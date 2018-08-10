function set!(msg::vector_3d_t, trans::AbstractVector)
    @boundscheck checkbounds(trans, 1:3)
    @inbounds begin
        msg.x = trans[1]
        msg.y = trans[2]
        msg.z = trans[3]
    end
    msg
end

function set!(msg::quaternion_t, rot::Rotation{3})
    quat = Quat(rot)
    msg.w = quat.w
    msg.x = quat.x
    msg.y = quat.y
    msg.z = quat.z
    msg
end

function set!(msg::position_3d_t, tf::Transform3D)
    set!(msg.translation, translation(tf))
    set!(msg.rotation, rotation(tf))
    msg
end

function set!(msg::twist_t, twist::Twist)
    set!(msg.linear_velocity, linear(twist))
    set!(msg.angular_velocity, angular(twist))
    msg
end

function set!(msg::force_torque_t,
        left_foot_wrench::Wrench, right_foot_wrench::Wrench,
        left_hand_wrench::Wrench, right_hand_wrench::Wrench)
    msg.l_foot_force_z = linear(left_foot_wrench)[3]
    msg.l_foot_torque_x = angular(left_foot_wrench)[1]
    msg.l_foot_torque_y = angular(left_foot_wrench)[2]
    msg.r_foot_force_z = linear(right_foot_wrench)[3]
    msg.r_foot_torque_x = angular(right_foot_wrench)[1]
    msg.r_foot_torque_y = angular(right_foot_wrench)[2]
    msg.l_hand_force = linear(left_hand_wrench)
    msg.l_hand_torque = angular(left_hand_wrench)
    msg.r_hand_force = linear(right_hand_wrench)
    msg.r_hand_torque = angular(right_hand_wrench)
    msg
end

# Type piracy, but whatever:
StaticArrays.SVector(msg::vector_3d_t) = SVector(msg.x, msg.y, msg.z)
Rotations.Quat(msg::quaternion_t) = Quat(msg.w, msg.x, msg.y, msg.z)

function contact_wrench_in_body_frame(state::MechanismState, result::DynamicsResult, body::RigidBody)
    transform(RigidBodyDynamics.contact_wrench(result, body), inv(transform_to_root(state, body)))
end

function set!(msg::robot_state_t, result::DynamicsResult, robot_info::HumanoidRobotInfo,
        τprev::AbstractVector, t::Number, state::MechanismState)
    # time
    msg.utime = floor(Int, t * 1e6)

    # pose
    floatingbody = robot_info.floatingbody
    pose = transform_to_root(state, floatingbody)
    set!(msg.pose, pose)

    # twist
    # twist in message should be expressed in world-aligned body frame (for some reason)
    twist = twist_wrt_world(state, robot_info.floatingbody)
    to_world_aligned_floating_body = Transform3D(pose.to, robot_info.world_aligned_floating_body_frame, -translation(pose))
    twist = transform(twist, to_world_aligned_floating_body)
    set!(msg.twist, twist)

    # state info for actuated joints
    q = configuration(state)
    v = velocity(state)
    for i = 1 : msg.num_joints
        actuator = robot_info.actuators[i]
        position_ind = range_to_ind(configuration_range(state, actuator.jointid))
        velocity_ind = range_to_ind(velocity_range(state, actuator.jointid))
        msg.joint_position[i] = q[position_ind]
        msg.joint_velocity[i] = v[velocity_ind]
        msg.joint_effort[i] = τprev[velocity_ind]
        i += 1
    end

    # contact wrenches
    RigidBodyDynamics.contact_dynamics!(result, state)
    set!(msg.force_torque,
        contact_wrench_in_body_frame(state, result, robot_info.feet[Sides.left]),
        contact_wrench_in_body_frame(state, result, robot_info.feet[Sides.right]),
        contact_wrench_in_body_frame(state, result, robot_info.hands[Sides.left]),
        contact_wrench_in_body_frame(state, result, robot_info.hands[Sides.right]))

    msg
end

function set!(τ::AbstractVector, Δt::Number, state::MechanismState, τprev::AbstractVector,
        msg::atlas_command_t, robot_info::HumanoidRobotInfo)
    τ .= 0
    q = configuration(state)
    v = velocity(state)
    for i = 1 : msg.num_joints
        actuator = findactuator(robot_info, msg.joint_names[i])
        position_ind = range_to_ind(configuration_range(state, actuator.jointid))
        velocity_ind = range_to_ind(velocity_range(state, actuator.jointid))
        gains = LowLevelJointGains(
            msg.k_q_p[i],
            msg.k_q_i[i],
            msg.k_qd_p[i],
            msg.k_f_p[i],
            msg.ff_qd[i],
            msg.ff_qd_d[i],
            msg.ff_f_d[i],
            msg.ff_const[i])
        joint_state = JointState(q[position_ind], v[velocity_ind], τprev[velocity_ind])
        joint_state_des = JointState(msg.position[i], msg.velocity[i], msg.effort[i])
        torque_scale = msg.k_effort[i] / typemax(UInt8)
        τ[velocity_ind] = torque_scale * command_effort(gains, joint_state, joint_state_des, Δt)
    end
    τ
end

function set!(state::MechanismState, msg::robot_state_t, robot_info::HumanoidRobotInfo)
    floatingjoint = robot_info.floatingjoint
    world_aligned_floating_body_frame = robot_info.world_aligned_floating_body_frame

    trans = SVector(msg.pose.translation)
    rot = Quat(msg.pose.rotation)
    pose = Transform3D(frame_after(floatingjoint), frame_before(floatingjoint), rot, trans)
    set_configuration!(state, floatingjoint, pose)

    # twist in message is expressed in world-aligned body frame (for some reason)
    to_body = Transform3D(world_aligned_floating_body_frame, frame_after(floatingjoint), inv(rot))
    angular = SVector(msg.twist.angular_velocity)
    linear = SVector(msg.twist.linear_velocity)
    twist = Twist(frame_after(floatingjoint), frame_before(floatingjoint), world_aligned_floating_body_frame,
        angular, linear)
    twist = transform(twist, to_body)
    set_velocity!(state, floatingjoint, twist)

    msg.num_joints == length(robot_info.revolutejoints) || throw(ArgumentError("Number of joints incorrect."))
    for i = 1 : msg.num_joints
        joint = robot_info.revolutejoints[i]
        @boundscheck msg.joint_name[i] == joint.name || throw(ArgumentError("Joint name mismatch."))
        set_configuration!(state, joint, msg.joint_position[i])
        set_velocity!(state, joint, msg.joint_velocity[i])
    end
    state
end

function set!(msg::atlas_command_t, τ::AbstractVector, t::Float64,
        state_des::MechanismState, robot_info::HumanoidRobotInfo, desired_controller_period_ms::Integer)
        msg.utime = floor(Int, t * 1e6)
    q = configuration(state_des)
    v = velocity(state_des)
    for i = 1 : msg.num_joints
        # simple torque control for now; really, this method should take a Dict{Actuator, LowLevelJointGains}
        actuator = findactuator(robot_info, msg.joint_names[i])
        position_ind = range_to_ind(configuration_range(state_des, actuator.jointid))
        velocity_ind = range_to_ind(velocity_range(state_des, actuator.jointid))
        gains = LowLevelJointGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)

        msg.position[i] = q[position_ind]
        msg.velocity[i] = v[velocity_ind]
        msg.effort[i] = τ[velocity_ind]
        msg.k_q_p[i] = gains.k_q_p
        msg.k_q_i[i] = gains.k_q_i
        msg.k_qd_p[i] = gains.k_qd_p
        msg.k_f_p[i] = gains.k_f_p
        msg.ff_qd[i] = gains.ff_qd
        msg.ff_qd_d[i] = gains.ff_qd_d
        msg.ff_f_d[i] = gains.ff_f_d
        msg.ff_const[i] = gains.ff_const
        msg.k_effort[i] = typemax(UInt8) # TODO: ?
    end
    msg.desired_controller_period_ms = desired_controller_period_ms
    msg
end

# type piracy to avoid allocations when decoding robot_state_t
@generated function LCMCore.decodefield!(x::robot_state_t, io::IO)
    field_assignments = Vector{Expr}(undef, fieldcount(x))
    for (i, fieldname) in enumerate(fieldnames(x))
        F = fieldtype(x, fieldname)
        field_assignments[i] = if fieldname == :joint_name
            quote
                if isempty(x.joint_name)
                    # decode as normal the first time
                    LCMCore.resizearrayfield!(x, $(Val(fieldname)), $F)
                    LCMCore.decodefield!(x.joint_name, io)
                else
                    # check that bytes match joint names in robot_state_t
                    @inbounds for name in x.$fieldname
                        len = ntoh(read(io, UInt32))
                        length(name) == len - 1 || error("Mismatch in joint_name field.")
                        for j = 1 : len - 1
                            name[j] === Char(read(io, UInt8)) || error("Mismatch in joint_name field.")
                        end
                        read(io, UInt8) # strip off null
                    end
                end
            end
        else
            # just do the normal thing
            quote
                $F <: Array && LCMCore.resizearrayfield!(x, $(Val(fieldname)), $F)
                if LCMCore.decode_in_place($F)
                    LCMCore.decodefield!(x.$fieldname, io)
                else
                    x.$fieldname = LCMCore.decodefield(io, $F)
                end
            end
        end
    end
    return quote
        $(field_assignments...)
        return x
    end
end
