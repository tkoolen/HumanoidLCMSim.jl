struct LCMController
    result::DynamicsResult{Float64, Float64}
    tprev::Base.RefValue{Float64}
    τprev::Vector{Float64}
    robot_info::HumanoidRobotInfo{Float64}
    lcm::LCM
    robot_state_channel::String
    robot_state_msg::RobotStateT
    atlas_command_msg::AtlasCommandT
    encodebuffer::IOBuffer
    new_command::Base.RefValue{Bool}
end

# TODO: reduce cost of encoding

function LCMController(robot_info::HumanoidRobotInfo;
        robot_state_channel::String = "EST_ROBOT_STATE",
        robot_command_channel::String = "ROBOT_COMMAND")
    mechanism = robot_info.mechanism
    result = DynamicsResult(mechanism)
    tprev = Ref(0.)
    τprev = zeros(num_velocities(mechanism))
    lcm = LCM()
    robot_state_msg = RobotStateT()
    atlas_command_msg = AtlasCommandT()
    encodebuffer = IOBuffer(false, true)
    controller = LCMController(result, tprev, τprev, robot_info, lcm, robot_state_channel, robot_state_msg, atlas_command_msg, encodebuffer, Ref(false))

    for joint in tree_joints(mechanism)
        num_velocities(joint) == 1 || continue
        push!(robot_state_msg.joint_name, string(joint))
    end
    robot_state_msg.num_joints = length(robot_state_msg.joint_name)
    resize!(robot_state_msg)

    subscribe(lcm, robot_command_channel, (channel, data) -> handle_robot_command_msg(controller, data))

    controller
end

function initialize!(controller::LCMController)
    controller.tprev[] = 0
    controller.τprev[:] = 0
    nothing
end

function handle_robot_command_msg(controller::LCMController, data::Vector{UInt8})
    io = BufferedInputStream(data)
    msg = controller.atlas_command_msg
    decode!(msg, io)
    controller.new_command[] = true
end

range_to_ind(range) = (@assert length(range) == 1; first(range))

function set!(msg::Vector3DT, trans::AbstractVector)
    @boundscheck checkbounds(trans, 1:3)
    @inbounds begin
        msg.x = trans[1]
        msg.y = trans[2]
        msg.z = trans[3]
    end
    msg
end

function set!(msg::QuaternionT, rot::Rotation{3})
    quat = Quat(rot)
    msg.w = quat.w
    msg.x = quat.x
    msg.y = quat.y
    msg.z = quat.z
    msg
end

function set!(msg::Position3DT, tf::Transform3D)
    set!(msg.translation, translation(tf))
    set!(msg.rotation, rotation(tf))
    msg
end

function set!(msg::TwistT, twist::Twist)
    set!(msg.linear_velocity, linear(twist))
    set!(msg.angular_velocity, angular(twist))
    msg
end

function set!(msg::ForceTorqueT, left_foot_wrench::Wrench, right_foot_wrench::Wrench, left_hand_wrench::Wrench, right_hand_wrench::Wrench)
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

function contact_wrench_in_body_frame(state::MechanismState, result::DynamicsResult, body::RigidBody)
    transform(RigidBodyDynamics.contact_wrench(result, body), inv(transform_to_root(state, body)))
end

function set!(msg::RobotStateT, result::DynamicsResult, robot_info::HumanoidRobotInfo, τprev::AbstractVector, t::Number, state::MechanismState)
    # time
    msg.utime = floor(Int, t * 1e6)

    # pose
    floating_body = robot_info.floating_body
    pose = transform_to_root(state, floating_body)
    set!(msg.pose, pose)

    # twist
    # twist in message should be expressed in world-aligned body frame (for some reason)
    twist = twist_wrt_world(state, robot_info.floating_body)
    to_world_aligned_floating_body = Transform3D(pose.to, robot_info.world_aligned_floating_body_frame, -translation(pose))
    twist = transform(twist, to_world_aligned_floating_body)
    set!(msg.twist, twist)

    # state info for actuated joints
    i = 1
    for joint in tree_joints(robot_info.mechanism)
        num_velocities(joint) == 1 || continue
        velocity_index = range_to_ind(velocity_range(state, joint))
        msg.joint_position[i] = configuration(state, joint)[1]
        msg.joint_velocity[i] = velocity(state, joint)[1]
        msg.joint_effort[i] = τprev[velocity_index]
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

function compute_torques!(τ::AbstractVector, Δt::Number, state::MechanismState, τprev::AbstractVector, msg::AtlasCommandT, robot_info::HumanoidRobotInfo)
    τ[:] = 0
    for i = 1 : msg.num_joints
        jointid = findjointid(robot_info, findactuator(robot_info, msg.joint_names[i]))
        velocity_ind = range_to_ind(velocity_range(state, jointid))
        gains = LowLevelJointGains(
            msg.k_q_p[i],
            msg.k_q_i[i],
            msg.k_qd_p[i],
            msg.k_f_p[i],
            msg.ff_qd[i],
            msg.ff_qd_d[i],
            msg.ff_f_d[i],
            msg.ff_const[i])
        joint_state = JointState(configuration(state, jointid)[1], velocity(state, jointid)[1], τprev[velocity_ind])
        joint_state_des = JointState(msg.position[i], msg.velocity[i], msg.effort[i])
        τ[velocity_ind] = command_effort(gains, joint_state, joint_state_des, Δt)
    end
    τ
end

function publish_robot_state(controller::LCMController, t::Number, state::MechanismState)
    set!(controller.robot_state_msg, controller.result, controller.robot_info, controller.τprev, t, state)
    encode(controller.encodebuffer, controller.robot_state_msg)
    bytes = take!(controller.encodebuffer)
    publish(controller.lcm, controller.robot_state_channel, bytes)
end

struct NoCommandError <: Exception end
Base.showerror(io::IO, e::NoCommandError) = print(io, "Didn't receive a command message.")

function (controller::LCMController)(τ::AbstractVector, t::Number, state::MechanismState)
    # send state info on first tick to get things started
    if t == 0 # TODO: consider creating a separate flag in the controller for doing this
        publish_robot_state(controller, t, state)
        sleep(0.5); publish_robot_state(controller, t, state) # send a second time to work around a bug in the controller
    end

    # process command
    handle(controller.lcm, Dates.Second(1))
    controller.new_command[] || throw(NoCommandError())
    compute_torques!(τ, controller.tprev[] - t, state, controller.τprev, controller.atlas_command_msg, controller.robot_info)
    τ .-= 0.1 .* velocity(state) # FIXME: parse damping from URDF
    controller.new_command[] = false
    controller.tprev[] = t
    controller.τprev[:] = τ

    # send state info
    publish_robot_state(controller, t, state)
    τ
end

function RigidBodySim.PeriodicController(Δt::Number, controller::LCMController) # TODO: switch order?
    initialize = let controller = controller
        (c, t, u, integrator) -> initialize!(controller)
    end
    PeriodicController(zeros(controller.τprev), Δt, controller; initialize = initialize)
end
