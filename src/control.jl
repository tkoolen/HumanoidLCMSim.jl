
struct LCMDrakeQPController{T}
    result::DynamicsResult{T, T}
    τprev::Vector{T}
    gains::OrderedDict{GenericJoint{T}, LowLevelJointGains{T}}
    lcm::LCM
    robot_state_msg::PyCall.PyObject
end

function MITQPLCMController
    (mechanism::Mechanism{T}) where T
    result = DynamicsResult(mechanism)
    τprev = zeros(T, num_velocities(mechanism))
    gains = OrderedDict{GenericJoint{T}, LowLevelJointGains{T}}()
    lcm = LCM()
    robot_state_msg = bot_core[:robot_state_t]()

    # TODO:
#     subscribe(lcm, "MY_CHANNEL", handle_msg, bot_core[:robot_state_t])

    MITQPLCMController(result, τprev, gains, lcm, robot_state_msg)
end

function update_gains!(controller::MITQPLCMController, atlas_command_msg::PyCall.PyObject)
    msg = atlas_command_msg
    for i = length(controller.gains)
        msg[:joint_names]
    end
end

function (controller::MITQPLCMController)(τ::AbstractVector, t::Number, state::MechanismState)
    # compute contact wrenches
    RigidBodyDynamics.contact_dynamics!(controller.result, state)

    # populate robot_state_t
    robot_state_msg = controller.robot_state_msg
    robot_state_msg[:utime] = floor(Int, t * 1e3)

    # send robot_state_t
    publish(controller.lcm, "EST_ROBOT_STATE", robot_state_msg)

    # wait for atlas_command_t
#     while true
#         handle(lcm)
#     end
    msg = bot_core[:atlas_command_t]() # TODO

    # compute torques
    for i = 1 : msg[:num_joints]
        gains = LowLevelJointGains(
            msg[:k_q_p][i],
            msg[:k_q_i][i],
            msg[:k_qd_p][i],
            msg[:k_f_p][i],
            msg[:ff_qd][i],
            msg[:ff_qd_d][i],
            msg[:ff_qd][i])
    end

    τ[:] = 0
    for (joint, gains) in controller.gains
        fastview(τ, velocity_range(state, joint))[:] = command_effort(gains, qj, qj_des, τj, τj_des, Δt)
    end

    controller.τprev[:] = τ
    τ
end
