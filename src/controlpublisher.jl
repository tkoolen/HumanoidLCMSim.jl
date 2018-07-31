struct LCMControlPublisher{M <:MechanismState, Tau, C}
    state::M
    controller::C
    desired_controller_period_ms::Int
    τ::Tau
    robot_info::HumanoidRobotInfo{Float64}
    lcm::LCM
    robot_command_channel::String
    atlas_command_msg::atlas_command_t
    robot_state_msg::robot_state_t
    encodebuffer::IOBuffer
end

function LCMControlPublisher(robot_info::HumanoidRobotInfo, controller::C;
        robot_state_channel::String = "EST_ROBOT_STATE",
        robot_command_channel::String = "ROBOT_COMMAND",
        desired_controller_period_ms = 1) where {C}
    mechanism = robot_info.mechanism
    state = MechanismState(mechanism)
    τ = similar(velocity(state))
    lcm = LCM()
    atlas_command_msg = atlas_command_t()
    robot_state_msg = robot_state_t()
    encodebuffer = IOBuffer(false, true)
    publisher = LCMControlPublisher(state, controller, desired_controller_period_ms, τ, robot_info,
        lcm, robot_command_channel, atlas_command_msg, robot_state_msg, encodebuffer)

    atlas_command_msg.num_joints = num_actuators(robot_info)
    resize!(atlas_command_msg)
    for (i, actuator) in enumerate(actuators(robot_info))
        atlas_command_msg.joint_names[i] = actuator.name
    end

    subscribe(lcm, robot_state_channel, (channel, data) -> handle_robot_state_msg(publisher, data))

    @async while true
        handle(lcm)
    end

    publisher
end

function handle_robot_state_msg(publisher::LCMControlPublisher, data::Vector{UInt8})
    io = BufferedInputStream(data) # TODO: allocations
    decode!(publisher.robot_state_msg, io)
    set!(publisher.state, publisher.robot_state_msg, publisher.robot_info)
    t = publisher.robot_state_msg.utime / 1e6
    publisher.controller(publisher.τ, t, publisher.state)
    state_des = publisher.state # TODO
    set!(publisher.atlas_command_msg, publisher.τ, t, state_des, publisher.robot_info, publisher.desired_controller_period_ms)
    encode(publisher.encodebuffer, publisher.atlas_command_msg)
    bytes = take!(publisher.encodebuffer)
    publish(publisher.lcm, publisher.robot_command_channel, bytes)
    nothing
end
