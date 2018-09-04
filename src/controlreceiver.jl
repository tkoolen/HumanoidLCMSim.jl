struct LCMControlReceiver
    result::DynamicsResult{Float64, Float64}
    tprev::Base.RefValue{Float64}
    τprev::Vector{Float64}
    robot_info::HumanoidRobotInfo{Float64}
    lcm::LCM
    robot_state_channel::String
    robot_state_msg::robot_state_t
    atlas_command_msg::atlas_command_t
    encodebuffer::FastWriteBuffer
    decodebuffer::FastReadBuffer
    new_command::Base.RefValue{Bool}
end

# TODO: reduce cost of encoding

function LCMControlReceiver(robot_info::HumanoidRobotInfo;
        robot_state_channel::String = "EST_ROBOT_STATE",
        robot_command_channel::String = "ROBOT_COMMAND")
    mechanism = robot_info.mechanism
    result = DynamicsResult(mechanism)
    tprev = Ref(0.)
    τprev = zeros(num_velocities(mechanism))
    lcm = LCM()
    robot_state_msg = robot_state_t()
    atlas_command_msg = atlas_command_t()
    encodebuffer = FastWriteBuffer()
    decodebuffer = FastReadBuffer()
    receiver = LCMControlReceiver(result, tprev, τprev, robot_info, lcm, robot_state_channel,
        robot_state_msg, atlas_command_msg, encodebuffer, decodebuffer, Ref(false))

    for joint in tree_joints(mechanism)
        num_velocities(joint) == 1 || continue
        push!(robot_state_msg.joint_name, joint.name)
    end
    robot_state_msg.num_joints = length(robot_state_msg.joint_name)
    resize!(robot_state_msg)

    let receiver = receiver
        sub = subscribe(lcm, robot_command_channel, (channel, data) -> handle_robot_command_msg(receiver, data))
        set_queue_capacity(sub, 1)
    end

    receiver
end

function initialize!(receiver::LCMControlReceiver)
    receiver.tprev[] = 0
    receiver.τprev .= 0
    nothing
end

function handle_robot_command_msg(receiver::LCMControlReceiver, data::Vector{UInt8})
    setdata!(receiver.decodebuffer, data)
    msg = receiver.atlas_command_msg
    decode!(msg, receiver.decodebuffer)
    receiver.new_command[] = true
end

function publish_robot_state(receiver::LCMControlReceiver, t::Number, state::MechanismState)
    set!(receiver.robot_state_msg, receiver.result, receiver.robot_info, receiver.τprev, t, state)
    encode(receiver.encodebuffer, receiver.robot_state_msg)
    bytes = take!(receiver.encodebuffer)
    publish(receiver.lcm, receiver.robot_state_channel, bytes)
end

struct NoCommandError <: Exception end
Base.showerror(io::IO, e::NoCommandError) = print(io, "Didn't receive a command message.")

function (receiver::LCMControlReceiver)(τ::AbstractVector, t::Number, state::MechanismState)
    # send state info on first tick to get things started
    if t == 0 # TODO: consider creating a separate flag in the controller for doing this
        publish_robot_state(receiver, t, state)
        sleep(0.5); publish_robot_state(receiver, t, state) # send a second time to work around a bug in the controller
    end

    # process command
    LCMCore.handle(receiver.lcm, Second(1))
    receiver.new_command[] || throw(NoCommandError())
    set!(τ, receiver.tprev[] - t, state, receiver.τprev, receiver.atlas_command_msg, receiver.robot_info)
    receiver.new_command[] = false
    receiver.tprev[] = t
    receiver.τprev .= τ

    # send state info
    publish_robot_state(receiver, t, state)
    τ
end

function RigidBodySim.PeriodicController(Δt::Number, receiver::LCMControlReceiver) # TODO: switch order?
    initialize = let receiver = receiver
        (c, t, u, integrator) -> initialize!(receiver)
    end
    PeriodicController(zero(receiver.τprev), Δt, receiver; initialize = initialize)
end
