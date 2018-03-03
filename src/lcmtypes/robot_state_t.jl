mutable struct RobotStateT <: LCMType
    utime::Int64
    pose::Position3DT
    twist::TwistT
    num_joints::Int16
    joint_name::Vector{String}
    joint_position::Vector{Float32}
    joint_velocity::Vector{Float32}
    joint_effort::Vector{Float32}
    force_torque::ForceTorqueT
end

LCMCore.fingerprint(::Type{RobotStateT}) = SVector(0x7e, 0xca, 0x9a, 0x4a, 0x9f, 0x35, 0x75, 0xc7)
LCMCore.size_fields(::Type{RobotStateT}) = (:num_joints,)

function Base.resize!(msg::RobotStateT)
    resize!(msg.joint_name, msg.num_joints)
    resize!(msg.joint_position, msg.num_joints)
    resize!(msg.joint_velocity, msg.num_joints)
    resize!(msg.joint_effort, msg.num_joints)
end

function LCMCore.check_valid(msg::RobotStateT)
    @assert length(msg.joint_name) == msg.num_joints
    @assert length(msg.joint_position) == msg.num_joints
    @assert length(msg.joint_velocity) == msg.num_joints
    @assert length(msg.joint_effort) == msg.num_joints
end
