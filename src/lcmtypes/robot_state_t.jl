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

@lcmtypesetup(RobotStateT,
    joint_name => (num_joints,),
    joint_position => (num_joints,),
    joint_velocity => (num_joints,),
    joint_effort => (num_joints,)
)
