mutable struct ForceTorqueT <: LCMType
    l_foot_force_x::Float32
    l_foot_force_y::Float32
    l_foot_force_z::Float32
    l_foot_torque_x::Float32
    l_foot_torque_y::Float32
    l_foot_torque_z::Float32
    
    r_foot_force_x::Float32
    r_foot_force_y::Float32
    r_foot_force_z::Float32
    r_foot_torque_x::Float32
    r_foot_torque_y::Float32
    r_foot_torque_z::Float32

    l_hand_force::SVector{3, Float32}
    l_hand_torque::SVector{3, Float32}

    r_hand_force::SVector{3, Float32}
    r_hand_torque::SVector{3, Float32}
end

LCMCore.fingerprint(::Type{ForceTorqueT}) = SVector(0xba, 0x9c, 0x10, 0xf6, 0xd7, 0x67, 0x29, 0x20)
LCMCore.size_fields(::Type{ForceTorqueT}) = ()
Base.resize!(msg::ForceTorqueT) = nothing
LCMCore.check_valid(msg::ForceTorqueT) = nothing
