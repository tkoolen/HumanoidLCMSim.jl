mutable struct ForceTorqueT <: LCMType
    l_foot_force_z::Float32
    l_foot_torque_x::Float32
    l_foot_torque_y::Float32
    r_foot_force_z::Float32
    r_foot_torque_x::Float32
    r_foot_torque_y::Float32
    l_hand_force::SVector{3, Float32}
    l_hand_torque::SVector{3, Float32}
    r_hand_force::SVector{3, Float32}
    r_hand_torque::SVector{3, Float32}
end

LCMCore.fingerprint(::Type{ForceTorqueT}) = SVector(0x1e, 0xc5, 0x3c, 0x5d, 0x2c, 0x3c, 0x03, 0xf8)
LCMCore.size_fields(::Type{ForceTorqueT}) = ()
Base.resize!(msg::ForceTorqueT) = nothing
LCMCore.check_valid(msg::ForceTorqueT) = nothing
