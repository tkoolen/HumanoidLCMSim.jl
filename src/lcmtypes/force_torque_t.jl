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

@lcmtypesetup ForceTorqueT
