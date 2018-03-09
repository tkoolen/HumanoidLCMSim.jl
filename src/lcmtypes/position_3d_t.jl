mutable struct Position3DT <: LCMType
    translation::Vector3DT
    rotation::QuaternionT
end

@lcmtypesetup Position3DT
