mutable struct Position3DT <: LCMType
    translation::Vector3DT
    rotation::QuaternionT
end

LCMCore.fingerprint(::Type{Position3DT}) = SVector(0xee, 0x9f, 0xf4, 0x46, 0x47, 0xaf, 0x3f, 0x79)
LCMCore.size_fields(::Type{Position3DT}) = ()
Base.resize!(msg::Position3DT) = nothing
LCMCore.check_valid(msg::Position3DT) = nothing
