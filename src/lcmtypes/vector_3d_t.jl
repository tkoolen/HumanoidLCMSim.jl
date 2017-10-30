mutable struct Vector3DT <: LCMType
    x::Float64
    y::Float64
    z::Float64
end

LCMCore.fingerprint(::Type{Vector3DT}) = SVector(0xae, 0x7e, 0x5f, 0xba, 0x5e, 0xec, 0xa1, 0x1e)
LCMCore.size_fields(::Type{Vector3DT}) = ()
Base.resize!(msg::Vector3DT) = nothing
LCMCore.check_valid(msg::Vector3DT) = nothing
