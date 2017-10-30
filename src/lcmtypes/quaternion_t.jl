mutable struct QuaternionT <: LCMType
    w::Float64
    x::Float64
    y::Float64
    z::Float64
end

LCMCore.fingerprint(::Type{QuaternionT}) = SVector(0x36, 0x5b, 0xdd, 0x4b, 0xf9, 0x10, 0x0a, 0x1f)
LCMCore.size_fields(::Type{QuaternionT}) = ()
Base.resize!(msg::QuaternionT) = nothing
LCMCore.check_valid(msg::QuaternionT) = nothing
