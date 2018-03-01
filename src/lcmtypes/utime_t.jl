mutable struct UtimeT <: LCMType
    utime::Int64
end

LCMCore.fingerprint(::Type{UtimeT}) = SVector(0x4d, 0xd, 0x41, 0xc1, 0xf1, 0x5, 0xb1, 0x2f)
LCMCore.size_fields(::Type{UtimeT}) = ()
Base.resize!(msg::UtimeT) = nothing
LCMCore.check_valid(msg::UtimeT) = nothing
