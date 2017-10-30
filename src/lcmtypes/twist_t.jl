mutable struct TwistT <: LCMType
    linear_velocity::Vector3DT
    angular_velocity::Vector3DT
end

LCMCore.fingerprint(::Type{TwistT}) = SVector(0x65, 0x05, 0xe8, 0xbe, 0xf0, 0x50, 0xb3, 0x4b)
LCMCore.size_fields(::Type{TwistT}) = ()
Base.resize!(msg::TwistT) = nothing
LCMCore.check_valid(msg::TwistT) = nothing
