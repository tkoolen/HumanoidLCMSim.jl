mutable struct QuaternionT <: LCMType
    w::Float64
    x::Float64
    y::Float64
    z::Float64
end

@lcmtypesetup QuaternionT
