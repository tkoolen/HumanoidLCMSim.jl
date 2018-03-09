mutable struct Vector3DT <: LCMType
    x::Float64
    y::Float64
    z::Float64
end

@lcmtypesetup Vector3DT
