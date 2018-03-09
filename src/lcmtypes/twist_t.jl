mutable struct TwistT <: LCMType
    linear_velocity::Vector3DT
    angular_velocity::Vector3DT
end

@lcmtypesetup TwistT
