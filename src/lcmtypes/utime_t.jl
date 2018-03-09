mutable struct UtimeT <: LCMType
    utime::Int64
end

@lcmtypesetup UtimeT
