mutable struct AtlasCommandT <: LCMType
    utime::Int64
    num_joints::Int32
    joint_names::Vector{String}
    position::Vector{Float64}
    velocity::Vector{Float64}
    effort::Vector{Float64}
    k_q_p::Vector{Float64}
    k_q_i::Vector{Float64}
    k_qd_p::Vector{Float64}
    k_f_p::Vector{Float64}
    ff_qd::Vector{Float64}
    ff_qd_d::Vector{Float64}
    ff_f_d::Vector{Float64}
    ff_const::Vector{Float64}
    k_effort::Vector{UInt8}
    desired_controller_period_ms::UInt8
end

@lcmtypesetup(AtlasCommandT,
    joint_names => (num_joints,),
    position => (num_joints,),
    velocity => (num_joints,),
    effort => (num_joints,),
    k_q_p => (num_joints,),
    k_q_i => (num_joints,),
    k_qd_p => (num_joints,),
    k_f_p => (num_joints,),
    ff_qd => (num_joints,),
    ff_qd_d => (num_joints,),
    ff_f_d => (num_joints,),
    ff_const => (num_joints,),
    k_effort => (num_joints,)
)
