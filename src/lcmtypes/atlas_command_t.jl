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

LCMCore.fingerprint(::Type{AtlasCommandT}) = SVector(0x36, 0x60, 0xf8, 0xc2, 0x34, 0x8e, 0x35, 0x12)
LCMCore.size_fields(::Type{AtlasCommandT}) = (:num_joints,)

function Base.resize!(msg::AtlasCommandT)
    resize!(msg.joint_names, msg.num_joints)
    resize!(msg.position, msg.num_joints)
    resize!(msg.velocity, msg.num_joints)
    resize!(msg.effort, msg.num_joints)
    resize!(msg.k_q_p, msg.num_joints)
    resize!(msg.k_q_i, msg.num_joints)
    resize!(msg.k_qd_p, msg.num_joints)
    resize!(msg.k_f_p, msg.num_joints)
    resize!(msg.ff_qd, msg.num_joints)
    resize!(msg.ff_qd_d, msg.num_joints)
    resize!(msg.ff_f_d, msg.num_joints)
    resize!(msg.ff_const, msg.num_joints)
    resize!(msg.k_effort, msg.num_joints)
end

function LCMCore.check_valid(msg::AtlasCommandT)
    @assert length(msg.joint_names) == msg.num_joints
    @assert length(msg.position) == msg.num_joints
    @assert length(msg.velocity) == msg.num_joints
    @assert length(msg.effort) == msg.num_joints
    @assert length(msg.k_q_p) == msg.num_joints
    @assert length(msg.k_q_i) == msg.num_joints
    @assert length(msg.k_qd_p) == msg.num_joints
    @assert length(msg.k_f_p) == msg.num_joints
    @assert length(msg.ff_qd) == msg.num_joints
    @assert length(msg.ff_qd_d) == msg.num_joints
    @assert length(msg.ff_f_d) == msg.num_joints
    @assert length(msg.ff_const) == msg.num_joints
    @assert length(msg.k_effort) == msg.num_joints
end
