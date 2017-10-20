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

    function AtlasCommandT()
        new(0, Int32(0), String[], Float64[], Float64[], Float64[], Float64[], Float64[],
        Float64[], Float64[], Float64[], Float64[], Float64[], Float64[], UInt8[], UInt8(0))
    end
end

fingerprint(::Type{AtlasCommandT}) = SVector(0x36, 0x60, 0xf8, 0xc2, 0x34, 0x8e, 0x35, 0x12)

function check_valid(cmd::AtlasCommandT)
    @assert length(cmd.joint_names) == cmd.num_joints
    @assert length(cmd.position) == cmd.num_joints
    @assert length(cmd.velocity) == cmd.num_joints
    @assert length(cmd.effort) == cmd.num_joints
    @assert length(cmd.k_q_p) == cmd.num_joints
    @assert length(cmd.k_q_i) == cmd.num_joints
    @assert length(cmd.k_qd_p) == cmd.num_joints
    @assert length(cmd.k_f_p) == cmd.num_joints
    @assert length(cmd.ff_qd) == cmd.num_joints
    @assert length(cmd.ff_qd_d) == cmd.num_joints
    @assert length(cmd.ff_f_d) == cmd.num_joints
    @assert length(cmd.ff_const) == cmd.num_joints
    @assert length(cmd.k_effort) == cmd.num_joints
end

function decode!(cmd::AtlasCommandT, io::IO)
    check_fingerprint(io, typeof(cmd))
    cmd.utime = decode!(cmd.utime, io)
    cmd.num_joints = decode!(cmd.num_joints, io)
    resize!(cmd.joint_names, cmd.num_joints); decode!(cmd.joint_names, io)
    resize!(cmd.position, cmd.num_joints); decode!(cmd.position, io)
    resize!(cmd.velocity, cmd.num_joints); decode!(cmd.velocity, io)
    resize!(cmd.effort, cmd.num_joints); decode!(cmd.effort, io)
    resize!(cmd.k_q_p, cmd.num_joints); decode!(cmd.k_q_p, io)
    resize!(cmd.k_q_i, cmd.num_joints); decode!(cmd.k_q_i, io)
    resize!(cmd.k_qd_p, cmd.num_joints); decode!(cmd.k_qd_p, io)
    resize!(cmd.k_f_p, cmd.num_joints); decode!(cmd.k_f_p, io)
    resize!(cmd.ff_qd, cmd.num_joints); decode!(cmd.ff_qd, io)
    resize!(cmd.ff_qd_d, cmd.num_joints); decode!(cmd.ff_qd_d, io)
    resize!(cmd.ff_f_d, cmd.num_joints); decode!(cmd.ff_f_d, io)
    resize!(cmd.ff_const, cmd.num_joints); decode!(cmd.ff_const, io)
    resize!(cmd.k_effort, cmd.num_joints); decode!(cmd.k_effort, io)
    cmd.desired_controller_period_ms = decode!(cmd.desired_controller_period_ms, io)
    cmd
end
