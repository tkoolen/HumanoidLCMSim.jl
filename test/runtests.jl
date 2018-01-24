module HumanoidLCMSimTests

using RigidBodySim
using RigidBodyDynamics
using RigidBodyTreeInspector
using OrdinaryDiffEq
using DiffEqCallbacks
using LCMCore
using PyLCM
using BotCoreLCMTypes
using StaticArrays

using Base.Test

using HumanoidLCMSim
using HumanoidLCMSim.NativeBotCoreLCMTypes


rand_python_msg(::Type{T}) where {T <: LCMType} = rand_python_msg(Base.Random.GLOBAL_RNG, T)

function rand_python_msg(rng::AbstractRNG, ::Type{AtlasCommandT})
    utime = 234
    num_joints = 2
    msg = bot_core[:atlas_command_t]()
    msg[:utime] = utime
    msg[:num_joints] = num_joints
    msg[:joint_names] = [randstring(rng, rand(rng, 1 : 10)) for i = 1 : num_joints]
    msg[:position] = rand(rng, Float64, num_joints)
    msg[:velocity] = rand(rng, Float64, num_joints)
    msg[:effort] = rand(rng, Float64, num_joints)
    msg[:k_q_p] = rand(rng, Float64, num_joints)
    msg[:k_q_i] = rand(rng, Float64, num_joints)
    msg[:k_qd_p] = rand(rng, Float64, num_joints)
    msg[:k_f_p] = rand(rng, Float64, num_joints)
    msg[:ff_qd] = rand(rng, Float64, num_joints)
    msg[:ff_qd_d] = rand(rng, Float64, num_joints)
    msg[:ff_f_d] = rand(rng, Float64, num_joints)
    msg[:ff_const] = rand(rng, Float64, num_joints)
    msg[:k_effort] = rand(rng, UInt8, num_joints)
    msg[:desired_controller_period_ms] = rand(rng, UInt8)
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{ForceTorqueT})
    msg = bot_core[:force_torque_t]()
    msg[:l_foot_force_z] = rand(rng, Float32)
    msg[:l_foot_torque_x] = rand(rng, Float32)
    msg[:l_foot_torque_y] = rand(rng, Float32)
    msg[:r_foot_force_z] = rand(rng, Float32)
    msg[:r_foot_torque_x] = rand(rng, Float32)
    msg[:r_foot_torque_y] = rand(rng, Float32)
    msg[:l_hand_force] = rand(rng, SVector{3, Float32})
    msg[:l_hand_torque] = rand(rng, SVector{3, Float32})
    msg[:r_hand_force] = rand(rng, SVector{3, Float32})
    msg[:r_hand_torque] = rand(rng, SVector{3, Float32})
    msg
end

function test_lcm_message(::Type{T}) where T <: LCMType
    lcmtype = T()
    pylcmtype = rand_python_msg(T)
    bytes = LCMCore.encode(pylcmtype)
    decode!(lcmtype, bytes)
    for fieldname in fieldnames(lcmtype)
        @test pylcmtype[fieldname] == getfield(lcmtype, fieldname)
    end
    bytes_back = encode(lcmtype)
    @test bytes == bytes_back
end

@testset "LCMTypes" begin
    # for T in names(names(HumanoidLCMSim.NativeBotCoreLCMTypes))
    for T in (:AtlasCommandT, :ForceTorqueT)
        @eval test_lcm_message($T)
    end
end

end # module
