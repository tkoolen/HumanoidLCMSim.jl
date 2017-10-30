using ValkyrieLCMSim
using Base.Test
using LCMCore
using PyLCM
using BotCoreLCMTypes
using ValkyrieLCMSim.NativeBotCoreLCMTypes
using StaticArrays

function example_atlas_command_msg(rng = Base.Random.GLOBAL_RNG)
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

function example_force_torque_msg(rng = Base.Random.GLOBAL_RNG)
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

function test_lcm_message(lcmtype::LCMType, pylcmtype)
    bytes = LCMCore.encode(pylcmtype)
    decode!(lcmtype, bytes)
    for fieldname in fieldnames(lcmtype)
        @test pylcmtype[fieldname] == getfield(lcmtype, fieldname)
    end
    bytes_back = encode(lcmtype)
    @test bytes == bytes_back
end

@testset "atlas_command_t" begin
    test_lcm_message(NativeBotCoreLCMTypes.AtlasCommandT(), example_atlas_command_msg())
end

@testset "force_torque_t" begin
    test_lcm_message(NativeBotCoreLCMTypes.ForceTorqueT(), example_force_torque_msg())
end
