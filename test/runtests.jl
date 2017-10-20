using ValkyrieLCMSim
using Base.Test
using PyLCM
using BotCoreLCMTypes
# using LCMCore

function example_atlas_command_msg(rng = Base.Random.GLOBAL_RNG)
    msg = bot_core[:atlas_command_t]()

    utime = 234
    num_joints = 2

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

@testset "atlas_command_t" begin
    import ValkyrieLCMSim: AtlasCommandT, decode!, encode

    pymsg = example_atlas_command_msg()
    bytes = LCMCore.encode(pymsg)
    cmd = AtlasCommandT()
    decode!(cmd, bytes)

    @test pymsg[:utime] == cmd.utime
    @test pymsg[:num_joints] == cmd.num_joints
    @test pymsg[:position] == cmd.position
    @test pymsg[:velocity] == cmd.velocity
    @test pymsg[:effort] == cmd.effort
    @test pymsg[:k_q_p] == cmd.k_q_p
    @test pymsg[:k_q_i] == cmd.k_q_i
    @test pymsg[:k_qd_p] == cmd.k_qd_p
    @test pymsg[:k_f_p] == cmd.k_f_p
    @test pymsg[:ff_qd] == cmd.ff_qd
    @test pymsg[:ff_qd_d] == cmd.ff_qd_d
    @test pymsg[:ff_f_d] == cmd.ff_f_d
    @test pymsg[:ff_const] == cmd.ff_const
    @test pymsg[:k_effort] == cmd.k_effort
    @test pymsg[:desired_controller_period_ms] == cmd.desired_controller_period_ms

    bytes_back = encode(cmd)
    @test bytes == bytes_back
end
