module HumanoidLCMSimTests

module SideTest
using Test, Random
using HumanoidLCMSim

if isdefined(Random, :seed!)
    using Random: seed!
else
    const seed! = srand
end

seed!(1)

@testset "side" begin
    @test -Sides.left == Sides.right
    @test -Sides.right == Sides.left

    sides = [rand(Side) for i = 1 : 10000];
    @test isapprox(count(side -> side == Sides.left, sides) / length(sides), 0.5; atol = 0.05)

    @test Sides.flipsign_if_right(2., Sides.left) == 2.
    @test Sides.flipsign_if_right(2., Sides.right) == -2.
end
end # module


module ControlTest
using Test
using Random
using HumanoidLCMSim
using ValkyrieRobot
using RigidBodyDynamics
using BotCoreLCMTypes
import HumanoidLCMSim: set!

@testset "robot_state_t -> MechanismState" begin
    mechanism = Valkyrie().mechanism
    remove_fixed_tree_joints!(mechanism)
    feet = Dict(Sides.left => findbody(mechanism, "leftFoot"), Sides.right => findbody(mechanism, "rightFoot"))
    hands = Dict(Sides.left => findbody(mechanism, "leftPalm"), Sides.right => findbody(mechanism, "rightPalm"))
    robot_info = HumanoidRobotInfo(mechanism, feet, hands, parse_actuators(mechanism, ValkyrieRobot.urdfpath()))
    msg = robot_state_t()
    for joint in tree_joints(mechanism)
        num_velocities(joint) == 1 || continue
        push!(msg.joint_name, string(joint))
    end
    msg.num_joints = length(msg.joint_name)
    resize!(msg)
    state = MechanismState(mechanism)
    rand!(state)
    result = DynamicsResult(mechanism)
    dynamics!(result, state)
    τprev = similar(velocity(state))
    τprev .= 0
    t = 1.0
    set!(msg, result, robot_info, τprev, t, state)
    state_back = MechanismState(mechanism)
    set!(state_back, msg, robot_info)

    # terrible precision due to float32:
    principal_value!(state)
    principal_value!(state_back)
    @test configuration(state) ≈ configuration(state_back) atol=1e-6
    @test velocity(state) ≈ velocity(state_back) atol=1e-6
end
end # module


module AtlasTest
using Test
using LinearAlgebra
using HumanoidLCMSim
using RigidBodyDynamics
using AtlasRobot

@testset "no controller" begin
    # just make sure we get all the way to simulating:
    @test_throws HumanoidLCMSim.NoCommandError AtlasSim.run(headless = true)
end

@testset "standing controller" begin
    # run controller
    AtlasControl.run(async=true)

    # run simulation
    sol = AtlasSim.run(controlΔt = 1 / 500, final_time = 5.0);

    # checks
    mechanism = AtlasRobot.mechanism(add_flat_ground=true)
    finalstate = MechanismState(mechanism)
    copyto!(finalstate, sol.u[end])
    @test sol.retcode == :Success
    @test center_of_mass(finalstate).v[3] ≈ 1.06 atol=5e-2
    @test norm(velocity(finalstate)) ≈ 0 atol=5e-3
end

end # module

end # module
