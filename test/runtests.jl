module HumanoidLCMSimTests

module SideTest
using Compat.Test
using HumanoidLCMSim

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
using Compat.Test
using HumanoidLCMSim
using ValkyrieRobot
using RigidBodyDynamics
using BotCoreLCMTypes
import HumanoidLCMSim: set!

@testset "robot_side_t -> MechanismState" begin
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
    τprev[:] = 0
    t = 1.0
    set!(msg, result, robot_info, τprev, t, state)
    state_back = MechanismState(mechanism)
    set!(state_back, msg, robot_info)

    # terrible precision due to float32:
    @test configuration(state) ≈ configuration(state_back) atol=1e-7
    @test velocity(state) ≈ velocity(state_back) atol=1e-7
end
end # module


module AtlasTest
using Compat.Test
using HumanoidLCMSim

@testset "atlas" begin
    # just make sure we get all the way to simulating:
    @test_throws HumanoidLCMSim.NoCommandError AtlasSim.run(headless = true)
end
end # module

include("notebooks.jl")

end # module
