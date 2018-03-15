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
