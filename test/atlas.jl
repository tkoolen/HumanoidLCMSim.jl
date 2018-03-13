using Compat.Test
using HumanoidLCMSim

# just make sure we get all the way to simulating:
@test_throws HumanoidLCMSim.NoCommandError AtlasSim.run(headless = true)
