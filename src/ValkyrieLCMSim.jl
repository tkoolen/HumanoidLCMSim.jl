module ValkyrieLCMSim

export
    HumanoidRobotInfo,
    LCMController

using RigidBodySim
using Rotations
using LCMCore
using BufferedStreams
import RigidBodyDynamics: contact_dynamics!, contact_wrench

include("lcmtypes/lcmtypes.jl")

using .NativeBotCoreLCMTypes

include("gains.jl")
include("humanoid_robot_info.jl")
include("control.jl")

end # module
