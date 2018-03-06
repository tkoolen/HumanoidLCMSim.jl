__precompile__()

module HumanoidLCMSim

export # types
    HumanoidRobotInfo,
    LCMController,
    Actuator,
    Side

export # functions
    parse_actuators,
    old_actuator_config

export # submodules
    Sides,
    AtlasSim

using RigidBodySim
using RigidBodyDynamics
using RigidBodyTreeInspector
using OrdinaryDiffEq
using DiffEqCallbacks
using Rotations
using LCMCore
using BufferedStreams
using LightXML
import DataStructures: OrderedDict

include("lcmtypes/lcmtypes.jl")
include("gains.jl")
include("sides.jl")

using .Sides
using .NativeBotCoreLCMTypes

include("humanoid_robot_info.jl")
include("control.jl")

include("atlas.jl")

end # module
