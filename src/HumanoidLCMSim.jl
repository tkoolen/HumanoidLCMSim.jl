__precompile__()

module HumanoidLCMSim

export # types
    HumanoidRobotInfo,
    LCMController,
    Actuator,
    Side

export # functions
    parse_actuators

export # submodules
    Sides,
    AtlasSim

using Compat
using RigidBodySim
using RigidBodyDynamics
using RigidBodyTreeInspector
using OrdinaryDiffEq
using DiffEqCallbacks
using StaticArrays
using Rotations
using LCMCore
using BufferedStreams
using LightXML
using BotCoreLCMTypes
import DataStructures: OrderedDict

include("gains.jl")
include("sides.jl")

using .Sides

include("humanoid_robot_info.jl")
include("control.jl")

include("atlas.jl")

end # module
