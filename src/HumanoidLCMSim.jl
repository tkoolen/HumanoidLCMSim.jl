__precompile__()

module HumanoidLCMSim

export # types
    HumanoidRobotInfo,
    LCMControlReceiver,
    LCMControlPublisher,
    Actuator,
    Side

export # functions
    parse_actuators,
    handle

export # submodules
    Sides,
    AtlasSim

using Compat
using RigidBodySim
using RigidBodyDynamics
using OrdinaryDiffEq
using DiffEqCallbacks
using StaticArrays
using Rotations
using LCMCore
using FastIOBuffers
using LightXML
using BotCoreLCMTypes
import DataStructures: OrderedDict
using Reexport

@reexport using JSExpr # FIXME: needed because of https://github.com/JuliaGizmos/JSExpr.jl/issues/13

include("gains.jl")
include("sides.jl")

using .Sides

include("util.jl")
include("humanoid_robot_info.jl")
include("lcm_interop.jl")
include("controlreceiver.jl")
include("controlpublisher.jl")

include("atlas.jl")

end # module
