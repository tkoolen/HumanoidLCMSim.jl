__precompile__()

module HumanoidLCMSim

export # types
    HumanoidRobotInfo,
    LCMControlReceiver,
    LCMControlPublisher,
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

include("util.jl")
include("humanoid_robot_info.jl")
include("lcm_interop.jl")
include("controlreceiver.jl")
include("controlpublisher.jl")

include("atlas.jl")

end # module
