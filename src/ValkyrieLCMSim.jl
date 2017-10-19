module ValkyrieLCMSim

export
    MITQPLCMController

using RigidBodySim
using PyLCM
using BotCoreLCMTypes
import DataStructures.OrderedDict

include("gains.jl")
# include("control.jl")
include("lcmcontrib.jl")
include(joinpath("lcmtypes", "lcmtypes.jl"))

end # module
