module ValkyrieLCMSim

export
    MITQPLCMController

using RigidBodySim
import DataStructures.OrderedDict

include("gains.jl")
# include("control.jl")
include("lcmcontrib.jl")
include("lcmtypes/lcmtypes.jl")

end # module
