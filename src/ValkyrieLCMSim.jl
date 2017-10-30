module ValkyrieLCMSim

export
    MITQPLCMController

using RigidBodySim
import DataStructures.OrderedDict

include("lcmtypes/lcmtypes.jl")
include("gains.jl")
# include("control.jl")

end # module
