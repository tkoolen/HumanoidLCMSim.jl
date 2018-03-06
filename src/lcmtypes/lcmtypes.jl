module NativeBotCoreLCMTypes # TODO

export
    UtimeT,
    Vector3DT,
    QuaternionT,
    TwistT,
    Position3DT,
    ForceTorqueT,
    AtlasCommandT,
    RobotStateT

using LCMCore, StaticArrays

include("utime_t.jl")
include("vector_3d_t.jl")
include("quaternion_t.jl")
include("twist_t.jl")
include("position_3d_t.jl")
include("force_torque_t.jl")
include("atlas_command_t.jl")
include("robot_state_t.jl")

end
