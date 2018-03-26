module AtlasSim

using RigidBodySim
using HumanoidLCMSim
using RigidBodyDynamics
using RigidBodyDynamics.Contact
using DrakeVisualizer

import RigidBodyTreeInspector
import AtlasRobot
import LCMCore: LCM, publish
import DiffEqBase: CallbackSet, solve, init
import OrdinaryDiffEq: Tsit5
import DataStructures: OrderedDict

function addflatground!(mechanism::Mechanism)
    frame = root_frame(mechanism)
    ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
    add_environment_primitive!(mechanism, ground)
    mechanism
end

function atlasrobotinfo(mechanism::Mechanism)
    actuatorconfig = OrderedDict{Actuator, JointID}(
        Actuator(string(j)) => JointID(j) for j in  tree_joints(mechanism) if num_positions(j) == 1)
    feet = Dict(
        Sides.left => findbody(mechanism, "l_foot"),
        Sides.right => findbody(mechanism, "r_foot"))
    hands = Dict(
        Sides.left => findbody(mechanism, "l_hand"),
        Sides.right => findbody(mechanism, "r_hand"))
    HumanoidRobotInfo(mechanism, feet, hands, actuatorconfig)
end

function initialize!(state::MechanismState, info::HumanoidRobotInfo)
    mechanism = state.mechanism
    zero!(state)
    kneebend = 1.1
    hipbendextra = 0.1
    for sideprefix in ('l', 'r')
        knee = findjoint(mechanism, "$(sideprefix)_leg_kny")
        hippitch = findjoint(mechanism, "$(sideprefix)_leg_hpy")
        anklepitch = findjoint(mechanism, "$(sideprefix)_leg_aky")
        set_configuration!(state, knee, [kneebend])
        set_configuration!(state, hippitch, [-kneebend / 2 + hipbendextra])
        set_configuration!(state, anklepitch, [-kneebend / 2 - hipbendextra])
    end
    floatingjoint = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(state, floatingjoint, [1; 0; 0; 0; 0; 0; 0.85])
    state
end

function visualizer(mechanism::Mechanism)
    # Open a new RigidBodySim window if necessary
    any_open_visualizer_windows() || (new_visualizer_window(); sleep(1));

    # Create and set up Visualizer
    vis = Visualizer()[:atlas]
    # ellipsoids = RigidBodyTreeInspector.create_geometry(mechanism, show_inertias=true))
    meshgeometry = RigidBodyTreeInspector.parse_urdf(
        AtlasRobot.urdfpath(), mechanism; package_path = [AtlasRobot.packagepath()])
    setgeometry!(vis, mechanism, meshgeometry)

    vis
end

function send_init_messages(state::MechanismState, lcmcontroller::LCMController)
    HumanoidLCMSim.publish_robot_state(lcmcontroller, 0.0, state)
    utime = HumanoidLCMSim.utime_t()
    lcm = LCM()
    println("Publishing START_MIT_STAND")
    publish(lcm, "START_MIT_STAND", utime)
    sleep(2)
    nothing
end

function make_callback(state::MechanismState, headless::Bool, max_rate)
    callback = CallbackSet()
    if max_rate < Inf
        callback = CallbackSet(callback, RealtimeRateLimiter(max_rate = max_rate))
    end
    if !headless
        vis = visualizer(state.mechanism)
        settransform!(vis, state)
        callback = CallbackSet(callback, CallbackSet(vis, state))
    end
    callback
end

function simulate(dynamics::Dynamics, state0, callback)
    problem = ODEProblem(dynamics, state, (0., Inf), callback = callback)
    integrator = init(problem, Tsit5(); abs_tol = 1e-10, dtmin = 0.0)
    solve!(integrator)
end

"""
    run(; controlΔt::Float64 = 1 / 300; headless = false)

Start a simulation of the Atlas robot, controlled using a `HumanoidLCMSim.LCMController` running
at a rate of `1 / controlΔt` Hertz.

# Examples

Default usage:

```julia
using HumanoidLCMSim; AtlasSim.run()
```
"""
function run(; controlΔt::Float64 = 1 / 300, headless = false, max_rate = Inf)
    BLAS.set_num_threads(4) # leave some cores for other processes
    mechanism = addflatground!(AtlasRobot.mechanism())
    info = atlasrobotinfo(mechanism)
    state0 = MechanismState(mechanism)
    initialize!(state0, info)
    lcmcontroller = LCMController(info;
        robot_state_channel="EST_ROBOT_STATE",
        robot_command_channel="ATLAS_COMMAND")
    send_init_messages(state0, lcmcontroller)
    callback = make_callback(state0, headless, max_rate)
    simulate(Dynamics(mechanism, PeriodicController(controlΔt, lcmcontroller)), state0, callback)
end

end # module
