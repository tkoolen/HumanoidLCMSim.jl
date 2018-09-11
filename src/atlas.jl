module AtlasSim

using Compat
using Compat.LinearAlgebra
using RigidBodySim
using HumanoidLCMSim
using RigidBodyDynamics
using RigidBodyDynamics.Contact

import AtlasRobot

using LCMCore: LCM, publish
using DiffEqBase: CallbackSet, solve, init
using OrdinaryDiffEq: Tsit5
using MechanismGeometries: URDFVisuals
using Blink: Window

function addflatground!(mechanism::Mechanism)
    frame = root_frame(mechanism)
    ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
    add_environment_primitive!(mechanism, ground)
    mechanism
end

function atlasrobotinfo(mechanism::Mechanism)
    actuators = [Actuator(string(j), JointID(j)) for j in tree_joints(mechanism) if num_positions(j) == 1]
    feet = Dict(
        Sides.left => findbody(mechanism, "l_foot"),
        Sides.right => findbody(mechanism, "r_foot"))
    hands = Dict(
        Sides.left => findbody(mechanism, "l_hand"),
        Sides.right => findbody(mechanism, "r_hand"))
    HumanoidRobotInfo(mechanism, feet, hands, actuators)
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

function send_init_messages(state::MechanismState, receiver::LCMControlReceiver)
    HumanoidLCMSim.publish_robot_state(receiver, 0.0, state)
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
    if headless
        controls = SimulationControls()
        open(controls, Window())
        callback = CallbackSet(callback, CallbackSet(controls))
    else
        visuals = URDFVisuals(AtlasRobot.urdfpath(); package_path = [AtlasRobot.packagepath()])
        gui = GUI(state.mechanism, visuals)
        open(gui)
        Compat.copyto!(gui.visualizer, state)
        callback = CallbackSet(callback, CallbackSet(gui))
    end
    callback
end

function simulate(dynamics::Dynamics, state0, final_time, callback)
    problem = ODEProblem(dynamics, state0, (0., final_time), callback = callback)
    integrator = init(problem, Tsit5(); abs_tol = 1e-8, dt = 1e-6, dtmin = 0.0)
    walltime = @elapsed solve!(integrator)
    integrator.sol, walltime
end

"""
    run(; controlΔt::Float64 = 1 / 300; headless = false)

Start a simulation of the Atlas robot, controlled using a `HumanoidLCMSim.LCMControlReceiver` running
at a rate of `1 / controlΔt` Hertz.

# Examples

Default usage:

```julia
using HumanoidLCMSim; AtlasSim.run()
```
"""
function run(; final_time = Inf, controlΔt::Float64 = 1 / 300, headless = false, max_rate = Inf)
    cpu_threads = VERSION >= v"0.7-" ? Sys.CPU_THREADS : Sys.CPU_CORES
    BLAS.set_num_threads(max(floor(Int, cpu_threads / 2  - 1), 1)) # leave some cores for other processes
    mechanism = addflatground!(AtlasRobot.mechanism())
    info = atlasrobotinfo(mechanism)
    state0 = MechanismState(mechanism)
    initialize!(state0, info)
    receiver = LCMControlReceiver(info;
        robot_state_channel="EST_ROBOT_STATE",
        robot_command_channel="ATLAS_COMMAND")
    pcontroller = PeriodicController(controlΔt, receiver)
    control! = let pcontroller = pcontroller
        function (τ, t, state)
            pcontroller(τ, t, state)
            # add some damping. TODO: do in a more appropriate place
            v = velocity(state)
            τ .-= 1.0 .* v
            τ
        end
    end
    send_init_messages(state0, receiver)
    callback = CallbackSet(make_callback(state0, headless, max_rate), PeriodicCallback(pcontroller))
    sol, walltime = simulate(Dynamics(mechanism, control!), state0, final_time, callback)
    simtime = sol.t[end]
    println("Simulated $simtime s in $walltime s ($(simtime / walltime) × realtime).")
    sol
end

end # module
