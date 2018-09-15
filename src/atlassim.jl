module AtlasSim

using LinearAlgebra
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

function send_init_messages(state::MechanismState, receiver::LCMControlReceiver)
    HumanoidLCMSim.publish_robot_state(receiver, 0.0, state)
    utime = HumanoidLCMSim.utime_t()
    lcm = LCM()
    @info "Publishing START_MIT_STAND."
    publish(lcm, "START_MIT_STAND", utime)
    sleep(2)
    nothing
end

function make_callback(state::MechanismState, headless::Bool, max_rate; controls::Bool = true)
    callback = CallbackSet()
    if max_rate < Inf
        callback = CallbackSet(callback, RealtimeRateLimiter(max_rate = max_rate))
    end
    if headless
        if controls
            controls = SimulationControls()
            open(controls, Window())
            callback = CallbackSet(callback, CallbackSet(controls))
        end
    else
        visuals = URDFVisuals(AtlasRobot.urdfpath(); package_path = [AtlasRobot.packagepath()])
        gui = GUI(state.mechanism, visuals)
        open(gui)
        copyto!(gui.visualizer, state)
        callback = CallbackSet(callback, CallbackSet(gui))
    end
    callback
end

function simulate(dynamics::Dynamics, state0, final_time, callback)
    problem = ODEProblem(dynamics, state0, (0., final_time), callback = callback)
    integrator = init(problem, Tsit5(); abs_tol = 1e-8, dt = 1e-6, dtmin = 0.0)
    @info "Starting simulation."
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
function run(; final_time = Inf, controlΔt::Float64 = 1 / 300, headless = false, max_rate = Inf, controls = true)
    BLAS.set_num_threads(max(floor(Int, Sys.CPU_THREADS / 2  - 1), 1)) # leave some cores for other processes
    mechanism = AtlasRobot.mechanism(add_flat_ground=true)
    info = atlasrobotinfo(mechanism)
    state0 = MechanismState(mechanism)
    AtlasRobot.setnominal!(state0)
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
    callback = CallbackSet(make_callback(state0, headless, max_rate; controls = controls), 
        PeriodicCallback(pcontroller))
    sol, walltime = simulate(Dynamics(mechanism, control!), state0, final_time, callback)
    simtime = sol.t[end]
    println("Simulated $simtime s in $walltime s ($(simtime / walltime) × realtime).")
    sol
end

end # module
