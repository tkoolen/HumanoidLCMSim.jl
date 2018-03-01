using Revise
using HumanoidLCMSim
using RigidBodySim
using RigidBodyDynamics
using AtlasRobot
using OrdinaryDiffEq
using DiffEqCallbacks
using RigidBodyTreeInspector
using RigidBodyDynamics.Contact
using LCMCore: LCM, publish

mechanism = AtlasRobot.mechanism(remove_fixed_tree_joints = false)
remove_fixed_tree_joints!(mechanism);
actuatorconfig = old_actuator_config(mechanism);

# add environment
rootframe = root_frame(mechanism)
ground = HalfSpace3D(Point3D(rootframe, 0., 0., 0.), FreeVector3D(rootframe, 0., 0., 1.))
add_environment_primitive!(mechanism, ground);

# Create visualizer
any_open_visualizer_windows() || (new_visualizer_window(); sleep(1));
vis = Visualizer()[:atlas]
setgeometry!(vis, mechanism, RigidBodyTreeInspector.parse_urdf(AtlasRobot.urdfpath(), mechanism; package_path = [AtlasRobot.packagepath()]))

feet = Dict(Sides.left => findbody(mechanism, "l_foot"), Sides.right => findbody(mechanism, "r_foot"))
hands = Dict(Sides.left => findbody(mechanism, "l_hand"), Sides.right => findbody(mechanism, "r_hand"))
robot_info = HumanoidRobotInfo(mechanism, feet, hands, actuatorconfig);

state = MechanismState(mechanism)

function initialize(state::MechanismState, atlas::Mechanism, vis::Visualizer)
    zero!(state)
    kneebend = 1.1
    hipbendextra = 0.1
    for sideprefix in ('l', 'r')
        set_configuration!(state, findjoint(atlas, "$(sideprefix)_leg_kny"), [kneebend])
        set_configuration!(state, findjoint(atlas, "$(sideprefix)_leg_hpy"), [-kneebend / 2 + hipbendextra])
        set_configuration!(state, findjoint(atlas, "$(sideprefix)_leg_aky"), [-kneebend / 2 - hipbendextra])
    end
    floatingjoint = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(state, floatingjoint, [1; 0; 0; 0; 0; 0; 0.85])
    settransform!(vis, state)
end

lcmcontroller = LCMController(robot_info, robot_state_channel="EST_ROBOT_STATE", robot_command_channel="ATLAS_COMMAND")
controller = PeriodicController(1 / 300, lcmcontroller)
initialize(state, mechanism, vis)
final_time = 1000.
problem = ODEProblem(state, (0., final_time), controller)

HumanoidLCMSim.publish_robot_state(lcmcontroller, 0.0, state)

msg = HumanoidLCMSim.UtimeT()
lcm = LCM()
publish(lcm, "START_MIT_STAND", msg)
sleep(2)

sol = solve(problem, Tsit5(), abs_tol = 1e-10, dt = 0.05, callback = CallbackSet(vis, state));
