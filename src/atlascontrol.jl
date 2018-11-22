module AtlasControl

using LinearAlgebra
using QPControl
using RigidBodyDynamics
using AtlasRobot
using HumanoidLCMSim
using MathOptInterface

import OSQP
using OSQP.MathOptInterfaceOSQP: OSQPSettings

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

function run(; async=false)
    # load URDF
    mechanism = AtlasRobot.mechanism(add_flat_ground=true)

    # create optimizer
    MOI = MathOptInterface
    optimizer = OSQP.Optimizer()
    MOI.set(optimizer, OSQPSettings.Verbose(), false)
    MOI.set(optimizer, OSQPSettings.EpsAbs(), 1e-5)
    MOI.set(optimizer, OSQPSettings.EpsRel(), 1e-5)
    MOI.set(optimizer, OSQPSettings.MaxIter(), 5000)
    MOI.set(optimizer, OSQPSettings.AdaptiveRhoInterval(), 25) # required for deterministic behavior

    # create low level controller
    info = AtlasSim.atlasrobotinfo(mechanism)
    lowlevel = MomentumBasedController{4}(mechanism, optimizer, floatingjoint=info.floatingjoint)
    for body in bodies(mechanism)
        for point in RigidBodyDynamics.contact_points(body)
            position = RigidBodyDynamics.Contact.location(point)
            normal = FreeVector3D(default_frame(body), 0.0, 0.0, 1.0)
            μ = point.model.friction.μ
            contact = addcontact!(lowlevel, body, position, normal, μ)
            contact.maxnormalforce[] = 1e6 # TODO
            contact.weight[] = 1e-3
        end
    end

    # create standing controller
    nominalstate = MechanismState(mechanism)
    AtlasRobot.setnominal!(nominalstate)
    pelvis = successor(info.floatingjoint, mechanism)
    comref = center_of_mass(nominalstate) - FreeVector3D(root_frame(mechanism), 0., 0., 0.05)
    standingcontroller = StandingController(
        lowlevel, collect(values(info.feet)), pelvis, nominalstate, comref=comref);

    # create LCM control publisher
    publisher = LCMControlPublisher(info, standingcontroller;
        robot_state_channel="EST_ROBOT_STATE",
        robot_command_channel="ATLAS_COMMAND");

    # start listening for state messages and publishing command messages in response
    @info "Starting control loop."
    handle(publisher, async=async)
end

end
