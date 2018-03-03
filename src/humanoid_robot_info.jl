struct Actuator
    name::String
end

function parse_actuators(mechanism::Mechanism{T}, urdffile::String) where T
    xdoc = parse_file(urdffile)
    xroot = LightXML.root(xdoc)
    @assert LightXML.name(xroot) == "robot"

    xml_transmissions = get_elements_by_tagname(xroot, "transmission")
    actuatorconfig = OrderedDict{Actuator, JointID}()

    for joint in tree_joints(mechanism) # order matters
        num_velocities(joint) == 0 && continue
        for xml_transmission in xml_transmissions
            xml_jointref = find_element(xml_transmission, "joint")
            if string(joint) == attribute(xml_jointref, "name")
                xml_actuator = find_element(xml_transmission, "actuator")
                xml_reduction = find_element(xml_actuator, "mechanicalReduction")
                xml_reduction == nothing || @assert parse(content(xml_reduction)) == 1
                actuator = Actuator(attribute(xml_actuator, "name"))
                actuatorconfig[actuator] = JointID(joint)
            end
        end
    end

    actuatorconfig
end

function old_actuator_config(mechanism::Mechanism)
    OrderedDict{Actuator, JointID}(Actuator(string(j)) => JointID(j) for j in tree_joints(mechanism) if num_positions(j) == 1)
end

struct HumanoidRobotInfo{T}
    mechanism::Mechanism{T}
    feet::Dict{Side, RigidBody{T}}
    hands::Dict{Side, RigidBody{T}}
    floating_body::RigidBody{T}
    world_aligned_floating_body_frame::CartesianFrame3D
    actuatorconfig::OrderedDict{Actuator, JointID}

    function HumanoidRobotInfo(
            mechanism::Mechanism{T},
            feet::Associative{Side, RigidBody{T}},
            hands::Associative{Side, RigidBody{T}},
            actuatorconfig::OrderedDict{Actuator, JointID}) where {T}
        sides = instances(Side)
        @assert isempty(setdiff(keys(feet), sides))
        @assert isempty(setdiff(keys(hands), sides))
        floating_joints = filter(isfloating, tree_joints(mechanism))
        @assert length(floating_joints) == 1
        floating_body = successor(first(floating_joints), mechanism)
        world_aligned_floating_body_frame = CartesianFrame3D("world_aligned_floating_body_frame")
        new{T}(mechanism, feet, hands, floating_body, world_aligned_floating_body_frame, actuatorconfig)
    end
end

num_actuators(info::HumanoidRobotInfo) = length(info.actuatorconfig)
actuators(info::HumanoidRobotInfo) = keys(info.actuatorconfig)

function findactuator(info::HumanoidRobotInfo, name::String)
    for actuator in actuators(info)
        actuator.name == name && return actuator
    end
    throw(ArgumentError("Actuator with name \"$name\" not found"))
end

findjointid(info::HumanoidRobotInfo, actuator::Actuator) = info.actuatorconfig[actuator]
