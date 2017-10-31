struct HumanoidRobotInfo{T}
    mechanism::Mechanism{T}
    feet::Dict{Symbol, RigidBody{T}}
    hands::Dict{Symbol, RigidBody{T}}
    floating_body::RigidBody{T}
    name_to_joint::Dict{String, GenericJoint{T}}

    function HumanoidRobotInfo(mechanism::Mechanism{T}, feet::Associative{Symbol, RigidBody{T}}, hands::Associative{Symbol, RigidBody{T}}) where {T}
        sides = [:left, :right]
        @assert isempty(setdiff(keys(feet), sides))
        @assert isempty(setdiff(keys(hands), sides))
        floating_joints = filter(isfloating, tree_joints(mechanism))
        @assert length(floating_joints) == 1
        floating_body = successor(first(floating_joints), mechanism)
        name_to_joint = Dict(string(j) => j for j in tree_joints(mechanism))
        new{T}(mechanism, feet, hands, floating_body, name_to_joint)
    end
end

RigidBodyDynamics.findjoint(info::HumanoidRobotInfo, name::String) = info.name_to_joint[name]
