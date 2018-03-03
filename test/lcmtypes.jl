module NativeBotCoreLCMTypesTests

using Compat
using LCMCore
using PyLCM
using BotCoreLCMTypes
using StaticArrays
using Rotations

using HumanoidLCMSim
using HumanoidLCMSim.NativeBotCoreLCMTypes
using Compat.Test


rand_python_msg(::Type{T}) where {T <: LCMType} = rand_python_msg(Base.Random.GLOBAL_RNG, T)

function rand_python_msg(rng::AbstractRNG, ::Type{AtlasCommandT})
    utime = 234
    num_joints = 2
    msg = bot_core[:atlas_command_t]()
    msg[:utime] = utime
    msg[:num_joints] = num_joints
    msg[:joint_names] = [randstring(rng, rand(rng, 1 : 10)) for i = 1 : num_joints]
    msg[:position] = rand(rng, Float64, num_joints)
    msg[:velocity] = rand(rng, Float64, num_joints)
    msg[:effort] = rand(rng, Float64, num_joints)
    msg[:k_q_p] = rand(rng, Float64, num_joints)
    msg[:k_q_i] = rand(rng, Float64, num_joints)
    msg[:k_qd_p] = rand(rng, Float64, num_joints)
    msg[:k_f_p] = rand(rng, Float64, num_joints)
    msg[:ff_qd] = rand(rng, Float64, num_joints)
    msg[:ff_qd_d] = rand(rng, Float64, num_joints)
    msg[:ff_f_d] = rand(rng, Float64, num_joints)
    msg[:ff_const] = rand(rng, Float64, num_joints)
    msg[:k_effort] = rand(rng, UInt8, num_joints)
    msg[:desired_controller_period_ms] = rand(rng, UInt8)
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{ForceTorqueT})
    msg = bot_core[:force_torque_t]()
    msg[:l_foot_force_z] = rand(rng, Float32)
    msg[:l_foot_torque_x] = rand(rng, Float32)
    msg[:l_foot_torque_y] = rand(rng, Float32)
    msg[:r_foot_force_z] = rand(rng, Float32)
    msg[:r_foot_torque_x] = rand(rng, Float32)
    msg[:r_foot_torque_y] = rand(rng, Float32)
    msg[:l_hand_force] = rand(rng, SVector{3, Float32})
    msg[:l_hand_torque] = rand(rng, SVector{3, Float32})
    msg[:r_hand_force] = rand(rng, SVector{3, Float32})
    msg[:r_hand_torque] = rand(rng, SVector{3, Float32})
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{Vector3DT})
    msg = bot_core[:vector_3d_t]()
    msg[:x] = rand(rng)
    msg[:y] = rand(rng)
    msg[:z] = rand(rng)
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{QuaternionT})
    quat = rand(rng, Quat{Float64})
    msg = bot_core[:quaternion_t]()
    msg[:w] = quat.w
    msg[:x] = quat.x
    msg[:y] = quat.y
    msg[:z] = quat.z
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{Position3DT})
    msg = bot_core[:position_3d_t]()
    msg[:translation] = rand_python_msg(rng, Vector3DT)
    msg[:rotation] = rand_python_msg(rng, QuaternionT)
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{TwistT})
    msg = bot_core[:twist_t]()
    msg[:linear_velocity] = rand_python_msg(rng, Vector3DT)
    msg[:angular_velocity] = rand_python_msg(rng, Vector3DT)
    msg
end

function rand_python_msg(rng::AbstractRNG, ::Type{RobotStateT})
    num_joints = 7
    msg = bot_core[:robot_state_t]()
    msg[:utime] = rand(rng, Int64)
    msg[:pose] = rand_python_msg(rng, Position3DT)
    msg[:twist] = rand_python_msg(rng, TwistT)
    msg[:num_joints] = num_joints
    msg[:joint_name] = [randstring(rng) for i = 1 : num_joints]
    msg[:joint_position] = [rand(rng, Float32) for i = 1 : num_joints]
    msg[:joint_velocity] = [rand(rng, Float32) for i = 1 : num_joints]
    msg[:joint_effort] = [rand(rng, Float32) for i = 1 : num_joints]
    msg[:force_torque] = rand_python_msg(rng, ForceTorqueT)
    msg
end

function Base.:(==)(pylcmtype::PyCall.PyObject, lcmtype::LCMType)
    for fieldname in fieldnames(lcmtype)
        pylcmtype[fieldname] == getfield(lcmtype, fieldname) || return false
    end
    return true
end

function test_lcm_message(::Type{T}) where T <: LCMType
    lcmtype = T()
    pylcmtype = rand_python_msg(T)
    bytes = LCMCore.encode(pylcmtype)
    decode!(lcmtype, bytes)
    @test pylcmtype == lcmtype
    bytes_back = encode(lcmtype)
    @test bytes == bytes_back
end

@testset "LCMTypes" begin
    for T in subtypes(NativeBotCoreLCMTypes, LCMType)
        test_lcm_message(T)
    end
end

end # module
