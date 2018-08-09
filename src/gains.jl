struct LowLevelJointGains{T}
    k_q_p::T # corresponds to kp_position in drcsim API
    k_q_i::T # corresponds to ki_position in drcsim API
    k_qd_p::T # corresponds to kp_velocity in drcsim API
    k_f_p::T
    ff_qd::T # maps to kd_position in drcsim API (there isnt an equivalent gain in the bdi api)
    ff_qd_d::T
    ff_f_d::T
    ff_const::T
end

struct JointState{T}
    position::T
    velocity::T
    effort::T
end

function command_effort(gains::LowLevelJointGains, state::JointState, state_des::JointState, Δt::Number)
    effort = gains.ff_const
    effort += gains.k_q_p * (state_des.position - state.position)
    effort += gains.k_q_i * (state_des.position - state.position) * Δt
    effort += gains.k_qd_p * (state_des.velocity - state.velocity)
    effort += gains.k_f_p * (state_des.effort - state.effort)
    effort += gains.ff_qd * state.velocity
    effort += gains.ff_qd_d * state_des.velocity
    effort += gains.ff_f_d * state_des.effort
    effort
end
