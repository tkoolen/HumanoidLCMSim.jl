mutable struct LowLevelJointGains{T}
    k_q_p::T # corresponds to kp_position in drcsim API
    k_q_i::T # corresponds to ki_position in drcsim API
    k_qd_p::T # corresponds to kp_velocity in drcsim API
    k_f_p::T
    ff_qd::T # maps to kd_position in drcsim API (there isnt an equivalent gain in the bdi api)
    ff_qd_d::T
    ff_f_d::T
    ff_const::T

    function LowLevelJointGains{T}() where T
        new{T}(0, 0, 0, 0, 0, 0, 0, 0)
    end
end

LowLevelJointGains() = LowLevelJointGains{Float64}()

function command_effort(gains::LowLevelJointGains, q::Number, q_desired::Number, f::Number, f_desired::Number, Δt::Number)
    effort =
        gains.k_q_p * (q_desired - q) +
        gains.k_q_i * (q_desired - q) * Δt +
        gains.k_qd_p * (qd_desired - qd) +
        gains.k_f_p * (f_desired - f) +
        gains.ff_qd * (qd) +
        gains.ff_qd_d * (qd_desired) +
        gains.ff_f_d * (f_desired) +
        gains.ff_const
end
