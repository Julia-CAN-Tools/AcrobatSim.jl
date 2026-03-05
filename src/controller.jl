import SystemSimulator as SS

mutable struct AcrobotController <: SS.AbstractController
    params::Dict{String,Float64}
    state::Vector{Float64}   # [θ1, θ2, ω1, ω2]
end

function AcrobotController()
    params = Dict{String,Float64}(
        # Physical parameters
        "m1"  => 1.0,
        "m2"  => 1.0,
        "l1"  => 1.0,
        "l2"  => 1.0,
        "lc1" => 0.5,
        "lc2" => 0.5,
        "I1"  => 0.0833,
        "I2"  => 0.0833,
        "g"   => 9.81,
        "b1"  => 0.1,
        "b2"  => 0.1,
        # Integrator
        "n_substeps" => 10.0,
        # Initial conditions
        "theta1_0" => 1.0,
        "theta2_0" => 0.5,
        "omega1_0" => 0.0,
        "omega2_0" => 0.0,
        # Reset flag (set to 1.0 to reset state to ICs)
        "reset" => 0.0,
    )
    state = [1.0, 0.5, 0.0, 0.0]
    return AcrobotController(params, state)
end

"""
    acrobot_callback(ctrl, inputs, outputs, dt)

Control callback for double pendulum simulation.
"""
function acrobot_callback(ctrl::AcrobotController, inputs, outputs, dt)
    p = ctrl.params

    # Reset state to initial conditions if requested
    if p["reset"] >= 1.0
        ctrl.state[1] = p["theta1_0"]
        ctrl.state[2] = p["theta2_0"]
        ctrl.state[3] = p["omega1_0"]
        ctrl.state[4] = p["omega2_0"]
        p["reset"] = 0.0
    end

    # Read torque commands from CAN input (default 0.0)
    τ1 = get(inputs, "can_torque.Tau1", 0.0)
    τ2 = get(inputs, "can_torque.Tau2", 0.0)

    # Build physics params and integrate
    ap = AcrobotParams(p)
    n_sub = max(1, round(Int, p["n_substeps"]))
    rk4_step!(ctrl.state, τ1, τ2, ap, dt, n_sub)

    # Reset to ICs if integration produced NaN/Inf (numerical blowup)
    if any(!isfinite, ctrl.state)
        @warn "Numerical blowup detected — resetting state to initial conditions"
        ctrl.state[1] = p["theta1_0"]
        ctrl.state[2] = p["theta2_0"]
        ctrl.state[3] = p["omega1_0"]
        ctrl.state[4] = p["omega2_0"]
    end

    # Write state to CAN output, clamped to encodable range
    outputs["can_state.Theta1"] = clamp(ctrl.state[1], -30.0, 35.535)
    outputs["can_state.Theta2"] = clamp(ctrl.state[2], -30.0, 35.535)
    outputs["can_state.Omega1"] = clamp(ctrl.state[3], -300.0, 355.35)
    outputs["can_state.Omega2"] = clamp(ctrl.state[4], -300.0, 355.35)

    return nothing
end
