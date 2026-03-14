import SystemSimulator as SS

struct AcrobotSlots
    tau1::Int
    tau2::Int
    reset::Int
    n_substeps::Int
    theta1_0::Int
    theta2_0::Int
    omega1_0::Int
    omega2_0::Int
    out_theta1::Int
    out_theta2::Int
    out_omega1::Int
    out_omega2::Int
end

mutable struct AcrobotSystem <: SS.AbstractSystem
    param_names::Vector{String}
    initial_values::Dict{String,Float64}
    state::Vector{Float64}   # [θ1, θ2, ω1, ω2]
    lifecycle::SS.SystemLifecycle
    lifecycle_slots::Union{SS.LifecycleSlots,Nothing}
    slots::Union{AcrobotSlots,Nothing}
    _rk4_ws::RK4Workspace
    _cached_ap::AcrobotParams
end

function AcrobotSystem()
    param_names = [
        "m1", "m2", "l1", "l2", "lc1", "lc2", "I1", "I2", "g", "b1", "b2",
        "n_substeps",
        "theta1_0", "theta2_0", "omega1_0", "omega2_0",
        "reset",
        "start_cmd", "stop_cmd", "running", "elapsed", "duration",
    ]
    initial_values = Dict{String,Float64}(
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
        # Simulation lifecycle
        "start_cmd" => 0.0,
        "stop_cmd"  => 0.0,
        "running"   => 0.0,
        "elapsed"   => 0.0,
        "duration"  => 300.0,
    )
    state = [1.0, 0.5, 0.0, 0.0]
    return AcrobotSystem(
        param_names,
        initial_values,
        state,
        SS.SystemLifecycle(),
        nothing,
        nothing,
        RK4Workspace(4),
        AcrobotParams(initial_values),
    )
end

"""
    acrobot_callback(ctrl, inputs, outputs, dt)

Control callback for double pendulum simulation.
Integrates physics only when active (between start/stop commands).
"""
function _reset_to_ics!(ctrl::AcrobotSystem, params, slots::AcrobotSlots)
    ctrl.state[1] = params[slots.theta1_0]
    ctrl.state[2] = params[slots.theta2_0]
    ctrl.state[3] = params[slots.omega1_0]
    ctrl.state[4] = params[slots.omega2_0]
    return nothing
end

SS.parameter_names(ctrl::AcrobotSystem) = copy(ctrl.param_names)

function SS.monitor_parameter_names(ctrl::AcrobotSystem)
    return String[name for name in ctrl.param_names if name != "running" && name != "elapsed"]
end

function SS.initialize_parameters!(ctrl::AcrobotSystem, params)::Nothing
    for name in ctrl.param_names
        params[name] = ctrl.initial_values[name]
    end
    return nothing
end

function SS.bind!(ctrl::AcrobotSystem, runtime)::Nothing
    ctrl.lifecycle_slots = SS.bind_lifecycle(runtime.params)
    ctrl.slots = AcrobotSlots(
        SS.signal_slot(runtime.inputs, "can_torque.Tau1"),
        SS.signal_slot(runtime.inputs, "can_torque.Tau2"),
        SS.signal_slot(runtime.params, "reset"),
        SS.signal_slot(runtime.params, "n_substeps"),
        SS.signal_slot(runtime.params, "theta1_0"),
        SS.signal_slot(runtime.params, "theta2_0"),
        SS.signal_slot(runtime.params, "omega1_0"),
        SS.signal_slot(runtime.params, "omega2_0"),
        SS.signal_slot(runtime.outputs, "can_state.Theta1"),
        SS.signal_slot(runtime.outputs, "can_state.Theta2"),
        SS.signal_slot(runtime.outputs, "can_state.Omega1"),
        SS.signal_slot(runtime.outputs, "can_state.Omega2"),
    )
    return nothing
end

function SS.parameters_updated!(ctrl::AcrobotSystem, params)::Nothing
    ctrl._cached_ap = AcrobotParams(params)
    return nothing
end

function SS.control_step!(ctrl::AcrobotSystem, inputs, outputs, params, dt)
    slots = ctrl.slots
    event = SS.update_lifecycle!(ctrl.lifecycle, params, ctrl.lifecycle_slots, dt)

    # Reset state to ICs on start
    if event == :started
        _reset_to_ics!(ctrl, params, slots)
    end

    # Reset state to initial conditions if requested (works while active)
    if params[slots.reset] >= 1.0
        _reset_to_ics!(ctrl, params, slots)
        params[slots.reset] = 0.0
    end

    if ctrl.lifecycle.active
        # Read torque commands from CAN input (default 0.0)
        τ1 = inputs[slots.tau1]
        τ2 = inputs[slots.tau2]

        n_sub = max(1, round(Int, params[slots.n_substeps]))
        rk4_step!(ctrl.state, τ1, τ2, ctrl._cached_ap, dt, n_sub, ctrl._rk4_ws)

        # Reset to ICs if integration produced NaN/Inf (numerical blowup)
        if any(!isfinite, ctrl.state)
            @warn "Numerical blowup detected — resetting state to initial conditions"
            _reset_to_ics!(ctrl, params, slots)
        end
    end

    # Always write current state to CAN output (frozen when not active)
    outputs[slots.out_theta1] = clamp(ctrl.state[1], -30.0, 35.535)
    outputs[slots.out_theta2] = clamp(ctrl.state[2], -30.0, 35.535)
    outputs[slots.out_omega1] = clamp(ctrl.state[3], -300.0, 355.35)
    outputs[slots.out_omega2] = clamp(ctrl.state[4], -300.0, 355.35)

    return nothing
end

function acrobot_callback(ctrl::AcrobotSystem, inputs, outputs, params, dt)
    return SS.control_step!(ctrl, inputs, outputs, params, dt)
end
