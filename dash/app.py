"""
Plotly Dash dashboard for AcrobatSim double pendulum.

Connects to SystemSimulator TcpMonitor:
  - Port 9100: send tunable parameters
  - Port 9101: receive all streamed signals

Usage:
    pip install -e ../../srt-dash
    python app.py
"""

from srt_dash import AppConfig, ParamDef, PlotDef, TraceDef, build_app

config = AppConfig(
    title="AcrobatSim",
    port=8050,
    stream_port=9101,
    param_port=9100,
    include_reset=True,
    extra_params={"n_substeps": 10.0},
    params=[
        # Physical parameters
        ParamDef("m1",  "m1 [kg]",          0.1,  10.0, 0.1,   1.0,    group="Parameters"),
        ParamDef("m2",  "m2 [kg]",          0.1,  10.0, 0.1,   1.0,    group="Parameters"),
        ParamDef("l1",  "l1 [m]",           0.1,   5.0, 0.1,   1.0,    group="Parameters"),
        ParamDef("l2",  "l2 [m]",           0.1,   5.0, 0.1,   1.0,    group="Parameters"),
        ParamDef("lc1", "lc1 [m]",         0.05,   2.5, 0.05,  0.5,    group="Parameters"),
        ParamDef("lc2", "lc2 [m]",         0.05,   2.5, 0.05,  0.5,    group="Parameters"),
        ParamDef("I1",  "I1 [kg*m^2]",    0.001,   5.0, 0.001, 0.0833, group="Parameters"),
        ParamDef("I2",  "I2 [kg*m^2]",    0.001,   5.0, 0.001, 0.0833, group="Parameters"),
        ParamDef("g",   "g [m/s^2]",        0.0,  20.0, 0.01,  9.81,   group="Parameters"),
        ParamDef("b1",  "b1 [N*m*s/rad]",   0.0,   5.0, 0.01,  0.1,    group="Parameters"),
        ParamDef("b2",  "b2 [N*m*s/rad]",   0.0,   5.0, 0.01,  0.1,    group="Parameters"),
        # Initial conditions
        ParamDef("theta1_0", "theta1_0 [rad]",   -3.14, 3.14, 0.01, 1.0, group="Initial Conditions"),
        ParamDef("theta2_0", "theta2_0 [rad]",   -3.14, 3.14, 0.01, 0.5, group="Initial Conditions"),
        ParamDef("omega1_0", "omega1_0 [rad/s]", -10.0, 10.0, 0.1,  0.0, group="Initial Conditions"),
        ParamDef("omega2_0", "omega2_0 [rad/s]", -10.0, 10.0, 0.1,  0.0, group="Initial Conditions"),
        # Duration
        ParamDef("duration", "Duration [s]", 1.0, 600.0, 1.0, 300.0, group="Experiment"),
    ],
    plots=[
        PlotDef("graph-angles", "Joint Angles", [
            TraceDef("can_state.Theta1", "theta1"),
            TraceDef("can_state.Theta2", "theta2"),
        ], yaxis="Angle [rad]"),
        PlotDef("graph-velocities", "Joint Velocities", [
            TraceDef("can_state.Omega1", "omega1"),
            TraceDef("can_state.Omega2", "omega2"),
        ], yaxis="Angular Velocity [rad/s]"),
        PlotDef("graph-torques", "Applied Torques", [
            TraceDef("can_torque.Tau1", "Tau1"),
            TraceDef("can_torque.Tau2", "Tau2"),
        ], yaxis="Torque [N*m]"),
        PlotDef("graph-phase", "Phase Portrait", [
            TraceDef("can_state.Theta1", "Link 1"),
            TraceDef("can_state.Theta2", "Link 2"),
        ], xaxis="Angle [rad]", yaxis="Angular Velocity [rad/s]",
           x_signal="can_state.Omega1"),
    ],
)

app = build_app(config)

if __name__ == "__main__":
    app.run(debug=True, use_reloader=False, host="0.0.0.0", port=config.port)
