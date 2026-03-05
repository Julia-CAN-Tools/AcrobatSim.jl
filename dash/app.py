"""
Plotly Dash dashboard for AcrobatSim double pendulum.

Connects to SystemSimulator TcpMonitor:
  - Port 9100: send tunable parameters
  - Port 9101: receive all streamed signals

Usage:
    pip install -r requirements.txt
    python app.py
"""

import socket
import struct
import threading
from collections import deque

import numpy as np
from dash import Dash, dcc, html, Input, Output, State, callback

# ---------------------------------------------------------------------------
# TCP client for TcpMonitor binary protocol
# ---------------------------------------------------------------------------

class SimulatorClient:
    """Connects to SystemSimulator TcpMonitor and exchanges data."""

    def __init__(self, host="localhost", stream_port=9101, param_port=9100):
        self.host = host
        self.stream_port = stream_port
        self.param_port = param_port

        self.signal_names = []
        self.param_names = []
        self.history = {}           # name -> deque of floats
        self.maxlen = 3000
        self.lock = threading.Lock()
        self._running = False

        self._stream_sock = None
        self._param_sock = None
        self._reader_thread = None

    def connect_stream(self):
        """Connect to the stream port, read handshake, start reader thread."""
        self._stream_sock = socket.create_connection((self.host, self.stream_port))
        self.signal_names = self._read_handshake(self._stream_sock)
        with self.lock:
            for name in self.signal_names:
                self.history[name] = deque(maxlen=self.maxlen)
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def connect_params(self):
        """Connect to the param port and read handshake."""
        self._param_sock = socket.create_connection((self.host, self.param_port))
        self.param_names = self._read_handshake(self._param_sock)

    def _read_handshake(self, sock):
        """Read TcpMonitor handshake: count + name strings."""
        raw = self._recv_exact(sock, 4)
        n = struct.unpack("<I", raw)[0]
        names = []
        for _ in range(n):
            slen_raw = self._recv_exact(sock, 2)
            slen = struct.unpack("<H", slen_raw)[0]
            name = self._recv_exact(sock, slen).decode("utf-8")
            names.append(name)
        return names

    def _recv_exact(self, sock, nbytes):
        """Receive exactly nbytes from socket."""
        buf = bytearray()
        while len(buf) < nbytes:
            chunk = sock.recv(nbytes - len(buf))
            if not chunk:
                raise ConnectionError("Socket closed")
            buf.extend(chunk)
        return bytes(buf)

    def _reader_loop(self):
        """Background thread: read Float64 frames, append to history."""
        n = len(self.signal_names)
        frame_size = n * 8
        while self._running:
            try:
                data = self._recv_exact(self._stream_sock, frame_size)
            except (ConnectionError, OSError):
                break
            values = struct.unpack(f"<{n}d", data)
            with self.lock:
                for name, val in zip(self.signal_names, values):
                    self.history[name].append(val)

    def send_params(self, param_dict):
        """Send parameter values in declared order."""
        if self._param_sock is None or not self.param_names:
            return
        values = [param_dict.get(name, 0.0) for name in self.param_names]
        payload = struct.pack(f"<{len(values)}d", *values)
        try:
            self._param_sock.sendall(payload)
        except (ConnectionError, OSError):
            pass

    def get_history(self, name):
        """Return a copy of the history buffer for a signal."""
        with self.lock:
            buf = self.history.get(name)
            if buf is None:
                return []
            return list(buf)

    def get_latest(self, name):
        """Return the latest value of a signal, or None."""
        with self.lock:
            buf = self.history.get(name)
            if buf is None or len(buf) == 0:
                return None
            return buf[-1]

    def close(self):
        self._running = False
        for s in (self._stream_sock, self._param_sock):
            if s:
                try:
                    s.close()
                except OSError:
                    pass


# ---------------------------------------------------------------------------
# Dash application
# ---------------------------------------------------------------------------

client = SimulatorClient()
_prev_reset_clicks = 0

# Try to connect (non-fatal if simulator isn't running yet)
try:
    client.connect_stream()
    print(f"Stream connected: {len(client.signal_names)} signals")
except Exception as e:
    print(f"Stream connection failed (start simulator first): {e}")

try:
    client.connect_params()
    print(f"Params connected: {client.param_names}")
except Exception as e:
    print(f"Param connection failed: {e}")

# Physical parameter sliders
PARAM_SLIDERS = [
    ("m1",  0.1, 10.0, 0.1,  1.0,   "m1 [kg]"),
    ("m2",  0.1, 10.0, 0.1,  1.0,   "m2 [kg]"),
    ("l1",  0.1,  5.0, 0.1,  1.0,   "l1 [m]"),
    ("l2",  0.1,  5.0, 0.1,  1.0,   "l2 [m]"),
    ("lc1", 0.05, 2.5, 0.05, 0.5,   "lc1 [m]"),
    ("lc2", 0.05, 2.5, 0.05, 0.5,   "lc2 [m]"),
    ("I1",  0.001, 5.0, 0.001, 0.0833, "I1 [kg·m²]"),
    ("I2",  0.001, 5.0, 0.001, 0.0833, "I2 [kg·m²]"),
    ("g",   0.0, 20.0, 0.01, 9.81,  "g [m/s²]"),
    ("b1",  0.0,  5.0, 0.01, 0.1,   "b1 [N·m·s/rad]"),
    ("b2",  0.0,  5.0, 0.01, 0.1,   "b2 [N·m·s/rad]"),
]

IC_SLIDERS = [
    ("theta1_0", -3.14, 3.14, 0.01, 1.0,   "θ1₀ [rad]"),
    ("theta2_0", -3.14, 3.14, 0.01, 0.5,   "θ2₀ [rad]"),
    ("omega1_0", -10.0, 10.0, 0.1,  0.0,   "ω1₀ [rad/s]"),
    ("omega2_0", -10.0, 10.0, 0.1,  0.0,   "ω2₀ [rad/s]"),
]

def make_slider(pid, lo, hi, step, val, label):
    return html.Div([
        html.Label(label, style={"fontSize": "12px"}),
        dcc.Slider(
            id=f"slider-{pid}",
            min=lo, max=hi, step=step, value=val,
            marks=None,
            tooltip={"placement": "right", "always_visible": True},
        ),
    ], style={"marginBottom": "6px"})

sidebar_children = [html.H3("Parameters")]
for args in PARAM_SLIDERS:
    sidebar_children.append(make_slider(*args))
sidebar_children.append(html.Hr())
sidebar_children.append(html.H3("Initial Conditions"))
for args in IC_SLIDERS:
    sidebar_children.append(make_slider(*args))

# Duration slider
sidebar_children.append(html.Hr())
sidebar_children.append(html.Label("Duration [s]", style={"fontSize": "12px", "fontWeight": "bold"}))
sidebar_children.append(make_slider("duration", 1.0, 600.0, 1.0, 300.0, "Duration [s]"))

# Simulation controls: Start / Stop / Reset buttons + status display
sidebar_children.append(html.Hr())
sidebar_children.append(html.Div([
    html.Div([
        html.Button(
            "Start",
            id="btn-start",
            n_clicks=0,
            style={
                "backgroundColor": "#28a745", "color": "white", "border": "none",
                "padding": "8px 20px", "fontSize": "14px", "fontWeight": "bold",
                "borderRadius": "4px", "cursor": "pointer", "marginRight": "6px",
            },
        ),
        html.Button(
            "Stop",
            id="btn-stop",
            n_clicks=0,
            style={
                "backgroundColor": "#dc3545", "color": "white", "border": "none",
                "padding": "8px 20px", "fontSize": "14px", "fontWeight": "bold",
                "borderRadius": "4px", "cursor": "pointer", "marginRight": "6px",
            },
        ),
        html.Button(
            "Reset",
            id="btn-reset",
            n_clicks=0,
            style={
                "backgroundColor": "#6c757d", "color": "white", "border": "none",
                "padding": "8px 20px", "fontSize": "14px", "fontWeight": "bold",
                "borderRadius": "4px", "cursor": "pointer",
            },
        ),
    ], style={"display": "flex", "justifyContent": "center", "marginBottom": "8px"}),
    html.Div(
        id="sim-status",
        children="Idle",
        style={
            "textAlign": "center", "fontSize": "13px", "fontWeight": "bold",
            "padding": "4px", "borderRadius": "4px", "backgroundColor": "#f0f0f0",
        },
    ),
], style={"marginTop": "4px"}))

# Hidden stores for click counts sent to Julia
sidebar_children.append(dcc.Store(id="store-start-clicks", data=0))
sidebar_children.append(dcc.Store(id="store-stop-clicks", data=0))

app = Dash(__name__)
app.layout = html.Div([
    dcc.Interval(id="interval", interval=200, n_intervals=0),
    html.Div([
        # Sidebar
        html.Div(sidebar_children,
                 style={"width": "260px", "padding": "10px",
                        "overflowY": "auto", "borderRight": "1px solid #ccc",
                        "height": "100vh"}),
        # Plot grid
        html.Div([
            html.Div([
                dcc.Graph(id="graph-angles", style={"height": "45vh"}),
                dcc.Graph(id="graph-velocities", style={"height": "45vh"}),
            ], style={"flex": "1"}),
            html.Div([
                dcc.Graph(id="graph-torques", style={"height": "45vh"}),
                dcc.Graph(id="graph-phase", style={"height": "45vh"}),
            ], style={"flex": "1"}),
        ], style={"display": "flex", "flex": "1"}),
    ], style={"display": "flex", "height": "100vh"}),
])

# Collect all slider IDs
ALL_SLIDER_IDS = [f"slider-{p[0]}" for p in PARAM_SLIDERS + IC_SLIDERS]
ALL_SLIDER_IDS.append("slider-duration")


# ---------------------------------------------------------------------------
# Button click callbacks: increment stored click counts
# ---------------------------------------------------------------------------

@callback(
    Output("store-start-clicks", "data"),
    Input("btn-start", "n_clicks"),
    prevent_initial_call=True,
)
def on_start_click(n):
    return n


@callback(
    Output("store-stop-clicks", "data"),
    Input("btn-stop", "n_clicks"),
    prevent_initial_call=True,
)
def on_stop_click(n):
    return n


# ---------------------------------------------------------------------------
# Main update callback
# ---------------------------------------------------------------------------

@callback(
    Output("graph-angles", "figure"),
    Output("graph-velocities", "figure"),
    Output("graph-torques", "figure"),
    Output("graph-phase", "figure"),
    Output("sim-status", "children"),
    Output("sim-status", "style"),
    Input("interval", "n_intervals"),
    Input("btn-reset", "n_clicks"),
    [State(sid, "value") for sid in ALL_SLIDER_IDS],
    State("store-start-clicks", "data"),
    State("store-stop-clicks", "data"),
)
def update_graphs(n_intervals, n_clicks_reset, *args):
    global _prev_reset_clicks

    n_sl = len(ALL_SLIDER_IDS)
    slider_values = args[:n_sl]
    start_clicks  = args[n_sl]
    stop_clicks   = args[n_sl + 1]

    # Send params to simulator
    param_dict = {}
    phys_ic_params = PARAM_SLIDERS + IC_SLIDERS
    for (pid, *_rest), val in zip(phys_ic_params, slider_values[:len(phys_ic_params)]):
        param_dict[pid] = float(val)

    # Duration slider (last in ALL_SLIDER_IDS)
    dur_val = slider_values[len(phys_ic_params)]
    param_dict["duration"] = float(dur_val) if dur_val is not None else 300.0

    # Reset button (edge-detect via click count)
    clicks = n_clicks_reset or 0
    param_dict["reset"] = 1.0 if clicks > _prev_reset_clicks else 0.0
    _prev_reset_clicks = clicks

    # Keep n_substeps at default
    param_dict["n_substeps"] = 10.0

    # Start/stop commands (click counts; Julia detects rising counts)
    param_dict["start_cmd"] = float(start_clicks or 0)
    param_dict["stop_cmd"]  = float(stop_clicks or 0)

    client.send_params(param_dict)

    # Read history
    time_data = client.get_history("Time")
    theta1 = client.get_history("can_state.Theta1")
    theta2 = client.get_history("can_state.Theta2")
    omega1 = client.get_history("can_state.Omega1")
    omega2 = client.get_history("can_state.Omega2")
    tau1 = client.get_history("can_torque.Tau1")
    tau2 = client.get_history("can_torque.Tau2")

    # Read simulation state from stream
    running_val = client.get_latest("running")
    elapsed_val = client.get_latest("elapsed")
    duration_val = client.get_latest("duration")

    is_running = running_val is not None and running_val >= 0.5
    elapsed_s  = elapsed_val if elapsed_val is not None else 0.0
    dur_s      = duration_val if duration_val is not None else param_dict["duration"]

    if is_running:
        status_text = f"Running: {elapsed_s:.1f} / {dur_s:.0f} s"
        status_style = {
            "textAlign": "center", "fontSize": "13px", "fontWeight": "bold",
            "padding": "4px", "borderRadius": "4px",
            "backgroundColor": "#d4edda", "color": "#155724",
        }
    elif elapsed_s > 0.5:
        status_text = f"Finished: {elapsed_s:.1f} s"
        status_style = {
            "textAlign": "center", "fontSize": "13px", "fontWeight": "bold",
            "padding": "4px", "borderRadius": "4px",
            "backgroundColor": "#fff3cd", "color": "#856404",
        }
    else:
        status_text = "Idle"
        status_style = {
            "textAlign": "center", "fontSize": "13px", "fontWeight": "bold",
            "padding": "4px", "borderRadius": "4px",
            "backgroundColor": "#f0f0f0", "color": "#333",
        }

    # Angles plot
    fig_angles = {
        "data": [
            {"x": time_data, "y": theta1, "name": "θ1", "type": "scatter"},
            {"x": time_data, "y": theta2, "name": "θ2", "type": "scatter"},
        ],
        "layout": {"title": "Joint Angles", "xaxis": {"title": "Time [s]"},
                   "yaxis": {"title": "Angle [rad]"}, "margin": {"t": 40}},
    }

    # Velocities plot
    fig_vel = {
        "data": [
            {"x": time_data, "y": omega1, "name": "ω1", "type": "scatter"},
            {"x": time_data, "y": omega2, "name": "ω2", "type": "scatter"},
        ],
        "layout": {"title": "Joint Velocities", "xaxis": {"title": "Time [s]"},
                   "yaxis": {"title": "Angular Velocity [rad/s]"}, "margin": {"t": 40}},
    }

    # Torques plot
    fig_torque = {
        "data": [
            {"x": time_data, "y": tau1, "name": "τ1", "type": "scatter"},
            {"x": time_data, "y": tau2, "name": "τ2", "type": "scatter"},
        ],
        "layout": {"title": "Applied Torques", "xaxis": {"title": "Time [s]"},
                   "yaxis": {"title": "Torque [N·m]"}, "margin": {"t": 40}},
    }

    # Phase portrait
    fig_phase = {
        "data": [
            {"x": theta1, "y": omega1, "name": "Link 1", "type": "scatter", "mode": "lines"},
            {"x": theta2, "y": omega2, "name": "Link 2", "type": "scatter", "mode": "lines"},
        ],
        "layout": {"title": "Phase Portrait", "xaxis": {"title": "Angle [rad]"},
                   "yaxis": {"title": "Angular Velocity [rad/s]"}, "margin": {"t": 40}},
    }

    return fig_angles, fig_vel, fig_torque, fig_phase, status_text, status_style


if __name__ == "__main__":
    app.run(debug=True, use_reloader=False, host="0.0.0.0", port=8050)
