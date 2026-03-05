import SystemSimulator as SS
import CANInterface as CI

using AcrobatSim

sf = SS.StopSignal()

# ── IO endpoints ───────────────────────────────────────────────────────────
# vcan0: receive torque commands (read-only)
rx_io = SS.CanIO(
    CI.SocketCanDriver("vcan0"),
    TORQUE_CMD_MESSAGES,
    typeof(TORQUE_CMD_MESSAGES[1])[],
)

# vcan2: publish state output (write-only)
tx_io = SS.CanIO(
    CI.SocketCanDriver("vcan2"),
    typeof(STATE_OUTPUT_MESSAGES[1])[],
    deepcopy(STATE_OUTPUT_MESSAGES),
)

# ── System configuration ──────────────────────────────────────────────────
cfg = SS.SystemConfig(
    10,   # 10 ms → 100 Hz control loop
    [
        SS.IOConfig(:can_torque, rx_io, 256, SS.IO_MODE_READONLY),
        SS.IOConfig(:can_state,  tx_io, 256, SS.IO_MODE_WRITEONLY),
    ],
    joinpath(@__DIR__, "..", "AcrobotLog.csv"),
    SS.MonitorConfig("0.0.0.0", 9100, 9101),
)

# ── Build and start ───────────────────────────────────────────────────────
ctrl = AcrobotController()
runtime = SS.SystemRuntime(cfg, sf, ctrl)

@info "Starting AcrobotSim" dt_ms=10 monitor_params=9100 monitor_stream=9101
@info "Params:" keys=sort(collect(keys(runtime.params)))
@info "Signals:" keys=runtime.monitor.out_names

SS.start!(runtime, acrobot_callback)

RUN_SECONDS = 300.0
@info "Running for $(RUN_SECONDS)s — press Ctrl+C to stop early"
try
    sleep(RUN_SECONDS)
catch e
    e isa InterruptException || rethrow(e)
end

# Shutdown: send waker frame to unblock vcan0 reader
SS.request_stop!(runtime.stop_signal)
_waker = CI.SocketCanDriver("vcan0")
try
    CI.write(_waker, UInt32(0x18FF0000), ntuple(_ -> UInt8(0), 8))
catch err
    @warn "Wake frame write failed" exception=(err, catch_backtrace())
finally
    CI.close(_waker)
end

@info "Stopping" steps=runtime.step_count[] timestamp=runtime.timestamp
SS.stop!(runtime)
