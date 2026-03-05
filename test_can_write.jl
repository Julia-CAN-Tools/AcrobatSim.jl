import SystemSimulator as SS
import CANInterface as CI

using AcrobatSim

sf = SS.StopSignal()

rx_io = SS.CanIO(
    CI.SocketCanDriver("vcan0"),
    TORQUE_CMD_MESSAGES,
    typeof(TORQUE_CMD_MESSAGES[1])[],
)

tx_io = SS.CanIO(
    CI.SocketCanDriver("vcan2"),
    typeof(STATE_OUTPUT_MESSAGES[1])[],
    deepcopy(STATE_OUTPUT_MESSAGES),
)

cfg = SS.SystemConfig(
    10,
    [
        SS.IOConfig(:can_torque, rx_io, 256, SS.IO_MODE_READONLY),
        SS.IOConfig(:can_state,  tx_io, 256, SS.IO_MODE_WRITEONLY),
    ],
    joinpath(@__DIR__, "TestLog.csv"),
)

ctrl = AcrobotController()
runtime = SS.SystemRuntime(cfg, sf, ctrl)

@info "Starting without TcpMonitor"
SS.start!(runtime, acrobot_callback)

@info "Running for 5s..."
sleep(5)

SS.request_stop!(runtime.stop_signal)
_waker = CI.SocketCanDriver("vcan0")
try
    CI.write(_waker, UInt32(0x18FF0000), ntuple(_ -> UInt8(0), 8))
catch; end
CI.close(_waker)

@info "Stopping" steps=runtime.step_count[]
SS.stop!(runtime)
