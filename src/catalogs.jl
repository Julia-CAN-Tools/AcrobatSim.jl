# CAN message catalogs for double pendulum simulation.

import J1939Parser as CP

# ── AcrobotTorqueCmd (RX on vcan0, CAN ID 0x0CFF0110) ─────────────────────
# Torque commands from external controller.
const TORQUE_CMD_MESSAGES = CP.CanMessage[
    CP.CanMessage(
        "AcrobotTorqueCmd",
        CP.CanId(3, 0xFF, 0x01, 0x10),
        CP.Signal[
            CP.Signal("Tau1", 1, 1, 16, 0.01, -200.0),
            CP.Signal("Tau2", 3, 1, 16, 0.01, -200.0),
        ],
    ),
]

# ── AcrobotState (TX on vcan2, CAN ID 0x0CFF0210) ─────────────────────────
# State output published by simulation.
const STATE_OUTPUT_MESSAGES = CP.CanMessage[
    CP.CanMessage(
        "AcrobotState",
        CP.CanId(3, 0xFF, 0x02, 0x10),
        CP.Signal[
            CP.Signal("Theta1", 1, 1, 16, 0.001, -30.0),
            CP.Signal("Theta2", 3, 1, 16, 0.001, -30.0),
            CP.Signal("Omega1", 5, 1, 16, 0.01, -300.0),
            CP.Signal("Omega2", 7, 1, 16, 0.01, -300.0),
        ],
    ),
]
