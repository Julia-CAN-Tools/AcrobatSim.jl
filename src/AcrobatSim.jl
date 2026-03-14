module AcrobatSim

import SystemSimulator as SS

include("dynamics.jl")
include("catalogs.jl")
include("system.jl")

export AcrobotParams, acrobot_dynamics!, rk4_step!, RK4Workspace, total_energy,
       TORQUE_CMD_MESSAGES, STATE_OUTPUT_MESSAGES,
       AcrobotSystem, acrobot_callback

end # module AcrobatSim
