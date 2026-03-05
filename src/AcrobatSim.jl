module AcrobatSim

include("dynamics.jl")
include("catalogs.jl")
include("controller.jl")

export AcrobotParams, acrobot_dynamics!, rk4_step!, total_energy,
       TORQUE_CMD_MESSAGES, STATE_OUTPUT_MESSAGES,
       AcrobotController, acrobot_callback

end # module AcrobatSim
