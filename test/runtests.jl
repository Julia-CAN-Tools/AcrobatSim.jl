using Test
using AcrobatSim
import CANUtils as CU
import J1939Parser as CP

@testset "AcrobatSim" begin

    @testset "Equilibrium — zero state, zero torque gives zero derivatives" begin
        x = [0.0, 0.0, 0.0, 0.0]
        dx = zeros(4)
        p = AcrobotParams(Dict{String,Float64}())
        acrobot_dynamics!(dx, x, 0.0, 0.0, p)
        @test all(abs.(dx) .< 1e-12)
    end

    @testset "Gravity sign — displaced angle produces restoring acceleration" begin
        p = AcrobotParams(Dict{String,Float64}())

        # θ1 > 0 (displaced right) → α1 should be negative (restoring)
        x = [0.3, 0.0, 0.0, 0.0]
        dx = zeros(4)
        acrobot_dynamics!(dx, x, 0.0, 0.0, p)
        @test dx[3] < 0.0  # restoring angular acceleration for link 1

        # θ1 < 0 (displaced left) → α1 should be positive (restoring)
        x = [-0.3, 0.0, 0.0, 0.0]
        acrobot_dynamics!(dx, x, 0.0, 0.0, p)
        @test dx[3] > 0.0
    end

    @testset "Energy conservation — no damping, no torque" begin
        p = AcrobotParams(Dict{String,Float64}("b1" => 0.0, "b2" => 0.0))
        x = [1.0, 0.5, 0.0, 0.0]
        E0 = total_energy(x, p)

        dt = 0.001
        n_steps = 10_000  # 10 seconds total
        for _ in 1:n_steps
            rk4_step!(x, 0.0, 0.0, p, dt, 10)
        end
        E_final = total_energy(x, p)

        drift = abs(E_final - E0) / abs(E0)
        @test drift < 0.001  # < 0.1% energy drift
    end

    @testset "CAN round-trip — encode state then decode, values match" begin
        msg_def = deepcopy(STATE_OUTPUT_MESSAGES[1])

        # Physical values to encode
        sig_out = Dict{String,Float64}(
            "Theta1" => 0.785,
            "Theta2" => -1.571,
            "Omega1" => 2.5,
            "Omega2" => -3.14,
        )

        # Encode to CAN frame
        frame = CU.encode(msg_def, sig_out)

        # Decode back
        sig_in = CU.create_signal_dict(CP.CanMessage[msg_def])
        matched = CU.match_and_decode!(frame, CP.CanMessage[msg_def], sig_in)
        @test matched

        # Values should match within scaling resolution
        @test abs(sig_in["Theta1"] - sig_out["Theta1"]) < 0.001   # scaling = 0.001
        @test abs(sig_in["Theta2"] - sig_out["Theta2"]) < 0.001
        @test abs(sig_in["Omega1"] - sig_out["Omega1"]) < 0.01    # scaling = 0.01
        @test abs(sig_in["Omega2"] - sig_out["Omega2"]) < 0.01
    end

    @testset "CAN torque decode — known raw bytes give correct physical values" begin
        msg_def = deepcopy(TORQUE_CMD_MESSAGES[1])

        # Encode known torques
        sig_out = Dict{String,Float64}("Tau1" => 10.0, "Tau2" => -5.0)
        frame = CU.encode(msg_def, sig_out)

        # Decode and verify
        sig_in = CU.create_signal_dict(CP.CanMessage[msg_def])
        matched = CU.match_and_decode!(frame, CP.CanMessage[msg_def], sig_in)
        @test matched
        @test abs(sig_in["Tau1"] - 10.0) < 0.01   # scaling = 0.01
        @test abs(sig_in["Tau2"] - (-5.0)) < 0.01
    end

end
