"""
    AcrobotParams

Physical parameters for the double pendulum.
"""

struct AcrobotParams
    m1::Float64    # Link 1 mass [kg]
    m2::Float64    # Link 2 mass [kg]
    l1::Float64    # Link 1 length [m]
    l2::Float64    # Link 2 length [m]
    lc1::Float64   # Link 1 center-of-mass distance [m]
    lc2::Float64   # Link 2 center-of-mass distance [m]
    I1::Float64    # Link 1 moment of inertia [kg⋅m²]
    I2::Float64    # Link 2 moment of inertia [kg⋅m²]
    g::Float64     # Gravity [m/s²]
    b1::Float64    # Joint 1 damping [N⋅m⋅s/rad]
    b2::Float64    # Joint 2 damping [N⋅m⋅s/rad]
end

function AcrobotParams(d::AbstractDict{String,<:Real})
    return AcrobotParams(
        Float64(get(d, "m1", 1.0)),
        Float64(get(d, "m2", 1.0)),
        Float64(get(d, "l1", 1.0)),
        Float64(get(d, "l2", 1.0)),
        Float64(get(d, "lc1", 0.5)),
        Float64(get(d, "lc2", 0.5)),
        Float64(get(d, "I1", 0.0833)),
        Float64(get(d, "I2", 0.0833)),
        Float64(get(d, "g", 9.81)),
        Float64(get(d, "b1", 0.1)),
        Float64(get(d, "b2", 0.1)),
    )
end

function AcrobotParams(d::SS.SignalBuffer)
    return AcrobotParams(
        Float64(get(d, "m1", 1.0)),
        Float64(get(d, "m2", 1.0)),
        Float64(get(d, "l1", 1.0)),
        Float64(get(d, "l2", 1.0)),
        Float64(get(d, "lc1", 0.5)),
        Float64(get(d, "lc2", 0.5)),
        Float64(get(d, "I1", 0.0833)),
        Float64(get(d, "I2", 0.0833)),
        Float64(get(d, "g", 9.81)),
        Float64(get(d, "b1", 0.1)),
        Float64(get(d, "b2", 0.1)),
    )
end

"""
    acrobot_dynamics!(dx, x, τ1, τ2, p::AcrobotParams)

Compute state derivative for double pendulum via Euler-Lagrange equations.

State: `x = [θ1, θ2, ω1, ω2]` — angles from hanging-down equilibrium.

Equations: `M(q)q̈ = τ - C(q,q̇) - G(q) - Bq̇`
"""
function acrobot_dynamics!(dx::Vector{Float64}, x::Vector{Float64},
                           τ1::Float64, τ2::Float64, p::AcrobotParams)
    θ1, θ2, ω1, ω2 = x[1], x[2], x[3], x[4]
    Δ = θ2 - θ1
    cosΔ = cos(Δ)
    sinΔ = sin(Δ)

    # Mass matrix M(q) — derived from T = ½q̇ᵀMq̇ with absolute angles
    M11 = p.m1 * p.lc1^2 + p.m2 * p.l1^2 + p.I1
    M12 = p.m2 * p.l1 * p.lc2 * cosΔ
    M22 = p.m2 * p.lc2^2 + p.I2

    # Coriolis/centrifugal vector C(q,q̇)
    h = p.m2 * p.l1 * p.lc2 * sinΔ
    C1 = -h * ω2^2
    C2 =  h * ω1^2

    # Gravity vector G(q) = ∂V/∂q (positive gradient of potential energy)
    G1 = (p.m1 * p.lc1 + p.m2 * p.l1) * p.g * sin(θ1)
    G2 = p.m2 * p.lc2 * p.g * sin(θ2)

    # Damping
    B1 = p.b1 * ω1
    B2 = p.b2 * ω2

    # Right-hand side: τ - C - G - B
    rhs1 = τ1 - C1 - G1 - B1
    rhs2 = τ2 - C2 - G2 - B2

    # Solve M * q̈ = rhs via closed-form 2×2 inverse
    det = M11 * M22 - M12 * M12
    if abs(det) < 1e-12
        # Singular mass matrix — zero accelerations to avoid NaN
        α1 = 0.0
        α2 = 0.0
    else
        α1 = ( M22 * rhs1 - M12 * rhs2) / det
        α2 = (-M12 * rhs1 + M11 * rhs2) / det
    end

    dx[1] = ω1
    dx[2] = ω2
    dx[3] = α1
    dx[4] = α2
    return nothing
end

"""
    RK4Workspace

Pre-allocated scratch vectors for the RK4 integrator. Avoids 5 heap allocations
per callback invocation when stored on the system struct.
"""
struct RK4Workspace
    k1::Vector{Float64}
    k2::Vector{Float64}
    k3::Vector{Float64}
    k4::Vector{Float64}
    xtmp::Vector{Float64}
end

RK4Workspace(n::Int=4) = RK4Workspace(
    Vector{Float64}(undef, n),
    Vector{Float64}(undef, n),
    Vector{Float64}(undef, n),
    Vector{Float64}(undef, n),
    Vector{Float64}(undef, n),
)

"""
    rk4_step!(x, τ1, τ2, params, dt, n_substeps=10)

Integrate state `x` forward by `dt` seconds using RK4 with `n_substeps` sub-steps.
"""
function rk4_step!(x::Vector{Float64}, τ1::Float64, τ2::Float64,
                   p::AcrobotParams, dt::Float64, n_substeps::Int=10)
    rk4_step!(x, τ1, τ2, p, dt, n_substeps, RK4Workspace(length(x)))
end

"""
    rk4_step!(x, τ1, τ2, params, dt, n_substeps, ws::RK4Workspace)

Pre-allocated variant — zero heap allocations per call.
"""
function rk4_step!(x::Vector{Float64}, τ1::Float64, τ2::Float64,
                   p::AcrobotParams, dt::Float64, n_substeps::Int,
                   ws::RK4Workspace)
    h = dt / n_substeps
    k1, k2, k3, k4, xtmp = ws.k1, ws.k2, ws.k3, ws.k4, ws.xtmp

    @inbounds for _ in 1:n_substeps
        acrobot_dynamics!(k1, x, τ1, τ2, p)

        @. xtmp = x + 0.5 * h * k1
        acrobot_dynamics!(k2, xtmp, τ1, τ2, p)

        @. xtmp = x + 0.5 * h * k2
        acrobot_dynamics!(k3, xtmp, τ1, τ2, p)

        @. xtmp = x + h * k3
        acrobot_dynamics!(k4, xtmp, τ1, τ2, p)

        @. x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    end
    return nothing
end

"""
    total_energy(x, p::AcrobotParams)

Compute total mechanical energy (kinetic + potential) of the double pendulum.
Potential energy reference: both links hanging straight down (θ1=θ2=0).
"""
function total_energy(x::Vector{Float64}, p::AcrobotParams)
    θ1, θ2, ω1, ω2 = x[1], x[2], x[3], x[4]

    # Positions of centers of mass
    x1 =  p.lc1 * sin(θ1)
    y1 = -p.lc1 * cos(θ1)
    x2 =  p.l1  * sin(θ1) + p.lc2 * sin(θ2)
    y2 = -p.l1  * cos(θ1) - p.lc2 * cos(θ2)

    # Velocities of centers of mass
    vx1 =  p.lc1 * cos(θ1) * ω1
    vy1 =  p.lc1 * sin(θ1) * ω1
    vx2 =  p.l1  * cos(θ1) * ω1 + p.lc2 * cos(θ2) * ω2
    vy2 =  p.l1  * sin(θ1) * ω1 + p.lc2 * sin(θ2) * ω2

    # Kinetic energy
    T = 0.5 * p.m1 * (vx1^2 + vy1^2) + 0.5 * p.I1 * ω1^2 +
        0.5 * p.m2 * (vx2^2 + vy2^2) + 0.5 * p.I2 * ω2^2

    # Potential energy (relative to hanging-down: y1_ref = -lc1, y2_ref = -l1-lc2)
    V = p.m1 * p.g * (y1 + p.lc1) + p.m2 * p.g * (y2 + p.l1 + p.lc2)

    return T + V
end
