#!/usr/bin/env bash
# Start AcrobotSim: Julia simulator + Dash UI
# Usage: bash start_acrobatsim.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASH_DIR="$SCRIPT_DIR/dash"
VENV_DIR="$DASH_DIR/.venv"

# ── Ensure vcan0 and vcan2 are up ─────────────────────────────────────────
for iface in vcan0 vcan2; do
    if ! ip link show "$iface" &>/dev/null; then
        echo "ERROR: $iface not found. Run: sudo bash J1939Parser.jl/logs/setupVirtualCAN.sh"
        exit 1
    fi
done
echo "✓ vcan0 and vcan2 are up"

# ── Free ports 8050, 9100, 9101 if already in use ────────────────────────
for port in 8050 9100 9101; do
    pids=$(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K[0-9]+' | sort -u || true)
    if [ -n "$pids" ]; then
        echo "Killing processes on port $port: $pids"
        for pid in $pids; do
            kill "$pid" 2>/dev/null || true
        done
    fi
done
# Wait for ports to actually free up
sleep 2
for port in 8050 9100 9101; do
    if ss -tln | grep -q ":${port} "; then
        echo "WARN: port $port still in use, force-killing..."
        pids=$(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K[0-9]+' | sort -u || true)
        for pid in $pids; do
            kill -9 "$pid" 2>/dev/null || true
        done
        sleep 1
    fi
done
echo "✓ Ports 8050, 9100, 9101 are free"

# ── Start Julia simulator in background ───────────────────────────────────
echo "Starting Julia AcrobotSim simulator..."
cd "$SCRIPT_DIR"
julia --threads=auto --project=. src/runscript.jl &
JULIA_PID=$!
echo "  Julia PID: $JULIA_PID"

# Wait for TcpMonitor to be listening on port 9100 before starting Dash
echo "Waiting for TcpMonitor (port 9100)..."
for i in $(seq 1 60); do
    if ss -tln | grep -q ':9100 '; then
        echo "✓ TcpMonitor ready on port 9100"
        break
    fi
    if ! kill -0 "$JULIA_PID" 2>/dev/null; then
        echo "ERROR: Julia process exited unexpectedly"
        exit 1
    fi
    sleep 1
done

if ! ss -tln | grep -q ':9100 '; then
    echo "ERROR: TcpMonitor did not start within 60s"
    kill "$JULIA_PID" 2>/dev/null || true
    exit 1
fi

# ── Start Dash app in background ──────────────────────────────────────────
echo "Starting Dash UI on port 8050..."
source "$VENV_DIR/bin/activate"
cd "$DASH_DIR"
python app.py &
DASH_PID=$!
echo "  Dash PID: $DASH_PID"

echo ""
echo "═══════════════════════════════════════════"
echo "  AcrobotSim running"
echo "  Dash UI:  http://localhost:8050"
echo "  Params:   TCP port 9100"
echo "  Stream:   TCP port 9101"
echo "═══════════════════════════════════════════"
echo "  Press Ctrl+C to stop both processes"
echo ""

# ── Cleanup on exit ───────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Shutting down..."
    kill "$DASH_PID" 2>/dev/null || true
    kill "$JULIA_PID" 2>/dev/null || true
    wait "$DASH_PID" 2>/dev/null || true
    wait "$JULIA_PID" 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

# Wait for Julia (the long-running process)
wait "$JULIA_PID" 2>/dev/null || true
