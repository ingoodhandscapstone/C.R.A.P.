#!/usr/bin/env bash
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="$ROOT/build/app"
UI_DIR="$ROOT/UI"
PYTHON="$UI_DIR/.venv/bin/python"
LOG_DIR="$ROOT/logs"

MOSQUITTO_PID=""
CPP_PID=""
UI_PID=""
CLEANED_UP=0

is_running() {
    [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null
}

port_1883_open() {
    (true <>/dev/tcp/127.0.0.1/1883) >/dev/null 2>&1
}

stop_process() {
    local name="$1"
    local pid="$2"

    if ! is_running "$pid"; then
        return
    fi

    echo "Stopping $name..."
    kill -TERM "$pid" 2>/dev/null || true

    for _ in {1..50}; do
        is_running "$pid" || break
        sleep 0.1
    done

    if is_running "$pid"; then
        echo "$name did not stop after SIGTERM; forcing shutdown."
        kill -KILL "$pid" 2>/dev/null || true
    fi

    wait "$pid" 2>/dev/null || true
}

cleanup() {
    [[ "$CLEANED_UP" -eq 1 ]] && return
    CLEANED_UP=1

    stop_process "UI" "$UI_PID"
    stop_process "C++ app" "$CPP_PID"

    if [[ -n "$MOSQUITTO_PID" ]]; then
        stop_process "Mosquitto" "$MOSQUITTO_PID"
    fi
}

trap 'cleanup' EXIT
trap 'cleanup; exit 130' INT TERM

mkdir -p "$LOG_DIR"

if ! command -v mosquitto >/dev/null 2>&1; then
    echo "mosquitto is not installed or is not on PATH."
    exit 1
fi

if [[ ! -x "$PYTHON" ]]; then
    echo "UI virtual environment not found at $UI_DIR/.venv"
    echo "Create it with: python3 -m venv UI/.venv && UI/.venv/bin/pip install -r UI/requirements.txt"
    exit 1
fi

if ! "$PYTHON" -c 'import tkinter; import paho.mqtt.client; from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg' >/dev/null 2>&1; then
    echo "The selected virtual environment is missing UI dependencies."
    echo "Install them with: $PYTHON -m pip install -r $UI_DIR/requirements.txt"
    echo "If tkinter is missing, install Python Tk support for this Linux system."
    exit 1
fi

if [[ ! -x "$APP" ]]; then
    if [[ ! -f "$ROOT/build/CMakeCache.txt" ]]; then
        echo "Build directory not configured; configuring it now..."
        cmake -S "$ROOT" -B "$ROOT/build" || exit 1
    fi

    echo "build/app not found; building it now..."
    cmake --build "$ROOT/build" --target app || exit 1
fi

if port_1883_open; then
    echo "Using existing MQTT broker on localhost:1883."
else
    echo "Starting Mosquitto..."
    mosquitto -p 1883 >"$LOG_DIR/mosquitto.log" 2>&1 &
    MOSQUITTO_PID=$!

    for _ in {1..50}; do
        port_1883_open && break
        sleep 0.1
    done

    if ! port_1883_open; then
        echo "Mosquitto did not start. Check $LOG_DIR/mosquitto.log"
        exit 1
    fi
fi

echo "Starting C++ app..."
"$APP" &
CPP_PID=$!

sleep 0.5
if ! is_running "$CPP_PID"; then
    echo "C++ app exited during startup. Check the latest session log in $LOG_DIR"
    exit 1
fi

echo "Starting UI..."
(cd "$UI_DIR" && "$PYTHON" mqtt_session_ui.py) &
UI_PID=$!

echo "Close the UI window or press Ctrl+C to stop everything."
wait -n "$UI_PID" "$CPP_PID"
