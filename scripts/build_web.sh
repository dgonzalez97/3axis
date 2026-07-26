#!/bin/sh

set -eu

if ! command -v emcc >/dev/null 2>&1; then
    echo "Emscripten was not found."
    echo "Install and activate emsdk, then run: make web"
    exit 1
fi

mkdir -p build/web

emcc \
    -Iinclude \
    -std=c11 \
    -O2 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Wconversion \
    -Wshadow \
    src/rate_damping.c \
    src/health_monitor.c \
    web/wasm_bridge.c \
    --no-entry \
    -s EXPORTED_FUNCTIONS='[
        "_web_reset",
        "_web_step",
        "_web_get_body_rate",
        "_web_get_wheel_torque",
        "_web_get_wheel_saturated",
        "_web_health_reset",
        "_web_health_update_aocs",
        "_web_health_update_battery",
        "_web_health_update_gnss",
        "_web_health_get_severity",
        "_web_health_get_faults",
        "_web_health_get_actions"
    ]' \
    -o build/web/controller.js

cp web/index.html build/web/index.html
cp web/style.css build/web/style.css
cp web/app.js build/web/app.js

echo "Browser demo built in build/web/"
echo "Run: python3 -m http.server 8000 --directory build/web"
