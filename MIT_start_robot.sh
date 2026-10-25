#!/bin/bash

ARDUINO_DIR="/Users/kaifaucette/Documents/PlatformIO/Projects/Robotic Arm Arduino"
PORT="/dev/cu.usbserial-0001"
PIO="$HOME/.platformio/penv/bin/pio"

# Kill old server if running
pkill -f "node server.js" 2>/dev/null || true

echo "Uploading firmware..."
cd "$ARDUINO_DIR"
"$PIO" run --target upload --upload-port "$PORT"

echo "Starting server..."
cd - > /dev/null
nohup node server.js > server.log 2>&1 &

# Give second to boot
sleep 1

if curl -s http://localhost:3000/status >/dev/null; then
    echo "Ready! Open http://localhost:3000/jointjogger.html"
else
    echo "Server started, but didn't respond to health check. Check server.log"
fi
