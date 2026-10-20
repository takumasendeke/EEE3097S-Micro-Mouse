#!/bin/sh
# Build src/main.py into build/s4.mpy and copy it, plus the launcher, to the mouse.
#   ./tools/build_deploy.sh
# The board can't compile the 38 KB source itself (MemoryError), so it runs the
# mpy-cross bytecode through the 2-line launcher src/board/main.py.
set -e
cd "$(dirname "$0")/.."
mkdir -p build
MPY=$(command -v mpy-cross || echo "python3 -m mpy_cross")
$MPY -march=armv7emsp -o build/s4.mpy src/main.py
echo "built build/s4.mpy ($(wc -c < build/s4.mpy | tr -d ' ') bytes): $(strings build/s4.mpy | grep -m1 -E '^v[0-9]+')"
DRIVE=$(ls -d /Volumes/UCT_MMOUSE 2>/dev/null || true)
if [ -n "$DRIVE" ]; then
    cp build/s4.mpy "$DRIVE/s4.mpy"
    cp src/board/main.py "$DRIVE/main.py"
    sync
    echo "copied to $DRIVE - eject it, then press reset"
else
    python3 -m mpremote cp build/s4.mpy :s4.mpy
    python3 -m mpremote cp src/board/main.py :main.py
    echo "copied with mpremote - press reset"
fi
