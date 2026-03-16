#!/usr/bin/env bash
# setup.sh — bootstrap the g1_io development environment
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "=== G1 IO setup ==="

# PYTHONPATH
PYPATH_LINE="export PYTHONPATH=$SCRIPT_DIR:\$PYTHONPATH"
if   [ -n "$ZSH_VERSION" ];  then SHELL_RC="$HOME/.zshrc"
elif [ -n "$BASH_VERSION" ]; then SHELL_RC="$HOME/.bashrc"
else SHELL_RC=""; fi
if [ -n "$SHELL_RC" ] && ! grep -qF "$PYPATH_LINE" "$SHELL_RC" 2>/dev/null; then
    echo "$PYPATH_LINE" >> "$SHELL_RC"
    echo "Added PYTHONPATH to $SHELL_RC"
fi
eval "$PYPATH_LINE"

# Install pixi if missing
if ! command -v pixi &>/dev/null; then
    echo "Installing pixi…"
    curl -fsSL https://pixi.prefix.dev/install.sh | bash
    export PATH="$HOME/.pixi/bin:$PATH"
fi

# Install environment
cd "$SCRIPT_DIR/third_party/setup"
pixi install
echo ""
echo "=== Setup complete ==="
echo "Next steps:"
echo "  1. cd third_party/unitree_ros2 && git clone https://github.com/unitreerobotics/unitree_ros2 ."
echo "  2. cd third_party/unitree_sdk2 && git clone https://github.com/unitreerobotics/unitree_sdk2 .  (optional)"
echo "  3. cd third_party/setup && pixi shell"
echo "  4. Build unitree_ros2 first, then: colcon build --symlink-install"
echo "  5. source install/setup.bash"
echo ""
echo "Run targets (inside pixi shell):"
echo "  pixi run run_real              real robot only"
echo "  pixi run run_real_isaac        real + Isaac simultaneously"
echo "  pixi run run_real_mujoco       real + MuJoCo simultaneously"
echo "  pixi run run_isaac_mujoco      Isaac + MuJoCo simultaneously"
echo "  pixi run foxglove              Foxglove at ws://localhost:8765"
echo "  pixi run realsense             RealSense camera"
echo ""
echo "To enable BMS/board/pressure topics (optional):"
echo "  colcon build --cmake-args -DUSE_SDK2=ON"
