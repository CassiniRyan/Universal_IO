#!/bin/bash
# Source this file from each terminal before running G1 IO:
#   source setup.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Setup unitree ros2 environment"
if [ -n "$ROS_SETUP" ] && [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: no ROS2 setup.bash found under /opt/ros."
fi

if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
elif [ -f "$SCRIPT_DIR/third_party/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/third_party/unitree_ros2/cyclonedds_ws/install/setup.bash"
else
    echo "Warning: unitree_ros2 CycloneDDS setup.bash was not found."
fi

if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
export G1_IO_PROJECT_DIR="$SCRIPT_DIR"
export G1_IO_LOG_DIR="$SCRIPT_DIR/g1_logs"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
UNITREE_NETWORK_MAC="${UNITREE_NETWORK_MAC:-6c:1f:f7:71:a8:b5}"

detect_unitree_interface() {
    if [ -n "$UNITREE_NETWORK_INTERFACE" ]; then
        printf '%s' "$UNITREE_NETWORK_INTERFACE"
        return
    fi

    for iface_path in /sys/class/net/*; do
        [ -d "$iface_path" ] || continue
        [ -f "$iface_path/address" ] || continue
        iface_mac="$(cat "$iface_path/address")"
        if [ "$iface_mac" = "$UNITREE_NETWORK_MAC" ]; then
            basename "$iface_path"
            return
        fi
    done

    if [ -d /sys/class/net/enp3s0 ]; then
        printf '%s' "enp3s0"
        return
    fi

    for iface_path in /sys/class/net/en* /sys/class/net/eth*; do
        [ -d "$iface_path" ] || continue
        iface="$(basename "$iface_path")"
        [ "$iface" = "lo" ] && continue
        [ -f "$iface_path/operstate" ] && [ "$(cat "$iface_path/operstate")" != "up" ] && continue
        printf '%s' "$iface"
        return
    done

    for iface_path in /sys/class/net/*; do
        [ -d "$iface_path" ] || continue
        iface="$(basename "$iface_path")"
        case "$iface" in
            lo|docker*|lxc*|br*) continue ;;
        esac
        [ -f "$iface_path/operstate" ] && [ "$(cat "$iface_path/operstate")" != "up" ] && continue
        printf '%s' "$iface"
        return
    done
}

UNITREE_SELECTED_INTERFACE="$(detect_unitree_interface)"
if [ -n "$UNITREE_SELECTED_INTERFACE" ] && [ -d "/sys/class/net/$UNITREE_SELECTED_INTERFACE" ]; then
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"$UNITREE_SELECTED_INTERFACE\" priority=\"default\" multicast=\"default\" />
                        </Interfaces></General></Domain></CycloneDDS>"
else
    unset CYCLONEDDS_URI
    echo "Warning: no valid Unitree network interface found. CYCLONEDDS_URI was unset."
    echo "Set UNITREE_NETWORK_INTERFACE=<iface> before sourcing setup.sh if needed."
fi

echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "G1_IO_LOG_DIR=$G1_IO_LOG_DIR"
if [ -n "$CYCLONEDDS_URI" ]; then
    echo "CycloneDDS network interface: $UNITREE_SELECTED_INTERFACE"
    echo "CycloneDDS interface MAC: $(cat "/sys/class/net/$UNITREE_SELECTED_INTERFACE/address" 2>/dev/null)"
fi
