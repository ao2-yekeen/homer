#!/usr/bin/env bash
# Read-only RPLIDAR evidence logger.  It does not restart ROS, touch USB
# power, or change any device configuration.
interval="${RPLIDAR_LOG_INTERVAL:-10}"
log_dir="${RPLIDAR_LOG_DIR:-$HOME/rplidar-diagnostics}"
mkdir -p "$log_dir"
log_file="$log_dir/$(date +%Y-%m-%dT%H-%M-%S%z).log"
exec >>"$log_file" 2>&1

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
set -u

echo "timestamp=$(( $(date +%s) )) iso=$(date --iso-8601=seconds) event=logger_start"
echo "host=$(hostname) kernel=$(uname -r)"
echo "log_file=$log_file interval=${interval}s"

while :; do
    now="$(date --iso-8601=seconds)"
    echo "=== sample=$now ==="
    echo "-- throttled --"
    vcgencmd get_throttled 2>&1 || true
    echo "-- usb --"
    lsusb 2>&1 || true
    echo "-- serial --"
    ls -l /dev/ttyUSB* /dev/robot-lidar 2>&1 || true
    for dev in /dev/ttyUSB0 /dev/ttyUSB1 /dev/robot-lidar; do
        if [[ -e "$dev" ]]; then
            echo "-- udev $dev --"
            udevadm info -q property -n "$dev" 2>&1 | grep -E '^(DEVNAME|ID_VENDOR|ID_MODEL|ID_SERIAL|ID_PATH|DEVLINKS)=' || true
        fi
    done
    echo "-- usb power --"
    for control in /sys/bus/usb/devices/*/power/control; do
        [[ -e "$control" ]] && printf '%s=' "$control" && cat "$control"
    done
    echo "-- kernel recent (permission permitting) --"
    dmesg --ctime 2>&1 | tail -40 || true
    echo "-- kernel journal recent (permission permitting) --"
    journalctl -k --since '20 seconds ago' --no-pager -o short-iso 2>&1 | tail -80 || true
    echo "-- rplidar service --"
    systemctl --user show rplidar.service -p ActiveState -p SubState -p MainPID -p NRestarts -p ExecMainStatus -p ExecMainCode 2>&1 || true
    journalctl --user -u rplidar.service --since '20 seconds ago' --no-pager -o short-iso 2>&1 | tail -80 || true
    echo "-- ROS nodes --"
    timeout 5 ros2 node list 2>&1 || true
    echo "-- scan hz (5 seconds) --"
    timeout 6 ros2 topic hz /scan 2>&1 || true
    echo "-- end sample --"
    sleep "$interval"
done
