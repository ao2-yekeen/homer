"""Fixed Pi USB topology. Keep LiDAR on 1.1 and ESP32 on 1.2.3.

Both CP2102 adapters report serial number 0001, so /dev/serial/by-id is
ambiguous. These by-path links identify the physical USB sockets without
opening either serial port. Update these paths if the cabling changes.
"""

from pathlib import Path

LIDAR_PORT = Path(
    "/dev/serial/by-path/"
    "platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0"
)
ESP32_PORT = Path(
    "/dev/serial/by-path/"
    "platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2.3:1.0-port0"
)
RGB_CAMERA = Path("/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e-video-index0")


def fixed_serial_port(port: Path, other: Path) -> bool:
    """Accept only a present character device distinct from the other port."""
    return port.is_char_device() and (not other.exists() or port.resolve() != other.resolve())
