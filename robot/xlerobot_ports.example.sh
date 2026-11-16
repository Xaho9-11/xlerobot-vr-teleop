# Stable device paths of this XLeRobot build. Copy to xlerobot_ports.sh and fill in your own; every script sources it.
# Find them with:  ls -l /dev/serial/by-id/   and   ls -l /dev/v4l/by-path/   (and docs/hardware-notes.md)
# bus 1 = left arm + head, bus 2 = right arm + base (USB-serial adapters, by serial number)
export XLR_PORT1=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<SERIAL1>-if00
export XLR_PORT2=/dev/serial/by-id/usb-1a86_USB_Single_Serial_<SERIAL2>-if00
# Cameras by USB port: the robot's cameras share one serial number, so the port is the only stable identity.
# At most two UVC cameras per USB 2.0 bus (see docs/hardware-notes.md).
export XLR_CAM_HEAD=/dev/v4l/by-path/pci-<controller>-usb-0:<port>:1.0-video-index0
export XLR_CAM_SCENE=/dev/v4l/by-path/pci-<controller>-usb-0:<port>:1.0-video-index0   # third-person, optional
export XLR_CAM_LEFT_WRIST=/dev/v4l/by-path/pci-<controller>-usb-0:<port>:1.0-video-index0
export XLR_CAM_RIGHT_WRIST=/dev/v4l/by-path/pci-<controller>-usb-0:<port>:1.0-video-index0
export XLR_ID=xlerobot
