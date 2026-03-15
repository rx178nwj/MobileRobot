#!/bin/bash
#
# Disable autofocus on Raspberry Pi Camera Module 3
# This script must be run at startup before launching the camera node
#
# Camera Module 3 has autofocus which is problematic for SLAM because:
# - Constant focus changes create inconsistent feature matching
# - Makes visual odometry unstable
#
# Solution: Set fixed focus mode with a specific focus distance

DEVICE="/dev/video0"

echo "Configuring Camera Module 3 focus settings..."

# Disable continuous autofocus
v4l2-ctl -d $DEVICE --set-ctrl=focus_automatic_continuous=0

# Set fixed focus value (0-1023, typical range)
# 450 = approximately 50cm-infinity focus (good for indoor navigation)
# Adjust this value based on your robot's typical operating distance:
#   - Close range (20-50cm): 200-350
#   - Medium range (50cm-2m): 400-500
#   - Far range (1m-infinity): 500-650
v4l2-ctl -d $DEVICE --set-ctrl=focus_absolute=450

echo "Camera focus set to fixed mode (focus_absolute=450)"
echo "Current camera controls:"
v4l2-ctl -d $DEVICE --list-ctrls | grep focus
