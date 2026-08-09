#!/usr/bin/env python3
"""
main.py

Main entry point for the WRO Navigation System.

Author : Jithu Joseph
"""

import serial
import sys
import time

from lidar_reader import LidarC1
from navigation import Navigator
from logger import Logger


# ======================================================
# Configuration
# ======================================================

SERIAL_PORT = "/dev/ttyAMA0"
SERIAL_BAUD = 460800

ENABLE_LOGGER = True

# ======================================================
# Initialise
# ======================================================

print("--------------------------------")
print("Starting LiDAR...")
print("--------------------------------")

lidar = LidarC1()
lidar.start()

print("--------------------------------")
print("Creating Navigator...")
print("--------------------------------")

navigator = Navigator()

if not navigator.bridge.start():

    print("Failed to connect to Teensy")

    sys.exit(1)

if ENABLE_LOGGER:

    navigator.logger = Logger()

print("--------------------------------")
print("System Ready")
print("--------------------------------")



# ======================================================
# Main Loop
# ======================================================

try:

    while True:

        angles_deg, distances_mm = lidar.get_scan()

        if len(angles_deg) == 0:
            continue

        navigator.run(
            angles_deg,
            distances_mm
        )

except KeyboardInterrupt:

    print("\nKeyboard interrupt received.")

finally:

    print("\nStopping robot...")

    try:
        navigator.stop_robot()
    except Exception as e:
        print(f"Stop robot failed: {e}")

    try:
        if navigator.logger is not None:
            navigator.logger.close()
    except Exception as e:
        print(f"Logger close failed: {e}")

    try:
        lidar.stop()
    except Exception as e:
        print(f"LiDAR stop failed: {e}")

    try:
        navigator.bridge.stop()
    except Exception as e:
        print(f"Bridge stop failed: {e}")

    print("Shutdown complete.")

    sys.exit(0)
