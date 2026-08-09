#!/usr/bin/env python3
"""
main_obstacle.py

Main entry point for the WRO Obstacle Challenge Navigation System.

Author : Jithu Joseph
"""

import sys

from lidar_reader import LidarC1
from navigator_obstacle import Navigator
from logger import Logger


# ======================================================
# Configuration
# ======================================================

ENABLE_LOGGER = True


# ======================================================
# Logger
# ======================================================

logger = Logger() if ENABLE_LOGGER else None


# ======================================================
# Initialise LiDAR
# ======================================================

print("--------------------------------")
print("Starting LiDAR...")
print("--------------------------------")

lidar = LidarC1()
lidar.start()


# ======================================================
# Initialise Navigator
# ======================================================

print("--------------------------------")
print("Creating Navigator...")
print("--------------------------------")

navigator = Navigator(logger=logger)


# ======================================================
# Connect Teensy
# ======================================================

print("--------------------------------")
print("Connecting to Teensy...")
print("--------------------------------")

if not navigator.bridge.start():

    print("Failed to connect to Teensy")

    sys.exit(1)


# ======================================================
# Start Camera
# ======================================================

print("--------------------------------")
print("Starting Camera...")
print("--------------------------------")

navigator.camera.start()


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

    print("Keyboard interrupt")

except Exception:

    import traceback
    traceback.print_exc()

finally:

    print("\nStopping robot...")

    try:
        navigator.stop_robot()
    except Exception as e:
        print(e)

    try:
        navigator.camera.stop()
    except Exception as e:
        print(e)

    try:
        lidar.stop()
    except Exception as e:
        print(e)

    try:
        navigator.bridge.stop()
    except Exception as e:
        print(e)

    print("Shutdown complete.")

    sys.exit(0)
