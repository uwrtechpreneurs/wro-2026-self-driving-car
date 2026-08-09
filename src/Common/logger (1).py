#!/usr/bin/env python3
"""
logger.py

CSV Logger
==========

Logs all important robot data to a timestamped CSV file.

Each run creates a new CSV inside

logs/

Author : Jithu Joseph
"""

import csv
import os
from datetime import datetime


class Logger:

    # ======================================================
    # Constructor
    # ======================================================

    def __init__(self):
        
        self.flush_counter = 0

        # --------------------------------------------------
        # Create log directory
        # --------------------------------------------------

        os.makedirs("logs", exist_ok=True)

        # --------------------------------------------------
        # Timestamped filename
        # --------------------------------------------------

        timestamp = datetime.now().strftime(
            "%Y-%m-%d_%H-%M-%S"
        )

        filename = os.path.join(
            "logs",
            f"{timestamp}.csv"
        )

        # --------------------------------------------------
        # Open CSV
        # --------------------------------------------------

        self.file = open(
            filename,
            "w",
            newline=""
        )
        
        self.filename = filename

        print(
            f"Logging to: {self.filename}"
        )

        self.writer = csv.writer(self.file)

        # --------------------------------------------------
        # Write Header
        # --------------------------------------------------

        self.writer.writerow([

            "time",
            
            "cycle_time",

            "state",

            "imu_heading",
            
            "imu_raw",

            "imu_sys_cal",

            "imu_gyro_cal",

            "imu_accel_cal",

            "imu_mag_cal",

            "heading_offset",

            "rotation",

            "wall_front",

            "wall_left",

            "wall_right",

            "track_width",

            "distance_error",

            "heading_error",

            "steering",

            "target_rpm",

            "actual_rpm",

            "encoder_ticks",

            "lidar_heading",

            "heading_confidence",

            "left_far_points",

            "left_far_ratio",

            "right_far_points",

            "right_far_ratio",

            "turn_direction",
            
            "wall_used",
            
            "left_pca_heading",
            
            "left_pca_linearity",
            
            "left_pca_points",
            
            "right_pca_heading",
            
            "right_pca_linearity",
            
            "right_pca_points"

        ])
        
        # --------------------------------------------------
        # Start Time
        # --------------------------------------------------

        self.start_time = datetime.now()
        self.last_log_time = self.start_time
        
    # ======================================================
    # Log One Navigation Cycle
    # ======================================================

    def log(self, navigator):
        """
        Log one navigation cycle.
        """
        current = datetime.now()

        elapsed_time = (
            current -
            self.start_time
        ).total_seconds()

        cycle_time = (
            current -
            self.last_log_time
        ).total_seconds()

        self.last_log_time = current
        self.writer.writerow([

            elapsed_time,
            
            cycle_time,
            
            navigator.state.name,

            navigator.imu_heading,
            
            navigator.imu_raw,

            navigator.imu_sys_cal,

            navigator.imu_gyro_cal,

            navigator.imu_accel_cal,

            navigator.imu_mag_cal,

            navigator.heading_offset,

            navigator.rotation,

            navigator.wall_front,

            navigator.wall_left,

            navigator.wall_right,

            navigator.track_width,

            navigator.distance_error,

            navigator.heading_error,

            navigator.steering,

            navigator.target_rpm,

            navigator.current_rpm,

            navigator.encoder_ticks,

            navigator.lidar_heading,

            navigator.heading_confidence,

            navigator.left_far_points,

            navigator.left_far_ratio,

            navigator.right_far_points,

            navigator.right_far_ratio,

            navigator.turn_direction,
            
            navigator.wall_used,
            
            navigator.left_pca_heading,
            
            
            navigator.left_pca_linearity,
            
            navigator.left_pca_points,
            
            navigator.right_pca_heading,
            
            navigator.right_pca_linearity,
            
            navigator.right_pca_points

        ])

        self.flush_counter += 1

        if self.flush_counter >= 10:

            self.file.flush()

            self.flush_counter = 0
    # ======================================================
    # Close Logger
    # ======================================================

    def close(self):

        if self.file is None:
            return

        self.file.flush()
        self.file.close()
        self.file = None
