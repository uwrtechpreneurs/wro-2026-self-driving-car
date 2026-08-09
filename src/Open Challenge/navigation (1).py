#!/usr/bin/env python3
"""
navigation.py

World-Frame Navigation
======================

This module controls the complete navigation of the robot.

Navigation is divided into four states

    INITIALIZE
    SEARCH
    TURN
    CALIBRATE

The robot uses

• IMU for continuous heading estimation

• LiDAR for wall geometry

• PCA only during calibration

The primary heading used throughout the navigation
system is

    rotation

where

    rotation = imu_heading + heading_offset

heading_offset is obtained from LiDAR calibration

    heading_offset = lidar_heading - imu_heading

Author : Jithu Joseph
"""

from enum import Enum

import time
import math

import numpy as np

from wall_extractor import WallExtractor
from teensy_bridge import TeensyBridge


# ==========================================================
# Navigation States
# ==========================================================

class NavState(Enum):

    INITIALIZE = 0

    SEARCH = 1

    TURN = 2

    CALIBRATE = 3

    REVERSE_FOR_TURN = 4
    
    FINISH = 5

# ==========================================================
# Navigation
# ==========================================================

class Navigator:

    # ======================================================
    # Configuration
    # ======================================================

    # ---------- Speed ----------

    SEARCH_RPM = 210

    TURN_RPM = SEARCH_RPM*0.85

    # ---------- Steering ----------

    MAX_STEERING = 35.0

    # ---------- Controller Gains ----------

    KP_DISTANCE = 0.14

    KP_HEADING = 1.50

    # ---------- Desired Wall Distance ----------

    DESIRED_OFFSET = 0.0

    # ---------- Corner Detection ----------

    FRONT_WALL_DISTANCE = 1100.0

    FAR_RATIO_THRESHOLD = 0.25
    
    MIN_FAR_POINTS = 20

    # ---------- Calibration ----------

    CONFIDENCE_THRESHOLD = 0.9

    # ---------- Turn ----------

    TURN_ANGLE = 90.0

    TURN_TOLERANCE = 1.0
    
    # ---------- Turn Controller ----------

    TURN_KP = 1.08

    TURN_MIN_STEERING = 8.0

    TURN_MAX_STEERING = 35.0
    
    CRITICAL_WALL_DISTANCE = 150.0 
    
    # ---------- Single Wall Following ----------

    SINGLE_WALL_DISTANCE = 400.0
    
    # ---------- Narrow Corridor ----------

    CORRIDOR_CLASSIFICATION_DISTANCE = 690

    TURN_START_DISTANCE = 690    
    REVERSE_RPM = -130
    
    # ---------- FINISH ----------

    TOTAL_CORNERS = 12
    FINISH_DISTANCE = 1850.0

    # ======================================================
    # Constructor
    # ======================================================

    def __init__(self):

        # ----------------------------------------------
        # Navigation State
        # ----------------------------------------------

        self.state = NavState.INITIALIZE
        self.last_state = NavState.INITIALIZE

        # ----------------------------------------------
        # Wall Extractor
        # ----------------------------------------------

        self.extractor = WallExtractor()

        # ----------------------------------------------
        # Heading
        # ----------------------------------------------

        self.imu_heading = 0.0
        
        self.imu_raw = 0.0

        self.imu_sys_cal = 0
        self.imu_gyro_cal = 0
        self.imu_accel_cal = 0
        self.imu_mag_cal = 0

        self.heading_offset = 0.0

        self.rotation = 0.0
        
        self.turn_start_rotation = 0.0
        
        # Calibration debug data
        self.wall_used = ""

        self.left_pca_heading = None
        self.left_pca_linearity = None
        self.left_pca_points = None

        self.right_pca_heading = None
        self.right_pca_linearity = None
        self.right_pca_points = None
        
        # ----------------------------------------------
        # Wall Distances
        # ----------------------------------------------

        self.wall_front = None

        self.wall_left = None

        self.wall_right = None

        self.track_width = None

        # ----------------------------------------------
        # Controller
        # ----------------------------------------------

        self.distance_error = 0.0

        self.heading_error = 0.0

        self.steering = 0.0

        # ----------------------------------------------
        # Corner Detection
        # ----------------------------------------------

        self.turn_direction = None

        # ----------------------------------------------
        # Calibration
        # ----------------------------------------------

        self.calibration_attempts = 0
        
        self.lidar_heading = None

        self.heading_confidence = None

        # ----------------------------------------------
        # Robot Interface
        # ----------------------------------------------

        self.bridge = TeensyBridge()

        self.current_rpm = 0.0

        self.target_rpm = 0.0

        self.encoder_ticks = 0
        
        self.stall_start_time = None
        
        self.STALL_RPM_THRESHOLD = 15.0
        
        self.STALL_TIMEOUT_SEC = 0.4
        
        #-----------------------------------------------
        #Logger
        #-----------------------------------------------
        self.logger = None
        
        # ----------------------------------------------
        # Corner Detection
        # ----------------------------------------------

        self.turn_direction = None

        self.left_far_points = 0
        self.left_far_ratio = 0.0

        self.right_far_points = 0
        self.right_far_ratio = 0.0
        
        self.reverse_distance_mm = 0.0

        self.reverse_start_ticks = 0
        self.reverse_target_ticks = 0

        self.corner_front_distance = None
        
        # ----------------------------------------------
        # FINISH
        # ----------------------------------------------
        
        self.completed_corners = 0
        self.mission_complete = False
        self.first_search_after_calibration = False
        
    @staticmethod
    def wrap180(angle):
        """
        Wrap angle to [-180, 180).
        """
        return (angle + 180.0) % 360.0 - 180.0
                
    # ======================================================
    # Update Rotation
    # ======================================================

    def update_rotation(self):

        self.rotation = self.wrap180(

            self.imu_heading -

            self.heading_offset

        )

    # ======================================================
    # Robot Communication
    # ======================================================

    def send_drive_command(
            self,
            rpm,
            steering):
                
        self.target_rpm = rpm

        self.bridge.send_command(
            rpm,
            steering
        )
    # ======================================================
    # Zero IMU
    # ======================================================

    def zero_imu(self):

        self.bridge.zero_imu()


    # ======================================================
    # Update Telemetry
    # ======================================================

    def update_telemetry(self):
        """
        Update the latest telemetry from TeensyBridge.
        """

        telemetry = self.bridge.get_telemetry()

        if telemetry is None:
            return False

        self.current_rpm = telemetry["actual_rpm"]

        self.encoder_ticks = telemetry["total_ticks"]

        self.imu_heading = telemetry["heading"]

        self.imu_raw = telemetry["imu_raw"]

        self.imu_sys_cal = telemetry["imu_sys_cal"]
        self.imu_gyro_cal = telemetry["imu_gyro_cal"]
        self.imu_accel_cal = telemetry["imu_accel_cal"]
        self.imu_mag_cal = telemetry["imu_mag_cal"]

        self.update_rotation()

        return True

    def check_stall(self):
            """
            Returns True if the drive motor has been commanded to move
            but hasn't actually turned for STALL_TIMEOUT_SEC.
            """
            if self.target_rpm != 0 and abs(self.current_rpm) < self.STALL_RPM_THRESHOLD:
                if self.stall_start_time is None:
                    self.stall_start_time = time.time()
                elif time.time() - self.stall_start_time > self.STALL_TIMEOUT_SEC:
                    return True
            else:
                self.stall_start_time = None

            return False

    # ======================================================
    # Stop Robot
    # ======================================================

    def stop_robot(self):
        """
        Stop drive motor.
        """

        self.send_drive_command(

            0,

            0

        )
    # ======================================================
    # Distance Controller
    # ======================================================

    def compute_distance_error(self):
        """
        Compute lateral distance error.

        Modes

        • Both walls visible
            -> Stay centred.

        • Left wall only
            -> Maintain fixed distance
               from left wall.

        • Right wall only
            -> Maintain fixed distance
               from right wall.

        • No walls
            -> Zero error.
        """

        # ------------------------------------------
        # Both Walls Visible
        # ------------------------------------------

        if (

            self.wall_left is not None

            and

            self.wall_right is not None

        ):

            self.distance_error = (

                self.wall_right

                -

                self.wall_left

            ) / 2.0

            return

        # ------------------------------------------
        # Left Wall Only
        # ------------------------------------------

        if self.wall_left is not None:

            self.distance_error = (

                self.SINGLE_WALL_DISTANCE

                -

                self.wall_left

            )

            return

        # ------------------------------------------
        # Right Wall Only
        # ------------------------------------------

        if self.wall_right is not None:

            self.distance_error = (

                self.wall_right

                -

                self.SINGLE_WALL_DISTANCE

            )

            return

        # ------------------------------------------
        # No Walls Visible
        # ------------------------------------------

        self.distance_error = 0.0


    # ======================================================
    # Heading Controller
    # ======================================================

    def compute_heading_error(self):
        """
        Heading correction.

        Uses the fused rotation value.
        """

        self.heading_error = self.rotation


    # ======================================================
    # Steering Controller
    # ======================================================

    def compute_steering(self):
        """
        Combine distance and heading
        corrections.
        """

        steering = (

            self.KP_DISTANCE

            * self.distance_error

            -

            self.KP_HEADING

            * self.heading_error

        )

        steering = max(

            -self.MAX_STEERING,

            min(

                self.MAX_STEERING,

                steering

            )

        )

        self.steering = steering


    # ======================================================
    # Drive Forward
    # ======================================================

    def drive_forward(self):
        """
        Drive using the current steering command.
        """

        self.send_drive_command(

            self.SEARCH_RPM,

            self.steering

        )
    # ======================================================
    # Navigation State Machine
    # ======================================================

    def run(self,
            angles_deg,
            distances_mm):
        """
        Execute one navigation cycle.
        """
        if not self.bridge.is_alive():

            self.stop_robot()

            return

        self.update_telemetry()

        if self.check_stall():       
            self.stop_robot()
            return
            
        if (
            angles_deg is None
            or
            distances_mm is None
        ):
            self.stop_robot()
            return

        if len(angles_deg) == 0:
            self.stop_robot()
            return

        if self.state == NavState.INITIALIZE:

            self.state_initialize(
                angles_deg,
                distances_mm
            )

        elif self.state == NavState.SEARCH:

            self.state_search(
                angles_deg,
                distances_mm
            )

        elif self.state == NavState.TURN:

            self.state_turn()

        elif self.state == NavState.CALIBRATE:

            self.state_calibrate(
                angles_deg,
                distances_mm
            )
            
        elif self.state == NavState.REVERSE_FOR_TURN:

            self.state_reverse_for_turn(
                angles_deg,
                distances_mm
            )
    
        if self.logger is not None:
            self.logger.log(self)
        self.last_state = self.state
        
    # ======================================================
    # INITIALIZE
    # ======================================================

    def state_initialize(
            self,
            angles_deg,
            distances_mm):
        """
        Initialise the navigation system.
        """

        self.zero_imu()

        self.heading_offset = 0.0

        self.rotation = 0.0

        self.calibration_attempts = 0

        self.state = NavState.CALIBRATE           
    # ======================================================
    # TURN
    # ======================================================

    def state_turn(self):
        """
        Execute a 90° turn using proportional control.
        """

        # ----------------------------------------------
        # Target Rotation
        # ----------------------------------------------

        if self.turn_direction == "RIGHT":
            target_rotation = self.TURN_ANGLE

        else:
            target_rotation = -self.TURN_ANGLE

        # ----------------------------------------------
        # Compute Heading Error
        # ----------------------------------------------

        error = self.wrap180(
            target_rotation - self.rotation
        )

        # ----------------------------------------------
        # Turn Complete
        # ----------------------------------------------

        if abs(error) <= self.TURN_TOLERANCE:

            self.stop_robot()

            self.zero_imu()
            
            self.completed_corners += 1
            if self.completed_corners >= self.TOTAL_CORNERS:
                self.mission_complete = True


            self.calibration_attempts = 0

            self.state = NavState.CALIBRATE

            return

        # ----------------------------------------------
        # Proportional Steering
        # ----------------------------------------------

        steering = self.TURN_KP * error

        # Maintain minimum steering torque

        if steering > 0:

            steering = max(
                steering,
                self.TURN_MIN_STEERING
            )

        else:

            steering = min(
                steering,
                -self.TURN_MIN_STEERING
            )

        # Clamp steering

        steering = max(

            -self.TURN_MAX_STEERING,

            min(

                self.TURN_MAX_STEERING,

                steering

            )

        )

        # ----------------------------------------------
        # Execute Turn
        # ----------------------------------------------

        if abs(error) < 20:
            turn_rpm = self.TURN_RPM * 0.9   # slow down on approach
        else:
            turn_rpm = self.TURN_RPM
        self.send_drive_command(turn_rpm, steering)
 
    # ======================================================
    # REVERSE
    # ======================================================
            
    def state_reverse_for_turn(
            self,
            angles_deg,
            distances_mm):
                
        result = self.extractor.extract(
            angles_deg,
            distances_mm,
            self.rotation
        )

        if result is None:
            self.stop_robot()
            return

        self.wall_front = result["wall_F"]

        if self.wall_front < self.TURN_START_DISTANCE:

            self.send_drive_command(
                self.REVERSE_RPM,
                0
            )
            return

        self.stop_robot()
        self.state = NavState.TURN
        return
        
    # ======================================================
    # CALIBRATE
    # ======================================================

    def state_calibrate(self, angles_deg, distances_mm):
        
        if not self.bridge.is_heading_settled():
            self.send_drive_command(0, 0)
            return

        # --------------------------------------------------
        # Startup calibration
        # --------------------------------------------------

        if self.turn_direction is None:

            result = self.extractor.calibrate_corridor_heading(
                angles_deg,
                distances_mm,
                rotation=0.0,
                outer_wall=None
            )
            self.wall_used = result["wall_used"]

            self.left_pca_heading = result["left_pca_heading"]
            self.left_pca_linearity = result["left_pca_linearity"]
            self.left_pca_points = result["left_pca_points"]

            self.right_pca_heading = result["right_pca_heading"]
            self.right_pca_linearity = result["right_pca_linearity"]
            self.right_pca_points = result["right_pca_points"]

        # --------------------------------------------------
        # Calibration after a turn
        # --------------------------------------------------

        else:

            if self.turn_direction == "RIGHT":
                outer_wall = "LEFT"

            elif self.turn_direction == "LEFT":
                outer_wall = "RIGHT"

            else:
                raise RuntimeError(
                    f"Invalid turn direction: {self.turn_direction}"
                )

            result = self.extractor.calibrate_corridor_heading(
                angles_deg,
                distances_mm,
                rotation=0.0,
                outer_wall=outer_wall
            )

            self.wall_used = result["wall_used"]

            self.left_pca_heading = result["left_pca_heading"]
            self.left_pca_linearity = result["left_pca_linearity"]
            self.left_pca_points = result["left_pca_points"]

            self.right_pca_heading = result["right_pca_heading"]
            self.right_pca_linearity = result["right_pca_linearity"]
            self.right_pca_points = result["right_pca_points"]

        self.lidar_heading = result["heading"]
        self.heading_confidence = result["confidence"]

        if result["heading"] is not None and result["confidence"] >= self.CONFIDENCE_THRESHOLD:
            self.heading_offset = result["heading"] - self.imu_heading
            self.update_rotation()
            self.calibration_attempts = 0
            
            self.first_search_after_calibration = True
            
            self.state = NavState.SEARCH
            self.turn_direction = None
            return

        # Confidence too low (or no reading) — keep driving forward and try again
        self.send_drive_command(self.SEARCH_RPM, 0)

        if self.calibration_attempts == 0:
            self.calibration_start_time = time.time()

        self.calibration_attempts += 1

        if time.time() - self.calibration_start_time > 3.0:
            print("WARNING: calibration stuck > 3s, confidence never cleared threshold")

            # Keep previous offset rather than using an unreliable one
            self.update_rotation()

            self.calibration_attempts = 0
            
            self.first_search_after_calibration = True
            
            self.turn_direction = None
            self.state = NavState.SEARCH                      
    # ======================================================
    # SEARCH
    # ======================================================

    def state_search(
            self,
            angles_deg,
            distances_mm):
        """
        Normal wall following.

        During this state the robot

        • follows the corridor

        • keeps itself centred

        • detects corners
        """

        # --------------------------------------------------
        # Update Rotation
        # --------------------------------------------------

        self.update_rotation()

        # --------------------------------------------------
        # Extract Corridor Geometry
        # --------------------------------------------------

        result = self.extractor.extract(

            angles_deg,

            distances_mm,

            self.rotation

        )

        if result is None:

            self.stop_robot()

            return

        self.left_far_points = result["left"]["far_points"]
        self.left_far_ratio = result["left"]["far_ratio"]

        self.right_far_points = result["right"]["far_points"]
        self.right_far_ratio = result["right"]["far_ratio"]
        

        # --------------------------------------------------
        # Store Geometry
        # --------------------------------------------------

        self.wall_front = result["wall_F"]

        self.wall_left = result["wall_L"]

        self.wall_right = result["wall_R"]

        self.track_width = result["track_width"]
        
        if self.mission_complete:

            # Ignore the first SEARCH cycle after calibration
            if self.first_search_after_calibration:

                self.first_search_after_calibration = False

                return

            # Now use the fresh LiDAR measurement
            if self.wall_front > self.FINISH_DISTANCE:

                self.send_drive_command(
                    self.SEARCH_RPM,
                    0
                )

            else:

                self.stop_robot()

            return
        
        if self.wall_left is not None and self.wall_left < self.CRITICAL_WALL_DISTANCE:
            self.send_drive_command(self.SEARCH_RPM * 0.6, self.MAX_STEERING)   # hard steer away, right
            return
        if self.wall_right is not None and self.wall_right < self.CRITICAL_WALL_DISTANCE:
            self.send_drive_command(self.SEARCH_RPM * 0.6, -self.MAX_STEERING)
            return

        # --------------------------------------------------
        # Controller
        # --------------------------------------------------

        self.compute_distance_error()

        self.compute_heading_error()

        self.compute_steering()

        self.drive_forward()

        # --------------------------------------------------
        # Corner Detection
        # --------------------------------------------------

        front_wall = (

            self.wall_front is not None

            and

            self.wall_front

            <

            self.FRONT_WALL_DISTANCE

        )

        # --------------------------------------------------
        # Right Corner
        # --------------------------------------------------

        if (

            front_wall

            and

            self.right_far_points > self.MIN_FAR_POINTS

            and

            self.right_far_ratio > self.FAR_RATIO_THRESHOLD
        ):

            self.turn_direction = "RIGHT"

            self.turn_start_rotation = self.rotation

            self.stop_robot()

            self.corner_front_distance = self.wall_front

            if self.wall_front >= self.CORRIDOR_CLASSIFICATION_DISTANCE:

                print("Wide corridor")

                self.state = NavState.TURN

            else:

                print("Narrow corridor")

                self.state = NavState.REVERSE_FOR_TURN
            return

        # --------------------------------------------------
        # Left Corner
        # --------------------------------------------------

        if (

            front_wall

            and

            self.left_far_points > self.MIN_FAR_POINTS

            and

            self.left_far_ratio > self.FAR_RATIO_THRESHOLD

        ):

            self.turn_direction = "LEFT"

            self.turn_start_rotation = self.rotation

            self.stop_robot()

            self.corner_front_distance = self.wall_front

            if self.wall_front >= self.CORRIDOR_CLASSIFICATION_DISTANCE:

                print("Wide corridor")

                self.state = NavState.TURN

            else:

                print("Narrow corridor")

                self.state = NavState.REVERSE_FOR_TURN
            return
