# ============================================================
# navigator_obstacle.py
#
# WRO Future Engineers 2026
# Obstacle Challenge Navigator
#
# Navigation Philosophy
#
#   SEARCH
#       │
#       ▼
#   Update Wall Geometry
#       │
#       ▼
#   Update Pillar Detection
#       │
#       ▼
#   Update Behaviour
#       │
#       ▼
#   Wall Following
#       │
#       ▼
#   Front Wall <= Turn Distance ?
#       │
#       ├── No → Continue SEARCH
#       │
#       └── Yes → TURN
#
# ============================================================

import math
import time

import numpy as np

from enum import Enum, auto

from teensy_bridge import TeensyBridge
from wall_extractor import WallExtractor
from camera_detector import CameraDetector
from pillar_tracker import PillarTracker


# ============================================================
# Navigation State
# ============================================================

class NavState(Enum):

    INITIALIZE = auto()

    EXIT_PARKING = auto()

    SEARCH = auto()

    TURN = auto()

    CALIBRATE = auto()

    PARK = auto()

    REVERSE_FOR_CORNER = auto()

    REVERSE_FOR_PILLAR = auto()


# ============================================================
# Behaviour
# ============================================================

class Behaviour(Enum):

    NORMAL = auto()

    STRAIGHT_RED = auto()

    STRAIGHT_GREEN = auto()

    CORNER_RED = auto()

    CORNER_GREEN = auto()
    
# ============================================================
# Turn Detection
# ============================================================

class TurnDirection(Enum):

    LEFT = auto()

    RIGHT = auto()

# ============================================================
# Navigator
# ============================================================

class Navigator:


    # ========================================================
    #
    # Robot Parameters
    #
    # ========================================================

    SEARCH_RPM = 120

    TURN_RPM = 100

    MAX_STEERING = 30


    # ========================================================
    #
    # Wall Following
    #
    # ========================================================

    KP_DISTANCE = 0.2

    KP_HEADING = 1.25

    SINGLE_WALL_DISTANCE = 525
    
    # ========================================================
    #
    # Calibration
    #
    # ========================================================

    CONFIDENCE_THRESHOLD = 0.9


    # ========================================================
    #
    # Turning
    #
    # ========================================================

    TURN_ANGLE = 90

    TURN_KP = 1.2
    
    TURN_MIN_STEERING = 8.0

    TURN_MAX_STEERING = 35.0
    
    TURN_HEADING_TOLERANCE = 3
    
    FAR_RATIO_THRESHOLD = 0.4
    
    MIN_FAR_POINTS = 30


    # ========================================================
    #
    # Obstacle Parameters
    #
    # ========================================================

    DEFAULT_TURN_DISTANCE = 900

    CORNER_RED_TURN_DISTANCE = 700

    CORNER_GREEN_TURN_DISTANCE = 1150

    STRAIGHT_RED_OFFSET = -350

    STRAIGHT_GREEN_OFFSET = 350

    STRAIGHT_THRESHOLD = 800
    
    # Distance to travel after detecting a straight pillar
    # before returning to the centre line

    STRAIGHT_PASS_DISTANCE_TICKS = 5347      # To be tuned

    # Minimum distance (mm) a straight pillar must be at before
    # the robot stops reversing and begins avoidance.
    # TODO: tune on the real field.
    STRAIGHT_PILLAR_MIN_DISTANCE = 400


    # ========================================================
    #
    # Reverse Maneuvers
    #
    # ========================================================

    # Reused from navigation.py — same reverse speed used for
    # narrow-corridor reversal.
    REVERSE_RPM = -130

    # Safety cutoff so a reverse maneuver can never run away if
    # the target distance is never reached (sensor glitch, robot
    # physically stuck, etc.).
    REVERSE_TIMEOUT_SEC = 2.0


    # ========================================================
    #
    # Parking
    #
    # ========================================================

    PARKING_SEARCH_DISTANCE = 1500
    
    # ========================================================
    #
    # FINISH
    #
    # ========================================================

    TOTAL_CORNERS = 12

    FINISH_DISTANCE = 1850.0


    # ========================================================
    #
    # Constructor
    #
    # ========================================================

    def __init__(self, logger):

        self.logger = logger


        # ----------------------------------------------------
        # Robot Interface
        # ----------------------------------------------------

        self.bridge = TeensyBridge()


        # ----------------------------------------------------
        # Sensors
        # ----------------------------------------------------

        self.extractor = WallExtractor()

        self.camera = CameraDetector()

        self.pillar_tracker = PillarTracker()


        # ----------------------------------------------------
        # Navigation
        # ----------------------------------------------------

        self.state = NavState.INITIALIZE


        self.turn_direction = None

        self.direction_locked = False
        
        # ----------------------------------------------------
        # Calibration
        # ----------------------------------------------------

        self.initial_calibration_done = False

        self.calibration_attempts = 0

        self.calibration_start_time = 0.0

        self.wall_used = None

        self.heading_confidence = 0.0

        self.lidar_heading = 0.0
        
        self.first_search_after_calibration = False
        
        # Calibration debug data
        self.left_pca_heading = None
        self.left_pca_linearity = None
        self.left_pca_points = None

        self.right_pca_heading = None
        self.right_pca_linearity = None
        self.right_pca_points = None

        # ----------------------------------------------------
        # Mission
        # ----------------------------------------------------

        self.completed_corners = 0

        self.completed_laps = 0
        
        self.mission_complete = False

        # ----------------------------------------------------
        # Wall Geometry
        # ----------------------------------------------------
        
        self.wall_front = None

        self.wall_left = None

        self.wall_right = None

        self.track_width = None

        # Opening detection (used only before turn direction
        # is locked)

        self.left_far_points = 0

        self.right_far_points = 0

        self.left_far_ratio = 0.0

        self.right_far_ratio = 0.0

        # ----------------------------------------------------
        # Wall Following
        # ----------------------------------------------------

        self.distance_error = 0.0

        self.heading_error = 0.0

        self.steering = 0.0

        # ----------------------------------------------------
        # Turn Controller
        # ----------------------------------------------------

        self.turn_start_distance = (
            self.DEFAULT_TURN_DISTANCE
        )

        self.target_heading = 0.0

        # ----------------------------------------------------
        # Reverse Maneuvers
        # ----------------------------------------------------

        # Frozen wall_front target for REVERSE_FOR_CORNER
        # (set to turn_start_distance at the moment the turn
        # trigger fires).
        self.reverse_target_distance = 0.0

        # Start time for the reverse-maneuver safety timeout,
        # shared by REVERSE_FOR_CORNER and REVERSE_FOR_PILLAR.
        self.reverse_start_time = 0.0


        # ----------------------------------------------------
        # Vehicle State
        # ----------------------------------------------------

        # Raw IMU heading from Teensy
        self.imu_heading = 0.0

        # Heading offset applied after every completed turn
        self.heading_offset = 0.0

        # Robot heading in world frame
        self.rotation = 0.0

        self.actual_rpm = 0.0

        self.encoder_ticks = 0


        # ----------------------------------------------------
        # Timing
        # ----------------------------------------------------

        self.state_start_time = time.time()

        self.last_loop_time = time.time()
        
        # ----------------------------------------------------
        # Behaviour
        # ----------------------------------------------------

        self.behaviour = Behaviour.NORMAL
        
        self.previous_behaviour = Behaviour.NORMAL

        self.straight_start_ticks = 0

        self.desired_offset = 0.0
        
# ============================================================
#
# General Helpers
#
# ============================================================

    @staticmethod
    def normalize_angle(angle):
        """
        Wrap angle to [-180, 180).
        """
        return (angle + 180.0) % 360.0 - 180.0
    # --------------------------------------------------------

    def change_state(self, new_state):

        if self.state != new_state:

            print(
                f"[STATE] {self.state.name} → {new_state.name}"
            )

            self.state = new_state
            self.state_start_time = time.time()


    # --------------------------------------------------------

    def reset_behaviour(self):

        self.behaviour = Behaviour.NORMAL

        self.desired_offset = 0

        self.turn_start_distance = (
            self.DEFAULT_TURN_DISTANCE
        )

        self.straight_start_ticks = 0

        self.reverse_target_distance = 0.0

        self.reverse_start_time = 0.0


    # --------------------------------------------------------

    def reset_wall_geometry(self):

        self.wall_front = None

        self.wall_left = None

        self.wall_right = None

        self.track_width = None

        self.left_far_points = 0

        self.right_far_points = 0

        self.left_far_ratio = 0.0

        self.right_far_ratio = 0.0


    # --------------------------------------------------------

    def reset_turn_controller(self):

        self.target_heading = self.rotation


# ============================================================
#
# Robot Interface
#
# ============================================================

    def update_telemetry(self):

        """
        Read latest telemetry from Teensy.

        Returns
        -------
        bool
            True if a new packet was received.
        """

        telemetry = self.bridge.get_telemetry()

        if telemetry is None:
            return False

        self.imu_heading = telemetry["heading"]

        self.actual_rpm = telemetry["actual_rpm"]

        self.encoder_ticks = telemetry["total_ticks"]
        return True


    # --------------------------------------------------------
    
    def update_rotation(self):
        """
        Compute robot heading in the world frame.

        rotation =
            imu_heading - heading_offset
        """

        self.rotation = self.normalize_angle(
            self.imu_heading -
            self.heading_offset
        )

    # --------------------------------------------------------

    def zero_imu(self):

        print("[IMU] Zeroing...")

        self.bridge.zero_imu()

        timeout = time.time() + 2.0

        while time.time() < timeout:

            telemetry = self.bridge.get_telemetry()

            if telemetry is None:
                continue

            self.imu_heading = telemetry["heading"]
            if self.bridge.is_heading_settled():

                print("[IMU] Zero complete")

                return True

        print("[IMU] Zero timeout")

        return False


    # --------------------------------------------------------

    def send_drive_command(
            self,
            rpm,
            steering
    ):

        steering = max(
            -self.MAX_STEERING,
            min(self.MAX_STEERING, steering)
        )

        self.bridge.send_command(
            rpm,
            steering
        )


    # --------------------------------------------------------

    def stop_robot(self):

        self.send_drive_command(0, 0)


    # --------------------------------------------------------

    def drive_forward(self):

        self.send_drive_command(
            self.SEARCH_RPM,
            self.steering
        )


    # --------------------------------------------------------

    def check_stall(self):

        """
        Returns True if robot is commanded to move
        but encoder RPM stays close to zero.
        """

        if self.actual_rpm is None:
            return False

        if abs(self.actual_rpm) > 5:
            return False

        return True
# ============================================================
#
# Sensor Processing
#
# ============================================================

    def update_wall_geometry(self, angles_deg, distances_mm):
        """
        Update wall geometry from the current LiDAR scan.

        Parameters
        ----------
        angles_deg : ndarray
            LiDAR angles in degrees.

        distances_mm : ndarray
            LiDAR distances in millimetres.

        Returns
        -------
        bool
            True if extraction succeeded.
        """

        result = self.extractor.extract(
            angles_deg,
            distances_mm,
            self.rotation        # <- we'll add rotation in Step 4
        )

        if result is None:
            return False

        # ----------------------------------------------------
        # Store wall geometry
        # ----------------------------------------------------

        self.wall_front = result["wall_F"]

        self.wall_left = result["wall_L"]

        self.wall_right = result["wall_R"]

        self.track_width = result["track_width"]

        # Store opening information
        # (used only to determine initial turn direction)

        self.left_far_points = result["left"]["far_points"]
        self.left_far_ratio = result["left"]["far_ratio"]

        self.right_far_points = result["right"]["far_points"]
        self.right_far_ratio = result["right"]["far_ratio"]

        return True


    # --------------------------------------------------------

    def update_pillar(self):
        """
        Read the latest camera detection and update the
        pillar tracker.
        """

        detection = self.camera.get_detections()

        if detection is None:

            self.pillar_tracker.update(None)
            return

        self.pillar_tracker.update(
            detection["pillar"]
        )
# ============================================================
#
# Behaviour
#
# ============================================================

    def update_behaviour(self):
        """
        Update obstacle behaviour based on the currently
        tracked pillar.

        This function ONLY updates:

            • behaviour
            • desired_offset
            • turn_start_distance

        It never sends motor commands or changes state.

        Classification lock
        --------------------
        Once a pillar is classified as CORNER_* or STRAIGHT_*,
        that classification is frozen — it will NOT be
        re-derived or flip to a different type/colour on
        later cycles, even if the camera briefly misclassifies
        or loses the pillar. This prevents the turn-distance
        threshold (or offset) from being silently reset while
        still approaching/passing the same pillar.

        Only two things can end a lock:

            • CORNER_*   → the turn actually triggers
                            (handled entirely by
                            check_turn_trigger(), which uses
                            only LiDAR wall_front — camera
                            visibility is irrelevant here by
                            design, since it's expected/normal
                            to lose the pillar as the robot
                            gets close).

            • STRAIGHT_* → BOTH of the following are true:
                             - ticks travelled since first
                               detection exceed
                               STRAIGHT_PASS_DISTANCE_TICKS
                               (distance covered)
                             - the pillar tracker currently
                               reports None (out of sight)
        """

        pillar = self.pillar_tracker.get_active()

        # ----------------------------------------------------
        # Locked on a CORNER pillar — hold completely.
        # Resolved only by check_turn_trigger() firing.
        # ----------------------------------------------------

        if self.behaviour in (
            Behaviour.CORNER_RED,
            Behaviour.CORNER_GREEN
        ):
            return

        # ----------------------------------------------------
        # Locked on a STRAIGHT pillar — hold until passed.
        # ----------------------------------------------------

        if self.behaviour in (
            Behaviour.STRAIGHT_RED,
            Behaviour.STRAIGHT_GREEN
        ):

            travelled = (
                self.encoder_ticks
                - self.straight_start_ticks
            )

            distance_covered = (
                travelled > self.STRAIGHT_PASS_DISTANCE_TICKS
            )

            out_of_sight = (
                pillar is None
            )

            if distance_covered and out_of_sight:

                self.reset_behaviour()

            return

        # ----------------------------------------------------
        # NORMAL — free to classify a newly detected pillar
        # ----------------------------------------------------

        if pillar is None:
            return

        if self.wall_front is None:
            return

        # ----------------------------------------------------
        # Classify pillar
        # ----------------------------------------------------

        delta = self.wall_front - pillar["distance"]

        is_straight = (
            delta > self.STRAIGHT_THRESHOLD
        )

        if is_straight:

            self.straight_start_ticks = self.encoder_ticks

            self.turn_start_distance = (
                self.DEFAULT_TURN_DISTANCE
            )

            if pillar["color"] == "RED":

                self.behaviour = Behaviour.STRAIGHT_RED

                self.desired_offset = (
                    self.STRAIGHT_RED_OFFSET
                )

            else:

                self.behaviour = Behaviour.STRAIGHT_GREEN

                self.desired_offset = (
                    self.STRAIGHT_GREEN_OFFSET
                )

        # ----------------------------------------------------
        # Corner pillar
        # ----------------------------------------------------

        else:

            self.desired_offset = 0

            if pillar["color"] == "RED":

                self.behaviour = Behaviour.CORNER_RED

                self.turn_start_distance = (
                    self.CORNER_RED_TURN_DISTANCE
                )

            else:

                self.behaviour = Behaviour.CORNER_GREEN

                self.turn_start_distance = (
                    self.CORNER_GREEN_TURN_DISTANCE
                )
        # ----------------------------------------------------
        # Behaviour transition logging
        # ----------------------------------------------------

        if self.behaviour != self.previous_behaviour:

            print(
                f"[BEHAVIOUR] "
                f"{self.previous_behaviour.name}"
                f" -> "
                f"{self.behaviour.name}"
            )

            self.previous_behaviour = self.behaviour

# ============================================================
#
# Controllers
#
# ============================================================

    def compute_distance_error(self):
        """
        Compute lateral wall following error.

        desired_offset

            0      -> normal centre

            +300   -> shift left

            -300   -> shift right
        """

        # ----------------------------------------------------
        # Both walls visible
        # ----------------------------------------------------

        if (
            self.wall_left is not None and
            self.wall_right is not None
        ):

            self.distance_error = (

                (self.wall_right - self.wall_left)

                / 2.0

                -

                self.desired_offset
            )

            return

        # ----------------------------------------------------
        # Left wall only
        # ----------------------------------------------------

        if self.wall_left is not None:

            self.distance_error = (

                self.SINGLE_WALL_DISTANCE

                -

                self.wall_left

                +

                self.desired_offset
            )

            return

        # ----------------------------------------------------
        # Right wall only
        # ----------------------------------------------------

        if self.wall_right is not None:

            self.distance_error = (

                self.wall_right

                -

                self.SINGLE_WALL_DISTANCE

                -

                self.desired_offset
            )

            return

        self.distance_error = 0.0
        
        
    def compute_heading_error(self):
        """
        Heading controller.

        Uses world-frame rotation.
        """

        self.heading_error = self.rotation
        
        
    def compute_steering(self):
        """
        Combine distance and heading controllers.
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


    def compute_reverse_steering(self):
        """
        Heading-only steering correction while driving backward
        (no lateral/offset term — this keeps the robot parallel
        to the walls rather than swerving toward a pillar).

        NOTE: the sign is flipped relative to compute_steering().
        Yaw rate is proportional to velocity * tan(steering
        angle), so reversing the direction of travel reverses the
        sign of the heading correction needed for the same
        physical response. This hasn't been verified on hardware
        yet — flip the sign here if reverse-heading correction
        turns the robot the wrong way during testing.
        """

        self.compute_heading_error()

        steering = (
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

        return steering


    def follow_corridor(self):
        """
        Main wall following controller.
        """

        self.compute_distance_error()

        self.compute_heading_error()

        self.compute_steering()

        self.drive_forward()

    # --------------------------------------------------------
    # Determine first turn direction
    # --------------------------------------------------------

    def determine_turn_direction(self):

        # Already determined

        if self.direction_locked:
            return None

        # Need a front wall before we can look for an opening

        if self.wall_front is None:
            return None

        # ----------------------------------------------------
        # Right opening
        # ----------------------------------------------------

        if (

            self.right_far_points >= self.MIN_FAR_POINTS

            and

            self.right_far_ratio >= self.FAR_RATIO_THRESHOLD

        ):

            print("[TURN] Right opening detected")

            return TurnDirection.RIGHT

        # ----------------------------------------------------
        # Left opening
        # ----------------------------------------------------

        if (

            self.left_far_points >= self.MIN_FAR_POINTS

            and

            self.left_far_ratio >= self.FAR_RATIO_THRESHOLD

        ):

            print("[TURN] Left opening detected")

            return TurnDirection.LEFT

        # ----------------------------------------------------
        # No opening yet
        # ----------------------------------------------------

        return None

    # --------------------------------------------------------
    # Check whether a turn should begin
    # --------------------------------------------------------

    def check_turn_trigger(self):

        # ----------------------------------------------------
        # First determine the course direction
        # ----------------------------------------------------

        if not self.direction_locked:

            direction = self.determine_turn_direction()

            if direction is not None:

                self.turn_direction = direction

                self.direction_locked = True

                print(
                    f"[TURN] Direction locked : "
                    f"{direction.name}"
                )

            # Keep driving until the front wall
            # reaches the turn distance.

            return

        # ----------------------------------------------------
        # Wait until front wall reaches trigger distance
        # ----------------------------------------------------

        if self.wall_front is None:
            return

        if self.wall_front > self.turn_start_distance:
            return

        print(
            f"[TURN] Trigger distance reached "
            f"(Front = {self.wall_front:.0f} mm, "
            f"target = {self.turn_start_distance:.0f} mm) "
            f"-> reversing to target before turning"
        )

        self.reverse_target_distance = self.turn_start_distance

        self.reverse_start_time = time.time()

        self.change_state(
            NavState.REVERSE_FOR_CORNER
        )

    # --------------------------------------------------------
    # Begin a turn
    # --------------------------------------------------------

    def begin_turn(self):
        """
        Lock in the target heading for this turn and switch
        to the TURN state.
        """

        self.stop_robot()

        if self.turn_direction == TurnDirection.RIGHT:
            delta = self.TURN_ANGLE
        else:
            delta = -self.TURN_ANGLE

        self.target_heading = self.normalize_angle(
            delta - self.rotation
        )

        self.change_state(
            NavState.TURN
        )


    # --------------------------------------------------------
    # REVERSE_FOR_CORNER
    # --------------------------------------------------------

    def state_reverse_for_corner(
        self,
        angles_deg,
        distances_mm
    ):
        """
        Reverse (with heading correction) until the front wall
        distance climbs back up to reverse_target_distance — the
        turn_start_distance that was in effect when the turn
        trigger fired — then begin the turn.

        This handles the case where a corner pillar's colour
        classification raises turn_start_distance (e.g. to
        CORNER_RED_TURN_DISTANCE) after the robot has already
        driven past that point using the smaller default
        threshold.
        """

        if not self.update_wall_geometry(
            angles_deg,
            distances_mm
        ):
            return

        # ----------------------------------------------------
        # Target distance reached -> begin the turn
        # ----------------------------------------------------

        if (
            self.wall_front is not None
            and
            self.wall_front >= self.reverse_target_distance
        ):

            self.begin_turn()

            return

        # ----------------------------------------------------
        # Safety timeout
        # ----------------------------------------------------

        if (
            time.time() - self.reverse_start_time
            > self.REVERSE_TIMEOUT_SEC
        ):

            print(
                "WARNING: corner reverse timed out "
                f"(Front = {self.wall_front}, "
                f"target = {self.reverse_target_distance:.0f} mm)"
            )

            self.begin_turn()

            return

        # ----------------------------------------------------
        # Heading-corrected reverse
        # ----------------------------------------------------

        self.compute_reverse_steering()

        self.send_drive_command(
            self.REVERSE_RPM,
            self.steering
        )


    # --------------------------------------------------------
    # REVERSE_FOR_PILLAR
    # --------------------------------------------------------

    def state_reverse_for_pillar(self):
        """
        Reverse (with heading correction, no lateral offset)
        until the tracked straight pillar is at or beyond
        STRAIGHT_PILLAR_MIN_DISTANCE, then resume SEARCH so
        avoidance (desired_offset) picks back up immediately.

        If the pillar is lost mid-reverse, treat it as "far
        enough" and resume immediately.
        """

        self.update_pillar()

        pillar = self.pillar_tracker.get_active()

        # ----------------------------------------------------
        # Pillar lost -> treat as far enough
        # ----------------------------------------------------

        if pillar is None:

            self.change_state(
                NavState.SEARCH
            )

            return

        # ----------------------------------------------------
        # Target distance reached -> resume forward
        # ----------------------------------------------------

        if (
            pillar["distance"]
            >=
            self.STRAIGHT_PILLAR_MIN_DISTANCE
        ):

            self.change_state(
                NavState.SEARCH
            )

            return

        # ----------------------------------------------------
        # Safety timeout
        # ----------------------------------------------------

        if (
            time.time() - self.reverse_start_time
            > self.REVERSE_TIMEOUT_SEC
        ):

            print(
                "WARNING: pillar reverse timed out "
                f"(pillar dist = {pillar['distance']:.0f} mm, "
                f"target = "
                f"{self.STRAIGHT_PILLAR_MIN_DISTANCE} mm)"
            )

            self.change_state(
                NavState.SEARCH
            )

            return

        # ----------------------------------------------------
        # Heading-corrected reverse
        # ----------------------------------------------------

        self.compute_reverse_steering()

        self.send_drive_command(
            self.REVERSE_RPM,
            self.steering
        )


# ============================================================
#
# Navigation State Machine
#
# ============================================================

    def run(self, angles_deg, distances_mm):
        """
        Main navigator entry point.

        Called once every control loop.
        """

        # ----------------------------------------------------
        # Update robot telemetry
        # ----------------------------------------------------

        if not self.update_telemetry():
            return

        # ----------------------------------------------------
        # Update world-frame heading
        # ----------------------------------------------------

        self.update_rotation()

        # ----------------------------------------------------
        # Execute current state
        # ----------------------------------------------------

        if self.state == NavState.INITIALIZE:

            self.state_initialize(
                angles_deg,
                distances_mm
            )

        elif self.state == NavState.EXIT_PARKING:

            self.state_exit_parking(
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

        elif self.state == NavState.PARK:

            self.state_park()

        elif self.state == NavState.REVERSE_FOR_CORNER:

            self.state_reverse_for_corner(
                angles_deg,
                distances_mm
            )

        elif self.state == NavState.REVERSE_FOR_PILLAR:

            self.state_reverse_for_pillar()

            
    # --------------------------------------------------------

    def state_initialize(
        self,
        angles_deg,
        distances_mm
    ):
        """
        Initialise robot before starting mission.
        """

        print("[INIT] Starting...")

        self.stop_robot()

        if not self.zero_imu():
            return

        self.reset_behaviour()

        self.reset_wall_geometry()

        self.reset_turn_controller()

        self.direction_locked = False

        self.completed_corners = 0

        self.completed_laps = 0
        
        self.heading_offset = 0.0

        self.rotation = 0.0

        self.initial_calibration_done = False

        self.calibration_attempts = 0

        self.mission_complete = False

        self.turn_direction = None

        print("[INIT] Complete")

        self.change_state(
            NavState.EXIT_PARKING
        )
        
        
    def state_exit_parking(self, angles_deg, distances_mm):

        if not self.update_wall_geometry(
            angles_deg,
            distances_mm
        ):
            return

        self.follow_corridor()

        if self.wall_front is not None and \
           self.wall_front > self.PARKING_SEARCH_DISTANCE:

            self.change_state(
                NavState.CALIBRATE
            )
                
    # --------------------------------------------------------

    def state_search(
        self,
        angles_deg,
        distances_mm
    ):
        """
        Main navigation state.

        Responsibilities
        ----------------
        • Update wall geometry
        • Update pillar tracker
        • Update behaviour
        • Follow corridor
        • Trigger turns
        """

        # ----------------------------------------------------
        # Update wall geometry
        # ----------------------------------------------------

        if not self.update_wall_geometry(
            angles_deg,
            distances_mm
        ):
            return

        # ----------------------------------------------------
        # Mission complete → drive to the finish line, then park
        # ----------------------------------------------------

        if self.mission_complete:

            # Ignore the first SEARCH cycle after calibration
            # (stale/settling LiDAR geometry)

            if self.first_search_after_calibration:

                self.first_search_after_calibration = False

                return

            if (
                self.wall_front is not None
                and
                self.wall_front > self.FINISH_DISTANCE
            ):

                self.send_drive_command(
                    self.SEARCH_RPM,
                    0
                )

            else:

                self.stop_robot()

                self.change_state(
                    NavState.PARK
                )

            return

        # ----------------------------------------------------
        # Update camera tracker
        # ----------------------------------------------------

        self.update_pillar()

        # ----------------------------------------------------
        # Determine obstacle behaviour
        # ----------------------------------------------------

        self.update_behaviour()

        # ----------------------------------------------------
        # Straight pillar too close → reverse before avoiding
        # ----------------------------------------------------

        if self.behaviour in (
            Behaviour.STRAIGHT_RED,
            Behaviour.STRAIGHT_GREEN
        ):

            pillar = self.pillar_tracker.get_active()

            if (
                pillar is not None
                and
                pillar["distance"]
                <
                self.STRAIGHT_PILLAR_MIN_DISTANCE
            ):

                print(
                    f"[REVERSE] Straight pillar too close "
                    f"({pillar['distance']:.0f} mm < "
                    f"{self.STRAIGHT_PILLAR_MIN_DISTANCE} mm) "
                    f"-> reversing"
                )

                self.reverse_start_time = time.time()

                self.change_state(
                    NavState.REVERSE_FOR_PILLAR
                )

                return

        # ----------------------------------------------------
        # Wall following controller
        # ----------------------------------------------------

        self.follow_corridor()

        # ----------------------------------------------------
        # Turn trigger
        # ----------------------------------------------------

        self.check_turn_trigger()
        
        if self.state != NavState.SEARCH:
            return
    # --------------------------------------------------------
    # TURN
    # --------------------------------------------------------

    def state_turn(self):
        """
        Execute a 90° turn using proportional control.

        Responsibilities
        ----------------
        • Rotate to target heading
        • Zero IMU on completion
        • Count completed corners
        • Transition to CALIBRATE

        Does NOT:
        • Reset behaviours
        """

        # ----------------------------------------------------
        # Compute heading error
        # ----------------------------------------------------

        error = self.normalize_angle(
            self.target_heading -
            self.rotation
        )

        # ----------------------------------------------------
        # Turn completed?
        # ----------------------------------------------------

        if abs(error) <= self.TURN_HEADING_TOLERANCE:

            self.stop_robot()

            self.zero_imu()

            # Count completed corner
            self.completed_corners += 1

            # Mission complete?
            if self.completed_corners >= self.TOTAL_CORNERS:

                self.mission_complete = True

            # Reset calibration attempts
            self.calibration_attempts = 0

            # Continue to calibration
            self.change_state(
                NavState.CALIBRATE
            )

            return

        # ----------------------------------------------------
        # Proportional steering
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Clamp steering
        # ----------------------------------------------------

        steering = max(

            -self.TURN_MAX_STEERING,

            min(

                self.TURN_MAX_STEERING,

                steering

            )

        )

        # ----------------------------------------------------
        # Reduce speed near target
        # ----------------------------------------------------

        if abs(error) < 20:

            turn_rpm = self.TURN_RPM * 0.9

        else:

            turn_rpm = self.TURN_RPM

        # ----------------------------------------------------
        # Execute turn
        # ----------------------------------------------------

        self.send_drive_command(
            turn_rpm,
            steering
        )
        
        
    # --------------------------------------------------------
    # CALIBRATE
    # --------------------------------------------------------

    def state_calibrate(
        self,
        angles_deg,
        distances_mm
    ):
        """
        Calibrate robot heading using LiDAR PCA.

        Initial calibration:
            Uses both walls.

        After a turn:
            Uses the outer wall only.
        """

        # ----------------------------------------------------
        # Wait until IMU has settled
        # ----------------------------------------------------

        if not self.bridge.is_heading_settled():

            self.send_drive_command(0, 0)

            return

        # ----------------------------------------------------
        # Determine calibration mode
        # ----------------------------------------------------

        if not self.initial_calibration_done:

            outer_wall = None

        else:

            if self.turn_direction == TurnDirection.RIGHT:

                outer_wall = "LEFT"

            else:

                outer_wall = "RIGHT"

        # ----------------------------------------------------
        # Perform PCA calibration
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Calibration successful
        # ----------------------------------------------------

        if (

            result["heading"] is not None

            and

            result["confidence"]
            >=
            self.CONFIDENCE_THRESHOLD

        ):

            self.heading_offset = (

                result["heading"]

                -

                self.imu_heading

            )

            self.update_rotation()

            self.initial_calibration_done = True

            self.calibration_attempts = 0

            self.first_search_after_calibration = True

            self.reset_behaviour()

            self.reset_turn_controller()

            self.reset_wall_geometry()

            self.pillar_tracker.clear()

            self.change_state(
                NavState.SEARCH
            )

            return

        # ----------------------------------------------------
        # Retry calibration
        # ----------------------------------------------------

        self.send_drive_command(
            self.SEARCH_RPM,
            0
        )

        if self.calibration_attempts == 0:

            self.calibration_start_time = time.time()

        self.calibration_attempts += 1

        if (

            time.time()

            -

            self.calibration_start_time

            >

            3.0

        ):

            print(

                "WARNING: Calibration timeout."

            )

            self.update_rotation()

            self.initial_calibration_done = True

            self.calibration_attempts = 0

            self.first_search_after_calibration = True

            self.reset_behaviour()

            self.reset_turn_controller()

            self.reset_wall_geometry()

            self.pillar_tracker.clear()

            self.change_state(
                NavState.SEARCH
            )


    # --------------------------------------------------------
    # PARK
    # --------------------------------------------------------

    def state_park(self):
        """
        Mission complete.

        Placeholder final state — holds the robot stopped once
        the finish line has been reached. Extend this with a
        magenta-marker parking maneuver (see CameraDetector's
        'parking' output) when that behaviour is defined.
        """

        self.stop_robot()
