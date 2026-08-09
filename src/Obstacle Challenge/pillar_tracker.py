# pillar_tracker.py
#
# Stabilises camera detections and maintains one active pillar.
# It does NOT make navigation decisions.
#
# Input:
#     CameraDetector output (pillar dict or None)
#
# Output:
#     Stable pillar dict or None


from collections import deque


class PillarTracker:

    # -----------------------------
    # Parameters
    # -----------------------------

    MAX_MISSED_FRAMES = 5

    DISTANCE_HISTORY = 5
    X_HISTORY = 5

    MAX_DISTANCE_JUMP = 250      # mm
    MAX_X_JUMP = 120             # pixels

    def __init__(self):

        self._pillar = None

        self._missed_frames = 0

        self._distance_history = deque(
            maxlen=self.DISTANCE_HISTORY
        )

        self._x_history = deque(
            maxlen=self.X_HISTORY
        )

    # ----------------------------------------------------------
    # Update tracker
    # ----------------------------------------------------------

    def update(self, detection):

        # ------------------------------------------------------
        # Nothing detected
        # ------------------------------------------------------

        if detection is None:

            if self._pillar is not None:

                self._missed_frames += 1

                if self._missed_frames > self.MAX_MISSED_FRAMES:

                    self.clear()

            return self._pillar

        # ------------------------------------------------------
        # First detection
        # ------------------------------------------------------

        if self._pillar is None:

            self._pillar = detection.copy()

            self._pillar["visible"] = True

            self._distance_history.clear()
            self._x_history.clear()

            self._distance_history.append(
                detection["distance"]
            )

            self._x_history.append(
                detection["screen_x"]
            )

            self._missed_frames = 0

            return self._pillar

        # ------------------------------------------------------
        # Reject colour flicker
        # ------------------------------------------------------

        if detection["color"] != self._pillar["color"]:

            return self._pillar

        # ------------------------------------------------------
        # Reject impossible jumps
        # ------------------------------------------------------

        if abs(
            detection["distance"] -
            self._pillar["distance"]
        ) > self.MAX_DISTANCE_JUMP:

            detection["distance"] = self._pillar["distance"]

        if abs(
            detection["screen_x"] -
            self._pillar["screen_x"]
        ) > self.MAX_X_JUMP:

            detection["screen_x"] = self._pillar["screen_x"]

        # ------------------------------------------------------
        # Moving average
        # ------------------------------------------------------

        self._distance_history.append(
            detection["distance"]
        )

        self._x_history.append(
            detection["screen_x"]
        )

        self._pillar["distance"] = int(
            sum(self._distance_history) /
            len(self._distance_history)
        )

        self._pillar["screen_x"] = int(
            sum(self._x_history) /
            len(self._x_history)
        )

        # Copy remaining values directly
        self._pillar["screen_y"] = detection["screen_y"]
        self._pillar["bbox"] = detection["bbox"]
        self._pillar["area"] = detection["area"]
        self._pillar["height_px"] = detection["height_px"]

        self._pillar["visible"] = True

        self._missed_frames = 0

        return self._pillar

    # ----------------------------------------------------------
    # Return active pillar
    # ----------------------------------------------------------

    def get_active(self):

        return self._pillar

    # ----------------------------------------------------------
    # Forget pillar
    # ----------------------------------------------------------

    def clear(self):

        self._pillar = None

        self._missed_frames = 0

        self._distance_history.clear()
        self._x_history.clear()
