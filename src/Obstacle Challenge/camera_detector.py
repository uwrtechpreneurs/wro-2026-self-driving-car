# camera_detector.py
# Step 10: Camera color detection for WRO FEC 2026
# Detects RED pillars, GREEN pillars, MAGENTA parking markers
# Returns structured detection data for use by navigation layer

from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import time


class CameraDetector:
    """
    Runs camera detection in background thread.
    Navigation code calls get_detections() to get latest result.

    Detection output per frame:
        {
          'pillar': {
              'color':    'RED' | 'GREEN' | None
              'screen_x': pixel x of pillar center (0=left, 320=right)
              'screen_y': pixel y of pillar center
              'distance': estimated distance in mm
              'area':     contour area in pixels
          } or None,

          'parking': {
              'left_marker':  {'screen_x', 'screen_y'} or None
              'right_marker': {'screen_x', 'screen_y'} or None
              'visible':      True if both markers seen
          },

          'frame_debug': annotated BGR frame for display (optional)
        }
    """

    # ── Camera settings ───────────────────────────────────────────────────────
    FRAME_W = 640
    FRAME_H = 480
    FPS     = 30

    # ── ROI: ignore top portion of frame (ceiling, far background) ────────────
    # 0.0 = use full frame, 0.3 = ignore top 30%
    ROI_TOP_FRACTION = 0.30

    # ── HSV color ranges ──────────────────────────────────────────────────────
    # Calibrate these on your actual competition field lighting
    # These are good starting values

    RED_LOWER_1  = np.array([0,   70,  80])
    RED_UPPER_1  = np.array([10, 255, 255])
    RED_LOWER_2  = np.array([170, 70,  80])
    RED_UPPER_2  = np.array([180, 255, 255])

    GREEN_LOWER  = np.array([40,  50,  50])
    GREEN_UPPER  = np.array([95, 255, 255])

    # Magenta parking markers: RGB(255,0,255)
    # In HSV: hue ~150°, high saturation, high value
    MAGENTA_LOWER = np.array([138, 100,  80])
    MAGENTA_UPPER = np.array([168, 255, 255])

    # ── Detection thresholds ──────────────────────────────────────────────────
    MIN_PILLAR_AREA    = 400    # pixels² — ignore tiny blobs
    MIN_PARKING_AREA   = 200
    ASPECT_MIN         = 0.3    # pillar is taller than wide
    ASPECT_MAX         = 2.0

    # ── Distance estimation ───────────────────────────────────────────────────
    # Pillar real height = 100mm (from rules)
    # Focal length: calibrate by placing pillar at known distance
    # and measuring its pixel height
    # focal_length = (pixel_height * real_distance) / real_height
    PILLAR_REAL_HEIGHT_MM = 100
    FOCAL_LENGTH_PX       = 615   # recalibrate for your camera + mount height

    def __init__(self):
        self._latest     = None
        self._lock       = threading.Lock()
        self._running    = False
        self._frame_count = 0

        # Smoothing
        self._prev_height = 0
        self._smooth_alpha = 0.7

        # Morphology kernel
        self._kernel = np.ones((3, 3), np.uint8)

    def start(self):
        """Initialize camera and start background detection thread."""

        try:

            self._picam2 = Picamera2()

            config = self._picam2.create_video_configuration(
                main={
                    "size": (self.FRAME_W, self.FRAME_H),
                    "format": "RGB888"
                },
                controls={
                    "FrameRate": self.FPS
                }
            )

            self._picam2.configure(config)

            self._picam2.start()

            time.sleep(1.5)

            self._picam2.set_controls({
                "AeEnable": True,
                "AwbEnable": True,
            })

            self._running = True

            self._thread = threading.Thread(
                target=self._detection_loop,
                daemon=False
            )

            self._thread.start()

            timeout = time.time() + 5

            while self._latest is None:

                if time.time() > timeout:

                    print("WARNING: Camera detection slow to start")

                    break

                time.sleep(0.05)

            print("Camera detector ready.")

        except Exception:

            if self._picam2 is not None:

                try:
                    self._picam2.close()
                except Exception:
                    pass

                self._picam2 = None

            raise

    def stop(self):
        """
        Stop detector thread and release the camera.
        """

        # ------------------------------------------
        # Stop detection thread
        # ------------------------------------------

        self._running = False

        if self._thread is not None:

            self._thread.join(timeout=2.0)

            self._thread = None

        # ------------------------------------------
        # Stop camera
        # ------------------------------------------

        if self._picam2 is not None:

            try:
                self._picam2.stop()

            except Exception as e:

                print(f"[CAMERA] stop() failed: {e}")

            try:
                self._picam2.close()

            except Exception as e:

                print(f"[CAMERA] close() failed: {e}")

            self._picam2 = None

        self._latest = None

        print("Camera detector stopped.")

    def get_detections(self):
        """
        Get latest detection result.
        Safe to call from any thread.
        Returns dict or None if no frame yet.
        """
        with self._lock:
            return self._latest

    def get_frame_count(self):
        return self._frame_count

    # ── Background detection loop ─────────────────────────────────────────────

    def _detection_loop(self):
        while self._running:
            # Capture frame (RGB from picamera2)
            frame_rgb = self._picam2.capture_array()

            # Rotate 180° if camera is mounted upside down
            # Comment out if not needed for your mount
            frame = cv2.rotate(frame_rgb, cv2.ROTATE_180)

            # Run detection
            result = self._process_frame(frame)
            result['frame_debug'] = frame  # pass raw frame for display

            with self._lock:
                self._latest = result
                self._frame_count += 1

    def _process_frame(self, frame):
        """
        Core detection pipeline.
        Returns structured detection dict.
        """
        h, w = frame.shape[:2]

        # ── ROI: crop top portion ─────────────────────────────────────────────
        roi_top = int(h * self.ROI_TOP_FRACTION)
        roi     = frame[roi_top:h, :]

        # ── Convert to HSV ────────────────────────────────────────────────────
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ── Color masks ───────────────────────────────────────────────────────
        red_mask = (
            cv2.inRange(hsv, self.RED_LOWER_1, self.RED_UPPER_1) |
            cv2.inRange(hsv, self.RED_LOWER_2, self.RED_UPPER_2)
        )
        green_mask   = cv2.inRange(hsv, self.GREEN_LOWER,   self.GREEN_UPPER)
        magenta_mask = cv2.inRange(hsv, self.MAGENTA_LOWER, self.MAGENTA_UPPER)

        # ── Morphology: remove noise ──────────────────────────────────────────
        red_mask     = cv2.morphologyEx(
            red_mask,     cv2.MORPH_OPEN, self._kernel)
        green_mask   = cv2.morphologyEx(
            green_mask,   cv2.MORPH_OPEN, self._kernel)
        magenta_mask = cv2.morphologyEx(
            magenta_mask, cv2.MORPH_OPEN, self._kernel)

        # ── Detect pillars ────────────────────────────────────────────────────
        pillar = self._detect_pillar(
            red_mask, green_mask, roi_top)

        # ── Detect parking markers ────────────────────────────────────────────
        parking = self._detect_parking(
            magenta_mask, roi_top)

        return {
            'pillar':  pillar,
            'parking': parking,
        }

    def _detect_pillar(self, red_mask, green_mask, roi_top):
        """
        Find the most prominent pillar in frame.
        Returns dict with color, position, distance — or None.
        """
        best        = None
        best_height = 0

        for mask, color in [(red_mask,   'RED'),
                             (green_mask, 'GREEN')]:

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.MIN_PILLAR_AREA:
                    continue

                x, y, bw, bh = cv2.boundingRect(cnt)
                aspect = bw / max(bh, 1)

                if not (self.ASPECT_MIN < aspect < self.ASPECT_MAX):
                    continue

                # Tallest contour = closest pillar
                if bh > best_height:
                    best_height = bh
                    # Convert y back to full frame coords
                    cx = x + bw // 2
                    cy = y + bh // 2 + roi_top
                    best = {
                        'color':    color,
                        'screen_x': cx,
                        'screen_y': cy,
                        'bbox':     (x, y + roi_top, bw, bh),
                        'area':     area,
                        'height_px': bh,
                    }

        if best is None:
            self._prev_height = 0
            return None

        # ── Spike rejection ───────────────────────────────────────────────────
        h = best['height_px']
        if self._prev_height > 0 and abs(h - self._prev_height) > 40:
            h = self._prev_height  # reject spike

        # ── Smooth height ─────────────────────────────────────────────────────
        if self._prev_height == 0:
            smooth_h = h
        else:
            smooth_h = int(
                self._smooth_alpha * self._prev_height +
                (1 - self._smooth_alpha) * h)
        self._prev_height = smooth_h

        # ── Distance from pinhole model ───────────────────────────────────────
        # dist_mm = (real_height_mm * focal_length_px) / pixel_height
        distance_mm = int(
            (self.PILLAR_REAL_HEIGHT_MM * self.FOCAL_LENGTH_PX) /
            max(smooth_h, 1))

        best['distance']  = distance_mm
        best['height_px'] = smooth_h

        return best

    def _detect_parking(self, magenta_mask, roi_top):
        """
        Detect magenta parking lot markers.
        Returns dict with left/right marker positions.
        """
        contours, _ = cv2.findContours(
            magenta_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        markers = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_PARKING_AREA:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            cx = x + bw // 2
            cy = y + bh // 2 + roi_top
            markers.append({
                'screen_x': cx,
                'screen_y': cy,
                'area':     area,
            })

        # Sort left to right by screen_x
        markers.sort(key=lambda m: m['screen_x'])

        result = {
            'left_marker':  None,
            'right_marker': None,
            'visible':      False,
        }

        if len(markers) >= 2:
            result['left_marker']  = markers[0]
            result['right_marker'] = markers[-1]
            result['visible']      = True
        elif len(markers) == 1:
            # Single marker visible
            if markers[0]['screen_x'] < self.FRAME_W // 2:
                result['left_marker'] = markers[0]
            else:
                result['right_marker'] = markers[0]

        return result
