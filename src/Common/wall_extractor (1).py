#!/usr/bin/env python3
"""
wall_extractor.py

World-Frame Wall Extractor
==========================

This extractor keeps all LiDAR sectors fixed with respect to
the corridor instead of the robot.

Rotation Compensation

rotation = imu_heading + corridor_heading

where

imu_heading
    Relative IMU heading from Teensy

corridor_heading
    Initial LiDAR heading measured when entering
    a new corridor.

The LiDAR points are NEVER rotated.

Instead, the sector boundaries are rotated.

Author : Jithu Joseph
"""

import numpy as np


class WallExtractor:

    # ==========================================================
    # Configuration
    # ==========================================================

    # Ignore very close readings

    MIN_DISTANCE = 50.0

    # Ignore anything beyond this

    MAX_DISTANCE = 3000.0

    # ----------------------------------------------------------
    # Main Sector Widths
    # ----------------------------------------------------------

    FRONT_HALF = 35

    RIGHT_HALF = 35

    REAR_HALF = 35

    LEFT_HALF = 35

    # ----------------------------------------------------------
    # Sector Centres
    #
    # Robot Frame
    #
    #           Front
    #             0°
    #
    #   Left             Right
    #   270°              90°
    #
    #           Rear
    #          180°
    # ----------------------------------------------------------

    FRONT_CENTER = 0

    RIGHT_CENTER = 90

    REAR_CENTER = 180

    LEFT_CENTER = 270

    # ==========================================================
    # Angle Helper
    # ==========================================================

    @staticmethod
    def _wrap_angle(angle):

        """
        Wrap angle into [0,360)
        """

        return angle % 360.0


    # ==========================================================
    # Sector Mask
    # ==========================================================

    def _sector_mask(self,
                     angles_deg,
                     center,
                     half_width):

        """
        Returns boolean mask for a sector.

        Supports wrap-around at 0°.
        """

        start = self._wrap_angle(
            center - half_width
        )

        end = self._wrap_angle(
            center + half_width
        )

        if start <= end:

            return (
                (angles_deg >= start)
                &
                (angles_deg <= end)
            )

        return (
            (angles_deg >= start)
            |
            (angles_deg <= end)
        )

    # ==========================================================
    # Clean Scan
    # ==========================================================

    def _clean_scan(self,
                    angles_deg,
                    distances_mm):

        """
        Remove

        • NaN
        • Inf
        • Very small values
        • Extremely large values
        """

        valid = (

            np.isfinite(distances_mm)

            &

            (distances_mm >= self.MIN_DISTANCE)

            &

            (distances_mm <= self.MAX_DISTANCE)

        )

        return (

            angles_deg[valid],

            distances_mm[valid]

        )

    # ==========================================================
    # Generate Rotated Sectors
    # ==========================================================

    def _build_sectors(self,
                          rotation):

        """
        Creates all four corridor-fixed sectors.

        rotation

        = imu_heading

        +

        corridor_heading
        """

        sectors = {

            "front": (
                (self.FRONT_CENTER - rotation) % 360,
                self.FRONT_HALF
            ),

            "right": (
                (self.RIGHT_CENTER - rotation) % 360,
                self.RIGHT_HALF
            ),

            "rear": (
                (self.REAR_CENTER - rotation) % 360,
                self.REAR_HALF
            ),

            "left": (
                (self.LEFT_CENTER - rotation) % 360,
                self.LEFT_HALF
            ),
        }

        return sectors


# ==========================================================
# Extract Sector
# ==========================================================

    def _get_sector(self,
                    angles_deg,
                    distances_mm,
                    sector):

        center, half = sector

        mask = self._sector_mask(
            angles_deg,
            center,
            half
        )

        return (
            angles_deg[mask],
            distances_mm[mask],
            mask
        )
    # ==========================================================
    # Wall Distance
    # ==========================================================

    SIDE_WALL_MAX_DISTANCE = 1000.0

    OPEN_DISTANCE = 2000.0

    MAD_THRESHOLD = 3.0


    def _wall_distance(
            self,
            distances_mm,
            side_wall=True):
        """
        Estimate wall distance.

        Side walls:
            Median (stable for wall following)

        Front / Rear:
            90th percentile (better estimate of open space ahead)
        """

        if len(distances_mm) == 0:
            return None

        # --------------------------------------------------
        # Side walls
        # --------------------------------------------------

        if side_wall:

            wall = float(np.median(distances_mm))

            if wall > self.SIDE_WALL_MAX_DISTANCE:
                return None

            return wall

        # --------------------------------------------------
        # Front / Rear walls
        # --------------------------------------------------

        return float(np.percentile(distances_mm, 90))


    # ==========================================================
    # Median Absolute Deviation Filter
    # ==========================================================

    def _mad_filter(self,
                    angles_deg,
                    distances_mm):
        """
        Removes statistical outliers using the
        Median Absolute Deviation (MAD).

        Returns

        filtered_angles

        filtered_distances
        """

        if len(distances_mm) < 5:

            return angles_deg, distances_mm

        median = np.median(distances_mm)

        deviation = np.abs(
            distances_mm - median
        )

        mad = np.median(deviation)

        if mad < 1e-6:

            return angles_deg, distances_mm

        threshold = self.MAD_THRESHOLD * mad

        mask = deviation <= threshold

        return (

            angles_deg[mask],

            distances_mm[mask]

        )


    # ==========================================================
    # PCA Wall Heading
    # ==========================================================

    def _pca_heading(self,
                     angles_deg,
                     distances_mm,
                     wall_distance):
        """
        Computes wall heading using PCA.

        Returns

        heading (degrees)

        None if heading cannot be estimated.
        """

        if wall_distance is None:

            return None

        if len(distances_mm) < 5:

            return None

        angles_deg, distances_mm = self._mad_filter(

            angles_deg,

            distances_mm

        )

        if len(distances_mm) < 5:

            return None

        theta = np.radians(
            angles_deg
        )

        x = distances_mm * np.cos(theta)

        y = distances_mm * np.sin(theta)

        points = np.column_stack((x, y))

        centroid = np.mean(
            points,
            axis=0
        )

        centered = points - centroid

        covariance = np.cov(
            centered.T
        )

        eigenvalues, eigenvectors = np.linalg.eigh(
            covariance
        )

        direction = eigenvectors[
            :,
            np.argmax(eigenvalues)
        ]

        heading = np.degrees(

            np.arctan2(

                direction[1],

                direction[0]

            )

        )

        heading = (
            heading + 180.0
        ) % 180.0

        if heading > 90.0:

            heading -= 180.0

        # --------------------------------------------------
        # PCA Linearity
        # --------------------------------------------------

        linearity = float(

            eigenvalues[1] /

            (eigenvalues[0] + eigenvalues[1])

        )

        return {

            "heading": float(heading),

            "linearity": linearity,

            "valid_points": len(distances_mm)

        }
# ==========================================================
# Sector Analysis
# ==========================================================

    def _analyse_sector(self,
                        distances_mm):
        """
        Analyse a sector for corner detection.

        Returns
        -------
        {
            valid_points,
            far_points,
            far_ratio
        }
        """

        valid_points = len(distances_mm)

        if valid_points == 0:

            return {

                "valid_points": 0,

                "far_points": 0,

                "far_ratio": 0.0

            }

        far_points = int(

            np.sum(

                distances_mm >

                self.OPEN_DISTANCE

            )

        )

        far_ratio = (

            far_points /

            valid_points

        )

        return {

            "valid_points": valid_points,

            "far_points": far_points,

            "far_ratio": far_ratio

        }


# ==========================================================
# Runtime Wall Extraction
# ==========================================================

    def extract(self,
                angles_deg,
                distances_mm,
                rotation=0.0):
        """
        Runtime perception pipeline.

        Parameters
        ----------
        angles_deg
            Raw LiDAR angles

        distances_mm
            Raw LiDAR distances

        rotation

            = imu_heading

            +

            corridor_heading

        Returns
        -------

        Dictionary containing

            wall distances

            sector analysis

            track width
        """

        # --------------------------------------------------
        # Clean Scan
        # --------------------------------------------------

        angles_deg, distances_mm = self._clean_scan(

            angles_deg,

            distances_mm

        )

        # --------------------------------------------------
        # Build Corridor-Fixed Sectors
        # --------------------------------------------------

        sectors = self._build_sectors(

            rotation

        )

        # --------------------------------------------------
        # Extract Sector Data
        # --------------------------------------------------

        front_angles, front_distances, _ = self._get_sector(

            angles_deg,

            distances_mm,

            sectors["front"]

        )

        right_angles, right_distances, _ = self._get_sector(

            angles_deg,

            distances_mm,

            sectors["right"]

        )

        rear_angles, rear_distances, _ = self._get_sector(

            angles_deg,

            distances_mm,

            sectors["rear"]

        )

        left_angles, left_distances, _ = self._get_sector(

            angles_deg,

            distances_mm,

            sectors["left"]

        )

        # --------------------------------------------------
        # Compute Wall Distances
        # --------------------------------------------------

        wall_F = self._wall_distance(

            front_distances,

            side_wall=False

        )

        wall_R = self._wall_distance(

            right_distances,

            side_wall=True

        )

        wall_L = self._wall_distance(

            left_distances,

            side_wall=True

        )

        # --------------------------------------------------
        # Analyse Sectors
        # --------------------------------------------------

        front_sector = self._analyse_sector(

            front_distances

        )

        right_sector = self._analyse_sector(

            right_distances

        )

        rear_sector = self._analyse_sector(

            rear_distances

        )

        left_sector = self._analyse_sector(

            left_distances

        )

        # --------------------------------------------------
        # Track Width
        # --------------------------------------------------

        track_width = None

        if (

            wall_L is not None

            and

            wall_R is not None

        ):

            track_width = (

                wall_L

                +

                wall_R

            )

        # --------------------------------------------------
        # Package Result
        # --------------------------------------------------

        result = {

            "wall_F": wall_F,

            "wall_R": wall_R,

            "wall_L": wall_L,

            "track_width": track_width,

            "front": front_sector,

            "right": right_sector,

            "rear": rear_sector,

            "left": left_sector

        }

        return result
# ==========================================================
# Corridor Heading Calibration
# ==========================================================

    def calibrate_corridor_heading(
            self,
            angles_deg,
            distances_mm,
            rotation=0.0,
            outer_wall=None):
        """
        Estimate corridor heading and confidence.

        Returns

        {
            heading

            confidence
        }
        """

        # --------------------------------------------------
        # Clean Scan
        # --------------------------------------------------

        angles_deg, distances_mm = self._clean_scan(

            angles_deg,

            distances_mm

        )

        # --------------------------------------------------
        # Build World Frame Sectors
        # --------------------------------------------------

        sectors = self._build_sectors(

            rotation

        )

        # --------------------------------------------------
        # Left Sector
        # --------------------------------------------------

        left_angles, left_distances, _ = self._get_sector(

            angles_deg,

            distances_mm,

            sectors["left"]

        )

        # --------------------------------------------------
        # Right Sector
        # --------------------------------------------------

        right_angles, right_distances, _ = self._get_sector(

            angles_deg,

            distances_mm,

            sectors["right"]

        )

        # --------------------------------------------------
        # Wall Distance
        # --------------------------------------------------

        left_wall = self._wall_distance(

            left_distances,

            side_wall=True

        )

        right_wall = self._wall_distance(

            right_distances,

            side_wall=True

        )

        # --------------------------------------------------
        # PCA
        # --------------------------------------------------

        left_pca = self._pca_heading(

            left_angles,

            left_distances,

            left_wall

        )

        right_pca = self._pca_heading(

            right_angles,

            right_distances,

            right_wall

        )

        # --------------------------------------------------
        # No valid wall
        # --------------------------------------------------

        if (

            left_pca is None

            and

            right_pca is None

        ):

            return {

                "heading": None,

                "confidence": 0.0

            }

        # --------------------------------------------------
        # Heading
        # --------------------------------------------------

        # --------------------------------------------------
        # Startup calibration
        # Uses both walls
        # --------------------------------------------------

        if outer_wall is None:

            headings = []

            if left_pca is not None:
                headings.append(left_pca["heading"])

            if right_pca is not None:
                headings.append(right_pca["heading"])

            heading = float(np.mean(headings))

            point_scores = []

            if left_pca is not None:
                point_scores.append(
                    min(left_pca["valid_points"] / 40.0, 1.0)
                )

            if right_pca is not None:
                point_scores.append(
                    min(right_pca["valid_points"] / 40.0, 1.0)
                )

            point_score = float(np.mean(point_scores))

            linearity_scores = []

            if left_pca is not None:
                linearity_scores.append(left_pca["linearity"])

            if right_pca is not None:
                linearity_scores.append(right_pca["linearity"])

            linearity_score = float(np.mean(linearity_scores))

            if left_pca is not None and right_pca is not None:

                agreement_error = abs(
                    left_pca["heading"] -
                    right_pca["heading"]
                )

                agreement_score = max(
                    0.0,
                    1.0 - agreement_error / 10.0
                )

            else:

                agreement_score = 1.0

            confidence = (
                0.40 * linearity_score +
                0.30 * point_score +
                0.30 * agreement_score
            )

            return {
                "heading": heading,
                "confidence": float(confidence),
                "wall_used": "BOTH",

                "left_pca_heading": None if left_pca is None else left_pca["heading"],
                "left_pca_linearity": None if left_pca is None else left_pca["linearity"],
                "left_pca_points": None if left_pca is None else left_pca["valid_points"],

                "right_pca_heading": None if right_pca is None else right_pca["heading"],
                "right_pca_linearity": None if right_pca is None else right_pca["linearity"],
                "right_pca_points": None if right_pca is None else right_pca["valid_points"],
            }

        # --------------------------------------------------
        # Post-turn calibration
        # Uses only the outer wall
        # --------------------------------------------------

        if outer_wall == "LEFT":

            selected = left_pca

        elif outer_wall == "RIGHT":

            selected = right_pca

        else:

            raise ValueError(
                f"Invalid outer_wall: {outer_wall}"
            )

        if selected is None:
            return {
                "heading": None,
                "confidence": 0.0
            }

        heading = selected["heading"]

        point_score = min(
            selected["valid_points"] / 40.0,
            1.0
        )

        linearity_score = selected["linearity"]

        confidence = (
            0.60 * linearity_score +
            0.40 * point_score
        )

        return {
            "heading": heading,
            "confidence": float(confidence),
            "wall_used": outer_wall,

            "left_pca_heading": None if left_pca is None else left_pca["heading"],
            "left_pca_linearity": None if left_pca is None else left_pca["linearity"],
            "left_pca_points": None if left_pca is None else left_pca["valid_points"],

            "right_pca_heading": None if right_pca is None else right_pca["heading"],
            "right_pca_linearity": None if right_pca is None else right_pca["linearity"],
            "right_pca_points": None if right_pca is None else right_pca["valid_points"],
        }
