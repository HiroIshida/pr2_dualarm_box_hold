import math
from dataclasses import dataclass

import numpy as np
import scipy.spatial


@dataclass
class PlanerBoundingBox:
    angle: float
    center: np.ndarray
    corners: np.ndarray

    def plot(self, fax, color="red") -> None:
        fig, ax = fax
        ax.scatter(self.corners[:, 0], self.corners[:, 1], c=color)
        for pair in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            ax.plot(self.corners[pair, 0], self.corners[pair, 1], color=color)

    @classmethod
    def from_points(cls, points: np.ndarray) -> "PlanerBoundingBox":
        hull = scipy.spatial.ConvexHull(points)
        angle, center, corners = cls.find_min_bound_rect(points[hull.vertices])
        return cls(angle, center, corners)

    @staticmethod
    def find_min_bound_rect(hull_points_2d):
        # took from https://github.com/dbworth/minimum-area-bounding-rectangle/blob/master/python/min_bounding_rect.py
        # Copyright (c) 2013, David Butterworth, University of Queensland
        # All rights reserved.

        # Compute edges (x2-x1,y2-y1)
        edges = np.zeros((len(hull_points_2d) - 1, 2))  # empty 2 column array
        for i in range(len(edges)):
            edge_x = hull_points_2d[i + 1, 0] - hull_points_2d[i, 0]
            edge_y = hull_points_2d[i + 1, 1] - hull_points_2d[i, 1]
            edges[i] = [edge_x, edge_y]

        # Calculate edge angles   atan2(y/x)
        edge_angles = np.zeros((len(edges)))  # empty 1 column array
        for i in range(len(edge_angles)):
            edge_angles[i] = math.atan2(edges[i, 1], edges[i, 0])

        # Check for angles in 1st quadrant
        for i in range(len(edge_angles)):
            edge_angles[i] = abs(edge_angles[i] % (math.pi / 2))  # want strictly positive answers

        # Remove duplicate angles
        edge_angles = np.unique(edge_angles)

        # Test each angle to find bounding box with smallest area
        large_int = 1000
        min_bbox = (
            0,
            large_int,
            0,
            0,
            0,
            0,
            0,
            0,
        )  # rot_angle, area, width, height, min_x, max_x, min_y, max_y
        for i in range(len(edge_angles)):

            # Create rotation matrix to shift points to baseline
            # R = [ cos(theta)      , cos(theta-PI/2)
            #       cos(theta+PI/2) , cos(theta)     ]
            R = np.array(
                [
                    [math.cos(edge_angles[i]), math.cos(edge_angles[i] - (math.pi / 2))],
                    [math.cos(edge_angles[i] + (math.pi / 2)), math.cos(edge_angles[i])],
                ]
            )

            # Apply this rotation to convex hull points
            rot_points = np.dot(R, np.transpose(hull_points_2d))  # 2x2 * 2xn

            # Find min/max x,y points
            min_x = np.nanmin(rot_points[0], axis=0)
            max_x = np.nanmax(rot_points[0], axis=0)
            min_y = np.nanmin(rot_points[1], axis=0)
            max_y = np.nanmax(rot_points[1], axis=0)

            # Calculate height/width/area of this bounding rectangle
            width = max_x - min_x
            height = max_y - min_y
            area = width * height

            # Store the smallest rect found first (a simple convex hull might have 2 answers with same area)
            if area < min_bbox[1]:
                min_bbox = (edge_angles[i], area, width, height, min_x, max_x, min_y, max_y)
            # Bypass, return the last found rect
            # min_bbox = ( edge_angles[i], area, width, height, min_x, max_x, min_y, max_y )

        # Re-create rotation matrix for smallest rect
        angle = min_bbox[0]
        R = np.array(
            [
                [math.cos(angle), math.cos(angle - (math.pi / 2))],
                [math.cos(angle + (math.pi / 2)), math.cos(angle)],
            ]
        )

        # min/max x,y points are against baseline
        min_x = min_bbox[4]
        max_x = min_bbox[5]
        min_y = min_bbox[6]
        max_y = min_bbox[7]

        # Calculate center point and project onto rotated frame
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        center_point = np.dot([center_x, center_y], R)

        # Calculate corner points and project onto rotated frame
        corner_points = np.zeros((4, 2))  # empty 2 column array
        corner_points[0] = np.dot([max_x, min_y], R)
        corner_points[1] = np.dot([min_x, min_y], R)
        corner_points[2] = np.dot([min_x, max_y], R)
        corner_points[3] = np.dot([max_x, max_y], R)

        return angle, center_point, corner_points
