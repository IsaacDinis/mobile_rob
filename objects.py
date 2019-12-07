
import numpy as np
import math


def dist(x1, x2):
    dist = math.sqrt((x1[0] - x2[0]) ** 2 + (x1[0] - x2[0]) ** 2)
    return dist


class IsoTriangle:

    def __init__(self, points):
        self.points = points
        self._orientation = int
        self._cm = float

    def _find_peak(self):
        points = self.points
        dist1 = dist(points[0, :], points[1, :])
        dist2 = dist(points[0, :], points[2, :])
        if abs(dist1-dist2) < 5:
            peak = points[0]

        elif dist1 > dist2:
            peak = points[1]

        else:
            peak = points[2]

        return peak

    def get_centroid(self):
        points = self.points
        c = list()
        c.append((points[0, 0]+points[1, 0]+points[2, 0])/3)
        c.append((points[0, 1] + points[1, 1] + points[2, 1])/3)
        return c

    def get_orientation(self):
        c = self.get_centroid()
        p = self._find_peak()
        v = [p[0]-c[0], p[1]-c[1]]
        v = v / np.linalg.norm(v)  # normalize the vector
        e = [0, -1]
        angle = np.sign(v[1])*np.arccos(np.clip(np.dot(v, e), -1.0, 1.0))

        return angle


class Coord:
    def __init__(self):
        self.x = float
        self.y = float


class Thymio:
    def __init__(self, circles):
        self.pos = Coord()
        if circles[0, 2] < circles[1, 2]:
            self.pos.x = circles[1, 0]
            self.pos.y = circles[1, 1]
            self.theta = self._calculate_orientation(circles[0])
        else:
            self.pos.x = circles[0, 0]
            self.pos.y = circles[0, 1]
            self.theta = self._calculate_orientation(circles[1])

    def _calculate_orientation(self, circle):
        v = [circle[0]-self.pos.x, circle[1]-self.pos.y]
        v = v / np.linalg.norm(v)  # normalize the vector
        e = [1, 0]
        angle = np.sign(v[1])*np.arccos(np.clip(np.dot(v, e), -1.0, 1.0))
        return angle
