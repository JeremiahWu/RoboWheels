"""Metric RGB-D ORB odometry core. IMU fusion is deliberately a separate backend."""

import cv2
import numpy as np


class OrbOdometry:
    def __init__(self, features=800, min_inliers=25):
        self.orb = cv2.ORB_create(nfeatures=features, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.min_inliers = min_inliers
        self.previous = None
        self.pose = np.eye(4)
        self.segment = 0

    def reset(self):
        self.previous = None
        self.pose = np.eye(4)
        self.segment += 1

    def update(self, image, depth, K, stamp):
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        current = (keypoints, descriptors, depth.copy(), K.copy(), stamp)
        result = {
            "state": "INITIALIZING",
            "features": len(keypoints),
            "inliers": 0,
            "pose": None,
            "segment": self.segment,
            "keypoints": keypoints,
        }
        old = self.previous
        if old is None:
            self.previous = current
            return result
        dt = stamp - old[4]
        if dt <= 0:
            result["state"] = "NON_MONOTONIC_INPUT"
            return result
        if (
            dt > 1.0
            or descriptors is None
            or old[1] is None
            or not np.allclose(K, old[3])
        ):
            self.reset()
            self.previous = current
            result.update(state="TRACKING_LOST", segment=self.segment)
            return result
        matches = sorted(
            self.matcher.match(old[1], descriptors), key=lambda m: m.distance
        )
        objects, pixels = [], []
        for match in matches:
            if match.distance > 60:
                continue
            u, v = old[0][match.queryIdx].pt
            x, y = int(round(u)), int(round(v))
            if not (0 <= y < old[2].shape[0] and 0 <= x < old[2].shape[1]):
                continue
            z = float(old[2][y, x])
            if not np.isfinite(z) or not 0.2 < z < 5.0:
                continue
            objects.append(
                [
                    (u - old[3][0, 2]) * z / old[3][0, 0],
                    (v - old[3][1, 2]) * z / old[3][1, 1],
                    z,
                ]
            )
            pixels.append(keypoints[match.trainIdx].pt)
        result["matches"] = len(objects)
        if len(objects) < self.min_inliers:
            self.reset()
            self.previous = current
            result.update(state="INSUFFICIENT_DEPTH_MATCHES", segment=self.segment)
            return result
        ok, rotation, translation, inliers = cv2.solvePnPRansac(
            np.asarray(objects, dtype=np.float32),
            np.asarray(pixels, dtype=np.float32),
            K,
            None,
            iterationsCount=100,
            reprojectionError=2.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_EPNP,
        )
        count = 0 if inliers is None else len(inliers)
        result["inliers"] = count
        valid = ok and count >= self.min_inliers and count >= 0.4 * len(objects)
        valid = valid and np.isfinite(rotation).all() and np.isfinite(translation).all()
        valid = (
            valid
            and np.linalg.norm(translation) < max(0.08, dt * 1.0)
            and np.linalg.norm(rotation) < max(0.1, dt * 2.0)
        )
        if not valid:
            self.reset()
            self.previous = current
            result.update(state="PNP_REJECTED", segment=self.segment)
            return result
        transform = np.eye(4)
        transform[:3, :3] = cv2.Rodrigues(rotation)[0]
        transform[:3, 3] = translation.ravel()
        self.pose = self.pose @ np.linalg.inv(transform)
        self.previous = current
        result.update(state="TRACKING_VO_ONLY", pose=self.pose.copy())
        return result
