import _opencv_util
import numpy as np

def fit_rigid_transform_2d(points1, points2):
    # This would be a good place to do some better type checking and conversions
    pt1 = np.array(points1, dtype=np.float32, order='C')
    pt2 = np.array(points2, dtype=np.float32, order='C')
    return _opencv_util._fit_rigid_transform_2d(pt1, pt2)


def apply_transform(transform, points):
    pts_ext = np.ones((3, points.shape[0]), np.float32)
    pts_ext[:2,:] = points.T

    pts_ext = np.dot(transform, pts_ext)
    return pts_ext[:2,:].T
