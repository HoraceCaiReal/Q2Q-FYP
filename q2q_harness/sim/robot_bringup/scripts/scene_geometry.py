"""Rigid scene geometry independent of ROS; model poses are world, links model-local."""
import numpy as np


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-9:
        raise ValueError('invalid rotation')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def world_to_model(scene, model, entity):
    root = scene[model]
    return rotation(root['quaternion']).T @ (
        np.asarray(scene[entity]['position']) - np.asarray(root['position']))


def local_point(scene, name):
    point = np.asarray(scene[name]['position'], dtype=float)
    if not np.isfinite(point).all():
        raise ValueError('invalid physical point: '+name)
    return point
