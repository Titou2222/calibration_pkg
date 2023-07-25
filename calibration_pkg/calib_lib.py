import numpy as np


def camera_intrinsics(intrinsic_param):
    """
    Computes the homogenous intrinsic matrix from camera intrinsic parameters.

    Args:
        intrinsic_param (List(float)) : List of the intrinsic parameters of the camera: 
            - cx (float): Principal point x-coordinate.
            - cy (float): Principal point y-coordinate.
            - fx (float): Focal length in the x-direction.
            - fy (float): Focal length in the y-direction.

    Returns:
        numpy.ndarray: Intrinsic matrix 3x4.
    """
    cx, cy, fx, fy = intrinsic_param
    intrinsic_matrix = np.array([
        [fx, 0, cx, 0],
        [0, fy, cy, 0],
        [0,  0,  1, 0]])
    
    return intrinsic_matrix

def camera_intrinsics3x3(intrinsic_param):
    """
    Computes the non-homogenous intrinsic matrix from camera intrinsic parameters.

    Args:
        intrinsic_param (np.array([float, float, float, float])) : Array of the intrinsic parameters of the camera: 
            - cx (float): Principal point x-coordinate.
            - cy (float): Principal point y-coordinate.
            - fx (float): Focal length in the x-direction.
            - fy (float): Focal length in the y-direction.

    Returns:
        numpy.ndarray: Intrinsic matrix 3x3.
    """
    cx, cy, fx, fy = intrinsic_param
    intrinsic_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0,  0,  1]])
    
    return intrinsic_matrix

def dist(point):
    """
    Computes the distance of a point to the origin thanks to the coordinates of the point.

    Args:
        point (np.array([float, float, float])) : Array of the coordinates of the point: 
            - x (float): x-Coordinate of point 
            - y (float): y-Coordinate of point 
            - z (float): z-Coordinate of point 

    Returns:
        float: distance
    """
    x,y,z = point
    return np.sqrt(x**2+y**2+z**2)


