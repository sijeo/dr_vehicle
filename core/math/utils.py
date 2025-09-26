"""
Mathematical utility functions for dead reckoning.
"""

import numpy as np
import math

def rotation_matrix(angle):
    """
    Create a 2D rotation matrix for the given angle.
    
    Args:
        angle (float): Angle in radians
        
    Returns:
        np.ndarray: 2x2 rotation matrix
    """
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    
    return np.array([
        [cos_a, -sin_a],
        [sin_a,  cos_a]
    ])

def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi] range.
    
    Args:
        angle (float): Angle in radians
        
    Returns:
        float: Normalized angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def wrap_angle(angle):
    """
    Wrap angle to [0, 2*pi] range.
    
    Args:
        angle (float): Angle in radians
        
    Returns:
        float: Wrapped angle in [0, 2*pi]
    """
    return angle % (2 * math.pi)

def degrees_to_radians(degrees):
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0

def radians_to_degrees(radians):
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points on Earth.
    
    Args:
        lat1, lon1: Latitude and longitude of first point (degrees)
        lat2, lon2: Latitude and longitude of second point (degrees)
        
    Returns:
        float: Distance in meters
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    # Earth radius in meters
    R = 6371000
    
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing between two GPS coordinates.
    
    Args:
        lat1, lon1: Starting latitude and longitude (degrees)
        lat2, lon2: Ending latitude and longitude (degrees)
        
    Returns:
        float: Bearing in radians [0, 2*pi]
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing = math.atan2(y, x)
    
    # Convert to [0, 2*pi] range
    return wrap_angle(bearing)