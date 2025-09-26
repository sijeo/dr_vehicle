"""
GPS sensor data processing for Ublox NEO6M.
"""

import numpy as np
import time
import math
from dataclasses import dataclass
from typing import Optional, List, Tuple
from .nmea import NMEAParser, NMEAFixData
from ..math.utils import haversine_distance, calculate_bearing
from ..math.constants import EARTH_RADIUS_M

@dataclass
class GPSData:
    """Processed GPS data for dead reckoning."""
    
    # Position (decimal degrees)
    latitude: float
    longitude: float
    altitude: Optional[float] = None
    
    # Local coordinates (meters from reference point)
    x: Optional[float] = None
    y: Optional[float] = None
    
    # Velocity
    speed: Optional[float] = None      # m/s
    course: Optional[float] = None     # radians
    
    # Quality indicators
    fix_quality: int = 0
    satellites: int = 0
    hdop: Optional[float] = None
    
    # Timestamp
    timestamp: Optional[float] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    @property
    def is_valid(self) -> bool:
        """Check if GPS data is valid."""
        return (self.fix_quality > 0 and 
                -90 <= self.latitude <= 90 and 
                -180 <= self.longitude <= 180)
    
    @property
    def position_local(self) -> Optional[np.ndarray]:
        """Get local position as numpy array."""
        if self.x is not None and self.y is not None:
            return np.array([self.x, self.y])
        return None
    
    @property
    def position_global(self) -> np.ndarray:
        """Get global position as numpy array."""
        return np.array([self.latitude, self.longitude])

class GPSProcessor:
    """
    Processes GPS data from Ublox NEO6M for dead reckoning.
    """
    
    def __init__(self, 
                 reference_lat: Optional[float] = None,
                 reference_lon: Optional[float] = None):
        """
        Initialize GPS processor.
        
        Args:
            reference_lat: Reference latitude for local coordinate system
            reference_lon: Reference longitude for local coordinate system
        """
        self.nmea_parser = NMEAParser()
        
        # Reference point for local coordinates
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        
        # Previous position for velocity calculation
        self.last_position = None
        self.last_timestamp = None
        
        # Statistics
        self.fix_count = 0
        self.last_update_time = time.time()
        
        # Quality tracking
        self.position_history = []
        self.max_history_length = 10
    
    def set_reference_point(self, latitude: float, longitude: float):
        """
        Set reference point for local coordinate system.
        
        Args:
            latitude: Reference latitude (degrees)
            longitude: Reference longitude (degrees)
        """
        self.reference_lat = latitude
        self.reference_lon = longitude
        
        print(f"GPS reference point set to: {latitude:.6f}, {longitude:.6f}")
    
    def global_to_local(self, latitude: float, longitude: float) -> Tuple[float, float]:
        """
        Convert global GPS coordinates to local Cartesian coordinates.
        
        Uses simple equirectangular projection for small areas.
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            
        Returns:
            (x, y) in meters from reference point
        """
        if self.reference_lat is None or self.reference_lon is None:
            # If no reference point set, use first GPS fix as reference
            self.set_reference_point(latitude, longitude)
            return (0.0, 0.0)
        
        # Convert to radians
        lat1 = math.radians(self.reference_lat)
        lon1 = math.radians(self.reference_lon)
        lat2 = math.radians(latitude)
        lon2 = math.radians(longitude)
        
        # Equirectangular projection
        x = (lon2 - lon1) * math.cos((lat1 + lat2) / 2) * EARTH_RADIUS_M
        y = (lat2 - lat1) * EARTH_RADIUS_M
        
        return (x, y)
    
    def local_to_global(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert local Cartesian coordinates to global GPS coordinates.
        
        Args:
            x: X coordinate in meters from reference
            y: Y coordinate in meters from reference
            
        Returns:
            (latitude, longitude) in degrees
        """
        if self.reference_lat is None or self.reference_lon is None:
            raise ValueError("Reference point not set")
        
        # Convert reference to radians
        lat1 = math.radians(self.reference_lat)
        lon1 = math.radians(self.reference_lon)
        
        # Inverse equirectangular projection
        lat2 = lat1 + y / EARTH_RADIUS_M
        lon2 = lon1 + x / (EARTH_RADIUS_M * math.cos((lat1 + lat2) / 2))
        
        # Convert back to degrees
        return (math.degrees(lat2), math.degrees(lon2))
    
    def process_nmea_sentence(self, sentence: str) -> Optional[GPSData]:
        """
        Process a single NMEA sentence.
        
        Args:
            sentence: NMEA sentence string
            
        Returns:
            GPSData if valid fix obtained, None otherwise
        """
        nmea_fix = self.nmea_parser.parse_sentence(sentence)
        
        if nmea_fix is None or not nmea_fix.is_valid:
            return None
        
        return self.convert_nmea_to_gps_data(nmea_fix)
    
    def convert_nmea_to_gps_data(self, nmea_fix: NMEAFixData) -> GPSData:
        """
        Convert NMEA fix data to GPSData.
        
        Args:
            nmea_fix: Parsed NMEA fix data
            
        Returns:
            GPSData object
        """
        # Convert to local coordinates
        x, y = self.global_to_local(nmea_fix.latitude, nmea_fix.longitude)
        
        # Calculate velocity if we have previous position
        speed = None
        course = None
        
        if (self.last_position is not None and 
            self.last_timestamp is not None and
            nmea_fix.timestamp is not None):
            
            dt = nmea_fix.timestamp - self.last_timestamp
            
            if dt > 0:
                # Calculate distance and bearing
                distance = haversine_distance(
                    self.last_position[0], self.last_position[1],
                    nmea_fix.latitude, nmea_fix.longitude
                )
                
                speed = distance / dt
                
                course = calculate_bearing(
                    self.last_position[0], self.last_position[1],
                    nmea_fix.latitude, nmea_fix.longitude
                )
        
        # Use NMEA speed/course if available and reasonable
        if nmea_fix.speed_knots is not None:
            nmea_speed = nmea_fix.speed_knots * 0.514444  # knots to m/s
            # Use NMEA speed if it's reasonable or if we don't have calculated speed
            if speed is None or abs(nmea_speed - speed) < 2.0:  # Within 2 m/s
                speed = nmea_speed
        
        if nmea_fix.course_radians is not None:
            course = nmea_fix.course_radians
        
        # Create GPS data
        gps_data = GPSData(
            latitude=nmea_fix.latitude,
            longitude=nmea_fix.longitude,
            altitude=nmea_fix.altitude,
            x=x,
            y=y,
            speed=speed,
            course=course,
            fix_quality=nmea_fix.fix_quality,
            satellites=nmea_fix.satellites_used,
            hdop=nmea_fix.hdop,
            timestamp=nmea_fix.timestamp
        )
        
        # Update history
        self.last_position = (nmea_fix.latitude, nmea_fix.longitude)
        self.last_timestamp = nmea_fix.timestamp
        
        # Add to position history for quality assessment
        self.position_history.append(gps_data)
        if len(self.position_history) > self.max_history_length:
            self.position_history.pop(0)
        
        # Update statistics
        self.fix_count += 1
        self.last_update_time = gps_data.timestamp
        
        return gps_data
    
    def get_measurement_for_ekf(self, gps_data: GPSData) -> Optional[np.ndarray]:
        """
        Convert GPS data to measurement vector for EKF.
        
        Args:
            gps_data: GPS data
            
        Returns:
            Measurement vector [x, y] in local coordinates
        """
        if not gps_data.is_valid or gps_data.position_local is None:
            return None
        
        return gps_data.position_local
    
    def estimate_position_accuracy(self) -> float:
        """
        Estimate current position accuracy based on recent fixes.
        
        Returns:
            Estimated position accuracy in meters
        """
        if len(self.position_history) < 3:
            return 10.0  # Default uncertainty
        
        # Calculate position scatter over recent fixes
        positions = np.array([[p.x, p.y] for p in self.position_history[-5:] 
                             if p.x is not None and p.y is not None])
        
        if len(positions) < 2:
            return 10.0
        
        # Calculate standard deviation of positions
        pos_std = np.std(positions, axis=0)
        scatter = np.sqrt(pos_std[0]**2 + pos_std[1]**2)
        
        # Use HDOP if available
        hdop_factor = 1.0
        if self.position_history[-1].hdop is not None:
            hdop_factor = max(1.0, self.position_history[-1].hdop)
        
        # Combine scatter and HDOP
        accuracy = max(2.0, scatter * hdop_factor)
        
        return min(accuracy, 50.0)  # Cap at 50m
    
    def is_stationary(self, speed_threshold: float = 0.5) -> bool:
        """
        Check if vehicle appears to be stationary.
        
        Args:
            speed_threshold: Speed threshold in m/s
            
        Returns:
            True if vehicle appears stationary
        """
        if not self.position_history:
            return True
        
        recent_speeds = [p.speed for p in self.position_history[-3:] 
                        if p.speed is not None]
        
        if not recent_speeds:
            return True
        
        avg_speed = np.mean(recent_speeds)
        return avg_speed < speed_threshold
    
    def get_statistics(self) -> dict:
        """Get processor statistics."""
        nmea_stats = self.nmea_parser.get_statistics()
        
        return {
            'fix_count': self.fix_count,
            'last_update_time': self.last_update_time,
            'reference_point': (self.reference_lat, self.reference_lon),
            'position_history_length': len(self.position_history),
            'estimated_accuracy': self.estimate_position_accuracy(),
            'is_stationary': self.is_stationary(),
            'nmea_statistics': nmea_stats
        }