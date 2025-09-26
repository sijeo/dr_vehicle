"""
NMEA sentence parsing for GPS data.
"""

import re
import time
import math
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass

@dataclass
class NMEAFixData:
    """NMEA GPS fix data."""
    
    # Position
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    
    # Quality indicators
    fix_quality: int = 0  # 0=invalid, 1=GPS fix, 2=DGPS fix
    satellites_used: int = 0
    hdop: Optional[float] = None  # Horizontal dilution of precision
    
    # Velocity
    speed_knots: Optional[float] = None
    course_degrees: Optional[float] = None
    
    # Time
    utc_time: Optional[str] = None
    date: Optional[str] = None
    timestamp: Optional[float] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    @property
    def is_valid(self) -> bool:
        """Check if GPS fix is valid."""
        return (self.fix_quality > 0 and 
                self.latitude is not None and 
                self.longitude is not None)
    
    @property
    def speed_ms(self) -> Optional[float]:
        """Get speed in m/s."""
        if self.speed_knots is not None:
            return self.speed_knots * 0.514444  # knots to m/s
        return None
    
    @property
    def course_radians(self) -> Optional[float]:
        """Get course in radians."""
        if self.course_degrees is not None:
            return math.radians(self.course_degrees)
        return None

class NMEAParser:
    """
    Parser for NMEA 0183 sentences from GPS modules.
    
    Supports common sentences:
    - GGA: Global Positioning System Fix Data
    - RMC: Recommended Minimum Navigation Information
    - GSA: GPS DOP and active satellites
    - GSV: GPS Satellites in view
    """
    
    def __init__(self):
        self.last_fix = NMEAFixData()
        self.sentence_count = 0
        self.parse_errors = 0
        
    def calculate_checksum(self, sentence: str) -> str:
        """Calculate NMEA checksum."""
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"{checksum:02X}"
    
    def validate_checksum(self, sentence: str) -> bool:
        """Validate NMEA sentence checksum."""
        if '*' not in sentence:
            return False
            
        data, checksum = sentence.split('*')
        data = data[1:]  # Remove '$' prefix
        
        calculated = self.calculate_checksum(data)
        return calculated == checksum.upper()
    
    def parse_coordinate(self, coord_str: str, direction: str) -> Optional[float]:
        """
        Parse NMEA coordinate format.
        
        Args:
            coord_str: Coordinate string (e.g., "4916.45")
            direction: Direction (N/S for latitude, E/W for longitude)
            
        Returns:
            Coordinate in decimal degrees
        """
        if not coord_str or not direction:
            return None
            
        try:
            # Parse DDMM.MMMM format
            if len(coord_str) < 4:
                return None
                
            # Find decimal point
            dot_pos = coord_str.find('.')
            if dot_pos == -1:
                return None
                
            # Extract degrees and minutes
            degrees_len = dot_pos - 2
            if degrees_len < 1:
                return None
                
            degrees = float(coord_str[:degrees_len])
            minutes = float(coord_str[degrees_len:])
            
            # Convert to decimal degrees
            decimal_degrees = degrees + minutes / 60.0
            
            # Apply direction
            if direction in ['S', 'W']:
                decimal_degrees = -decimal_degrees
                
            return decimal_degrees
            
        except (ValueError, IndexError):
            return None
    
    def parse_gga(self, fields: list) -> bool:
        """
        Parse GGA sentence: Global Positioning System Fix Data.
        
        Format: $GPGGA,time,lat,lat_dir,lon,lon_dir,quality,satellites,hdop,alt,alt_unit,geoid,geoid_unit,dgps_time,dgps_id*checksum
        """
        if len(fields) < 15:
            return False
            
        try:
            # Time
            if fields[1]:
                self.last_fix.utc_time = fields[1]
            
            # Position
            lat = self.parse_coordinate(fields[2], fields[3])
            lon = self.parse_coordinate(fields[4], fields[5])
            
            if lat is not None:
                self.last_fix.latitude = lat
            if lon is not None:
                self.last_fix.longitude = lon
            
            # Quality
            if fields[6]:
                self.last_fix.fix_quality = int(fields[6])
            
            # Satellites
            if fields[7]:
                self.last_fix.satellites_used = int(fields[7])
            
            # HDOP
            if fields[8]:
                self.last_fix.hdop = float(fields[8])
            
            # Altitude
            if fields[9]:
                self.last_fix.altitude = float(fields[9])
                
            return True
            
        except (ValueError, IndexError):
            return False
    
    def parse_rmc(self, fields: list) -> bool:
        """
        Parse RMC sentence: Recommended Minimum Navigation Information.
        
        Format: $GPRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum
        """
        if len(fields) < 12:
            return False
            
        try:
            # Time
            if fields[1]:
                self.last_fix.utc_time = fields[1]
            
            # Status (A=active, V=void)
            status = fields[2]
            if status != 'A':
                self.last_fix.fix_quality = 0
                return True
            
            # Position
            lat = self.parse_coordinate(fields[3], fields[4])
            lon = self.parse_coordinate(fields[5], fields[6])
            
            if lat is not None:
                self.last_fix.latitude = lat
            if lon is not None:
                self.last_fix.longitude = lon
            
            # Speed in knots
            if fields[7]:
                self.last_fix.speed_knots = float(fields[7])
            
            # Course in degrees
            if fields[8]:
                self.last_fix.course_degrees = float(fields[8])
            
            # Date
            if fields[9]:
                self.last_fix.date = fields[9]
            
            # If we got here with valid data, set fix quality
            if self.last_fix.latitude is not None and self.last_fix.longitude is not None:
                if self.last_fix.fix_quality == 0:
                    self.last_fix.fix_quality = 1  # GPS fix
                    
            return True
            
        except (ValueError, IndexError):
            return False
    
    def parse_sentence(self, sentence: str) -> Optional[NMEAFixData]:
        """
        Parse a single NMEA sentence.
        
        Args:
            sentence: NMEA sentence string
            
        Returns:
            NMEAFixData if parsing successful and fix is valid, None otherwise
        """
        self.sentence_count += 1
        
        # Clean up sentence
        sentence = sentence.strip()
        
        # Validate format
        if not sentence.startswith('$') or '*' not in sentence:
            self.parse_errors += 1
            return None
        
        # Validate checksum
        if not self.validate_checksum(sentence):
            self.parse_errors += 1
            return None
        
        # Split sentence
        data_part = sentence.split('*')[0]
        fields = data_part.split(',')
        
        if len(fields) < 1:
            self.parse_errors += 1
            return None
        
        sentence_type = fields[0][1:]  # Remove '$' prefix
        
        # Parse based on sentence type
        success = False
        
        if sentence_type.endswith('GGA'):
            success = self.parse_gga(fields)
        elif sentence_type.endswith('RMC'):
            success = self.parse_rmc(fields)
        else:
            # Unsupported sentence type, but not an error
            return None
        
        if not success:
            self.parse_errors += 1
            return None
        
        # Update timestamp
        self.last_fix.timestamp = time.time()
        
        # Return fix data if valid
        if self.last_fix.is_valid:
            return NMEAFixData(
                latitude=self.last_fix.latitude,
                longitude=self.last_fix.longitude,
                altitude=self.last_fix.altitude,
                fix_quality=self.last_fix.fix_quality,
                satellites_used=self.last_fix.satellites_used,
                hdop=self.last_fix.hdop,
                speed_knots=self.last_fix.speed_knots,
                course_degrees=self.last_fix.course_degrees,
                utc_time=self.last_fix.utc_time,
                date=self.last_fix.date,
                timestamp=self.last_fix.timestamp
            )
        
        return None
    
    def get_last_fix(self) -> NMEAFixData:
        """Get the most recent GPS fix data."""
        return self.last_fix
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get parser statistics."""
        return {
            'sentences_processed': self.sentence_count,
            'parse_errors': self.parse_errors,
            'error_rate': self.parse_errors / max(1, self.sentence_count),
            'last_fix_valid': self.last_fix.is_valid
        }