"""
Vehicle state representation for the EKF.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional
import time

@dataclass
class VehicleState:
    """
    Represents the vehicle state for dead reckoning.
    
    State vector: [x, y, vx, vy, heading, heading_rate]
    - x, y: Position in meters (local coordinate frame)
    - vx, vy: Velocity in m/s (local coordinate frame)
    - heading: Vehicle heading in radians
    - heading_rate: Angular velocity in rad/s
    """
    
    # Position (meters)
    x: float = 0.0
    y: float = 0.0
    
    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    
    # Orientation (radians)
    heading: float = 0.0
    heading_rate: float = 0.0
    
    # Timestamp
    timestamp: Optional[float] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    @property
    def state_vector(self) -> np.ndarray:
        """Get state as numpy vector."""
        return np.array([
            self.x,
            self.y, 
            self.vx,
            self.vy,
            self.heading,
            self.heading_rate
        ])
    
    @state_vector.setter
    def state_vector(self, vector: np.ndarray):
        """Set state from numpy vector."""
        if len(vector) != 6:
            raise ValueError("State vector must have 6 elements")
            
        self.x = vector[0]
        self.y = vector[1]
        self.vx = vector[2]
        self.vy = vector[3]
        self.heading = vector[4]
        self.heading_rate = vector[5]
    
    @property
    def position(self) -> np.ndarray:
        """Get position as [x, y] vector."""
        return np.array([self.x, self.y])
    
    @property
    def velocity(self) -> np.ndarray:
        """Get velocity as [vx, vy] vector."""
        return np.array([self.vx, self.vy])
    
    @property
    def speed(self) -> float:
        """Get vehicle speed in m/s."""
        return np.sqrt(self.vx**2 + self.vy**2)
    
    def copy(self) -> 'VehicleState':
        """Create a copy of the state."""
        return VehicleState(
            x=self.x,
            y=self.y,
            vx=self.vx,
            vy=self.vy,
            heading=self.heading,
            heading_rate=self.heading_rate,
            timestamp=self.timestamp
        )
    
    def __str__(self) -> str:
        return (
            f"VehicleState(pos=[{self.x:.2f}, {self.y:.2f}], "
            f"vel=[{self.vx:.2f}, {self.vy:.2f}], "
            f"heading={self.heading:.3f}, "
            f"heading_rate={self.heading_rate:.3f})"
        )