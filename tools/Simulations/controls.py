"""
controls.py

This module wraps Pygame event handling to provide high level control
commands for the ground vehicle simulation. It maps keyboard arrow 
keys to acceleration and yaw rate commands, and handle graceful
shutdown when the user presses ESC or closes the window.


Classes:
KeyboardController:
    Reads Pygame events and computes acceleration and yaw rate commands
    for the vehicle.

"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

try:
    import pygame
except ImportError: # pragma: no cover
    pygame = None   # type: ignore
    print("Pygame is not installed. KeyboardController will not function.")

@dataclass
class KeyboardController:
    """ Translate keyboard input into vehicle control commands.
    
    The controller maps the arrow key to forward accleration. braking, 
    and yaw rate commands. Use the poll() method in each simulation loop
    iteration to read input and obtain the commanded acceleration and yaw rate.
    
    If Pygame is not available, the controller will disable itself and 
    always output zero commands. In this case the simulation can still be run
    programmatically (e.g. with pre-determined control inputs) without interactive
    control.
    
    Parameters:
    accel_forward (float): Acceleration command (m/s^2) when UP arrow is pressed.
    accel_brake (float): Deceleration command (m/s^2) when DOWN arrow is pressed.
    yaw_rate_cmd (float): Yaw rate command (rad/s) when LEFT or RIGHT arrow is pressed
    Positive yaw corresponds to a left turn.
    """

    accel_forward: float = 1.0
    accel_brake: float = 1.0
    yaw_rate_deg: float = 15.0  # degrees per second
    # Internal state
    running : bool = True
    window_size: Tuple[int, int] = (300, 200)

    def __post_init__(self):
        if pygame is None:
            print("Pygame not available. KeyboardController disabled.")
            self._enabled = False
            return
        self._enabled = True
        pygame.init()
        self._screen = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption("Vehicle Keyboard Controller")
        self._clock = pygame.time.Clock()

    def poll(self) -> Tuple[float, float]:
        """ Poll the keyboard and compute control commands.
        
        Returns:
            Tuple[float, float]: (acceleration_cmd, yaw_rate_cmd)
            The commands forward acceleration (m/s^2) and yaw rate (rad/s).
            for the iteration. If the controller is disabled (Pygame unavailable)
            these will always be zero.

        """
        if not self._enabled:
            return 0.0, 0.0
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                self.running = False
        keys = pygame.key.get_pressed()
        acc_cmd = 0.0
        yaw_rate_cmd = 0.0
        # Forward/back commands
        if keys[pygame.K_UP]:
            acc_cmd += self.accel_forward
        if keys[pygame.K_DOWN]:
            acc_cmd -= self.accel_brake
        # Yaw commands (convert degrees/s to radians/s)
        yaw_rate_rad = self.yaw_rate_deg * (3.14159265 / 180.0)
        if keys[pygame.K_LEFT]:
            yaw_rate_cmd += yaw_rate_rad
        if keys[pygame.K_RIGHT]:
            yaw_rate_cmd -= yaw_rate_rad
        # Tick the clock to manage frame rate (optional)
        self._clock.tick()  
        return acc_cmd, yaw_rate_cmd
    

    


