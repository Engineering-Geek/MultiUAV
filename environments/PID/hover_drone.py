from typing import Tuple

import numpy as np
from mujoco import MjData, MjModel

from environments.PID.base_drone import BasePIDDrone
from environments.mujoco_xmls import make_model


class PIDError:
    """
    A class to represent a PID _error.
    
    Attributes:
        kp (float): The proportional gain.
        ki (float): The integral gain.
        kd (float): The derivative gain.
        integral_error (float): The integral _error.
        prev_error (float): The previous _error.
    """
    _error: float = 0.0
    _integral_error: float = 0.0
    _derivative_error: float = 0.0
    _prev_error: float = 0.0
    def __init__(self, kp: float = 0.1, ki: float = 0.1, kd: float = 0.1, max_integral_error: float = 1.0,
                 dt: float = 0.01):
        """
        Initializes the PID _error.
        Args:
            kp: The proportional gain.
            ki: The integral gain.
            kd: The derivative gain.
            max_integral_error: The maximum integral _error.
            dt: The time step.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral_error = max_integral_error
        self.dt = dt
    
    def __call__(self, error: float):
        """
        Updates the error, integral error, and derivative error.
        Args:
            error: The _error.
        """
        self._error = error
        self._integral_error = np.clip(
            self._integral_error + self._error * self.dt, -self.max_integral_error, self.max_integral_error)
        self._derivative_error = (self._error - self._prev_error) / self.dt
        self._prev_error = self._error
        
    @property
    def output(self):
        return self.kp * self._error + self.ki * self._integral_error + self.kd * self._derivative_error
    
    @property
    def error(self):
        return self._error
    
    @property
    def integral_error(self):
        return self._integral_error
    
    @property
    def derivative_error(self):
        return self._derivative_error


class PIDDrone(BasePIDDrone):
    def __init__(self, index: int, model: MjModel, data: MjData, render: bool):
        super().__init__(index, model, data, render)
        # Note, goal is to have 0 linear and angular velocities and accelerations
        kp = 1
        ki = 1
        kd = 1
        max_integral_error = 1
        
        self.pid_thrust = PIDError(kp, ki, kd, max_integral_error)
        self.pid_yaw = PIDError(kp, ki, kd, max_integral_error)
        self.pid_pitch = PIDError(kp, ki, kd, max_integral_error)
        self.pid_roll = PIDError(kp, ki, kd, max_integral_error)
        
        self.control_allocation_matrix = np.array([
            [1, -1, -1],
            [1, 1, 1],
            [1, 1, -1],
            [1, -1, 1]
        ])
    
    @property
    def control_error_vector(self):
        """
        Vector with elements [thrust, roll, pitch, yaw]
        """
        return np.array(
            [self.pid_thrust.output, self.pid_roll.output, self.pid_pitch.output, self.pid_yaw.output]
        ).transpose()
    
    @property
    def thrust(self):
        return self.control_allocation_matrix @ self.control_error_vector
    
    def update(self, goal: Tuple[float, float, float, float]):
        """
        Updates the PID _error.
        Args:
            goal: The goal state.
        """
        # Calculate the _error
        thrust_error = goal[0] - self.local_linear_acceleration[2]
        roll_error = goal[1] - self.roll
        pitch_error = goal[2] - self.pitch
        yaw_error = goal[3] - self.yaw
        
        # Update the PID _error
        self.pid_thrust(thrust_error)
        self.pid_roll(roll_error)
        self.pid_pitch(pitch_error)
        self.pid_yaw(yaw_error)
        
        # Calculate the thrust
        self._propeller_force = self.thrust
        
    def step(self, goal: Tuple[float, float, float, float]):
        """
        Steps the PID controller.
        Args:
            goal: The goal state.
        """
        self.update(goal)
        self.data.ctrl[self.sample_prop_body_id] = self._propeller_force

