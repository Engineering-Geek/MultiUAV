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
    def __init__(self, index: int, model: MjModel, data: MjData):
        super().__init__(index, model, data)
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

        self.target_thrust = 0
        self.target_pitch = 0
        self.target_roll = 0
        self.target_yaw = 0

        self.max_thrust = .5        # Newtons
        self.max_pitch = 0.15       # Radians
        self.max_roll = 0.15        # Radians
        self.max_yaw = 0.15         # Radians

        self.min_thrust = 0
        self.min_pitch = -0.15
        self.min_roll = -0.15
        self.min_yaw = -0.15

        self.thrust_increment = 0.05

    def key_callback(self, key_code: int):
        """
        Updates the goal state based on the key code.
        Controls:
            - Thrust Increase: + (plus) / Thrust Decrease: - (minus)
            - Roll: A, D
            - Pitch: S, W
            - Yaw: Q, E
        Args:
            key_code: The key code.
        """
        if chr(key_code) == '+':
            self.target_thrust += self.thrust_increment
        elif chr(key_code) == '-':
            self.target_thrust -= self.thrust_increment

        if chr(key_code) == 'a':
            self.target_roll = self.max_roll
        elif chr(key_code) == 'd':
            self.target_roll = self.min_roll
        else:
            self.target_roll = 0

        if chr(key_code) == 'w':
            self.target_pitch = self.max_pitch
        elif chr(key_code) == 's':
            self.target_pitch = self.min_pitch
        else:
            self.target_pitch = 0

        if chr(key_code) == 'q':
            self.target_yaw = self.max_yaw
        elif chr(key_code) == 'e':
            self.target_yaw = self.min_yaw
        else:
            self.target_yaw = 0

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

    def control_step(self):
        # Calculate the error
        thrust_error = self.target_thrust - self.local_linear_acceleration[2]
        roll_error = self.target_roll - self.roll
        pitch_error = self.target_pitch - self.pitch
        yaw_error = self.target_yaw - self.yaw

        # Update the PID _error
        self.pid_thrust(thrust_error)
        self.pid_roll(roll_error)
        self.pid_pitch(pitch_error)
        self.pid_yaw(yaw_error)

        # Calculate the thrust
        self.propeller_thrust = self.thrust


