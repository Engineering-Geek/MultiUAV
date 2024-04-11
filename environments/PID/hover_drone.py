import numpy as np

from environments.PID.base_drone import BasePIDDrone


class PIDError:
    integral_error: float = 0
    prev_error: float = 0


class HoverPIDDrone(BasePIDDrone):
    def __init__(self, index: int, model, data):
        super().__init__(index, model, data)
        # Note, goal is to have 0 linear and angular velocities and accelerations
        self.positional_error = PIDError()
        self.velocity_error = PIDError()
        self.acceleration_error = PIDError()
        self.set_point = 1.0
        self.kp = 0
        self.ki = 0
        self.kd = 0

    def step(self):
        self.propeller_force = self.pid_controller()

    def get_positional_error(self):
        return self.set_point - self.body_pos[2]

    def get_orientation_error(self):
        return self.body_quat

    def get_velocity_error(self):
        return np.linalg.norm(self.global_linear_velocity) + np.linalg.norm(self.global_rotational_velocity)

    def get_acceleration_error(self):
        return np.linalg.norm(self.global_linear_acceleration) + np.linalg.norm(self.global_rotational_acceleration)

