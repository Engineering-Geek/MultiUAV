import numpy as np
from mujoco import MjModel, MjData, mj_name2id, mjtSensor


class BasePIDDrone:
    def __init__(self, index: int, model: MjModel, data: MjData):
        self.index = index
        self.model = model
        self.data = data

        gyro_name = f"gyro_{index}"
        accel_name = f"accel_{index}"
        self.gyro_id: int = mj_name2id(model, mjtSensor.mjSENS_GYRO, gyro_name)
        self.accel_id: int = mj_name2id(model, mjtSensor.mjSENS_ACCELEROMETER, accel_name)

        body_name = f"drone_{index}"
        self.body_id: int = mj_name2id(model, mjtSensor.mjOBJ_BODY, body_name)

        sample_prop_body_name = f"ccw_front_right_{index}"
        self.sample_prop_body_id: int = mj_name2id(model, mjtSensor.mjOBJ_BODY, sample_prop_body_name)

        self.gyro_data = data.sensordata[self.gyro_id]
        self.accel_data = data.sensordata[self.accel_id]
        self.body_pos = data.xpos[self.body_id]
        self.body_quat = data.xquat[self.body_id]
        self.body_vel = data.cvel[self.body_id][:3]

        # Note that all propellers are identical and have the same distance from the body center.
        self.body_to_propeller = 0.04646  # meters, calculated from OnShape model, both x and y directions are the same

        self.rotation_matrix = self.data.xmat[self.body_id].reshape(3, 3)
        mass_matrix_upper_triangle = self.data.cinert[self.body_id][:5]
        self.inertia_matrix = np.array([
            [mass_matrix_upper_triangle[0], mass_matrix_upper_triangle[1], mass_matrix_upper_triangle[2]],
            [mass_matrix_upper_triangle[1], mass_matrix_upper_triangle[3], mass_matrix_upper_triangle[4]],
            [mass_matrix_upper_triangle[2], mass_matrix_upper_triangle[4], mass_matrix_upper_triangle[5]]
        ])
        self.global_linear_acceleration = self.data.cacc[self.body_id][:3]
        self.global_rotational_acceleration = self.data.cacc[self.body_id][3:]
        self.global_linear_velocity = self.data.cvel[self.body_id][:3]
        self.global_rotational_velocity = self.data.cvel[self.body_id][3:]

        self.local_linear_acceleration = np.dot(self.rotation_matrix.T, self.global_linear_acceleration)
        self.local_rotational_acceleration = np.dot(self.rotation_matrix.T, self.global_rotational_acceleration)
        self.local_linear_velocity = np.dot(self.rotation_matrix.T, self.global_linear_velocity)
        self.local_rotational_velocity = np.dot(self.rotation_matrix.T, self.global_rotational_velocity)

        self._propeller_force = np.zeros(4)

        self.pitch = np.arcsin(2 * (self.body_quat[1] * self.body_quat[3] - self.body_quat[0] * self.body_quat[2]))
        self.roll = np.arctan2(2 * (self.body_quat[0] * self.body_quat[1] + self.body_quat[2] * self.body_quat[3]),
                               1 - 2 * (self.body_quat[1] ** 2 + self.body_quat[2] ** 2))
        self.yaw = np.arctan2(2 * (self.body_quat[0] * self.body_quat[3] + self.body_quat[1] * self.body_quat[2]),
                              1 - 2 * (self.body_quat[2] ** 2 + self.body_quat[3] ** 2))

    @property
    def propeller_force(self):
        return self._propeller_force

    @propeller_force.setter
    def propeller_force(self, value):
        self._propeller_force = np.clip(value, 0, 1)
        self.data.ctrl[self.index * 4: (self.index + 1) * 4] = value