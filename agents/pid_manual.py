from time import sleep
from typing import Tuple

import numpy as np
from mujoco import MjData, mj_name2id, mjtObj, mj_id2name, viewer, mj_step, mj_resetData

from environments.mujoco_xmls import make_model


class PID:
    current_error = 0
    previous_error = 0
    integral_error = 0
    derivative_error = 0

    def __init__(self, kp, ki, kd, max_integral_error):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral_error = max_integral_error

    def update(self, error):
        self.current_error = error
        self.integral_error += error
        self.integral_error = np.clip(self.integral_error, -self.max_integral_error, self.max_integral_error)
        self.derivative_error = error - self.previous_error
        self.previous_error = error
        return self.kp * self.current_error + self.ki * self.integral_error + self.kd * self.derivative_error

    def reset(self):
        self.current_error = 0
        self.previous_error = 0
        self.integral_error = 0
        self.derivative_error = 0


class Propeller:
    def __init__(self, x, y, f):
        self.x = x
        self.y = y
        self.f = f


class ManualDrone:
    # region Initialization
    def __init__(self, roll_pid: PID, pitch_pid: PID, yaw_pid: PID, x_pid: PID, y_pid: PID, z_pid: PID,
                 max_roll: float, max_pitch: float, max_yaw: float):
        # region Initializing the Attributes
        index = 0
        self.model = make_model(1, [])
        self.data = MjData(self.model)
        drone_name = f"drone_{index}"
        self.drone_id = mj_name2id(self.model, mjtObj.mjOBJ_BODY, drone_name)
        self.propeller_body_ids = []
        self.propeller_masses = []
        self.motor_ids = []
        self.max_thrust = []
        self.max_torque = []
        self.drone_mass = 0
        self.I = np.zeros((3, 3))
        self.propellers = []
        self.A = np.zeros((6, len(self.propellers)))
        self.A_inv = np.zeros((len(self.propellers), 6))
        self.yaw_pid = yaw_pid
        self.pitch_pid = pitch_pid
        self.roll_pid = roll_pid
        self.x_pid = x_pid
        self.y_pid = y_pid
        self.z_pid = z_pid
        self.max_roll = max_roll
        self.max_pitch = max_pitch
        self.max_yaw = max_yaw
        # endregion

        # region Calling the Initialization Functions
        self.initialize_propellers(index)
        self.initialize_motors(index)
        self.initialize_mass_and_inertia()
        self.initialize_thrust_matrix()
        # endregion

    def initialize_propellers(self, index):
        propeller_body_names = [
            f"cw_front_left_{index}",
            f"ccw_back_left_{index}",
            f"cw_back_right_{index}",
            f"ccw_front_right_{index}",
        ]
        self.propeller_body_ids = [mj_name2id(self.model, mjtObj.mjOBJ_BODY, name) for name in propeller_body_names]
        self.propeller_masses = [self.model.body_mass[prop_id] for prop_id in self.propeller_body_ids]

    def initialize_motors(self, index):
        motor_names = [f"front_right_{index}", f"front_left_{index}", f"back_left_{index}", f"back_right_{index}"]
        self.motor_ids = [mj_name2id(self.model, mjtObj.mjOBJ_ACTUATOR, name) for name in motor_names]
        self.max_thrust = self.model.actuator_gear[self.motor_ids[:]][:, 2]
        self.max_torque = self.model.actuator_gear[self.motor_ids[:]][:, 5]

    def initialize_mass_and_inertia(self):
        self.drone_mass = self.model.body_mass[self.drone_id] + sum(self.propeller_masses)
        # dummy step to update the data; required to get the correct values
        mj_step(self.model, self.data)
        ixx, iyy, izz, ixy, ixz, iyz = self.data.cinert[self.drone_id][:6]
        self.I = np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz]
        ]).copy()  # Copy is necessary to avoid the changes in the original data
        mj_resetData(self.model, self.data)

    def initialize_thrust_matrix(self):
        f = self.max_torque / self.max_thrust
        self.propellers = [
            Propeller(
                *self.get_relative_position(self.drone_id, pid)[:2],  # Unpack the first two elements from the tuple
                f if "ccw" in mj_id2name(self.model, mjtObj.mjOBJ_BODY, pid) else -f
            ) for pid in self.propeller_body_ids
        ]

        self.A = np.array([
            [0 for _ in range(len(self.propellers))],
            [0 for _ in range(len(self.propellers))],
            [1 for _ in range(len(self.propellers))],
            [prop.y for prop in self.propellers],
            [prop.x for prop in self.propellers],
            [prop.f for prop in self.propellers]
        ])
        self.A_inv = np.linalg.pinv(self.A)
    # endregion

    # region Roll, Pitch, Yaw, Global and Relative Position
    @property
    def roll(self):
        quaternion = self.data.xquat[self.drone_id]
        return np.arctan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]),
                          1 - 2 * (quaternion[1] ** 2 + quaternion[2] ** 2))

    @property
    def pitch(self):
        quaternion = self.data.xquat[self.drone_id]
        return np.arcsin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))

    @property
    def yaw(self):
        quaternion = self.data.xquat[self.drone_id]
        return np.arctan2(2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
                          1 - 2 * (quaternion[2] ** 2 + quaternion[3] ** 2))

    @property
    def angles(self):
        return self.roll, self.pitch, self.yaw
    
    @property
    def position(self):
        return self.data.xpos[self.drone_id]

    def get_relative_position(self, parent_body_id: int, child_body_id: int) -> np.ndarray:
        parent_pos = self.data.xpos[parent_body_id]
        child_pos = self.data.xpos[child_body_id]

        parent_rotation_matrix = self.data.xmat[parent_body_id].reshape((3, 3))
        global_relative_position = child_pos - parent_pos

        return parent_rotation_matrix.T @ global_relative_position
    # endregion

    def thrust(self, global_linear_acceleration: np.ndarray,
               global_angular_acceleration: np.ndarray) -> np.ndarray:
        rotation_matrix = self.data.xmat[self.drone_id].reshape((3, 3))
        # Step 1
        mass_accel_product = self.drone_mass * global_linear_acceleration
        
        # Step 2
        inertia_accel_product = self.I @ global_angular_acceleration
        
        # Step 3
        rotated_mass_accel = rotation_matrix @ mass_accel_product
        rotated_inertia_accel = rotation_matrix @ inertia_accel_product
        
        # Step 4
        concatenated_array = np.concatenate([rotated_mass_accel, rotated_inertia_accel])
        
        # Step 5
        thrust_values = self.A_inv @ concatenated_array
        
        # Step 6
        normalized_thrust_values = np.divide(thrust_values, self.max_thrust)
        
        # Step 7
        clipped_thrust_values = np.clip(normalized_thrust_values, 0, 1)
        
        return clipped_thrust_values
    
    def desired_global_accelerations(self, desired_target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x_error, y_error, z_error = desired_target - self.position
        x_acceleration = self.x_pid.update(x_error)
        y_acceleration = self.y_pid.update(y_error)
        z_acceleration = self.z_pid.update(z_error)
        
        # Desired orientation is such that the drone's z-axis points towards the target. This will be the target roll,
        # pitch and yaw.
        
        desired_roll = np.arctan2(y_error, z_error)
        desired_pitch = np.arctan2(x_error, z_error)
        desired_yaw = 0
        
        roll_error = desired_roll - self.roll
        pitch_error = desired_pitch - self.pitch
        yaw_error = desired_yaw - self.yaw
        
        roll_acceleration = self.roll_pid.update(roll_error)
        pitch_acceleration = self.pitch_pid.update(pitch_error)
        yaw_acceleration = self.yaw_pid.update(yaw_error)
        
        return (np.array([x_acceleration, y_acceleration, z_acceleration]),
                np.array([roll_acceleration, pitch_acceleration, yaw_acceleration]))
    
    def step(self, desired_target: np.ndarray):
        thrusts = self.thrust(*self.desired_global_accelerations(desired_target))
        for motor_id, thrust in zip(self.motor_ids, thrusts):
            self.data.ctrl[motor_id] = thrust
            
            
class Simulation:
    def __init__(self, drone: ManualDrone):
        self.drone = drone
        self.timestep = drone.model.opt.timestep
        self.time = drone.data.time
        self.target = np.array([0, 0, 1])
        self.window = viewer.launch_passive(drone.model, drone.data)
        
    def step(self):
        self.drone.step(self.target)
        mj_step(self.drone.model, self.drone.data)
        self.window.sync()
        sleep(self.timestep)
        
    def close(self):
        self.window.close()
        
    def run(self, steps: int):
        for _ in range(steps):
            self.step()
            if not self.window.is_running():
                break
        self.close()
        
        
def test_pid():
    roll_pid = PID(10, 2, 5, 1.)
    pitch_pid = PID(10, 2, 5, 1.)
    yaw_pid = PID(10, 2, 5, 1.)
    x_pid = PID(10, 2, 5, 1.)
    y_pid = PID(10, 2, 5, 1.)
    z_pid = PID(10, 2, 5, 1.)
    
    drone = ManualDrone(roll_pid, pitch_pid, yaw_pid, x_pid, y_pid, z_pid, 0.1, 0.1, 0.1)
    simulation = Simulation(drone)
    simulation.run(10000)
    simulation.close()
    
def test_dummy():
    model = make_model(1, [])
    data = MjData(model)
    data.ctrl[:] = .2
    viewer.launch(model, data)
        
if __name__ == "__main__":
    # test_dummy()
    test_pid()
    
