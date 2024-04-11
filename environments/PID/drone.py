from mujoco import MjModel, MjData, mj_name2id, mjtSensor
from mujoco._structs import _MjDataSensorViews, _MjModelSensorViews

class Drone:
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
        self.body_to_propeller = 0.04646   # meters, calculated from OnShape model, both x and y directions are the same
        
        

