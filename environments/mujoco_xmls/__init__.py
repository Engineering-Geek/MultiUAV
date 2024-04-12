import logging
import os
from typing import List

from environments.mujoco_xmls._drone import DroneXML
from environments.mujoco_xmls._scene import SceneXML
from environments.mujoco_xmls._object import ObjectXML

from mujoco import MjModel, mj_saveLastXML

def urdf_to_xml(urdf_file: str, xml_file: str):
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    # Sanity checks
    if not os.path.isfile(urdf_file):
        logging.error(f"URDF file {urdf_file} does not exist.")
        raise FileNotFoundError(f"URDF file {urdf_file} does not exist.")
    if not os.access(urdf_file, os.R_OK):
        logging.error(f"URDF file {urdf_file} is not readable.")
        raise PermissionError(f"URDF file {urdf_file} is not readable.")

    try:
        # Convert URDF to XML
        mj_saveLastXML(filename=xml_file, m=MjModel.from_xml_path(urdf_file))
        logging.info(f"Converted {urdf_file} to {xml_file} successfully.")
    except Exception as e:
        logging.error(f"Failed to convert {urdf_file} to {xml_file}. Error: {str(e)}")
        raise e


def make_model(n_drones: int, objects: List[ObjectXML]) -> MjModel:
    scene = SceneXML()
    drones = [DroneXML(i) for i in range(n_drones)]
    positions = [(i / 2., 0, 0.025) for i in range(n_drones)]
    drone_rgba = (0.3, 0.7, 0.4, 1.0)
    prop_rgba = (0.5, 0.5, 0.5, 1.0)
    for i, drone in enumerate(drones):
        drone.set_position(positions[i])
        drone.set_drone_rgba(drone_rgba)
        drone.set_propeller_rgba(prop_rgba)
        drone.set_fovy(60)
        drone.set_gyro(0.001, 30)
        drone.set_accel(0.001, 30)
    
    scene.set_drones(drones)
    scene.set_objects(objects)
    scene.set_scene()
    
    # log the contents of the scene file
    with open(scene.file, "r") as f:
        logging.info(f"Scene file contents:\n{f.read()}")
    # log the contents of the mujoco_xmls files
    for drone in drones:
        with open(os.path.join(scene.temp_dir.name, f"drone_{drone.index}.xml"), "r") as f:
            logging.info(f"Drone {drone.index} file contents:\n{f.read()}")
    return MjModel.from_xml_path(scene.file)
    

if __name__ == "__main__":
    # Convert URDF to XML
    urdf_to_xml(
        urdf_file="assets/robot.urdf",
        xml_file="drone.xml"
    )

