import os
import tempfile
import xml.dom.minidom
from typing import List

from mujoco import MjModel

from environments.mujoco_xmls._drone import DroneXML
from environments.mujoco_xmls._object import ObjectXML


class SceneXML(object):
    """
    A class to represent a Scene in MuJoCo XML format.

    Attributes:
        temp_dir (TemporaryDirectory): A temporary directory where scene files are stored.
        drones (List[DroneXML]): A list of DroneXML objects representing drones in the scene.
        objects (List[ObjectXML]): A list of ObjectXML objects representing objects in the scene.
        file (str): Path to the scene XML file within the temporary directory.

    Methods:
        set_drones(drones: List[DroneXML]): Sets the drones in the scene, ensuring they are valid.
        set_objects(objects: List[ObjectXML]): Sets the objects in the scene, ensuring they are valid.
        set_scene(): Updates the scene XML with current drones and objects, then writes to file.
    """
    def __init__(self):
        """
        Initializes the SceneXML object by reading the scene XML file and creating a temporary directory.
        
        The scene XML file is read and stored in the `_xml` attribute. The temporary directory is created
        using the `tempfile.TemporaryDirectory` class and stored in the `temp_dir` attribute.
        """
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        with open("scene.xml") as f:
            self._xml = f.read()
        self.temp_dir = tempfile.TemporaryDirectory()
        self.drones: List[DroneXML] = []
        self.objects: List[ObjectXML] = []
        self.file = os.path.join(self.temp_dir.name, "scene.xml")
    
    def _copy_assets(self):
        """
        Copies assets from the `assets` directory to the temporary directory.
        :return:
        """
        os.makedirs(os.path.join(self.temp_dir.name, "assets"))
        for asset in os.listdir("assets"):
            with open(os.path.join("assets", asset), "rb") as f:
                with open(os.path.join(self.temp_dir.name, "assets", asset), "wb") as g:
                    g.write(f.read())
    
    def set_drones(self, drones: List[DroneXML]):
        """
        Sets the drones in the scene, ensuring they are valid.
        :param drones: A list of DroneXML objects representing drones in the scene.
        :return:
        """
        assert all([drone.valid for drone in drones]), "All drones must be valid."
        # Save each mujoco_xmls to a temporary file in the temporary directory
        for drone in drones:
            with open(os.path.join(self.temp_dir.name, f"drone_{drone.index}.xml"), "w") as f:
                f.write(drone.xml)
        self.drones = drones
    
    def set_objects(self, objects: List[ObjectXML]):
        """
        Sets the objects in the scene, ensuring they are valid.
        :param objects:
        :return:
        """
        assert all([obj.valid for obj in objects]), "All objects must be valid."
        self.objects = objects
    
    def set_scene(self):
        """
        Updates the scene XML with current drones and objects, then writes to file.
        :return:
        """
        assert self.drones, "No drones have been set."
        # replace the "{{files}}" placeholder with the mujoco_xmls files
        self._xml = self._xml.replace(
            "{{files}}",
            "\n\t".join([f'<include file="drone_{drone.index}.xml"/>' for drone in self.drones])
        )
        # replace the "{{objects}}" placeholder with the object XML
        
        
        self._xml = self._xml.replace(
            "{{objects}}",
            "\n\t".join([obj.xml for obj in self.objects])
        )
        dom = xml.dom.minidom.parseString(self._xml)
        pretty_xml = dom.toprettyxml()
        
        with open(os.path.join(self.temp_dir.name, "scene.xml"), "w") as f:
            f.write(pretty_xml)
            
        self._copy_assets()
        
    @property
    def mujoco_model(self):
        """
        Returns the MuJoCo model object created from the scene XML file.
        :return:
        """
        return MjModel.from_xml_path(self.file)
