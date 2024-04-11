import os
from typing import Tuple


class DroneXML(object):
    """
    A class to represent a drone in MuJoCo XML format.

    Attributes:
        index (int): An index to uniquely identify the drone.
        xml (str): The XML content of the drone.

    Methods:
        set_position(position: Tuple[float, float, float]): Sets the drone's position.
        set_drone_rgba(rgba: Tuple[float, float, float, float]): Sets the drone's RGBA color.
        set_propeller_rgba(rgba: Tuple[float, float, float, float]): Sets the propeller's RGBA color.
        set_fovy(fovy: float): Sets the field of view in the y-direction.
        set_gyro(noise: float, cutoff: float): Sets the gyro's noise and cutoff frequency.
        set_accel(noise: float, cutoff: float): Sets the accelerometer's noise and cutoff frequency.
    """
    def __init__(self, index: int):
        """
        Initializes the DroneXML object by reading the drone XML file and setting the index.
        
        The drone XML file is read and stored in the `_xml` attribute. The index is set to the provided value.
        
        :param index: An index to uniquely identify the drone.
        """
        os.chdir(os.path.dirname(os.path.abspath(__file__)))
        with open("drone.xml") as f:
            self._xml = f.read()
        self._set_index(index)
        self.index = index
            
    def set_position(self, position: Tuple[float, float, float]):
        """
        Sets the drone's position.
        :param position: A 3-tuple representing the drone's position.
        :return:
        """
        assert len(position) == 3, "Position must be a 3-tuple."
        self._xml = self._xml.replace(
            "{{x}} {{y}} {{z}}",
            f"{position[0]} {position[1]} {position[2]}"
        )
    
    def set_drone_rgba(self, rgba: Tuple[float, float, float, float]):
        """
        Sets the drone's RGBA color.
        :param rgba: A 4-tuple representing the drone's RGBA color.
        :return:
        """
        assert len(rgba) == 4, "RGBA must be a 4-tuple."
        self._xml = self._xml.replace(
            "{{drone_rgba}}",
            f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
        )
        
    def set_propeller_rgba(self, rgba: Tuple[float, float, float, float]):
        """
        Sets the propeller's RGBA color.
        :param rgba: A 4-tuple representing the propeller's RGBA color.
        :return:
        """
        assert len(rgba) == 4, "RGBA must be a 4-tuple."
        self._xml = self._xml.replace(
            "{{prop_rgba}}",
            f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
        )
        
    def _set_index(self, index: int):
        """
        Sets the drone's index.
        :param index: An index to uniquely identify the drone.
        :return:
        """
        self._xml = self._xml.replace(
            "{{index}}",
            str(index)
        )
    
    def set_fovy(self, fovy: float):
        """
        Sets the field of view in the y-direction. The x-direction field of view is calculated based on the aspect ratio.
        :param fovy: The field of view in the y-direction.
        :return:
        """
        self._xml = self._xml.replace(
            "{{fovy}}",
            str(fovy)
        )
        
    def set_gyro(self, noise: float, cutoff: float):
        """
        Sets the gyro's noise and cutoff frequency.
        :param noise: Gaussian noise in the gyro measurements.
        :param cutoff: Cutoff magnitude for the gyro measurements.
        :return:
        """
        self._xml = self._xml.replace(
            "{{gyro_noise}}",
            str(noise)
        )
        self._xml = self._xml.replace(
            "{{gyro_cutoff}}",
            str(cutoff)
        )
        
    def set_accel(self, noise: float, cutoff: float):
        """
        Sets the accelerometer's noise and cutoff frequency.
        :param noise: Gaussian noise in the accelerometer measurements.
        :param cutoff: Cutoff magnitude for the accelerometer measurements.
        :return:
        """
        self._xml = self._xml.replace(
            "{{accel_noise}}",
            str(noise)
        )
        self._xml = self._xml.replace(
            "{{accel_cutoff}}",
            str(cutoff)
        )
    
    @property
    def valid(self) -> bool:
        """
        Checks if the drone XML is valid.
        :return: True if the XML does not contain placeholders, False otherwise.
        """
        return "{{" not in self._xml
    
    @property
    def xml(self) -> str:
        """
        Returns the drone XML content.
        :return: The drone XML content.
        """
        return self._xml

