from typing import List, Tuple, Dict

class ObjectXML(object):
    """
    A class to represent an object in a MuJoCo scene using XML.

    Attributes:
        name (str): Name of the object.
        geom_type (str): The geometric type of the object (e.g., 'box').
        position (Tuple[float, float, float]): The position of the object in the scene.
        quat (Tuple[float, float, float, float]): Quaternion representing the object's orientation.
        mass (float): Mass of the object.
        dimensions (List[float]): Dimensions of the object, specific to its geometric type.
        other (Dict[str, str]): Additional attributes for the object's geometry.
        
    Methods:
        set_position(position: Tuple[float, float, float]): Sets the object's position.
        set_quat(quat: Tuple[float, float, float, float]): Sets the object's orientation using a quaternion.
        set_mass(mass: float): Sets the object's mass.
        set_dimensions(dimensions: List[float]): Sets the object's dimensions.
        set_other(other: Dict[str, str]): Sets additional geometry attributes.
        _generate_xml(): Generates the object's XML representation.
        xml: Public method to get the object's XML representation.
        valid: Checks if the generated XML is valid (does not contain placeholders).
    """
    def __init__(self, name: str, fixed: bool = False, geom_type: str = "box"):
        self.name = name
        self.base_xml = "<body name=\"{{name}}\"> {{FREEJOINT}} {{GEOM}} </body>"
        self.base_xml = self.base_xml.replace(
            "{{FREEJOINT}}", "<freejoint name=\"{{name}}\"/>" if not fixed else ""
        )
        self.geom_type = geom_type
        self.geom_xml = "<geom type=\"{{type}}\" size=\"{{size}}\" mass=\"{{mass}}\" group=\"3\" {{other}} />"
        self.position = (0, 0, 0)
        self.quat = (1, 0, 0, 0)
        self.mass = 1.0
        self.dimensions: List[float] = []
        self.other: Dict[str, str] = {}

    def set_position(self, position: Tuple[float, float, float]):
        """
        Sets the object's position.

        :param position: A 3-tuple representing the object's position in the scene.
        """
        self.position = position

    def set_quat(self, quat: Tuple[float, float, float, float]):
        """
        Sets the object's orientation using a quaternion.

        :param quat: A 4-tuple representing the object's orientation.
        """
        self.quat = quat

    def set_mass(self, mass: float):
        """
        Sets the object's mass.

        :param mass: A float representing the object's mass.
        """
        self.mass = mass

    def set_dimensions(self, dimensions: List[float]):
        """
        Sets the object's dimensions.

        :param dimensions: A list of floats representing the object's dimensions.
        """
        self.dimensions = dimensions

    def set_other(self, other: Dict[str, str]):
        """
        Sets additional geometry attributes.

        :param other: A dictionary of additional attributes for the object's geometry.
        """
        self.other = other

    def _generate_xml(self) -> str:
        """
        Generates and returns the object's XML representation.

        :return: A string representing the object's XML.
        """
        xml = self.geom_xml.replace("{{type}}", self.geom_type)
        xml = xml.replace("{{size}}", " ".join(map(str, self.dimensions)))
        xml = xml.replace("{{mass}}", str(self.mass))
        xml = xml.replace("{{other}}", " ".join([f'{k}="{v}"' for k, v in self.other.items()]))
        return self.base_xml.replace("{{name}}", self.name).replace("{{GEOM}}", xml)

    @property
    def valid(self) -> bool:
        """
        Checks if the generated XML is valid (does not contain placeholders).

        :return: True if the XML is valid, False otherwise.
        """
        return "{{" not in self._generate_xml()

    @property
    def xml(self) -> str:
        """
        Public method to get the object's XML representation.

        :return: A string representing the object's XML.
        """
        return self._generate_xml()
