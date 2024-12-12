class SampleClass:
    """A sample class to check how they can be imported by other ROS2 packages."""

    def __init__(self, name: str):
        self._name = name

    def get_name(self) -> str:
        """
        Gets the name of this instance.
        :return: This name.
        """
        return self._name