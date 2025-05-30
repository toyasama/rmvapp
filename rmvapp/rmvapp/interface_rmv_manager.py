from abc import abstractmethod
import numpy as np
from rmv_library.parameters.frame_parameter import FramesParameters
from rmv_library.parameters.visualization_parameter import VisualizationParameters


class IRmvException(Exception):
    """
    Exception class for RMV Manager.
    """

    pass


class IRmvManager:
    """
    Interface for RMV Manager.
    """

    def __init__(self):
        """
        Initialize the RMV Manager interface.
        """
        self._image: np.ndarray | None = None
        self._frame_params: FramesParameters = FramesParameters()
        self._visualization_params: VisualizationParameters = VisualizationParameters()

    @abstractmethod
    def get_image(self) -> None | np.ndarray:
        """
        Get the image from the RMV Manager.

        Returns:
            Image: The image from the RMV Manager.
        """
        raise IRmvException("get_image not implemented")

    @abstractmethod
    def set_image(self, image: np.ndarray) -> None:
        """
        Set the image for the RMV Manager.
        Args:
            image (Image): The image to set.
        """
        raise IRmvException("set_image not implemented")

    @abstractmethod
    def reset_image(self) -> None:
        """
        Reset the image in the RMV Manager.
        """
        raise IRmvException("reset_image not implemented")

    @abstractmethod
    def set_params(self, params):
        """
        Set the parameters for the RMV Manager.

        Args:
            params (dict): The parameters to set.
        """
        raise IRmvException("set_params not implemented")

    @abstractmethod
    def get_params(self) -> dict:
        """
        Get the parameters from the RMV Manager.

        Returns:
            dict: The parameters from the RMV Manager.
        """
        raise IRmvException("get_params not implemented")
