from abc import ABC, abstractmethod
import numpy as np


class IRmvManager:
    """
    Interface for RMV Manager.
    """

    def __init__(self):
        """
        Initialize the RMV Manager interface.
        """
        pass

    @abstractmethod
    def _update_image(self, image) -> None:
        """
        Update the image in the RMV Manager.
        Args:
            image (Image): The image to update.
        """
        pass

    @abstractmethod
    def get_image(self) -> None | np.ndarray:
        """
        Get the image from the RMV Manager.

        Returns:
            Image: The image from the RMV Manager.
        """
        pass

    @abstractmethod
    def set_params(self, params):
        """
        Set the parameters for the RMV Manager.

        Args:
            params (dict): The parameters to set.
        """
        pass

    @abstractmethod
    def get_params(self):
        """
        Get the parameters from the RMV Manager.

        Returns:
            dict: The parameters from the RMV Manager.
        """
        pass
