from .interface_rmv_manager import IRmvManager
from sensor_msgs.msg import Image
from threading import RLock
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from copy import deepcopy
import numpy as np
import numpy as np
from cv_bridge import CvBridge
from rmv_msgs.srv import GetParams
import json
from typing import Any


class RmvManager(IRmvManager):
    """
    RMV Manager class that implements the IRmvManager interface.
    """

    def __init__(self, node: Node):
        """
        Initialize the RMV Manager.
        """
        super().__init__()
        self._params: dict = {}
        self.image_lock = RLock()
        self.bridge = CvBridge()
        self.image_sub = node.create_subscription(
            Image,
            "/rmv/image",
            self._update_image,
            qos_profile_sensor_data,
        )
        self.get_params_srv_name: str = "/rmv/get_params"
        self._client = node.create_client(GetParams, self.get_params_srv_name)
        self._upload_params()
        print("RMV Manager initialized")

    def get_image(self):
        with self.image_lock:
            if self._image is None:
                return None
            else:
                image = deepcopy(self._image)
                self._image = None
                return image

    def set_image(self, image: np.ndarray):
        with self.image_lock:
            self._image = image

    def reset_image(self):
        with self.image_lock:
            self._image = None

    def _update_image(self, image: Image):
        """
        Update the image in the RMV Manager.
        Args:
            image (Image): The image to update.
        """
        if image is not None:
            self.set_image(self.bridge.imgmsg_to_cv2(image, "bgr8"))
        else:
            self.reset_image()

    def set_params(self, params: dict):
        self._params = params

    def get_params(self) -> dict:
        return self._params

    def _upload_params(self):
        if not self._client.wait_for_service(1):
            raise RuntimeError(f"Service {self.get_params_srv_name} not available")
        request = GetParams.Request()
        response = self._client.call(request)
        self._params = json.loads(response.dict_as_string)
        self._update_params(self._params)

    def _update_params(self, data: dict[str, Any]) -> None:
        """Update parameters from a dictionary."""
        self._update_visualization(data)
        self._update_frames(data)

    def _update_visualization(self, data: dict[str, Any]) -> None:
        """Update visualization parameters from dictionary."""
        self._visualization_params.updateFromDict(data)

    def _update_frames(self, data: dict[str, Any]) -> None:
        """Update frames parameters from dictionary."""
        self._frame_params.updateFromDict(data)
