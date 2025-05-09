from .interface_rmv_manager import IRmvManager
from sensor_msgs.msg import Image
from threading import RLock
from rclpy.qos import qos_profile_sensor_data
from copy import deepcopy
from numpy import array
import rclpy
import numpy as np
from cv_bridge import CvBridge


class RmvManager(IRmvManager):
    """
    RMV Manager class that implements the IRmvManager interface.
    """

    def __init__(self, node):
        """
        Initialize the RMV Manager.
        """
        super().__init__()
        self.node: rclpy.Node = node
        self.image: array = None
        self.params = {}
        self.image_lock = RLock()
        self.bridge = CvBridge()
        self.image_sub = self.node.create_subscription(
            Image,
            "/rmv/image",
            self._update_image,
            qos_profile_sensor_data,
        )
        print("RMV Manager initialized")

    def _update_image(self, image: Image):
        print("Image received")
        with self.image_lock:
            if image is not None:
                self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            else:
                self.image = None

    def get_image(self):
        with self.image_lock:
            if self.image is None:
                return None
            else:
                image = deepcopy(self.image)
                self.image = None
                return image

    def set_params(self, params):
        self.params = params
