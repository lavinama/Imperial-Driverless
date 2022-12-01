import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as im
import numpy as np
from YOLO_utility import detect_cones

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('cv')
        self.subscription = self.create_subscription(
            Image,
            '/fsds/camera/',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        """
        detect cones using YOLO
        Args:
            msg: cv2.Mat
        """
        array = np.asarray(msg)
        image = im.fromarray(msg)
        # Look at YOLO_utility.py/detect_cones function
        detect_cones(image)
