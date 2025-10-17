import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

class FaceDetectionNode(Node):
    def __init__(self, node_name):
        super()