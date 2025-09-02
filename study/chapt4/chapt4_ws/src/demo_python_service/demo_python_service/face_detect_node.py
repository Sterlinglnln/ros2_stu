import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import face_recognition
import time

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge = CvBridge()
        self.service = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback)
        self.default_image_path = get_package_share_directory('demo_python_service') + '/resource/face_test.png'
        self.upsample_times = 1
        self.model = "hog"
        # 声明和获取参数
        self.declare_parameter('face_locations_upsample_times', 1)
        self.declare_parameter('face_locations_model', "hog")
        self.model = self.get_parameter("face_locations_model").value
        self.upsample_times = self.get_parameter("face_locations_upsample_times").value
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f'参数 {parameter.name} 设置为：{parameter.value}')
            if parameter.name == 'face_locations_upsample_times':
                self.upsample_times = parameter.value
            if parameter.name == 'face_locations_model':
                self.model = parameter.value
        return SetParametersResult(successful=True)
        
    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
        start_time = time.time()
        self.get_logger().info('开始人脸检测...')
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.upsample_times, model=self.model)
        end_time = time.time()
        self.get_logger().info(f'人脸检测总计时 {end_time - start_time:.2f} 秒.')
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()