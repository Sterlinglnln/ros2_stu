import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import cv2
from cv_bridge import CvBridge

class FaceDetectorClient(Node):
    def __init__(self):
        super().__init__('face_detector_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.face_test_image_path = get_package_share_directory(
            'demo_python_service') + '/resource/face_test.png'
        self.image = cv2.imread(self.face_test_image_path)
        
    def send_request(self):
        # 1. 等待服务上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待人脸检测服务端上线...')
            
        # 2. 创建请求对象并赋值
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        
        # 3. 发送异步请求
        future = self.client.call_async(request)
        def request_callback(result_future):
            response = result_future.result()
            self.get_logger().info(f'检测到的人脸数量：{response.number}, 总计时：{response.use_time}')
            # self.show_face_locations(response)
        future.add_done_callback(request_callback)

    def call_set_parameters(self, parameters):
        # 1. 创建一个客户端，并等待服务上线
        client = self.create_client(SetParameters, '/face_detection_node/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待参数设置服务端上线...')
        # 2. 创建请求对象
        request = SetParameters.Request()
        request.parameters = parameters
        # 3. 异步调用、等待并获取结果
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    
    def update_detect_model(self, model):
        # 1. 创建参数对象
        param = Parameter()
        param.name = 'face_locations_model'
        # 2. 创建参数值对象并赋值
        new_model_value = ParameterValue()
        new_model_value.type = ParameterType.PARAMETER_STRING
        new_model_value.string_value = model
        param.value = new_model_value
        # 3. 请求更新参数并处理
        response = self.call_set_parameters([param])
        for result in response.results:
            if result.successful:
                self.get_logger().info(f'参数 {param.name} 设置为 {model}')
            else:
                self.get_logger().info(f'参数设置失败，原因为：{result.reason}')

def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectorClient()
    face_detect_client.update_detect_model('hog')
    face_detect_client.send_request()
    face_detect_client.update_detect_model('cnn')
    face_detect_client.send_request()
    rclpy.spin(face_detect_client)
    rclpy.shutdown()
