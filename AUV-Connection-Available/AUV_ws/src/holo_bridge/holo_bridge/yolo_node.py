import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class AUVVisionNode(Node):
    def __init__(self):
        super().__init__('auv_vision_node')
        
        # 1. 订阅 HoloOcean 的原始图像
        self.subscription = self.create_subscription(
            Image,
            '/auv/camera/image_raw',
            self.image_callback,
            10)
            
        # 2. 发布处理后的识别画面
        self.publisher_ = self.create_publisher(Image, '/auv/camera/yolo_result', 10)
        
        self.bridge = CvBridge()
        
        # 3. 加载模型 (yolov8n 是速度最快的版本，非常适合仿真实时处理)
        # 第一次运行会自动下载 yolov8n.pt 权重文件
        self.get_logger().info('正在初始化 YOLOv8...')
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info('视觉节点就绪！等待画面流入...')

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 执行推理
        # task='detect' 表示目标检测，stream=False 表示单帧处理
        results = self.model(frame, verbose=False)

        # 在图像上绘制识别结果（框和标签）
        annotated_frame = results[0].plot()

        # 将处理后的画面转回 ROS 消息并发布
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.publisher_.publish(out_msg)

def main():
    rclpy.init()
    node = AUVVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()