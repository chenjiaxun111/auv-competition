#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 引入多线程执行器
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch

class YoloTargetProposer(Node):
    def __init__(self):
        super().__init__('yolo_target_proposer')
        
        # 1. 订阅原始图像
        self.subscription = self.create_subscription(
            Image, '/auv/camera/image_raw', self.image_callback, 10)
            
        # 2. 发布带框图像
        self.publisher_ = self.create_publisher(Image, '/auv/camera/yolo_result', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        
        # 3. 初始化 YOLOv8 并强制搬运到 5090 显存
        self.get_logger().info('正在将视觉大脑加载到 RTX 5090...')
        self.model = YOLO('yolov8n.pt').to('cuda') 
        
        # 打印设备确认
        self.get_logger().info(f"成功！YOLO 正在运行设备: {self.model.device}")
        
        self.fx, self.fy = 320.0, 320.0
        self.cx, self.cy = 320.0, 240.0
        self.fixed_depth = 5.0 

    def image_callback(self, msg):
        # A. 转换图像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # B. 5090 极速推理
        # 使用 half=True 开启半精度，能显著降低延迟
        results = self.model(frame, 
                             conf=0.3, 
                             half=True, 
                             verbose=False)
        
        # C. 可视化发布
        annotated_frame = results[0].plot()
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.publisher_.publish(out_msg)

        # D. 坐标转换与 TF 发布
        boxes = results[0].boxes
        if len(boxes) > 0:
            box = boxes[0]
            u, v, w, h = box.xywh[0]
            
            z_cam = self.fixed_depth 
            x_cam = (u.item() - self.cx) * z_cam / self.fx
            y_cam = (v.item() - self.cy) * z_cam / self.fy
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'detected_target'

            t.transform.translation.x = float(z_cam)
            t.transform.translation.y = float(-x_cam)
            t.transform.translation.z = float(-y_cam)
            t.transform.rotation.w = 1.0 

            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = YoloTargetProposer()
    
    # 针对 5090 的多线程优化，让数据处理更顺畅
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()