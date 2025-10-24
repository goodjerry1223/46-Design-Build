#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FixedFramePublisher(Node):
    def __init__(self):
        super().__init__('frame_transformer')
        self.transform_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_frame_transform)
        self.get_logger().info("Broadcasting fixed transform between base_link and laser_link")

    def broadcast_frame_transform(self):
        # 创建并填充变换消息
        frame_transform = TransformStamped()
        
        # 设置时间戳和坐标系
        frame_transform.header.stamp = self.get_clock().now().to_msg()
        frame_transform.header.frame_id = 'base_link'
        frame_transform.child_frame_id = 'laser_link'
        
        # 设置位置偏移
        frame_transform.transform.translation.x = 0.0
        frame_transform.transform.translation.y = 0.0
        frame_transform.transform.translation.z = 0.0
        
        # 设置方向 (单位四元数表示无旋转)
        frame_transform.transform.rotation.x = 0.0
        frame_transform.transform.rotation.y = 0.0
        frame_transform.transform.rotation.z = 0.0
        frame_transform.transform.rotation.w = 1.0
        
        # 广播变换
        self.transform_broadcaster.sendTransform(frame_transform)

def main(args=None):
    rclpy.init(args=args)
    transformer = FixedFramePublisher()
    
    try:
        rclpy.spin(transformer)
    except KeyboardInterrupt:
        transformer.get_logger().info("Transform publisher shutting down")
    finally:
        transformer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()