import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import socket
import json
import threading


class GazeboPathReceiver(Node):
    def __init__(self):
        super().__init__('gazebo_path_receiver')
        self.start_pub = self.create_publisher(PoseStamped, 'start_pose', 10)
        self.end_pub = self.create_publisher(PoseStamped, 'end_pose', 10)
        self.path_pub = self.create_publisher(PoseStamped, 'path_point', 10)

        self.server_thread = threading.Thread(target=self.run_socket_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        self.get_logger().info("🚀 Gazebo 좌표 수신 서버 실행 중 (포트 9999)")

    def run_socket_server(self):
        HOST = 'localhost'
        PORT = 9999

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            while True:
                conn, _ = s.accept()
                with conn:
                    data = conn.recv(4096).decode()
                    try:
                        coords = json.loads(data)
                        self.publish_pose(coords)
                    except Exception as e:
                        self.get_logger().error(f"❌ JSON 처리 오류: {e}")

    def publish_pose(self, data):
        def make_pose(x, y):
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.w = 1.0
            return msg

        # Start
        start = data.get("start")
        if start:
            self.start_pub.publish(make_pose(start['x'], start['y']))
            self.get_logger().info(f"✅ Start published: {start}")

        # End
        end = data.get("end")
        if end:
            self.end_pub.publish(make_pose(end['x'], end['y']))
            self.get_logger().info(f"✅ End published: {end}")

        # Path
        path = data.get("path", [])
        for i, pt in enumerate(path):
            self.path_pub.publish(make_pose(pt['x'], pt['y']))
            self.get_logger().info(f"📍 Path point {i+1}/{len(path)} published")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPathReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
