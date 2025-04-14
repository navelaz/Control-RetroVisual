import rclpy
from rclpy.node import Node
import math
import time
import sys

sys.path.append('/home/navelaz/ros2_ws/src/Coppelia/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class SphereMover(Node):
    def __init__(self):
        super().__init__('sphere_mover')
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sphere = self.sim.getObject('/Sphere')
        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.move_sphere)

    def move_sphere(self):
        t = time.time() - self.start_time
        x = 1.0 * math.sin(0.5 * t)
        y = 0.0
        z = 0.1
        self.sim.setObjectPosition(self.sphere, -1, [x, y, z])

def main(args=None):
    rclpy.init(args=args)
    node = SphereMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
