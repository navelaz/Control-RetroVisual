import rclpy
from rclpy.node import Node
import math
import sys

# Ruta al ZMQ API
sys.path.append('/home/navelaz/ros2_ws/src/Coppelia/programming/zmqRemoteApi/clients/python/src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class PioneerFollower(Node):
    def __init__(self):
        super().__init__('pioneer_follower')

        # Conexión con CoppeliaSim
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        # Obtener objetos
        self.sphere = self.sim.getObject('/Sphere')
        self.robot = self.sim.getObject('/PioneerP3DX')
        self.left_motor = self.sim.getObject('/PioneerP3DX/leftMotor')
        self.right_motor = self.sim.getObject('/PioneerP3DX/rightMotor')

        # Parámetros del controlador PID
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.2
        self.integral = 0.0
        self.last_error = 0.0

        # Parámetros físicos
        self.wheel_distance = 0.381
        self.max_speed = 2.0

        # Radios estimados para detección de colisión
        self.robot_radius = 0.3
        self.sphere_radius = 0.15

        # Timer principal
        self.timer = self.create_timer(0.05, self.control_loop)

    def pid(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

    def control_loop(self):
        dt = 0.05

        # Posiciones y orientación
        sphere_pos = self.sim.getObjectPosition(self.sphere, -1)
        robot_pos = self.sim.getObjectPosition(self.robot, -1)
        robot_ori = self.sim.getObjectOrientation(self.robot, -1)

        dx = sphere_pos[0] - robot_pos[0]
        dy = sphere_pos[1] - robot_pos[1]
        distance = math.hypot(dx, dy)

        # Ángulo hacia la esfera
        angle_to_target = math.atan2(dy, dx)
        robot_theta = robot_ori[2]
        error_theta = (angle_to_target - robot_theta + math.pi) % (2 * math.pi) - math.pi

        # Umbral de seguridad
        collision_threshold = self.robot_radius + self.sphere_radius + 0.25

        # Comportamiento reactivo
        if distance < collision_threshold:
            # Muy cerca: retrocede y gira más fuerte
            v = -0.4
            w = 1.0 if error_theta > 0 else -1.0
        elif distance < (collision_threshold + 0.2):
            # Casi cerca: desacelera y ajusta rumbo con PID
            v = 0.1
            w = self.pid(error_theta, dt)
        else:
            # Distancia segura: sigue a la esfera normalmente
            v = min(0.5 * distance, self.max_speed)
            w = self.pid(error_theta, dt)

        # Cinemática diferencial
        vL = v - w * self.wheel_distance / 2
        vR = v + w * self.wheel_distance / 2

        # Limitar velocidades
        vL = max(-self.max_speed, min(self.max_speed, vL))
        vR = max(-self.max_speed, min(self.max_speed, vR))

        # Enviar velocidades
        self.sim.setJointTargetVelocity(self.left_motor, vL)
        self.sim.setJointTargetVelocity(self.right_motor, vR)


def main(args=None):
    rclpy.init(args=args)
    node = PioneerFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
