#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class LightDetector(Node):
    def __init__(self):
        super().__init__('light_detector')

        # ROS interfaces
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/light', 10)

        # Nuevo: publicador de imágenes comprimidas
        self.pub_compressed = self.create_publisher(CompressedImage, '/video_source/compressed', 10)

        # CV bridge
        self.bridge = CvBridge()
        self.img = None
        self.last_detected = "UNKNOWN"

        # HSV color ranges
        self.red1_lower = np.array([0, 120, 70])
        self.red1_upper = np.array([10, 255, 255])
        self.red2_lower = np.array([170, 120, 70])
        self.red2_upper = np.array([180, 255, 255])
    
    # Rango general (más flexible)
        self.yellow1_lower = np.array([10, 100, 100])  # Aumentar a un rango más bajo de tono
        self.yellow1_upper = np.array([40, 255, 255])  # Aumentar a un rango más alto de tono

        # Rango específico (más estricto)
        self.yellow2_lower = np.array([20, 80, 180])  # Relajar el rango inferior
        self.yellow2_upper = np.array([45, 130, 255])  # Relajar el rango superior

        self.green_lower = np.array([36, 100, 50])
        self.green_upper = np.array([85, 255, 255])

        # Morphological kernel and blob detector
        self.kernel = np.ones((5, 5), np.uint8)
        self.detector = self.configure_blob_detector()

        # Timer: ahora también publicamos comprimido
        self.timer = self.create_timer(0.1, self.process_image)

        self.get_logger().info("🚦 Nodo 'light_detector' iniciado.")

    def configure_blob_detector(self):
        params = cv.SimpleBlobDetector_Params()
        params.minThreshold = 30
        params.maxThreshold = 255
        params.filterByColor = True
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 30
        params.maxArea = 10000000
        params.filterByConvexity = True
        params.minConvexity = 0.1
        params.maxConvexity = 1
        params.filterByCircularity = True
        params.minCircularity = 0.5
        params.maxCircularity = 1
        params.filterByInertia = True
        params.minInertiaRatio = 0.5
        params.maxInertiaRatio = 1
        return cv.SimpleBlobDetector_create(params)
    
    def image_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"❌ Error al convertir imagen: {e}")

    def compress_and_publish(self, frame, quality=50, scale=0.5):
        # 1) Reducir resolución
        h, w = frame.shape[:2]
        small = cv.resize(frame, (int(w*scale), int(h*scale)),
                          interpolation=cv.INTER_LINEAR)

        # 2) Comprimir a JPEG
        ret, buf = cv.imencode('.jpg', small,
                               [cv.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            self.get_logger().warn("Fallo al comprimir frame")
            return

        # 3) Empaquetar en CompressedImage
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub_compressed.publish(msg)

    def detect_blob(self, hsv_img, lower, upper):
        mask = cv.inRange(hsv_img, lower, upper)
        result = cv.bitwise_and(self.img, self.img, mask=mask)
        gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        _, binary = cv.threshold(gray, 5, 255, cv.THRESH_BINARY)
        binary = cv.erode(binary, self.kernel, iterations=8)
        binary = cv.dilate(binary, self.kernel, iterations=8)
        keypoints = self.detector.detect(binary)
        return len(keypoints) > 0

    def process_image(self):
        if self.img is None:
            return

        # — Detección de color (igual que antes) —
        hsv_img = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
        detected = "UNKNOWN"
        # … blob logic …
        if self.detect_blob(hsv_img, self.red1_lower, self.red1_upper) or \
           self.detect_blob(hsv_img, self.red2_lower, self.red2_upper):
            detected = "RED"
        elif self.detect_blob(hsv_img, self.yellow1_lower, self.yellow1_upper) or \
             self.detect_blob(hsv_img, self.yellow2_lower, self.yellow2_upper):
            detected = "YELLOW"
        elif self.detect_blob(hsv_img, self.green_lower, self.green_upper):
            detected = "GREEN"
        self.publisher.publish(String(data=detected))

        # Sólo imprime cuando cambia el estado
        if detected != self.last_detected:
            self.last_detected = detected
            if detected == "RED":
                self.get_logger().info("🛑 Semáforo detectado: ROJO")
            elif detected == "YELLOW":
                self.get_logger().info("🟡 Semáforo detectado: AMARILLO")
            elif detected == "GREEN":
                self.get_logger().info("🟢 Semáforo detectado: VERDE")
            


        # — Publicación comprimida para vista rápida —
        # Ajusta quality (0–100) y scale (0.1–1.0) a tu gusto
        self.compress_and_publish(self.img, quality=60, scale=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = LightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()