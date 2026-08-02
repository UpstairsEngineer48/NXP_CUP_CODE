# Copyright 2024-2026 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np

# HINT: If you want to use pyzbar for QR code detection, you can install it using:
# pip install pyzbar
# And uncomment/import it here:
try:
    from pyzbar.pyzbar import decode
except ImportError:
    decode = None

class QRDetector(Node):
    """
    ROS 2 Node that processes raw camera images to scan for QR codes.
    It publishes the detected QR code payload on the `/qr_detection` topic.
    """
    def __init__(self):
        super().__init__('qr_detector')

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

        # Publisher for QR code detection results.
        self.publisher_qr = self.create_publisher(
            String,
            '/qr_detection',
            10)

        self.get_logger().info("QR Detector Node started. Waiting for images...")

    def camera_image_callback(self, message):
        """Processes incoming camera frames to detect QR codes."""
        # Convert compressed image message to OpenCV format
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("Camera", image)
        cv2.waitKey(1)
        qr_data = self.detect_qr_code(image)

        if qr_data is not None:
            # Publish the decoded QR payload
            msg = String()
            msg.data = qr_data
            self.publisher_qr.publish(msg)
            self.get_logger().info(f"Published QR Data: {qr_data}")


    def detect_qr_code(self, image):
        """
        Detect and decode a QR code using pyzbar first.
        Falls back to OpenCV if pyzbar is unavailable.
        """

        # Convert to grayscale (improves detection)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ---------- Method 1: pyzbar ----------
        if decode is not None:
            decoded_objects = decode(gray)

            for obj in decoded_objects:
                # Optional: draw bounding box
                pts = obj.polygon
                if len(pts) == 4:
                    pts = np.array([(p.x, p.y) for p in pts], dtype=np.int32)
                    cv2.polylines(image, [pts], True, (0, 255, 0), 2)

                data = obj.data.decode("utf-8")
                self.get_logger().info(f"QR detected: {data}")
                return data

        # ---------- Method 2: OpenCV fallback ----------
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(gray)

        if bbox is not None and data:
            bbox = bbox.astype(int)

            for i in range(len(bbox[0])):
                pt1 = tuple(bbox[0][i])
                pt2 = tuple(bbox[0][(i + 1) % len(bbox[0])])
                cv2.line(image, pt1, pt2, (255, 0, 0), 2)

            self.get_logger().info(f"QR detected (OpenCV): {data}")
            return data

        return None
def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
