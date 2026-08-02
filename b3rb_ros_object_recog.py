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

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np

# HINT: TensorFlow/Keras can be heavy and might not be installed by default.
# We wrap the import in a try-except block so the node runs even if TensorFlow is missing.
try:
    import tensorflow as tf
except ImportError:
    tf = None

# ------------------------------------------------------------------
# Mission sign semantics (see challenge doc "Sign to Building Map")
# ------------------------------------------------------------------
#   A, B, C -> PATIENT_1, PATIENT_2, PATIENT_3   (any patient counts -> priority pick)
#   X, Y, Z -> HOSPITAL_1, HOSPITAL_2, HOSPITAL_3 (must match server assignment -> exact pick)
PATIENT_LETTERS = ["A", "B", "C"]
HOSPITAL_LETTERS = ["X", "Y", "Z"]

PATIENT_MAP = {"A": "PATIENT_1", "B": "PATIENT_2", "C": "PATIENT_3"}
HOSPITAL_MAP = {"X": "HOSPITAL_1", "Y": "HOSPITAL_2", "Z": "HOSPITAL_3"}
# Reverse lookup, useful for whatever node publishes the server's hospital assignment.
HOSPITAL_NAME_TO_LETTER = {v: k for k, v in HOSPITAL_MAP.items()}

PANEL_LABELS = ["A", "B", "C", "X", "Y", "Z"]

# HSV range used to isolate the green route board from the background.
BOARD_HSV_LOWER = np.array([35, 50, 50])
BOARD_HSV_UPPER = np.array([90, 255, 255])


class ObjectRecognizer(Node):
    """
    ROS 2 Node that processes raw camera images to read the route board and
    decide which way to turn (LEFT / RIGHT / STRAIGHT).

    There are two modes, mirroring the mission flow:

      * SEARCH mode (default, target_letter is None):
          Used before a patient has been picked up. We don't care which
          patient we reach first, so we just take whichever of A/B/C shows
          up on the board, checked in that priority order: A first, then
          B, then C.

      * TARGET mode (target_letter is one of X/Y/Z):
          Used after a patient has been scanned and the Municipality
          Server has assigned a hospital. Now only ONE specific letter is
          correct -- we must ignore any other letters on the board and
          only act on the arrow for our assigned hospital letter.

      In both modes, if the letter(s) we care about aren't present on the
      current board (or no board is visible at all), we default to
      STRAIGHT and keep looking.

    Mode switching:
      Another node (e.g. the one handling QR scanning / Municipality
      Server communication) is what actually knows when a patient has
      been picked up and which hospital was assigned. This node just
      listens for that on `/target_destination` (std_msgs/String):
        - "" (empty string)  -> back to SEARCH mode (looking for a patient)
        - "X" / "Y" / "Z"    -> TARGET mode, follow only that hospital letter
      Publish HOSPITAL_NAME_TO_LETMAP[hospital_name] there once the server
      responds with an assignment.

    The turn decision is published as a plain string ("LEFT", "RIGHT", or
    "STRAIGHT") on `/sign_board_detection`.
    """
    def __init__(self):
        super().__init__('object_recognizer')

        # Current mission target letter. None => SEARCH mode (A>B>C priority).
        # "X"/"Y"/"Z" => TARGET mode (only that letter counts).
        self.target_letter = None

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

        # Subscription that tells us which mode/letter to follow.
        self.subscription_target = self.create_subscription(
            String,
            '/target_destination',
            self.target_destination_callback,
            10)

        # Publisher for the turn decision.
        self.publisher_sign = self.create_publisher(
            String,
            '/sign_board_detection',
            10)

        # Optional: pre-trained Keras model, kept for future/optional use.
        self.model = None
        if tf is not None:
            try:
                dir_path = os.path.dirname(os.path.abspath(__file__))
                model_path = os.path.join(dir_path, 'model.h5')
                if os.path.exists(model_path):
                    self.model = tf.keras.models.load_model(model_path)
                    self.get_logger().info(f"Loaded Keras model from {model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load Keras model: {e}")

        self.get_logger().info("Object Recognizer Node started in SEARCH mode (A>B>C).")

    # ------------------------------------------------------------------
    # Mode switching
    # ------------------------------------------------------------------
    def target_destination_callback(self, message):
        """
        Updates the mode based on what the mission-control node publishes.
        Empty string -> SEARCH mode. "X"/"Y"/"Z" -> TARGET mode.
        """
        letter = message.data.strip().upper()

        if letter == "":
            self.target_letter = None
            self.get_logger().info("Mode switched -> SEARCH (patient priority A>B>C)")
        elif letter in HOSPITAL_LETTERS:
            self.target_letter = letter
            hospital_name = HOSPITAL_MAP.get(letter, "UNKNOWN")
            self.get_logger().info(f"Mode switched -> TARGET {letter} ({hospital_name})")
        else:
            self.get_logger().warn(f"Ignoring unrecognized target_destination: '{message.data}'")

    # ------------------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------------------
    def camera_image_callback(self, message):
        """Processes incoming camera frames to classify the route board."""
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        turn_decision, followed_letter, route_map = self.classify_sign(image)

        msg = String()
        msg.data = turn_decision
        self.publisher_sign.publish(msg)

        mode = "SEARCH" if self.target_letter is None else f"TARGET:{self.target_letter}"
        self.get_logger().info(
            f"[{mode}] route_map={route_map} followed='{followed_letter}' -> {turn_decision}")

    # ------------------------------------------------------------------
    # Board isolation
    # ------------------------------------------------------------------
    def find_board(self, image):
        """Locates the green route board and returns the cropped board image."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BOARD_HSV_LOWER, BOARD_HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

        # Guard against spurious tiny green blobs being mistaken for the board.
        if w < 20 or h < 20:
            return None

        return image[y + 2:y + h - 2, x + 2:x + w - 2]

    # ------------------------------------------------------------------
    # Per-panel arrow detection
    # ------------------------------------------------------------------
    @staticmethod
    def get_contour(roi, upscale=1):
        if roi.size == 0:
            return None
        if upscale > 1:
            roi = cv2.resize(roi, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def detect_arrow(self, arrow_roi):
        """
        Returns "LEFT", "RIGHT", "STRAIGHT", or None.

        Upscale before thresholding: a raw, small arrow crop loses too much
        sub-pixel edge detail, which can cause a naive left/right pixel-count
        classifier to misfire on clipped/thin arrow tips.
        """
        cnt = self.get_contour(arrow_roi, upscale=6)
        if cnt is None:
            return None

        bx, by, bw, bh = cv2.boundingRect(cnt)
        if bh == 0:
            return None

        # A vertical (straight) arrow is much taller than wide relative to a
        # horizontal (left/right) arrow -- check this first.
        aspect = bw / bh
        if aspect < 1.2:
            return "STRAIGHT"

        # For a horizontal arrow, the arrowhead is bulkier than the thin shaft,
        # so the contour's centroid sits shifted toward whichever side holds
        # the head. This is more robust to a few stray/clipped pixels than
        # counting raw pixels in the left half vs. the right half of the ROI.
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return None
        centroid_x = M["m10"] / M["m00"]
        bbox_center_x = bx + bw / 2

        return "LEFT" if centroid_x < bbox_center_x else "RIGHT"

    def build_route_map(self, board):
        """Splits the board into its 6 fixed panels and reads each arrow."""
        route_map = {}
        panel_width = board.shape[1] // len(PANEL_LABELS)

        for i, label in enumerate(PANEL_LABELS):
            x1 = i * panel_width
            x2 = board.shape[1] if i == len(PANEL_LABELS) - 1 else (i + 1) * panel_width
            panel = board[:, x1:x2]

            panel_h = panel.shape[0]
            arrow_roi = panel[panel_h // 2:]

            route_map[label] = self.detect_arrow(arrow_roi)

        return route_map

    # ------------------------------------------------------------------
    # Turn decision
    # ------------------------------------------------------------------
    def decide_turn(self, route_map):
        """
        Returns (decision, followed_letter).

        SEARCH mode (target_letter is None):
          Walk A -> B -> C. First letter with a usable arrow wins -- we
          don't care which patient we find first.

        TARGET mode (target_letter is X/Y/Z):
          Only that exact letter counts. Any other letters present on the
          board (including other hospital letters) are ignored, since
          going to the wrong hospital is a penalty, not just a detour.

        Either way: nothing usable found -> STRAIGHT (keep driving/searching).
        """
        if self.target_letter is None:
            for label in PATIENT_LETTERS:
                arrow = route_map.get(label)
                if arrow is not None:
                    return arrow, label
            return "STRAIGHT", None
        else:
            arrow = route_map.get(self.target_letter)
            if arrow is not None:
                return arrow, self.target_letter
            return "STRAIGHT", None

    def classify_sign(self, image):
        """
        Detect the route board, build its letter->arrow map, and decide
        the turn per the current mode. Returns (decision, followed_letter, route_map).
        route_map is {} when no board is found.
        """
        board = self.find_board(image)
        if board is None:
            return "STRAIGHT", None, {}

        route_map = self.build_route_map(board)
        decision, followed_letter = self.decide_turn(route_map)
        return decision, followed_letter, route_map


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
