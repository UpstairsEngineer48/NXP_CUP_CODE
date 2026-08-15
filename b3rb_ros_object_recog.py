<<<<<<< HEAD
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
import cv2
import numpy as np

=======
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
>>>>>>> origin/main
try:
    import tensorflow as tf
except ImportError:
    tf = None

# ------------------------------------------------------------------
<<<<<<< HEAD
# Mission sign semantics
# ------------------------------------------------------------------
=======
# Mission sign semantics (see challenge doc "Sign to Building Map")
# ------------------------------------------------------------------
#   A, B, C -> PATIENT_1, PATIENT_2, PATIENT_3   (any patient counts -> priority pick)
#   X, Y, Z -> HOSPITAL_1, HOSPITAL_2, HOSPITAL_3 (must match server assignment -> exact pick)
>>>>>>> origin/main
PATIENT_LETTERS = ["A", "B", "C"]
HOSPITAL_LETTERS = ["X", "Y", "Z"]

PATIENT_MAP = {"A": "PATIENT_1", "B": "PATIENT_2", "C": "PATIENT_3"}
HOSPITAL_MAP = {"X": "HOSPITAL_1", "Y": "HOSPITAL_2", "Z": "HOSPITAL_3"}
<<<<<<< HEAD
=======
# Reverse lookup, useful for whatever node publishes the server's hospital assignment.
>>>>>>> origin/main
HOSPITAL_NAME_TO_LETTER = {v: k for k, v in HOSPITAL_MAP.items()}

PANEL_LABELS = ["A", "B", "C", "X", "Y", "Z"]

<<<<<<< HEAD
BOARD_HSV_LOWER = np.array([35, 50, 50])
BOARD_HSV_UPPER = np.array([90, 255, 255])

WHITE_HSV_LOWER = np.array([0, 0, 180])
WHITE_HSV_UPPER = np.array([180, 60, 255])


class ObjectRecognizer(Node):
    """
    ROS 2 Node that reads the green route board and decides LEFT / RIGHT /
    STRAIGHT for the current mission mode (SEARCH vs TARGET).

=======
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
>>>>>>> origin/main
    """
    def __init__(self):
        super().__init__('object_recognizer')

<<<<<<< HEAD
        self.target_letter = None
        self.board_read_cooldown = 3.0
        self.last_board_read_time = 0.0

        self.required_detections = 5
        self.last_direction = None
        self.direction_count = 0

        # Bootstrapped once from the first board detected.
        self.reference_contours = {}

=======
        # Current mission target letter. None => SEARCH mode (A>B>C priority).
        # "X"/"Y"/"Z" => TARGET mode (only that letter counts).
        self.target_letter = None

        # Subscription for camera images.
>>>>>>> origin/main
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            10)

<<<<<<< HEAD
=======
        # Subscription that tells us which mode/letter to follow.
>>>>>>> origin/main
        self.subscription_target = self.create_subscription(
            String,
            '/target_destination',
            self.target_destination_callback,
            10)

<<<<<<< HEAD
=======
        # Publisher for the turn decision.
>>>>>>> origin/main
        self.publisher_sign = self.create_publisher(
            String,
            '/sign_board_detection',
            10)

<<<<<<< HEAD
        self.publisher_sign_visible = self.create_publisher(
            Bool,
            '/sign_board_visible',
            10)

=======
        # Optional: pre-trained Keras model, kept for future/optional use.
>>>>>>> origin/main
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
<<<<<<< HEAD
        letter = message.data.strip().upper()
=======
        """
        Updates the mode based on what the mission-control node publishes.
        Empty string -> SEARCH mode. "X"/"Y"/"Z" -> TARGET mode.
        """
        letter = message.data.strip().upper()

>>>>>>> origin/main
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
<<<<<<< HEAD
=======
        """Processes incoming camera frames to classify the route board."""
>>>>>>> origin/main
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        turn_decision, followed_letter, route_map = self.classify_sign(image)

<<<<<<< HEAD
        visible = all(route_map.get(label) is not None for label in PANEL_LABELS)
        visible_msg = Bool()
        visible_msg.data = visible
        self.publisher_sign_visible.publish(visible_msg)

        if time.time() - self.last_board_read_time < self.board_read_cooldown:
            return

        if turn_decision == self.last_direction:
            self.direction_count += 1
        else:
            self.last_direction = turn_decision
            self.direction_count = 1

        if self.direction_count >= self.required_detections:
            msg = String()
            msg.data = turn_decision
            self.publisher_sign.publish(msg)

            mode = "SEARCH" if self.target_letter is None else f"TARGET:{self.target_letter}"
            self.get_logger().info(
                f"[{mode}] route_map={route_map} followed='{followed_letter}' -> {turn_decision}")

            self.last_board_read_time = time.time()
            self.last_direction = None
            self.direction_count = 0
=======
        msg = String()
        msg.data = turn_decision
        self.publisher_sign.publish(msg)

        mode = "SEARCH" if self.target_letter is None else f"TARGET:{self.target_letter}"
        self.get_logger().info(
            f"[{mode}] route_map={route_map} followed='{followed_letter}' -> {turn_decision}")
>>>>>>> origin/main

    # ------------------------------------------------------------------
    # Board isolation
    # ------------------------------------------------------------------
    def find_board(self, image):
<<<<<<< HEAD
=======
        """Locates the green route board and returns the cropped board image."""
>>>>>>> origin/main
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BOARD_HSV_LOWER, BOARD_HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

<<<<<<< HEAD
=======
        # Guard against spurious tiny green blobs being mistaken for the board.
>>>>>>> origin/main
        if w < 20 or h < 20:
            return None

        return image[y + 2:y + h - 2, x + 2:x + w - 2]

    # ------------------------------------------------------------------
<<<<<<< HEAD
    # Shared contour helper
    # ------------------------------------------------------------------
    @staticmethod
    def get_contour(roi):
        if roi is None or roi.size == 0:
            return None
=======
    # Per-panel arrow detection
    # ------------------------------------------------------------------
    @staticmethod
    def get_contour(roi, upscale=1):
        if roi.size == 0:
            return None
        if upscale > 1:
            roi = cv2.resize(roi, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
>>>>>>> origin/main
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

<<<<<<< HEAD
    # ------------------------------------------------------------------
    # Letter identification (position-independent, contour-shape based)
    # ------------------------------------------------------------------
    def bootstrap_reference_contours(self, board):
        """Seed reference letter contours assuming PANEL_LABELS order. Only
        runs once, on the first board where every panel yields a contour."""
        panel_width = board.shape[1] // len(PANEL_LABELS)
        refs = {}
=======
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

>>>>>>> origin/main
        for i, label in enumerate(PANEL_LABELS):
            x1 = i * panel_width
            x2 = board.shape[1] if i == len(PANEL_LABELS) - 1 else (i + 1) * panel_width
            panel = board[:, x1:x2]
<<<<<<< HEAD
            cnt = self.get_contour(panel[:panel.shape[0] // 2])
            if cnt is not None:
                refs[label] = cnt

        if len(refs) == len(PANEL_LABELS):
            self.reference_contours = refs
            self.get_logger().info("Bootstrapped reference letter contours from board.")

    def detect_letter(self, letter_roi):
        if not self.reference_contours:
            return None
        cnt = self.get_contour(letter_roi)
        if cnt is None:
            return None
        best, best_score = None, float("inf")
        for label, ref in self.reference_contours.items():
            score = cv2.matchShapes(cnt, ref, cv2.CONTOURS_MATCH_I1, 0)
            if score < best_score:
                best_score = score
                best = label
        return best

    # ------------------------------------------------------------------
    # Arrow direction detection
    # ------------------------------------------------------------------
    def detect_arrow(self, arrow_roi):
        if arrow_roi is None or arrow_roi.size == 0:
            return None

        arrow_roi = cv2.resize(
            arrow_roi, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC)

        gray = cv2.cvtColor(arrow_roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        h, w = thresh.shape

        def contour_area(segment):
            contours, _ = cv2.findContours(
                segment, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            return sum(cv2.contourArea(c) for c in contours)

        # CASE 1: ink concentrated at the top -> STRAIGHT arrow shaft/head.
        top_height = max(1, int(0.25 * h))
        top_area = contour_area(thresh[:top_height, :])
        if top_area > 0:
            return "STRAIGHT"

        # CASE 2: compare left vs right halves for LEFT/RIGHT.
        mid = w // 2
        left_area = contour_area(thresh[:, :mid])
        right_area = contour_area(thresh[:, mid:])

        return "LEFT" if left_area > right_area else "RIGHT"

    # ------------------------------------------------------------------
    # Panel trimming via topmost white pixel row
    # ------------------------------------------------------------------
    @staticmethod
    def trim_panel_to_glyphs(panel):
        """Crops a panel tightly around its printed glyphs using the
        topmost row of white pixels, mirroring the calibration behavior
        found while prototyping against reference board images."""
        hsv = cv2.cvtColor(panel, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, WHITE_HSV_LOWER, WHITE_HSV_UPPER)
        ys, _ = np.where(white_mask == 255)
        if len(ys) == 0:
            return None

        highest_y = int(np.min(ys))
        if highest_y > 3:
            k = 11
        elif highest_y > 2:
            k = 14
        elif highest_y == 2:
            k = 20
        else:  # highest_y == 0 or 1
            k = 40

        crop_h = max(1, highest_y * k)
        return panel[:crop_h, :]

    # ------------------------------------------------------------------
    # Route map construction
    # ------------------------------------------------------------------
    def build_route_map(self, board):
        route_map = {}
        panel_width = board.shape[1] // len(PANEL_LABELS)

        for i in range(len(PANEL_LABELS)):
            x1 = i * panel_width
            x2 = board.shape[1] if i == len(PANEL_LABELS) - 1 else (i + 1) * panel_width
            panel = board[:, x1:x2]

            # Trim off the margin the same way the experimental script did.
            ph, pw = panel.shape[:2]
            panel = panel[ph // 15: ph - ph // 15, pw // 15: pw - pw // 15]

            trimmed = self.trim_panel_to_glyphs(panel)
            if trimmed is None or trimmed.shape[0] < 2:
                continue

            th = trimmed.shape[0]
            letter_roi = trimmed[:th // 2]
            arrow_roi = trimmed[th // 2:]

            # Trim the arrow ROI's side margins.
            aw = arrow_roi.shape[1]
            ah = arrow_roi.shape[0]
            side_margin = max(1, ah // 12)
            arrow_roi = arrow_roi[:, aw // 8: max(aw // 8 + 1, aw - side_margin)]

            letter = self.detect_letter(letter_roi)
            if letter is None:
 
                letter = PANEL_LABELS[i]

            route_map[letter] = self.detect_arrow(arrow_roi)
=======

            panel_h = panel.shape[0]
            arrow_roi = panel[panel_h // 2:]

            route_map[label] = self.detect_arrow(arrow_roi)
>>>>>>> origin/main

        return route_map

    # ------------------------------------------------------------------
    # Turn decision
    # ------------------------------------------------------------------
    def decide_turn(self, route_map):
<<<<<<< HEAD
=======
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
>>>>>>> origin/main
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
<<<<<<< HEAD
=======
        """
        Detect the route board, build its letter->arrow map, and decide
        the turn per the current mode. Returns (decision, followed_letter, route_map).
        route_map is {} when no board is found.
        """
>>>>>>> origin/main
        board = self.find_board(image)
        if board is None:
            return "STRAIGHT", None, {}

<<<<<<< HEAD
        if not self.reference_contours:
            self.bootstrap_reference_contours(board)

=======
>>>>>>> origin/main
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
