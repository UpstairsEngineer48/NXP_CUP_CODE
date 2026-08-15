# Copyright 2024-2026 NXP
# Copyright 2016 Open Source Robotics Foundation, Inc.
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
#


<<<<<<< HEAD
import re
=======
>>>>>>> origin/main
import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
<<<<<<< HEAD
from std_msgs.msg import String, Bool
=======
from std_msgs.msg import String
>>>>>>> origin/main
from synapse_msgs.msg import EdgeVectors, ServerCommunication

QOS_PROFILE_DEFAULT = 10
PI = math.pi

<<<<<<< HEAD

=======
# ------------------ Control bounds (this year's convention) ------------------
>>>>>>> origin/main
SPEED_MIN = 0.0
SPEED_MAX = 1.0
TURN_MIN = -1.0
TURN_MAX = 1.0
SPEED_STOP = 0.0

# Speed presets
<<<<<<< HEAD
SPEED_10_PERCENT = SPEED_MAX * 0.5
SPEED_50_PERCENT = SPEED_MAX * 0.7
=======
SPEED_10_PERCENT = SPEED_MAX / 5.0
SPEED_50_PERCENT = SPEED_MAX / 2.0
>>>>>>> origin/main

# Obstacle-avoidance turn magnitudes
LEFT_TURN_OBSTACLE = +0.3
RIGHT_TURN_OBSTACLE = -0.3

<<<<<<< HEAD

INTERSECTION_TURN_LEFT = +0.5
INTERSECTION_TURN_RIGHT = -0.5


SIGN_REQUIRED_DETECTIONS = 1


SIGN_LOST_TIME = 2.5

QR_GAP_TIME = 7.0


VECTOR_DROPOUT_TIMER_DURATION = 5.0

SPEED_INCREMENT = 0.04
SPEED_INCREMENT_INTERVAL = 0.1

=======
# ------------------ Lane-loss timer ------------------
# use this to keep driving straight for a short window whenever
# both lane edges were briefly lost (e.g. crossing an intersection). Kept as
# a generic "vector dropout" handler; rename/retune per your track.
VECTOR_DROPOUT_TIMER_DURATION = 5.0

# ------------------ Speed ramping ------------------
SPEED_INCREMENT = 0.1
SPEED_INCREMENT_INTERVAL = 0.1

# ------------------ Collision handling ------------------
>>>>>>> origin/main
COLLISION_THRESHOLD = 0.1          # meters -- LIDAR min-distance collision trigger
REPEAT_VECTOR_THRESHOLD = 10       # consecutive identical single-vector reads => stuck/collision
REVERSE_TIME = 1.5                 # seconds to reverse before retrying
REVERSE_SPEED = -SPEED_50_PERCENT / 2.0

<<<<<<< HEAD

HOSPITAL_LETTER_TO_NAME = {"X": "HOSPITAL_1", "Y": "HOSPITAL_2", "Z": "HOSPITAL_3"}
PATIENT_LETTERS = {"A", "B", "C"}

PATIENT_NAMES = {"PATIENT_1", "PATIENT_2", "PATIENT_3"}
HOSPITAL_NAMES = {"HOSPITAL_1", "HOSPITAL_2", "HOSPITAL_3"}
FAKE_HOSPITAL_NAMES = {"FAKE_HOSPITAL_1", "FAKE_HOSPITAL_2"}

MISSION_PATIENT_TARGET_COUNT = 3


QR_ZONE_LIDAR_THRESHOLD = 5.0      # meters
PARKING_LIDAR_THRESHOLD = 0.6       # meters





LANE_CENTER_KP = 1.0    # proportional gain for lane-centering steer (current offset)
LANE_HEADING_KP = 0.6   # gain for anticipatory steer, based on the lane's slope/direction ahead


class LineFollower(Node):
=======
# CONFIGURATION:
# The buggy is driven in manual mode by publishing standard controller Joy messages to /cerebri/in/joy.
# The layout is: msg.axes = [0.0, speed, 0.0, turn]
# - speed: positive for forward, negative for reverse. Range: [-1.0, 1.0]
# - turn: positive for left steer, negative for right steer. Range: [-1.0, 1.0]
# msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1] (Keep buttons set to this pattern for manual override mode)


class PID:
    """Simple PID controller, ported from last year's line_follower for lane-centering."""

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


class LineFollower(Node):
    """
    Core controller Node for the B3RB buggy.

    Drive/steer logic (PID lane centering + LIDAR obstacle/collision handling) is
    ported from a previous year's working submission. Mission logic (patient/
    hospital state machine, zone checks, server communication, QR/sign parsing)
    is this year's scaffolding and still needs your implementation -- see the
    TODO(mission) markers.
    """
>>>>>>> origin/main

    def __init__(self):
        super().__init__('line_follower')

        # ------------------ Subscriptions ------------------

        # 1. Lane Edge Vectors (from edge_vectors_publisher)
        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT)

        # 2. LIDAR Obstacle Scanner
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT)

        # 3. Server Communication Feedback Loop
        self.subscription_server = self.create_subscription(
            ServerCommunication,
            '/ServerCommunication',
            self.server_communication_callback,
            QOS_PROFILE_DEFAULT)

        # 4. QR Code Detections (from qr_detector)
        self.subscription_qr = self.create_subscription(
            String,
            '/qr_detection',
            self.qr_detection_callback,
            QOS_PROFILE_DEFAULT)

        # 5. Sign Board Detections (from object_recognizer)
        self.subscription_signs = self.create_subscription(
            String,
            '/sign_board_detection',
            self.sign_board_callback,
            QOS_PROFILE_DEFAULT)

<<<<<<< HEAD
        # 6. Sign Board Visibility (from object_recognizer, every frame)
        self.subscription_sign_visible = self.create_subscription(
            Bool,
            '/sign_board_visible',
            self.sign_visible_callback,
            QOS_PROFILE_DEFAULT)

=======
>>>>>>> origin/main
        # ------------------ Publishers ------------------

        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT)

        self.publisher_server = self.create_publisher(
            ServerCommunication,
            '/ServerCommunication',
            QOS_PROFILE_DEFAULT)

<<<<<<< HEAD

        self.publisher_target_destination = self.create_publisher(
            String,
            '/target_destination',
            QOS_PROFILE_DEFAULT)

        # ------------------ Mission state ------------------
        self.obstacle_in_front = False
        self.mission_state = "SEARCH_PATIENT"
        self.assigned_hospital_letter = None
        self.assigned_hospital_name = None
        self.delivered_count = 0
        self.mission_completed = False
        self.parked_reported = False

        # Server-communication bookkeeping.
        self.next_uid = 0
        self.last_patient_uid = None
        self.last_delivery_uid = None
        self.last_park_uid = None


        self.last_patient_payload = None
        self.last_delivery_payload = None
        self.last_park_payload = "PARKED"

        # Response-timeout tracking: set whenever we start waiting on the
        # server, cleared the moment a data reply arrives.
        self.awaiting_since = None
        self.retry_count = 0

        # Sign-board memory, consumed at intersections (see get_sign_turn_override).
        # Kept for logging / backward compatibility -- reflects the raw last
        # reading, not necessarily what we've committed to.
        self.last_sign_direction = None
        self.last_sign_time = None

        # Sign-commit state machine (see sign_board_callback / get_sign_turn_override):
        #   candidate_direction/candidate_count -> building confidence in a reading
        #   committed_direction                 -> the direction we'll actually turn
        #   last_sign_seen_time                 -> last time ANY sign was detected
        #                                          (kept for logging only -- turn
        #                                          execution no longer waits on this)
        self.candidate_direction = None
        self.candidate_count = 0
        self.committed_direction = None
        self.last_sign_seen_time = None

        # Visibility-driven "board has left view" gate (see sign_visible_callback
        # / get_sign_turn_override): sign_lost_since is the timestamp the board
        # was last reported NOT fully visible; passed_current_sign flips true
        # once that's held for SIGN_LOST_TIME seconds straight.
        self.sign_board_visible = False
        self.sign_lost_since = None
        self.passed_current_sign = False

        # Latest LIDAR snapshot, used by the zone-proximity heuristics.
        self.last_lidar_ranges = None
        self.last_lidar_min_distance = None

        # QR "region ended" gate:don't act on a QR reading the instant
        # it's seen -- wait until QR_GAP_TIME has passed since the LAST
        # time it was seen, actually driven past the QR
        # region and are now (assumed) within the server's valid zone.
        self.last_qr_raw = None
        self.last_qr_building = None
        self.last_qr_seen_time = None
        self.qr_pending_process = False

        self.target_speed = 1.0
=======
        # ------------------ Mission state (this year's scaffolding, unchanged) ------------------
        self.obstacle_in_front = False
        self.patient_id = None
        self.hospital_id = None
        self.current_destination = None
        self.mission_completed = False

        # ------------------ Driving state (ported from old file) ------------------
        self.target_speed = 0.15
>>>>>>> origin/main
        self.target_turn = 0.0
        self.current_speed = SPEED_MIN
        self.last_speed_update_time = self.get_clock().now()

<<<<<<< HEAD
=======
        self.pid = PID(kp=1.2 ,ki=0.0, kd=0.0)
        self.last_time = self.get_clock().now()

>>>>>>> origin/main
        self.vector_dropout_timer = None
        self.vectors_lost = False

        self.obstacle_detected = False
        self.obstacle_left = False
        self.obstacle_right = False

        self.previous_vector = None
        self.repeat_vector_count = 0
        self.prev_reverse_turn = 'NOTURN'

        self.reversing = False
        self.reverse_start_time = None

        self.stopped = False

        # Timer to publish drive commands at 10Hz
        self.control_timer = self.create_timer(0.1, self.publish_drive_commands)

<<<<<<< HEAD
        # Timer to check for an unresponsive server while waiting on it.
        self.timeout_check_timer = self.create_timer(1.0, self.check_server_timeout)

=======
>>>>>>> origin/main
        self.get_logger().info("Line Follower controller initialized (PID lane-centering active).")

    # ------------------ Drive helpers ------------------

    def publish_drive_commands(self):
        """Timer callback that periodically publishes the current speed and steer command."""
<<<<<<< HEAD
        self.process_qr_after_gap()
=======
>>>>>>> origin/main
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, self.current_speed, 0.0, self.target_turn]
        self.publisher_joy.publish(msg)

    def rover_move_manual_mode(self, speed, turn):
        """Helper to immediately set control speed and steering angle."""
        self.target_speed = float(max(min(speed, SPEED_MAX), -SPEED_MAX))
        self.target_turn = float(max(min(turn, TURN_MAX), -TURN_MAX))
        self.current_speed = self.target_speed

    def update_speed(self, target_speed):
        """Gradually ramps speed up; decelerates instantly. Ported from old file."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_speed_update_time).nanoseconds / 1e9

        if target_speed > self.current_speed:
            if dt >= SPEED_INCREMENT_INTERVAL:
                self.current_speed = min(self.current_speed + SPEED_INCREMENT, target_speed)
                self.last_speed_update_time = current_time
        else:
            self.current_speed = target_speed

    def start_vector_dropout_timer(self):
        self.vector_dropout_timer = self.create_timer(
            VECTOR_DROPOUT_TIMER_DURATION, self.stop_vector_dropout_timer)

    def stop_vector_dropout_timer(self):
        if self.vector_dropout_timer:
            self.vector_dropout_timer.cancel()
            self.vector_dropout_timer = None
            self.vectors_lost = False

    def reverse_buggy(self):
<<<<<<< HEAD

=======
        """Reverse + steer away from the flagged obstacle side. Ported from old file."""
>>>>>>> origin/main
        self.get_logger().warn("Collision/stuck condition detected! Reversing...")
        self.reversing = True
        self.reverse_start_time = self.get_clock().now()

        if self.obstacle_left:
            reverse_turn = RIGHT_TURN_OBSTACLE
            self.prev_reverse_turn = 'RIGHT'
        elif self.obstacle_right:
            reverse_turn = LEFT_TURN_OBSTACLE
            self.prev_reverse_turn = 'LEFT'
        else:
            # Alternate turns if no specific obstacle side is detected.
            if self.prev_reverse_turn == 'LEFT':
                reverse_turn = RIGHT_TURN_OBSTACLE
                self.prev_reverse_turn = 'RIGHT'
            else:
                reverse_turn = LEFT_TURN_OBSTACLE
                self.prev_reverse_turn = 'LEFT'

        if not self.stopped:
            self.rover_move_manual_mode(REVERSE_SPEED, reverse_turn)
        else:
            self.rover_move_manual_mode(SPEED_STOP, 0.0)

    def stop_reversing(self):
        self.reversing = False
        self.rover_move_manual_mode(SPEED_STOP, 0.0)

<<<<<<< HEAD
    # ------------------ Mission helpers ------------------

    def get_sign_turn_override(self):

        if self.committed_direction is None or not self.passed_current_sign:
            return 0.0

        if self.committed_direction == "LEFT":
            return INTERSECTION_TURN_LEFT
        elif self.committed_direction == "RIGHT":
            return INTERSECTION_TURN_RIGHT
        return 0.0  # STRAIGHT

    def check_in_zone(self):

        if self.last_lidar_min_distance is None:
            return False
        return self.last_lidar_min_distance < QR_ZONE_LIDAR_THRESHOLD

    def check_in_parking_zone(self):

        if self.last_lidar_ranges is None:
            return False

        ranges = self.last_lidar_ranges
        n = len(ranges)
        if n < 6:
            return False

        front = np.nanmin(ranges[int(n * 0.4):int(n * 0.6)])
        left = np.nanmin(ranges[:int(n * 0.15)])
        right = np.nanmin(ranges[int(n * 0.85):])

        close_sides = sum(
            1 for d in (front, left, right) if not np.isnan(d) and d < PARKING_LIDAR_THRESHOLD)
        return close_sides >= 2

    def parse_qr_payload(self, raw_text):

        if not raw_text:
            return None
        text = raw_text.strip().upper()
        match = re.search(r'([A-Z]+(?:_[A-Z0-9]+)+)', text)
        if match:
            return match.group(1)
        return text if text else None

    def _next_uid(self):
        uid = self.next_uid
        self.next_uid = (self.next_uid + 1) % 256
        return uid

    def publish_target_destination(self, letter_or_empty):
        """Switches object_recognizer between SEARCH mode ("") and TARGET mode (X/Y/Z)."""
        msg = String()
        msg.data = letter_or_empty
        self.publisher_target_destination.publish(msg)

    def begin_exit_sequence(self):
        """Called once the 3rd hospital delivery is confirmed."""
        self.mission_completed = True
        self.assigned_hospital_letter = None
        self.assigned_hospital_name = None
        self.publish_target_destination("")
        self.mission_state = "EXITING_TO_PARKING"
        self.stopped = False
        self.parked_reported = False
        self.get_logger().info("All 3 patients delivered -- exiting toward parking.")

    def check_server_timeout(self):
        
        if self.awaiting_since is None:
            return

        elapsed = (self.get_clock().now() - self.awaiting_since).nanoseconds / 1e9
        if elapsed < SERVER_RESPONSE_TIMEOUT:
            return

        if self.retry_count >= MAX_RESEND_ATTEMPTS:
            self.get_logger().error(
                f"No server response after {self.retry_count} retries in state "
                f"{self.mission_state} -- giving up this wait.")
            self.handle_response_giveup()
            return

        self.retry_count += 1
        self.get_logger().warn(
            f"No server response after {SERVER_RESPONSE_TIMEOUT}s -- "
            f"retransmitting (attempt {self.retry_count}/{MAX_RESEND_ATTEMPTS}).")

        if self.mission_state == "AWAITING_HOSPITAL_ASSIGNMENT":
            self.send_server_update(self.last_patient_payload, ack=0, uid=self.last_patient_uid)
        elif self.mission_state == "AWAITING_DELIVERY_OUTCOME":
            self.send_server_update(self.last_delivery_payload, ack=0, uid=self.last_delivery_uid)
        elif self.mission_state == "AWAITING_PARK_RESULT":
            self.send_server_update(self.last_park_payload, ack=0, uid=self.last_park_uid)
        else:
            # Not actually in a wait state -- clear the stale timer.
            self.awaiting_since = None
            return

        self.awaiting_since = self.get_clock().now()

    def handle_response_giveup(self):
        """Fallback recovery once retries are exhausted for the current wait."""
        self.awaiting_since = None
        self.retry_count = 0

        if self.mission_state == "AWAITING_HOSPITAL_ASSIGNMENT":
            self.mission_state = "SEARCH_PATIENT"
            self.stopped = False
        elif self.mission_state == "AWAITING_DELIVERY_OUTCOME":
            self.mission_state = "EN_ROUTE_TO_HOSPITAL"
            self.stopped = False
        elif self.mission_state == "AWAITING_PARK_RESULT":
            self.get_logger().error(
                "Parking result never confirmed -- check connectivity; "
                "buggy remains stopped in the parking zone.")
            # Nothing safe to fall back to here -- PARKED was already sent
            # and the mission is otherwise complete.

    def report_parked(self):
        """
        Sends PARKED the instant the parking zone is detected, then begins
        slowing down -- being inside the zone when the message is sent is
        what counts, not being fully stopped first.
        """
        self.parked_reported = True
        self.last_park_payload = "PARKED"
        self.last_park_uid = self.send_server_update("PARKED", ack=0)
        self.mission_state = "AWAITING_PARK_RESULT"
        self.awaiting_since = self.get_clock().now()
        self.retry_count = 0
        self.stopped = True
        self.get_logger().info("Inside parking zone -- sent PARKED message to server.")

    # ------------------ Callback Implementations ------------------

    def edge_vectors_callback(self, message):
   
        
=======
    # ------------------ Callback Implementations ------------------

    def edge_vectors_callback(self, message):
        """
        Lane-centering, ported from last year's working PID implementation.

        - vector_count == 0: lane briefly lost -> keep a short straight-drive
          grace window (vector-dropout timer) before treating it as intersection/parking exit.
        - vector_count == 1: only one edge visible -> steer away from it using
          the vector's own slope as a proxy deviation.
        - vector_count == 2: both edges visible -> PID-center the midpoint.

        TODO(mission): at intersections you will also need to factor in
        sign_board_callback's last-seen direction (left/straight/right) to
        choose a turn instead of blindly centering -- this is challenge-specific
        and was not present in the old code.
        """
>>>>>>> origin/main
        if self.reversing:
            current_time = self.get_clock().now()
            dt = (current_time - self.reverse_start_time).nanoseconds / 1e9
            if dt >= REVERSE_TIME:
                self.stop_reversing()
            else:
                return  # Keep reversing until time is up.

        # Repeated-single-vector heuristic: buggy stuck against something.
        if self.previous_vector == message.vector_1 and message.vector_count == 1:
            self.repeat_vector_count += 1
        else:
            self.repeat_vector_count = 0
        self.previous_vector = message.vector_1 if message.vector_count == 1 else None

        if self.repeat_vector_count > REPEAT_VECTOR_THRESHOLD:
            self.reverse_buggy()
            return

        turn = 0.0
        half_width = message.image_width / 2.0

<<<<<<< HEAD
        if self.vectors_lost:
            if message.vector_count == 0:
                self.target_speed = SPEED_10_PERCENT
                turn = self.get_sign_turn_override()
            else:
                self.stop_vector_dropout_timer()
                self.clear_sign_state()
=======
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if self.vectors_lost:
            if message.vector_count == 0:
                self.target_speed = SPEED_10_PERCENT
                turn = 0.0
            else:
                self.stop_vector_dropout_timer()
>>>>>>> origin/main
        elif message.vector_count == 0:
            self.start_vector_dropout_timer()
            self.vectors_lost = True
            self.target_speed = SPEED_10_PERCENT
<<<<<<< HEAD
            turn = self.get_sign_turn_override()
=======
>>>>>>> origin/main
        elif message.vector_count == 1:
            self.target_speed = SPEED_50_PERCENT
            deviation = message.vector_1[1].x - message.vector_1[0].x
            turn = deviation / message.image_width
        else:  # vector_count == 2
            middle_x_left = (message.vector_1[0].x + message.vector_1[1].x) / 2.0
            middle_x_right = (message.vector_2[0].x + message.vector_2[1].x) / 2.0
            middle_x = (middle_x_left + middle_x_right) / 2.0
            deviation = half_width - middle_x

<<<<<<< HEAD
            # Slope of each edge vector (far point x - near point x) indicates
            # which way the lane is curving ahead, letting us start turning
            # before the offset (deviation) itself has built up.
            slope_left = message.vector_1[1].x - message.vector_1[0].x
            slope_right = message.vector_2[1].x - message.vector_2[0].x
            heading_error = (slope_left + slope_right) / 2.0

            turn = (LANE_CENTER_KP * (deviation / half_width)
                    + LANE_HEADING_KP * (heading_error / half_width))
=======
            turn = self.pid.compute(deviation / half_width, dt)
>>>>>>> origin/main
            turn = max(TURN_MIN, min(TURN_MAX, turn))

            if abs(deviation) > half_width / 2.0:
                self.target_speed = SPEED_10_PERCENT
            else:
                self.target_speed = SPEED_MAX

        if self.obstacle_detected:
            self.target_speed = SPEED_10_PERCENT
            if self.obstacle_left:
                turn = RIGHT_TURN_OBSTACLE
            elif self.obstacle_right:
                turn = LEFT_TURN_OBSTACLE

        if self.stopped:
            self.update_speed(SPEED_STOP)
        else:
            self.update_speed(self.target_speed)

        self.target_turn = max(TURN_MIN, min(TURN_MAX, turn))

    def lidar_callback(self, message):
<<<<<<< HEAD
  
=======
        """
        Obstacle/collision detection, ported from last year's implementation.

        TODO(mission): use ranges toward the sides/front to verify you are
        actually inside a patient/hospital zone boundary before you scan/act --
        this proximity-to-building check is specific to this year's challenge
        and was not present in the old code.
        """
>>>>>>> origin/main
        ranges = np.array(message.ranges, dtype=float)
        ranges[ranges < message.range_min] = np.nan
        ranges[ranges > message.range_max] = np.nan

<<<<<<< HEAD
        self.last_lidar_ranges = ranges
        self.last_lidar_min_distance = None if np.all(np.isnan(ranges)) else np.nanmin(ranges)



=======
>>>>>>> origin/main
        if np.all(np.isnan(ranges)):
            self.obstacle_detected = False
            self.obstacle_left = False
            self.obstacle_right = False
<<<<<<< HEAD
        else:
            min_distance = np.nanmin(ranges)

            if min_distance < COLLISION_THRESHOLD:
                self.obstacle_detected = True
                angle_of_min_distance = np.nanargmin(ranges) * message.angle_increment
                if angle_of_min_distance < PI / 2:
                    self.obstacle_left = True
                    self.obstacle_right = False
                else:
                    self.obstacle_left = False
                    self.obstacle_right = True
            else:
                self.obstacle_detected = False
                self.obstacle_left = False
                self.obstacle_right = False

        if (self.mission_state == "EXITING_TO_PARKING"
                and not self.parked_reported
                and self.check_in_parking_zone()):
            self.report_parked()
=======
            return

        min_distance = np.nanmin(ranges)

        if min_distance < COLLISION_THRESHOLD:
            self.obstacle_detected = True
            angle_of_min_distance = np.nanargmin(ranges) * message.angle_increment
            if angle_of_min_distance < PI / 2:
                self.obstacle_left = True
                self.obstacle_right = False
            else:
                self.obstacle_left = False
                self.obstacle_right = True
        else:
            self.obstacle_detected = False
            self.obstacle_left = False
            self.obstacle_right = False
>>>>>>> origin/main

    def server_communication_callback(self, message):
        """
        Receives coordination commands from the server.

<<<<<<< HEAD
        """
        if message.dest != 1:
            return

        if message.ack == 1:
            self.get_logger().info(f"Server ACKed uid={message.uid}")
            return

        payload = message.msg.strip().upper()
        self.get_logger().info(f"Server data uid={message.uid} msg='{payload}'")

        # Acknowledge receipt of this data message before acting on it.
        self.send_server_update("", ack=1, uid=message.uid)

        self.handle_server_data(payload)

    def handle_server_data(self, payload):
        """State transitions driven by the server's data messages."""

        self.awaiting_since = None
        self.retry_count = 0

        if self.mission_state == "AWAITING_HOSPITAL_ASSIGNMENT":
            if payload in HOSPITAL_LETTER_TO_NAME:
                self.assigned_hospital_letter = payload
                self.assigned_hospital_name = HOSPITAL_LETTER_TO_NAME[payload]
                self.get_logger().info(
                    f"Hospital assigned: {payload} ({self.assigned_hospital_name})")
                self.mission_state = "EN_ROUTE_TO_HOSPITAL"
                self.stopped = False
                self.publish_target_destination(payload)
            elif payload == "INVALID":
                self.get_logger().warn(
                    "Patient report rejected as INVALID -- resuming search to retry.")
                self.mission_state = "SEARCH_PATIENT"
                self.stopped = False
            else:
                self.get_logger().warn(f"Unexpected data while awaiting hospital assignment: {payload}")

        elif self.mission_state == "AWAITING_DELIVERY_OUTCOME":
            if payload in PATIENT_LETTERS:
                self.delivered_count += 1
                self.get_logger().info(f"Delivery confirmed. Total delivered: {self.delivered_count}")
                if self.delivered_count >= MISSION_PATIENT_TARGET_COUNT:
                    self.begin_exit_sequence()
                else:
                    self.assigned_hospital_letter = None
                    self.assigned_hospital_name = None
                    self.mission_state = "SEARCH_PATIENT"
                    self.stopped = False
                    self.publish_target_destination("")
            elif payload == "INVALID":
                self.get_logger().warn(
                    "Hospital delivery rejected as INVALID -- returning to re-attempt delivery.")
                self.mission_state = "EN_ROUTE_TO_HOSPITAL"
                self.stopped = False
            else:
                self.get_logger().warn(f"Unexpected data while awaiting delivery outcome: {payload}")

        elif self.mission_state == "AWAITING_PARK_RESULT":
            if payload == "OK":
                self.get_logger().info("Parking confirmed by server -- mission fully complete.")
                self.mission_state = "MISSION_DONE"
            elif payload == "INVALID":
                self.get_logger().warn(
                    "Parking rejected as INVALID -- buggy may not be fully inside the zone.")
                # Left for further tuning: could re-attempt positioning here.
            else:
                self.get_logger().warn(f"Unexpected data while awaiting park result: {payload}")

        else:
            self.get_logger().info(
                f"Server data '{payload}' received in state {self.mission_state} -- no action defined.")

    def send_server_update(self, text_msg, ack=0, uid=None):
        """
        Sends a message to the server. 
       
        """
        server_msg = ServerCommunication()
        server_msg.src = 1
        server_msg.dest = 2
        server_msg.uid = uid if uid is not None else self._next_uid()
        server_msg.ack = ack
        server_msg.msg = text_msg
        self.publisher_server.publish(server_msg)
        return server_msg.uid

    def qr_detection_callback(self, message):

        raw_text = message.data.strip()
        building_name = self.parse_qr_payload(raw_text)
        if building_name is None:
            return

        self.last_qr_raw = raw_text
        self.last_qr_building = building_name
        self.last_qr_seen_time = self.get_clock().now()
        self.qr_pending_process = True

    def process_qr_after_gap(self):
 
        if not self.qr_pending_process or self.last_qr_seen_time is None:
            return

        age = (self.get_clock().now() - self.last_qr_seen_time).nanoseconds / 1e9
        if age < QR_GAP_TIME:
            return

        # QR region has ended -- process this reading exactly once.
        self.qr_pending_process = False
        raw_text = self.last_qr_raw
        building_name = self.last_qr_building

        self.get_logger().info(f"QR decoded: raw='{raw_text}' -> '{building_name}'")

        is_mission_building = (
            building_name in PATIENT_NAMES
            or building_name in HOSPITAL_NAMES
            or building_name in FAKE_HOSPITAL_NAMES)

        if not self.check_in_zone():
            if is_mission_building:
                self.get_logger().warn(
                    f"Saw '{building_name}' but zone check failed -- not reporting yet.")
            return

        if self.mission_state == "SEARCH_PATIENT" and building_name in PATIENT_NAMES:
            self.stopped = True
            self.last_patient_payload = raw_text
            self.last_patient_uid = self.send_server_update(raw_text, ack=0)
            self.mission_state = "AWAITING_HOSPITAL_ASSIGNMENT"
            self.awaiting_since = self.get_clock().now()
            self.retry_count = 0
            self.get_logger().info(
                f"Reported patient '{raw_text}' -- awaiting hospital assignment.")

        elif self.mission_state == "EN_ROUTE_TO_HOSPITAL":
            if building_name in FAKE_HOSPITAL_NAMES:
                self.get_logger().warn(f"Ignoring fake hospital '{building_name}' -- not reporting.")
                return
            if building_name != self.assigned_hospital_name:
                self.get_logger().warn(
                    f"Reached '{building_name}' but assigned hospital is "
                    f"'{self.assigned_hospital_name}' -- not reporting.")
                return
            self.stopped = True
            self.last_delivery_payload = raw_text
            self.last_delivery_uid = self.send_server_update(raw_text, ack=0)
            self.mission_state = "AWAITING_DELIVERY_OUTCOME"
            self.awaiting_since = self.get_clock().now()
            self.retry_count = 0
            self.get_logger().info(f"Reported delivery at '{raw_text}'.")

        else:
            self.get_logger().info(
                f"QR '{building_name}' seen in state {self.mission_state} -- no action taken.")

    def sign_visible_callback(self, message):

        self.sign_board_visible = message.data
        now = self.get_clock().now()

        if self.sign_board_visible:
            self.sign_lost_since = None
            return

        if self.sign_lost_since is None:
            self.sign_lost_since = now
            return

        if not self.passed_current_sign:
            elapsed = (now - self.sign_lost_since).nanoseconds / 1e9
            if elapsed >= SIGN_LOST_TIME:
                self.passed_current_sign = True
                self.get_logger().info(
                    f"Sign board out of view for {SIGN_LOST_TIME}s -- turn armed.")

    def sign_board_callback(self, message):

        direction = message.data
        self.last_sign_direction = direction
        self.last_sign_time = self.get_clock().now()
        self.last_sign_seen_time = self.last_sign_time
        self.get_logger().info(f"Heard Sign Board: {direction}")

        if self.committed_direction is not None:
            # Already committed for this intersection -- ignore.
            return

        if direction == self.candidate_direction:
            self.candidate_count += 1
        else:
            self.candidate_direction = direction
            self.candidate_count = 1

        if self.candidate_count >= SIGN_REQUIRED_DETECTIONS:
            self.committed_direction = direction
            self.get_logger().info(f"Committed sign direction: {direction}")

    def clear_sign_state(self):
  
        self.candidate_direction = None
        self.candidate_count = 0
        self.committed_direction = None
        self.last_sign_seen_time = None
        self.sign_lost_since = None
        self.passed_current_sign = False
=======
        GUIDELINE (Server Communication) -- this year's scaffolding, unchanged:
        - Check if the message is destined for the Buggy (`message.dest == 1`).
        - Do not forget to check for ACK messages from server.
        - The server communicates mission info in the `message.msg` payload string.
        - Parse server instructions (e.g., patient pickup, target hospitals).
        - Call `self.send_server_update` to report your status when you reach a checkpoint.

        TODO(mission): implement patient/hospital assignment parsing and state
        transitions here -- no equivalent existed in the old code.
        """
        if message.dest == 1:
            self.get_logger().info(f"Received Server Message: {message.msg}")
            pass

    def send_server_update(self, text_msg):
        """Sends status messages to the server. (Do not forget to send ACK messages to server)"""
        server_msg = ServerCommunication()
        server_msg.src = 1
        server_msg.dest = 2
        server_msg.uid = 100
        server_msg.ack = 0
        server_msg.msg = text_msg
        self.publisher_server.publish(server_msg)

    def qr_detection_callback(self, message):
        """
        Receives QR codes scanned from the buildings.

        GUIDELINE -- this year's scaffolding, unchanged. TODO(mission): parse
        payload, verify zone via lidar/position, trigger send_server_update.
        """
        self.get_logger().info(f"Heard QR code: {message.data}")
        pass

    def sign_board_callback(self, message):
        """
        Receives traffic sign boards.

        GUIDELINE -- this year's scaffolding, unchanged. TODO(mission): store
        last-seen direction and consume it inside edge_vectors_callback at
        intersections.
        """
        self.get_logger().info(f"Heard Sign Board: {message.data}")
        pass
>>>>>>> origin/main


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
