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


import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from synapse_msgs.msg import EdgeVectors, ServerCommunication

QOS_PROFILE_DEFAULT = 10
PI = math.pi

# ------------------ Control bounds (this year's convention) ------------------
SPEED_MIN = 0.0
SPEED_MAX = 1.0
TURN_MIN = -1.0
TURN_MAX = 1.0
SPEED_STOP = 0.0

# Speed presets
SPEED_10_PERCENT = SPEED_MAX / 5.0
SPEED_50_PERCENT = SPEED_MAX / 2.0

# Obstacle-avoidance turn magnitudes
LEFT_TURN_OBSTACLE = +0.3
RIGHT_TURN_OBSTACLE = -0.3

# ------------------ Lane-loss timer ------------------
# use this to keep driving straight for a short window whenever
# both lane edges were briefly lost (e.g. crossing an intersection). Kept as
# a generic "vector dropout" handler; rename/retune per your track.
VECTOR_DROPOUT_TIMER_DURATION = 5.0

# ------------------ Speed ramping ------------------
SPEED_INCREMENT = 0.1
SPEED_INCREMENT_INTERVAL = 0.1

# ------------------ Collision handling ------------------
COLLISION_THRESHOLD = 0.1          # meters -- LIDAR min-distance collision trigger
REPEAT_VECTOR_THRESHOLD = 10       # consecutive identical single-vector reads => stuck/collision
REVERSE_TIME = 1.5                 # seconds to reverse before retrying
REVERSE_SPEED = -SPEED_50_PERCENT / 2.0

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

        # ------------------ Publishers ------------------

        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT)

        self.publisher_server = self.create_publisher(
            ServerCommunication,
            '/ServerCommunication',
            QOS_PROFILE_DEFAULT)

        # ------------------ Mission state (this year's scaffolding, unchanged) ------------------
        self.obstacle_in_front = False
        self.patient_id = None
        self.hospital_id = None
        self.current_destination = None
        self.mission_completed = False

        # ------------------ Driving state (ported from old file) ------------------
        self.target_speed = 0.15
        self.target_turn = 0.0
        self.current_speed = SPEED_MIN
        self.last_speed_update_time = self.get_clock().now()

        self.pid = PID(kp=1.2 ,ki=0.0, kd=0.0)
        self.last_time = self.get_clock().now()

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

        self.get_logger().info("Line Follower controller initialized (PID lane-centering active).")

    # ------------------ Drive helpers ------------------

    def publish_drive_commands(self):
        """Timer callback that periodically publishes the current speed and steer command."""
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
        """Reverse + steer away from the flagged obstacle side. Ported from old file."""
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

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if self.vectors_lost:
            if message.vector_count == 0:
                self.target_speed = SPEED_10_PERCENT
                turn = 0.0
            else:
                self.stop_vector_dropout_timer()
        elif message.vector_count == 0:
            self.start_vector_dropout_timer()
            self.vectors_lost = True
            self.target_speed = SPEED_10_PERCENT
        elif message.vector_count == 1:
            self.target_speed = SPEED_50_PERCENT
            deviation = message.vector_1[1].x - message.vector_1[0].x
            turn = deviation / message.image_width
        else:  # vector_count == 2
            middle_x_left = (message.vector_1[0].x + message.vector_1[1].x) / 2.0
            middle_x_right = (message.vector_2[0].x + message.vector_2[1].x) / 2.0
            middle_x = (middle_x_left + middle_x_right) / 2.0
            deviation = half_width - middle_x

            turn = self.pid.compute(deviation / half_width, dt)
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
        """
        Obstacle/collision detection, ported from last year's implementation.

        TODO(mission): use ranges toward the sides/front to verify you are
        actually inside a patient/hospital zone boundary before you scan/act --
        this proximity-to-building check is specific to this year's challenge
        and was not present in the old code.
        """
        ranges = np.array(message.ranges, dtype=float)
        ranges[ranges < message.range_min] = np.nan
        ranges[ranges > message.range_max] = np.nan

        if np.all(np.isnan(ranges)):
            self.obstacle_detected = False
            self.obstacle_left = False
            self.obstacle_right = False
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

    def server_communication_callback(self, message):
        """
        Receives coordination commands from the server.

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
