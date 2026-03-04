#!/usr/bin/env python3
# Script revisi kelompok G
# Author: Alif Gibran Muhammad Ervin
# Serial replaced with ROS2 topics:
#   - Publishes commands to:  /claw/command  (std_msgs/String)
#   - Subscribes to MCU from: /claw/response (std_msgs/String)
# Logic: Camera detect tag0(payload) -> send command dan posisi tag0 ke MCU lewat topic ->
# send "PAYLOAD" ke MCU -> tunggu MCU request drop lewat topic -> track tag1(droppoint) sampai MCU send "DONE"

import math
import os
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pupil_apriltags import Detector


class AprilTagNode(Node):

    TAG_SIZE = 0.108
    DIST_THRESHOLD_CM = 1
    SEND_INTERVAL = 1.0

    STATE_WAIT_PICK = 0
    STATE_WAIT_PICK_DONE = 1
    STATE_TRACK_DROP = 2
    STATE_WAIT_FINAL_DONE = 3

    def __init__(self) -> None:
        super().__init__("apriltag_node")

        calib_path = os.path.join(os.path.dirname(__file__), "calib.npz") # Bayu's webcam intrinsic NOT MINE
        calib = np.load(calib_path)
        camera_matrix = calib["cameraMatrix"]

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        self.camera_params = (fx, fy, cx, cy)

        self.cap = cv2.VideoCapture(0)

        self.detector = Detector(
            families="tagStandard41h12",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.cmd_pub = self.create_publisher(String, "claw/command", 10)

        self.create_subscription(String, "claw/response", self.response_callback, 10)

        self.state = self.STATE_WAIT_PICK
        self.payload_signal_sent = False
        self.last_target_sent_time = 0.0
        self.latest_response = ""

        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        self.get_logger().info("AprilTag ROS2 Node Started")
        self.get_logger().info("  Commands  → /claw/command  (std_msgs/String)")
        self.get_logger().info("  Responses ← /claw/response (std_msgs/String)")

    def response_callback(self, msg: String) -> None:
        """Receive MCU responses from /claw/response topic."""
        self.latest_response = msg.data.strip()
        self.get_logger().info(f"MCU: {self.latest_response}")

    def send_command(self, command: str) -> None:
        """Publish a command string to /claw/command topic."""
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"CMD: {command}")

    def meter_to_cm(self, value: float) -> int:
        return int(value * 100.0)

    def find_tag(self, detections, tag_id: int):
        for d in detections:
            if d.tag_id == tag_id:
                return d
        return None

    def loop(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Camera read failed")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.TAG_SIZE,
        )

        current_time = time.time()

        msg = self.latest_response
        self.latest_response = ""

        # STATE_WAIT_PICK
        if self.state == self.STATE_WAIT_PICK:
            det0 = self.find_tag(detections, 0)

            if det0:
                t = det0.pose_t
                x_cm = self.meter_to_cm(float(t[0]))
                y_cm = self.meter_to_cm(float(t[1]))

                distance = math.sqrt(x_cm**2 + y_cm**2)

                if current_time - self.last_target_sent_time >= self.SEND_INTERVAL:
                    self.send_command(f"TARGET {x_cm} {y_cm}")
                    self.last_target_sent_time = current_time

                if distance <= self.DIST_THRESHOLD_CM:
                    if not self.payload_signal_sent:
                        self.send_command("PAYLOAD")
                        self.payload_signal_sent = True
                        self.state = self.STATE_WAIT_PICK_DONE

        # STATE_WAIT_PICK_DONE
        if self.state == self.STATE_WAIT_PICK_DONE:
            if msg == "REQUEST_DROP":
                self.state = self.STATE_TRACK_DROP
                self.get_logger().info("Switching to DROP tracking")

        # STATE_TRACK_DROP
        if self.state == self.STATE_TRACK_DROP:

            if msg == "DONE":
                self.state = self.STATE_WAIT_PICK
                self.payload_signal_sent = False
                self.get_logger().info("Cycle Complete")
                return

            det1 = self.find_tag(detections, 1)

            if det1:
                t = det1.pose_t
                x_cm = self.meter_to_cm(float(t[0]))
                y_cm = self.meter_to_cm(float(t[1]))

                distance = math.sqrt(x_cm**2 + y_cm**2)

                if current_time - self.last_target_sent_time >= self.SEND_INTERVAL:
                    self.send_command(f"TARGET {x_cm} {y_cm}")
                    self.last_target_sent_time = current_time

                if distance <= self.DIST_THRESHOLD_CM:
                    self.send_command("DROP")
                    self.state = self.STATE_WAIT_FINAL_DONE

        cv2.imshow("apriltag", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = AprilTagNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()