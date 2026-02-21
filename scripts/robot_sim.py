#!/usr/bin/env python3
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Empty, SetBool


class RobotNavSim(Node):
    def __init__(self):
        super().__init__("robot_nav_sim")

        self.publisher = self.create_publisher(Image, "robot_sim_visualization", 10)

        self.bridge = CvBridge()

        self.robot_pos_mutex = threading.Lock()
        self.image_mutex = threading.Lock()
        self.pos_history_mutex = threading.Lock()
        self.max_width = 640
        self.max_height = 480
        self.navigation_delay = 0.1
        self.robot_radius = 20
        self.pos_history = set()
        self.happy = False

        self.clear_callback(None, None)
        self.reset_position_callback(None, None)

        self._navigate_to_pose_server = ActionServer(
            self, NavigateToPose, "~/navigate_to_pose", self.navigate_to_pose_callback
        )

        self._navigate_through_poses_server = ActionServer(
            self,
            NavigateThroughPoses,
            "~/navigate_through_poses",
            self.navigate_through_poses_callback,
        )

        self.reset_service = self.create_service(
            Empty, "~/reset_position", self.reset_position_callback
        )

        self.happy_service = self.create_service(
            SetBool, "~/set_happy_robot", self.happy_callback
        )

        self.clear_service = self.create_service(Empty, "~/clear", self.clear_callback)

        self.timer = self.create_timer(self.navigation_delay * 2, self.publish_image)

    def reset_position_callback(self, _, response):
        with self.robot_pos_mutex:
            self.robot_x = math.floor(self.max_height / 2)
            self.robot_y = math.floor(self.max_width / 2)
        return response

    def clear_callback(self, _, response):
        with self.pos_history_mutex:
            self.pos_history = set()
        return response

    def simulate_robot_nav(self, target_pose):
        x = 0
        y = 0
        tx = target_pose.position.x
        ty = target_pose.position.y
        with self.robot_pos_mutex:
            if self.robot_x > tx:
                self.robot_x -= 1
            elif self.robot_x < tx:
                self.robot_x += 1
            if self.robot_y > ty:
                self.robot_y -= 1
            elif self.robot_y < ty:
                self.robot_y += 1
            x = self.robot_x
            y = self.robot_y

        time.sleep(self.navigation_delay)
        if x == tx and y == ty:
            return 0
        return 1

    def valid_target_pose(self, target_pose):
        tx = target_pose.position.x
        ty = target_pose.position.y
        return (
            tx >= self.robot_radius
            and tx <= self.max_height - self.robot_radius
            and ty >= self.robot_radius
            and ty <= self.max_width - self.robot_radius
        )

    def happy_callback(self, request, response):
        self.happy = request.data
        response.success = True
        extra = "" if self.happy else "not "
        response.message = "The robot is " + extra + "happy!"
        return response

    def pose_distance(self, x1, y1, x2, y2):
        return math.dist([x1, y1], [x2, y2])

    def navigate_to_pose_callback(self, goal_handle):
        self.get_logger().info("Received target pose")
        if not self.valid_target_pose(goal_handle.request.pose.pose):
            self.get_logger().error("Invalid target pose")
            return NavigateToPose.Result()

        feedback_msg = NavigateToPose.Feedback()

        start_t = self.get_clock().now()
        while self.simulate_robot_nav(goal_handle.request.pose.pose) > 0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            with self.robot_pos_mutex:
                dist = self.pose_distance(
                    float(self.robot_x),
                    float(self.robot_y),
                    goal_handle.request.pose.pose.position.x,
                    goal_handle.request.pose.pose.position.y,
                )
                feedback_msg.current_pose.pose.position.x = float(self.robot_x)
                feedback_msg.current_pose.pose.position.y = float(self.robot_y)
                feedback_msg.distance_remaining = dist
                feedback_msg.estimated_time_remaining = rclpy.duration.Duration(
                    seconds=dist * self.navigation_delay
                ).to_msg()
            feedback_msg.current_pose.pose.orientation.w = 1.0
            feedback_msg.navigation_time = (self.get_clock().now() - start_t).to_msg()
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = NavigateToPose.Result()

        return result

    def navigate_through_poses_callback(self, goal_handle):
        self.get_logger().info("Received target poses")
        target_poses = goal_handle.request.poses
        for pose in target_poses:
            if not self.valid_target_pose(pose.pose):
                self.get_logger().error("Invalid target pose")
                return NavigateThroughPoses.Result()

        feedback_msg = NavigateThroughPoses.Feedback()

        start_t = self.get_clock().now()
        for pose_index, pose in enumerate(target_poses):
            while self.simulate_robot_nav(pose.pose) > 0:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return NavigateThroughPoses.Result()
                with self.robot_pos_mutex:
                    dist_to_next = self.pose_distance(
                        float(self.robot_x),
                        float(self.robot_y),
                        pose.pose.position.x,
                        pose.pose.position.y,
                    )
                    total_dist = dist_to_next
                    for pi, p in enumerate(target_poses[pose_index:]):
                        if pose_index + pi >= len(target_poses) - 1:
                            break
                        total_dist += self.pose_distance(
                            p.pose.position.x,
                            p.pose.position.y,
                            target_poses[pose_index + pi + 1].pose.position.x,
                            target_poses[pose_index + pi + 1].pose.position.y,
                        )

                    feedback_msg.current_pose.pose.position.x = float(self.robot_x)
                    feedback_msg.current_pose.pose.position.y = float(self.robot_y)
                    feedback_msg.distance_remaining = total_dist
                    feedback_msg.estimated_time_remaining = rclpy.duration.Duration(
                        seconds=total_dist * self.navigation_delay
                    ).to_msg()
                feedback_msg.current_pose.pose.orientation.w = 1.0
                feedback_msg.navigation_time = (
                    self.get_clock().now() - start_t
                ).to_msg()
                feedback_msg.number_of_poses_remaining = len(target_poses) - pose_index
                goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = NavigateThroughPoses.Result()

        return result

    def draw_state(self, image):
        pos_colour = (255, 0, 255)
        face_color = (255, 255, 0)
        outline_color = (255, 0, 0)
        with self.robot_pos_mutex:
            with self.pos_history_mutex:
                self.pos_history.add((self.robot_x, self.robot_y))
                for p in self.pos_history:
                    cv2.circle(
                        image,
                        (p[1], p[0]),
                        math.floor(self.robot_radius / 5),
                        pos_colour,
                        -1,
                    )

            cv2.circle(
                image,
                (self.robot_y, self.robot_x),
                self.robot_radius,
                face_color,
                -1,
            )

            cv2.circle(
                image,
                (self.robot_y, self.robot_x),
                self.robot_radius,
                outline_color,
                3,
            )

            eye_dist = math.floor(self.robot_radius / 3)
            eye_size = math.floor(self.robot_radius / 4)

            cv2.circle(
                image,
                (self.robot_y - eye_dist, self.robot_x - eye_dist),
                eye_size,
                outline_color,
                -1,
            )

            cv2.circle(
                image,
                (self.robot_y + eye_dist, self.robot_x - eye_dist),
                eye_size,
                outline_color,
                -1,
            )

            if self.happy:
                cv2.ellipse(
                    image,
                    (
                        self.robot_y,
                        self.robot_x + math.floor(self.robot_radius / 4),
                    ),
                    (
                        math.floor(self.robot_radius / 2),
                        math.floor(self.robot_radius / 4),
                    ),
                    0,
                    0,
                    180,
                    outline_color,
                    2,
                )
            else:
                cv2.line(
                    image,
                    (
                        self.robot_y - math.floor(self.robot_radius / 5),
                        self.robot_x + math.floor(self.robot_radius / 4),
                    ),
                    (
                        self.robot_y + math.floor(self.robot_radius / 5),
                        self.robot_x + math.floor(self.robot_radius / 4),
                    ),
                    outline_color,
                    2,
                )

    def publish_image(self):
        image = np.zeros((self.max_height, self.max_width, 3), dtype=np.uint8)

        self.draw_state(image)

        ros_image = self.bridge.cv2_to_imgmsg(image, "rgb8")
        ros_image.header = Header()
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "map"

        self.publisher.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavSim()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
