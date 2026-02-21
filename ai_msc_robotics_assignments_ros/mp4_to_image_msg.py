import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__("video_publisher_node")

        # Declare parameters with default values
        self.declare_parameter("video_path", "converted.mp4")
        self.declare_parameter("publish_topic", "/robot_car/front_cam/rgb/image_raw")
        self.declare_parameter("frame_rate", 30.0)

        # Get parameter values
        video_path = self.get_parameter("video_path").get_parameter_value().string_value
        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )
        frame_rate = self.get_parameter("frame_rate").get_parameter_value().double_value

        # Create publisher
        self.publisher = self.create_publisher(Image, publish_topic, 10)

        # Initialize OpenCV video capture
        self.cap = cv2.VideoCapture(video_path)

        # Check if video opened successfully
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video file: {video_path}")
            return

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Calculate timer period based on frame rate
        timer_period = 1.0 / frame_rate

        # Create timer to publish frames
        self.timer = self.create_timer(timer_period, self.publish_frame)

        # Log initialization
        self.get_logger().info(
            f"Video Publisher Node initialized. Publishing frames from {video_path}"
        )

    def publish_frame(self):
        # Read a frame from the video
        ret, frame = self.cap.read()

        # If frame is read correctly ret is True
        if ret:
            # Convert OpenCV image (BGR) to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the image
            self.publisher.publish(ros_image)
        else:
            # If no more frames, reset video to beginning
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def __del__(self):
        # Release video capture when node is destroyed
        if hasattr(self, "cap"):
            self.cap.release()


def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Create and spin the node
    video_publisher = VideoPublisherNode()

    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        video_publisher.get_logger().info("Shutting down video publisher node")
    finally:
        # Destroy the node and shut down ROS communication
        video_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
