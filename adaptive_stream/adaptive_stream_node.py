import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String


class AdaptiveStreamNode(Node):

    def __init__(self):
        super().__init__('adaptive_stream_node')

        # ROS parameter:
        # high / medium / low
        self.declare_parameter('mode', 'medium')

        self.bridge = CvBridge()

        # Three streaming profiles
        self.profiles = {
            'high': {
                'width': 1280,
                'height': 720,
                'fps': 20.0,
                'quality': 90
            },
            'medium': {
                'width': 640,
                'height': 480,
                'fps': 15.0,
                'quality': 70
            },
            'low': {
                'width': 320,
                'height': 240,
                'fps': 10.0,
                'quality': 40
            }
        }

        self.mode = self.get_parameter('mode').value
        # network state debounce
        self.last_network_state = None
        self.network_state_count = 0

        self.network_state_threshold = {
            'GOOD': 10,
            'MEDIUM': 5,
            'BAD': 5
        }

        if self.mode not in self.profiles:
            self.mode = 'medium'

        self.last_publish_time = 0.0
        self.frame_count = 0
        self.total_bytes = 0
        self.stats_start_time = time.monotonic()

        # Subscribe to raw ZED RGB image
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            self.image_callback,
            10
        )
        # Subscribe network state
        self.network_subscription = self.create_subscription(
            String,
            '/adaptive/network_state',
            self.network_callback,
            10
        )

        # Publish our own JPEG stream
        self.publisher = self.create_publisher(
            CompressedImage,
            '/adaptive/image/compressed',
            10
        )
        # Raw preview topic for web_video_server
        self.preview_publisher = self.create_publisher(
            Image,
            '/adaptive/image',
            10
        )

        # Allow mode to change while node is running
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        self.get_logger().info(
            'Adaptive stream node started'
        )

        self.print_current_profile()

    def print_current_profile(self):
        profile = self.profiles[self.mode]

        self.get_logger().info(
            f'Mode={self.mode.upper()} | '
            f'{profile["width"]}x{profile["height"]} | '
            f'{profile["fps"]:.0f} FPS | '
            f'JPEG Q={profile["quality"]}'
        )

    def network_callback(self, msg):

        state = msg.data

        if state == self.last_network_state:
            self.network_state_count += 1
        else:
            self.last_network_state = state
            self.network_state_count = 1


        threshold = self.network_state_threshold.get(
            state,
            5
        )


        if self.network_state_count >= threshold:


            target_mode = None

            if state == 'GOOD':
                target_mode = 'high'

            elif state == 'MEDIUM':
                target_mode = 'medium'

            elif state == 'BAD':
                target_mode = 'low'


            if target_mode and target_mode != self.mode:

                self.set_parameters(
                    [
                        Parameter(
                            'mode',
                            Parameter.Type.STRING,
                            target_mode
                        )
                    ]
                )


                self.get_logger().info(
                    f'Auto switch mode: {target_mode.upper()} '
                    f'(network={state}, stable={self.network_state_count}s)'
                )


            self.network_state_count = 0

    def parameter_callback(self, params):

        for param in params:

            if param.name == 'mode':

                new_mode = param.value.lower()

                if new_mode not in self.profiles:
                    self.get_logger().warn(
                        'Invalid mode. Use: high, medium, or low'
                    )

                    return SetParametersResult(
                        successful=False,
                        reason='Mode must be high, medium, or low'
                    )

                self.mode = new_mode

                # Allow the new profile to publish immediately
                self.last_publish_time = 0.0

                # Reset statistics after profile switching
                self.frame_count = 0
                self.total_bytes = 0
                self.stats_start_time = time.monotonic()

                self.get_logger().info(
                    f'Stream mode changed to {self.mode.upper()}'
                )

                self.print_current_profile()

        return SetParametersResult(successful=True)

    def image_callback(self, msg):

        profile = self.profiles[self.mode]

                # FPS control
        now = time.monotonic()
        frame_interval = 1.0 / profile['fps']

        if now - self.last_publish_time < frame_interval:
            return

        if self.last_publish_time == 0.0:
            self.last_publish_time = now
        else:
            self.last_publish_time += frame_interval

            # If processing/input was delayed too much,
            # resynchronize the timing reference.
            if now - self.last_publish_time > frame_interval:
                self.last_publish_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Resize according to current profile
            target_size = (
                profile['width'],
                profile['height']
            )

            if (
                frame.shape[1] != profile['width']
                or frame.shape[0] != profile['height']
            ):
                frame = cv2.resize(
                    frame,
                    target_size,
                    interpolation=cv2.INTER_AREA
                )

            # JPEG encoding
            success, encoded = cv2.imencode(
                '.jpg',
                frame,
                [
                    int(cv2.IMWRITE_JPEG_QUALITY),
                    profile['quality']
                ]
            )

            if not success:
                self.get_logger().warn(
                    'JPEG encoding failed'
                )
                return
            # Publish resized raw image for browser preview
            preview_msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8'
            )
            preview_msg.header = msg.header
            self.preview_publisher.publish(preview_msg)

            # Create ROS CompressedImage
            compressed_msg = CompressedImage()

            # Keep timestamp/frame reference from input image
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded.tobytes()

            self.publisher.publish(compressed_msg)

            self.frame_count += 1
            self.total_bytes += len(compressed_msg.data)

            # Print statistics approximately once per second
            elapsed = now - self.stats_start_time

            if elapsed >= 1.0:

                actual_fps = self.frame_count / elapsed

                bandwidth_mbps = (
                    self.total_bytes * 8.0
                    / elapsed
                    / 1_000_000.0
                )

                avg_frame_kb = (
                    self.total_bytes
                    / max(self.frame_count, 1)
                    / 1024.0
                )

                self.get_logger().info(
                    f'[{self.mode.upper()}] '
                    f'{profile["width"]}x{profile["height"]} | '
                    f'FPS={actual_fps:.1f} | '
                    f'Q={profile["quality"]} | '
                    f'Frame={avg_frame_kb:.1f} KB | '
                    f'Rate={bandwidth_mbps:.2f} Mbps'
                )

                self.frame_count = 0
                self.total_bytes = 0
                self.stats_start_time = now

        except Exception as e:

            self.get_logger().error(
                f'Image processing error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = AdaptiveStreamNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
