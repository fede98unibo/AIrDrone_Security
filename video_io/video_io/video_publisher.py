import os
import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

MAX_WIDTH = 800
VIDEO_DIR = "./src/video_io/video"

class VideoPublisher(Node):
    def __init__(self, video_source: str, image_topic: str, hz: int):
        super().__init__("video_publisher")
        self.publisher = self.create_publisher(Image, image_topic, 10)
        timer_period = 1 / hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bridge = CvBridge()
        
        self._set_source(video_source)
        self.video = cv2.VideoCapture(self._get_source())

        self.get_logger().info(f'Publishing video {video_source} on topic {image_topic} at {hz} fps')

    def _set_source(self, video_source):
        if video_source == "cam":
            self.video_source = 0
        elif video_source == "all":
            self.video_source = os.listdir(VIDEO_DIR)
            self.current = 0
        else:
            self.video_source = video_source
    
    def _get_source(self):
        if isinstance(self.video_source, list):
            source = f"{VIDEO_DIR}/{self.video_source[self.current]}"
            self.current = (self.current + 1) % len(self.video_source)
        else:
            source = self.video_source

        self.get_logger().info(f"Playing {source}")
        return source

    def timer_callback(self) -> None:
        ret, frame = self.video.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            scale = MAX_WIDTH / max(frame.shape)
            if scale < 1:
                frame = cv2.resize(
                    src=frame,
                    dsize=None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_AREA,
                )
            msg = self.bridge.cv2_to_imgmsg(frame)
            self.publisher.publish(msg)
        else:
            # self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.video.release()
            self.video = cv2.VideoCapture(self._get_source())


    def destroy_node(self) -> bool:
        self.video.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = VideoPublisher(
        # video_source="cam",
        # video_source="all",
        video_source=f"{VIDEO_DIR}/street.mp4",
        image_topic="/image_raw",
        hz=30,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Closing Node (Keyboard Interrupt)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
