import time
from collections import deque

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose2D

from yolo_ros.yolo import Yolo
import cv2

from yolo_ros.following import Follower

QUEUE_LEN = 1


class YoloNode(Node):
    def __init__(self, image_topic: str, detection_topic: str, network: Yolo, compressed: bool):
        super().__init__(network.name.replace("-", "_"))

        if compressed:
            self.subscriber = self.create_subscription(
                CompressedImage, image_topic + "/compressed", self.detect_compressed, QUEUE_LEN
            )
        else:
            self.subscriber = self.create_subscription(
                Image, image_topic, self.detect, QUEUE_LEN
            )

        self.publisher = self.create_publisher(
            Detection2DArray, detection_topic, QUEUE_LEN
        )

        self.bridge = CvBridge()
        self.follower = Follower(300, 5)

        self.network = network

        self.get_logger().info(
            f"Ready to receive images on '{image_topic}', "
            f"and publish detections on '{detection_topic}'"
        )

        self.time = deque(maxlen=100)

    def detect(self, img_msg: Image) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        self.process_image(cv_image)

    def detect_compressed(self, img_msg: CompressedImage) -> None:
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        self.process_image(cv_image)

    def process_image(self, cv_image):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        start = time.time()
        detected, classes, scores, bboxes = self.network(cv_image)
        self.time.append(time.time() - start)

        self.get_logger().info(f"FPS: {len(self.time)/sum(self.time):.2f}")

        if detected == 0:
            return

        self.get_logger().info(f"Det: {detected}")

        detections = Detection2DArray(
            detections=[
                Detection2D(
                    results=[
                        ObjectHypothesisWithPose(
                            id=cls_str, score=float(score)
                        )
                    ],
                    bbox=BoundingBox2D(
                        center=Pose2D(x=float(cx), y=float(cy)),
                        size_x=float(sx),
                        size_y=float(sy),
                    ),
                    
                )
                for cls_str, score, (cx, cy, sx, sy) in zip(classes, scores, bboxes)
            ]
        )

        detections = self.follower.follow(detections)

        self.publisher.publish(detections)


def main(args=None):
    rclpy.init(args=args)

    tiny = True
    input_size = 416
    confidence_threshold = 0.4
    nms_threshold = 0.4
    compressed = False

    if input_size not in (320, 416, 608, 1024):
        raise ValueError(f"{input_size=} not supported [320, 416, 608]")

    name = f"yolo-v4{'-tiny' if tiny else ''}-{input_size}"
    print(f"Loading {name}...")
    yolo = Yolo(name, input_size, confidence_threshold, nms_threshold)

    node = YoloNode(
        image_topic="/camera/image_raw",
        detection_topic="/detector_node/detections",
        network=yolo,
        compressed=compressed
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
