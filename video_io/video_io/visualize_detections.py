from typing import List
import colorsys
import random

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2D, Detection2DArray
from sensor_msgs.msg import Image

from video_io.video_publisher import VIDEO_DIR


QUEUE_LEN = 10


class VisualizeDetections(Node):
    def __init__(self, image_topic: str, detection_topic: str):
        super().__init__("visualize_detection")
        self.detections: List[Detection2D] = []
        self.fading = 0

        self.detection_subscriber = self.create_subscription(
            Detection2DArray, detection_topic, self.detect, QUEUE_LEN
        )
        self.image_subscriber = self.create_subscription(
            Image, image_topic, self.visualize, QUEUE_LEN
        )
        self.bridge = CvBridge()

        with open(f"{VIDEO_DIR}/coco.names") as f:
            labels = f.read().strip().split("\n")

        num_classes = len(labels)
        hsv_tuples = [(1.0 * x / num_classes, 1.0, 1.0) for x in range(num_classes)]
        colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors)
        )

        random.seed(0)
        random.shuffle(colors)
        random.seed(None)

        self.font_scale = 0.5

        self.colors = {cls: color for cls, color in zip(labels, colors)}

        self.get_logger().info(
            f"Ready! [Image topic: '{image_topic}', detections topic '{detection_topic}']"
        )

    def detect(self, msg: Detection2DArray):
        self.detections = msg.detections
        self.fading = 0

    def visualize(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_h, image_w, _ = cv_image.shape

        if self.fading > 60:
            self.detections = []

        for detection in self.detections:
            bbox = detection.bbox
            result = detection.results[0]

            x1 = bbox.center.x - bbox.size_x / 2
            y1 = bbox.center.y - bbox.size_y / 2
            x2 = x1 + bbox.size_x
            y2 = y1 + bbox.size_y

            bbox_color = self.colors[result.id]
            bbox_thick = int(0.6 * (image_h + image_w) / 600)

            bbox_mess = f"{result.id}: {result.score:.4f}"

            c1, c2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(cv_image, c1, c2, bbox_color, bbox_thick)

            t_size = cv2.getTextSize(
                bbox_mess, 0, self.font_scale, thickness=bbox_thick // 2
            )[0]

            c3 = (int(c1[0] + t_size[0]), int(c1[1] - t_size[1] - 3))
            cv2.rectangle(cv_image, c1, c3, bbox_color, -1)

            c4 = (c1[0], int(c1[1] - 2))
            cv2.putText(
                cv_image,
                bbox_mess,
                c4,
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                (0, 0, 0),
                bbox_thick // 2,
                lineType=cv2.LINE_AA,
            )

        self.fading += 1
        cv2.imshow("Detections", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    viz = VisualizeDetections(
        image_topic="/camera/image_raw", detection_topic="/detector_node/detections"
    )

    try:
        rclpy.spin(viz)
    except KeyboardInterrupt:
        viz.get_logger().info("Closing Node (Keyboard Interrupt)")
    finally:
        viz.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
