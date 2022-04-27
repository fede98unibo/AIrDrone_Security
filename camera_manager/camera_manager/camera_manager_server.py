import time
import threading

from airdrone_actions.srv import TakePicture
from airdrone_actions.action import TakeVideo
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from cv_bridge import CvBridge
import cv2

SAVE_PATH = "/home/simone/Videos"


class ImageSubNode(Node):
    def __init__(self):
        super().__init__("img_sub_node")
        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.save_callback, 1
        )
        self._image = None
        self.get_logger().info("Ready to grab frames")

    def save_callback(self, img: Image) -> None:
        self._image = img

    def get_frame(self) -> Image:
        self.get_logger().info("Getting Frame")

        self._image = None

        while self._not_ready():
            rclpy.spin_once(self)

        return self._image

    def _not_ready(self) -> bool:
        return self._image is None


class CameraService(Node):
    def __init__(self):
        super().__init__("camera_service")
        self._srv_take = self.create_service(
            TakePicture, "take_picture", self.take_picture_service
        )

        self.bridge = CvBridge()
        self.img_node = ImageSubNode()
        # self.video_node = VideoSubNode()

        self._action_server = ActionServer(
            self,
            TakeVideo,
            "take_video",
            self.take_video_action_service,
            cancel_callback=self.cancel_callback,
        )

        self._run = False
        self.video_writer = None
        self.fps = 30
        self.get_logger().info("Ready to serve")

    def take_picture_service(self, request, response):
        self.get_logger().info(
            f"""Incoming request:
                detections: {request.include_detections} 
                geo: {request.include_geolocalization}
            """
        )
        response.image = self.img_node.get_frame()

        return response

    def take_video_action_service(self, goal_handle):
        self.get_logger().info("Taking video...")
        result = TakeVideo.Result()
        goal_handle.succeed()

        fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
        self.video_writer = None

        feedback_msg = TakeVideo.Feedback()
        start = time.time()

        video_path = f"{SAVE_PATH}/{goal_handle.request.video_name}.mp4"
        result.video_path = video_path

        self._run = True

        rate = self.create_rate(self.fps)

        while rclpy.ok():
            if not self._run:
                break
            feedback_msg.elapsed_time = int(time.time() - start)
            frame = self.img_node.get_frame()
            image = self.bridge.imgmsg_to_cv2(frame)

            if self.video_writer is None:
                image_w, image_h, _ = image.shape
                self.video_writer = cv2.VideoWriter(
                    video_path,
                    fourcc,
                    self.fps,
                    (image_w, image_h),
                )
            feedback_msg.image = self.bridge.cv2_to_compressed_imgmsg(image)
            self.video_writer.write(image)
            goal_handle.publish_feedback(feedback_msg)
            rate.sleep()
        
        # già rilasciato nella cancel callback (?)
        if self.video_writer is not None:
            self.video_writer.release()
            
        return result

    def cancel_callback(self, cancel_request):
        self.get_logger().warn(f"{cancel_request}")
        self._run = False
        if self.video_writer is not None:
            self.video_writer.release()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def destroy_node(self) -> bool:
        if self.video_writer is not None:
            self.video_writer.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    minimal_service = CameraService()

    try:
        rclpy.spin(minimal_service, MultiThreadedExecutor(2))
    except KeyboardInterrupt:
        minimal_service.get_logger().info("Closing Node (Keyboard Interrupt)")
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
