import time
from airdrone_actions.srv import TakePicture, StartRecording, StopRecording
# from airdrone_actions.action import TakeVideo
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
# from rclpy.action import ActionServer


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

        while self._image is None:
            rclpy.spin_once(self)

        return self._image


class CameraService(Node):
    def __init__(self):
        super().__init__("camera_service")
        self._srv_take = self.create_service(
            TakePicture, "take_picture", self.take_picture_service
        )

        self._srv_start = self.create_service(
            StartRecording, "start_recording", self.start_recording_service
        )

        self._srv_stop = self.create_service(
            StopRecording, "stop_recording", self.stop_recording_service
        )

        self.img_node = ImageSubNode()

        # self._action_server = ActionServer(
        #     self, TakeVideo, "take_video", self.take_video_action_service
        # )

        self.get_logger().info("Ready to serve")

    # def take_video_action_service(self, goal_handle):
    #     self.get_logger().info("Taking video...")
    #     result = TakeVideo.Result()
    #     return result

    def start_recording_service(self, request, response):
        self.get_logger().info(
            f"Start recording {request.video_name}"
        )
        response.image = self.img_node.get_frame()

        return response

    def stop_recording_service(self, request, response):
        self.get_logger().info(
            f"""Incoming request:
                detections: {request.include_detections} 
                geo: {request.include_geolocalization}
            """
        )
        response.image = self.img_node.get_frame()

        return response

    def take_picture_service(self, request, response):
        self.get_logger().info(
            f"""Incoming request:
                detections: {request.include_detections} 
                geo: {request.include_geolocalization}
            """
        )
        response.image = self.img_node.get_frame()

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = CameraService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.get_logger().info("Closing Node (Keyboard Interrupt)")
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
