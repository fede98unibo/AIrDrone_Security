import time

from airdrone_actions.srv import TakePicture
from airdrone_actions.action import TakeVideo
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ActionServer, CancelResponse, ServerGoalHandle

from cv_bridge import CvBridge
import cv2

import numpy as np

SAVE_PATH = '/home/simone/Videos'


class ImageSubNode(Node):
    def __init__(self):
        super().__init__('img_sub_node')
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.save_callback, 1
        )
        self._image = None
        self.get_logger().info('Ready to grab frames')

    def save_callback(self, img: Image) -> None:
        self._image = img

    def get_frame(self) -> Image:
        self._image = None

        while self._not_ready():
            rclpy.spin_once(self)

        return self._image

    def _not_ready(self) -> bool:
        return self._image is None  # and geo is None ....


class CameraService(Node):
    def __init__(self):
        super().__init__('camera_service')
        self._srv_take = self.create_service(
            TakePicture, 'take_picture', self.take_picture_service
        )

        self.bridge = CvBridge()
        self.img_node = ImageSubNode()

        self._action_server = ActionServer(
            self,
            TakeVideo,
            'take_video',
            execute_callback=self.take_video_action_service,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
        )

        self.video_writer = None
        self.fps = 30
        self.get_logger().info('Ready to serve')

    def take_picture_service(self, request, response):
        self.get_logger().info(
            f'''Incoming request:
                detections: {request.include_detections} 
                geo: {request.include_geolocalization}
            '''
        )
        response.image = self.img_node.get_frame()

        return response

    async def take_video_action_service(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Taking video...')
        result = TakeVideo.Result()

        video_path = f'{SAVE_PATH}/{goal_handle.request.video_name}.avi'
        result.video_path = video_path

        fourcc = cv2.VideoWriter_fourcc(*'MJPG')

        feedback_msg = TakeVideo.Feedback()
        start = time.time()

        f = 0

        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                break

            feedback_msg.elapsed_time = int(time.time() - start)
            if f % 30 == 0:
                self.get_logger().info('Getting Frames')
                f = 0
            f += 1
            
            frame = self.img_node.get_frame()
            image = self.bridge.imgmsg_to_cv2(frame)

            self.get_logger().warn(f"{type(image).__name__}")

            if self.video_writer is None:
                self.get_logger().info('Init Writer')
                image_w, image_h, _ = image.shape
                self.video_writer = cv2.VideoWriter(
                    video_path,
                    fourcc,
                    self.fps, 
                    (640, 480) # TODO: controllare perchè fa il cazzone con queste shapes
                    # (int(image_w), int(image_h)),
                )

            feedback_msg.image = self.bridge.cv2_to_compressed_imgmsg(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.video_writer.write(image)
            goal_handle.publish_feedback(feedback_msg)

        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info('Releasing video')
            self.video_writer = None

        
        goal_handle.canceled()
        self.get_logger().info('Done')

        return result

    def cancel_callback(self, cancel_request):
        self.get_logger().warn(f'Received cancel request')
        return CancelResponse.ACCEPT


    def destroy_node(self) -> bool:
        if self.video_writer is not None:
            self.video_writer.release()
        self._action_server.destroy()
        self.img_node.destroy_node()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    minimal_service = CameraService()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(minimal_service, executor=executor)
    except KeyboardInterrupt:
        minimal_service.get_logger().info('Closing Node (Keyboard Interrupt)')
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
