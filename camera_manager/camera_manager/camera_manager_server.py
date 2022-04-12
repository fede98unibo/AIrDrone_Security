from airdrone_actions.srv import TakePicture
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node


class CameraService(Node):

    def __init__(self):
        super().__init__('camera_service')
        self.srv = self.create_service(TakePicture, 'take_picture', self.take_picture_service)
        self.image_subscriber = self.create_subscription(
            Image, "/image_raw", self.save, 1
        )
        self.image = None

    def take_picture_service(self, request, response):
        response.image = self.image
        self.get_logger().info(f'Incoming request\ndetections: {request.include_detections} geo: {request.include_geolocalization}')

        return response

    def save(self, image):
        self.image = image


def main(args=None):
    rclpy.init(args=args)

    minimal_service = CameraService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()