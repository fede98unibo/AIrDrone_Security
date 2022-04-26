from airdrone_actions.srv import TakePicture

import rclpy
import cv2
import cv_bridge

from rclpy.node import Node


class CameraClient(Node):
    def __init__(self):
        super().__init__("camera_client")
        self.cli = self.create_client(TakePicture, "take_picture")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = TakePicture.Request()

    def send_request(self):
        self.get_logger().info("Sending request")
        self.req.include_detections = True
        self.req.include_geolocalization = False
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = CameraClient()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info("Service call failed %r" % (e,))
            else:
                minimal_client.get_logger().info("image received")
            break

    minimal_client.destroy_node()
    rclpy.shutdown()

    img = cv_bridge.CvBridge().imgmsg_to_cv2(response.image)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imshow("frame", img)
    cv2.waitKey(5_000)


if __name__ == "__main__":
    main()
