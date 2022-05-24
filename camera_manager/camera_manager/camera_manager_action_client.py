# # Copyright 2019 Open Source Robotics Foundation, Inc.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# # from example_interfaces.action import Fibonacci
# from airdrone_actions.action import TakeVideo

# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node


# class MinimalActionClient(Node):

#     def __init__(self):
#         super().__init__('minimal_action_client')
#         self._action_client = ActionClient(self, TakeVideo, 'take_video')

#     def cancel_done(self, future):
#         cancel_response = future.result()
#         if len(cancel_response.goals_canceling) > 0:
#             self.get_logger().info('Goal successfully canceled')
#         else:
#             self.get_logger().info('Goal failed to cancel')

#         rclpy.shutdown()

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected :(')
#             return

#         self._goal_handle = goal_handle

#         self.get_logger().info('Goal accepted :)')

#         # Start a 2 second timer
#         self._timer = self.create_timer(2.0, self.timer_callback)

#     def feedback_callback(self, feedback):
#         self.get_logger().info(f'Received feedback: {type(feedback).__name__}')

#     def timer_callback(self):
#         self.get_logger().info('Canceling goal')
#         # Cancel the goal
#         future = self._goal_handle.cancel_goal_async()
#         future.add_done_callback(self.cancel_done)

#         # Cancel the timer
#         self._timer.cancel()

#     def send_goal(self):
#         self.get_logger().info('Waiting for action server...')
#         self._action_client.wait_for_server()

#         goal_msg = TakeVideo.Goal()
#         goal_msg.video_name = "cacca"

#         self.get_logger().info('Sending goal request...')

#         self._send_goal_future = self._action_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self.feedback_callback)

#         self._send_goal_future.add_done_callback(self.goal_response_callback)


# def main(args=None):
#     rclpy.init(args=args)

#     action_client = MinimalActionClient()

#     action_client.send_goal()

#     rclpy.spin(action_client)


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from airdrone_actions.action import TakeVideo
import cv2
from cv_bridge import CvBridge


class CameraActionClient(Node):
    def __init__(self):
        super().__init__("camera_action_client")
        self._action_client = ActionClient(self, TakeVideo, "take_video")
        self.elapsed_time = 0
        self.bridge = CvBridge()
        self.f = 0

    def send_goal(self, video_name):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        goal_msg = TakeVideo.Goal()
        goal_msg.video_name = video_name

        self._action_client.wait_for_server(10)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        self.get_logger().info("Canceling goal")
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._goal_handle = goal_handle

        # self._get_result_future = goal_handle.get_result_async()
        # self._get_result_future.add_done_callback(self.get_result_callback)

        # Start a 2 second timer
        self._timer = self.create_timer(2.0, self.cancel_goal)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.video_path}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.elapsed_time = feedback.elapsed_time

        if self.f % 30 == 0:
            self.f = 0 
            self.get_logger().info(f"Received feedback: {self.elapsed_time}")
        self.f += 1 
        
        image = self.bridge.compressed_imgmsg_to_cv2(feedback.image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cv2.imshow("video", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    action_client = CameraActionClient()

    action_client.send_goal("test")

    # while action_client.elapsed_time < 3:
    #     rclpy.spin_once(action_client)

    # action_client.cancel_goal()

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
