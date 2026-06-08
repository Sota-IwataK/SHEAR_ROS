#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import PoseStamped

from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.publisher import RosPublisher


DEFAULT_UNITY_PUBLISHERS = {
    "/palm_pose_world": PoseStamped,
    "/palm_pose_control_center_world": PoseStamped,
    "/palm_pose": PoseStamped,
    "/palm_pose_hmd_relative": PoseStamped,
}


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")

    publishers = {
        topic: RosPublisher(topic, message_class)
        for topic, message_class in DEFAULT_UNITY_PUBLISHERS.items()
    }
    tcp_server.loginfo(
        "Pre-registered Unity PoseStamped publish topics: {}".format(
            list(DEFAULT_UNITY_PUBLISHERS.keys())
        )
    )

    tcp_server.start(publishers=publishers)

    tcp_server.setup_executor()

    tcp_server.destroy_nodes()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
