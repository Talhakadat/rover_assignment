#!/usr/bin/env python3
# @author alpogant
# Date: 2022-08-10
# iturover team software subteam assignment part 2
from rcl_interfaces.msg import SetParametersResult
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import bz2
import binascii

class msg_class(Node):
    def __init__(self):
        super().__init__("bz2_node")
        self.declare_parameter("message", "somewhere, something incredible is waiting to be known.")
        self.pub = self.create_publisher(String, "/bz2_message", 10)
        self.msg = self.get_parameter("message").value.encode("utf-8")
        self.add_on_set_parameters_callback(self.param_callback)

        self.compress()
        self.get_logger().info("BZ2 Compressed Message Publisher Node Started!")
        self.timer = self.create_timer(0.2, self.run)

    def param_callback(self, params):
        for param in params:
            if param.name == "message" and param.type_ == param.Type.STRING:
                self.msg = param.value.encode("utf-8")
                self.compress()
                self.get_logger().info(f"Parameter updated: {param.value}")
        return SetParametersResult(successful=True)


    def compress(self):
        self.compressed_msg = bz2.compress(self.msg)
        self.get_logger().info(str(self.compressed_msg))
        self.compressed_ascii = binascii.hexlify(self.compressed_msg).decode("ascii")
        self.get_logger().info(self.compressed_ascii)

    def run(self):
        pubm = String()
        pubm.data = self.compressed_ascii
        self.pub.publish(pubm)

def main(args=None):
    rclpy.init(args=args)
    node = msg_class()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()