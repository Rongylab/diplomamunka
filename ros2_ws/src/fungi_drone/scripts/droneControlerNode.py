
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy




# import tf
# from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
# from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.msg import RCIn
# from mavros_msgs.srv import CommandBool
# from mavros_msgs.srv import SetMode
# from mavros_msgs.srv import CommandTOL
import math

pi_2 = math.pi / 2.0



class droneControler(Node):
    def __init__(self):
        super().__init__("droneControler")
        self.subscription = self.create_subscription(
            Joy,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%f"' % msg.axes[0])
        print("Running")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = droneControler()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()









def demo_fly():
    print("kuki")




if __name__=="__main__":
    demo_fly()