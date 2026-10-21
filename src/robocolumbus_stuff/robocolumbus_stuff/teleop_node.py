#!/usr/bin/env python3
"""Robocolumbus25 teleoperation node.

Converts joystick `sensor_msgs/Joy` messages from `/joy` into
`geometry_msgs/Twist` messages on `/cmd_vel/teleop`. Publishes JSON
payloads as `std_msgs/String` on `json_msg` for TTS and button events
(e.g. kill/do). Applies max speed and steering limits and computes
angular velocity from steering via wheelbase geometry.
"""

import rclpy
import math
import json
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class Robocolumbus25TeleopNode(Node):
    maxLinearX: float       = 1.0   # Meters per second
    maxSteerAngleRad: float = 0.698   # ~40 degrees
    wheelBase:float         = 0.490 # meters
    cmd_vel_zero:bool = False
    lastLinearX: float = 0.0

    def __init__(self):
        super().__init__('rc25_teleop_node')


        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)

        self.joy_subscription = self.create_subscription(Joy, '/joy'
                                    , self.joy_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel/teleop', 10)

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("Teleop Node Started")
        self.get_logger().info("Teleop Node Started")


    def tts(self, tts) -> None:
        """
        Send text to speaker
        """
        json_msg = {"speaker":{"tts":tts}}
        self.sendJsonMsg(json_msg)

    def sendJsonMsg(self, json_msg) -> None :
        #self.get_logger().info(f"{json_msg=}")
        str = json.dumps(json_msg)
        msg = String(data=str)
        self.json_msg_publisher.publish(msg)

    def json_msg_callback(self,msg) :
        pass

    # Get button commands from Joy message
    buttonsLast = [0]*10
    axesLast = [0]*10

    def joy_callback(self, msg:Joy):
        cmd_vel = Twist()

        axes = [0.0]*10
        buttons = [0]*10

        # get controller axes and button values
        for i in range(0,8): #0-7
            axes[i] = msg.axes[i]
        for i in range(0,10):
            buttons[i] = msg.buttons[i]


        # Send /cmd_vel to move robot
        if not(axes[1]==0.0 and axes[3]==0.0) :
            # GAMESIR controller
            # throttle = axes[1]
            # steer = axes[3]

            # NC300 controller
            throttle = axes[1]
            steer = axes[0]

            linearX       = throttle * self.maxLinearX
            steerAngleRad =    steer * self.maxSteerAngleRad

            # Basic steering calculation wheel angle to angular velocity
            if math.fabs(linearX) < 0.001 :
                linearX = 0.0
                angularZ = 0.0
            else :
                angularZ = math.tan(steerAngleRad) * linearX / self.wheelBase

            if abs(linearX-self.lastLinearX) > 0.05 :
                self.get_logger().info(f"{throttle=:.3f} {steer=:.3f} : {linearX=:.3f} {angularZ=:.3f} {steerAngleRad=:.3f}")
            self.lastLinearX = linearX
            
            # Publish /cmd_vel
            cmd_vel.linear.x = linearX
            cmd_vel.angular.z = angularZ
            self.cmd_vel_publisher.publish(cmd_vel)

        # Publish buttons when change occurs
        if buttons[4] != self.buttonsLast[4] :
            b:bool = (buttons[4]==1)
            json_msg = {"nav":{"buttonKill":b}}
            self.sendJsonMsg(json_msg)

        if buttons[5] != self.buttonsLast[5] :
            b:bool = (buttons[5]==1)
            json_msg = {"nav":{"buttonDo":b}}
            self.sendJsonMsg(json_msg)

        # save axes and buttons values
        for i in range(0,8):
            self.axesLast[i] = axes[i]
        for i in range(0,10):
            self.buttonsLast[i] = buttons[i]

        # self.get_logger().info(f"{buttons=} {self.buttonsLast=} {axes=} {self.axesLast=}")

    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = Robocolumbus25TeleopNode()

    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

    try :
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        from rclpy.impl import rcutils_logger
        logger = rcutils_logger.RcutilsLogger(name="node")
        logger.info('Received Keyboard Interrupt (^C). Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
