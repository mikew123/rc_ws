"""Robocolumbus25 Time-of-Flight (TOF) node.

Reads distance data from serial-connected RP2040 TOF sensors (front/rear, left/center/right),
publishes point clouds and raw distance arrays for each sensor, and emits TTS/status via `json_msg`.

Topics (partial):
- Subscribes: `json_msg` (`std_msgs/String`).
- Publishes: `tof_fc`, `tof_fl`, `tof_fr`, `tof_rc`, `tof_rl`, `tof_rr` (`sensor_msgs/PointCloud2`),
    `tof_fc_mid` (`robocolumbus_interfaces/Float32X8`), `tof_dist` (`robocolumbus_interfaces/TofDist`), `json_msg` (`std_msgs/String`).

Behavior summary:
- Opens serial connection, reads JSON packets for each TOF sensor, publishes 8x8 point clouds
    (with curvature correction) and raw distance arrays, and emits TTS/status messages at startup.
"""

import rclpy
import json
import serial
import math
import numpy as np
import time

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import String
from robocolumbus_interfaces.msg import Float32X8, TofDist

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class TofNode(Node):
    '''
    This processes the serial port from the RP2040 for the front and rear
    TOF sensors and creates tof_dist topics 
    '''

    timerRateHz = 100.0; # Rate to check serial port for messages
    serial_port = ["/dev/serial/by-id/usb-Waveshare_RP2040_Zero_45533065790A3B5A-if00"
                   ,"/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_45533065790A3B5A-if00"]
    serial_port_idx:int = 0

    baudrate = 1000000

    def __init__(self):
        super().__init__('tof_node')

        self.cb_group = MutuallyExclusiveCallbackGroup()


        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)


        self.openSerialPort()


        self.tof_dist_publisher = self.create_publisher(TofDist, 'tof_dist', 10)
      
        # timer to check serial port
        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback
                                       , callback_group=self.cb_group)
        
        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("Time of Flight Node Started")
        self.get_logger().info(f"TofNode Started")


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

    def openSerialPort(self) :
        # Open serial port to tof sensors controller over USB
        # continuously try to open it
        serialOpen = False
        while not serialOpen :
            try :
                self.ser = serial.Serial(self.serial_port[self.serial_port_idx], self.baudrate, timeout=0.1)
                self.get_logger().info(f"openSerialPort: Serial port {self.serial_port[self.serial_port_idx]} opened.")
                serialOpen = True

            except serial.SerialException as e :
                self.get_logger().info(f"openSerialPort: Failed to open serial port: {e}")
                self.get_logger().info("openSerialPort: Try opening serial port again, cycle between ports")
                self.serial_port_idx +=1
                if self.serial_port_idx > 1 : self.serial_port_idx = 0

    # get data from serial port, returns a line of text
    def getSerialData(self) -> str :
        # Check if a line has been received on the serial port
        err:bool=False
        try :
            if self.ser.in_waiting > 200 : # 200 is min TOF string size 
                received_data:str = self.ser.readline().decode().strip()
                # self.get_logger().info(f"getSerialData: {received_data=}")
                return received_data # Exit while 1 loop
            else :
                return None
            
        except Exception as ex :
            self.get_logger().info(f"getSerialData: serial read failure : {ex}")
            err=True

        if err :
            try :
                self.ser.close()    
                # self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.openSerialPort()
            except serial.SerialException as e:
                self.get_logger().info(f"getSerialData: Failed to open serial port: {e}")

    def send_json_cmd(self,cmd) :
        # self.get_logger().info(f"send_json_cmd: {cmd=}")
        if self.ser and self.ser.is_open:
            try:
                json_cmd = json.dumps(cmd) + '\n'
                self.ser.write(json_cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")
    
    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
    
        received_data = self.getSerialData()
        if received_data == None : return
    
        # self.get_logger().info(f"{received_data=}")

        try :
            unknown = True
            packet = json.loads(received_data)
            if "tof_fc" in packet :
                tof_ab = "tof_fc"
                unknown = False
            elif "tof_fl" in packet :
                tof_ab = "tof_fl"
                unknown = False
            elif "tof_fr" in packet :
                tof_ab = "tof_fr"
                unknown = False
            elif "tof_rc" in packet :
                tof_ab = "tof_rc"
                unknown = False
            elif "tof_rl" in packet :
                tof_ab = "tof_rl"
                unknown = False
            elif "tof_rr" in packet :
                tof_ab = "tof_rr"
                unknown = False

            if unknown :
                self.get_logger().info(f"TOF sensors serial json unknown : {received_data}")
                return  
        except Exception as ex:
            self.get_logger().error(f"TOF sensors serial json Exception {ex} : {received_data}")
            return
        
        # publish raw data from sensors
        self.tof_sensor_publish(tof_ab, packet)
        
    # publish the "raw" TOF sensor distance data
    # nav_node uses the distance data instead of pcd
    # maybe we process the data and send a minimum instead of all data
    def tof_sensor_publish(self, tof_ab, packet) -> None :
        if tof_ab in packet :
            tof = packet.get(tof_ab)
        else : return

        if "dist" in tof :
            dist = np.int16(tof.get("dist")).reshape(64)
        else : return
        # self.get_logger().info(f"tof_sensor_publish: {tof_ab} {dist=}")

        msg = TofDist()
        msg.tof = tof_ab
        msg.dist = dist
        self.tof_dist_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = TofNode()
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
    
if __name__ == '__main__':
    main()