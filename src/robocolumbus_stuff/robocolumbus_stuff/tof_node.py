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
    TOF sensors
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

        # publish a topic for each TOF sensor fc=front_center etc
        self.tof_fc_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fc', 10)
        self.tof_fl_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fl', 10)
        self.tof_fr_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fr', 10)
        self.tof_rc_pcd_publisher = self.create_publisher(PointCloud2, 'tof_rc', 10)
        self.tof_rl_pcd_publisher = self.create_publisher(PointCloud2, 'tof_rl', 10)
        self.tof_rr_pcd_publisher = self.create_publisher(PointCloud2, 'tof_rr', 10)
        self.tof_fc_mid_publisher = self.create_publisher(Float32X8, 'tof_fc_mid', 10)

        self.tof_dist_subscriber = self.create_subscription(TofDist, 'tof_dist'
                                            , self.tof_dist_callback, 10)

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

        # # publish front center mid row point cloud    
        # self.tof_pcd_publish(tof_ab, packet)
        
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

    # Process the TOF distance topics to create Point Clouds
    # Each topic message hass the TOF sensor name as well as the 64 distance points
    def tof_dist_callback(self, msg) -> None :
    
        tof_ab = msg.tof
        data = msg.dist # 64 int16 list
        self.tof_pcd_publish(tof_ab, data)

    # AMCL uses pcd for obstical detection
    # 8x8 point cloud for each sensor FOV 45degx45deg
    # calculate x,y,z for each point
    # TODO: Optimize math with numpy
    def tof_pcd_publish(self, tof_ab, data) -> None:
        # self.get_logger().info(f"tof_Publish: {tof_ab=} {data=}")

        fov = 45.0
        fovPt = fov/8 # FOV for each 8x8 sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians

        # There is a curvature in the distances of the sensors that needs to be corrected
        # Remove the curve by scaling each sensor with a inverted sin() curve over FOV
        # NOTE: This could be pre-computed outside the function since it is constant
        tofCurveCor = []
        for n in range(0,8) :
            theta:float = (n-4+0.5)*fovPtRad + math.pi/2
            s:float = math.sin(theta)
            if n == 0 : s0 = s
            tofCurveCor.append(s0/s)            

        # data is a liinear array of 64 values that needs to be converted to 8x8 array
        dist = np.array(data).reshape(8, 8)

        # pointcloud is a list of tupples(x,y,z)
        xyz0:list = []
        for m in range(0,8) : # Rows bottom to top
            theta_m = (m-4+0.5)*fovPtRad
            d_m = dist[m]
            for n in range(0, 8) : # Collumns left to right
                theta_n = (n-4+0.5)*fovPtRad
                d = d_m[n]
                if d==-1: 
                    # Bad data - set as infinate number (use NaN instead?)
                    xx0 = math.inf
                    yy0 = math.inf
                    zz0 = math.inf
                else :
                    Wx =  int(d*math.cos(theta_n)*tofCurveCor[n])
                    Wy = -int(d*math.sin(theta_n)*tofCurveCor[n])
                    Wz =  int(d*math.sin(theta_m)*tofCurveCor[n])
                    # Convert mm to meters
                    xx0 = np.float32(Wx/1000.0)
                    yy0 = np.float32(Wy/1000.0)
                    zz0 = np.float32(Wz/1000.0)
                
                xyz0.append((xx0,yy0,zz0))
        
        # self.get_logger().info(f"tof_Publish: {xyz0=}")

        if tof_ab == "tof_fc" :
            pcd = self.point_cloud(xyz0, 'tof_fc_link')
            self.tof_fc_pcd_publisher.publish(pcd)
        elif tof_ab == "tof_fl" :
            pcd = self.point_cloud(xyz0, 'tof_fl_link')
            self.tof_fl_pcd_publisher.publish(pcd)
        elif tof_ab == "tof_fr" :
            pcd = self.point_cloud(xyz0, 'tof_fr_link')
            self.tof_fr_pcd_publisher.publish(pcd)
        elif tof_ab == "tof_rc" :
            pcd = self.point_cloud(xyz0, 'tof_rc_link')
            self.tof_rc_pcd_publisher.publish(pcd)
        elif tof_ab == "tof_rl" :
            pcd = self.point_cloud(xyz0, 'tof_rl_link')
            self.tof_rl_pcd_publisher.publish(pcd)
        elif tof_ab == "tof_rr" :
            pcd = self.point_cloud(xyz0, 'tof_rr_link')
            self.tof_rr_pcd_publisher.publish(pcd)

        if tof_ab == "tof_fc" :
            # Publish the mid row distances as Float32X8 message for nav node
            msg = Float32X8()
            df = []
            for d in dist[3] :
                if d==-1 : d = math.inf
                else : d /= 1000.0 # convert mm to meters
                df.append(float(d))
            msg.data = df
            self.tof_fc_mid_publisher.publish(msg)

    def point_cloud(self, points_xy:list[tuple[np.float32]], parent_frame:str="map") -> PointCloud2:
        """
            Input list of tuples (x,y,z)
            Returns a point cloud to publish - Rviz can display it
        """
        points = np.asarray(points_xy)

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        #self.get_logger().info(f"{itemsize = } {fields = } {points = } {data = }")

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(
            frame_id=parent_frame,
            stamp = self.get_clock().now().to_msg(),
            )

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False, #Pi4
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of two float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )

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