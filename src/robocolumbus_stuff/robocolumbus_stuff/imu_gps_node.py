"""Robocolumbus25 IMU & GPS node.

Reads JSON-formatted sensor packets from a serial-connected RP2040 and
publishes standard ROS2 messages. Expected input over serial contains
`imu`, `gps`, and `cmp` JSON objects. The node publishes IMU, GPS and
pose information, publishes a simple compass azimuth, and publishes
calibration status messages.

Topics (partial):
- Subscribes: `json_msg` (`std_msgs/String`), `gps_nav` (`sensor_msgs/NavSatFix`)
- Publishes: `imu` (`sensor_msgs/Imu`), `imu/cal` (`robocolumbus_interfaces/ImuCal`),
    `gps_nav` (`sensor_msgs/NavSatFix`), `gps_pose` (`geometry_msgs/Pose`),
    `cmp_azi` (`std_msgs/Float32`), and `json_msg` (`std_msgs/String`).

Behavior summary:
- Opens a serial connection to an RP2040-based IMU/GPS controller and
    continuously reads lines of JSON. When packets contain `imu`, the
    node composes and publishes `sensor_msgs/Imu` (orientation, angular
    velocity, linear acceleration) and a custom `ImuCal` status message.
- When packets contain `gps` the node publishes `sensor_msgs/NavSatFix`
    and computes a local `gps_pose` offset relative to startup.
- When packets contain `cmp` the node can publish compass/azimuth
    information.

The node also sends configuration commands to the serial device and
emits simple TTS-style JSON messages via `json_msg` for status.
"""

from ctypes.wintypes import PMSG

from numpy import int32
import rclpy
import json
import serial
import math
import time
import tf_transformations

from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

from robocolumbus_interfaces.msg import ImuCal

class ImuGpsNode(Node):
    '''
    This processes the serial port from the RP2040 for the IMU and GPS sensors
    in the cabin
    '''

    timerRateHz = 100.0; # Rate to check serial port for messages
    serial_port = ["/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6635C469F25492A-if00"
                   ,"/dev/serial/by-id/usb-Waveshare_RP2040_PiZero_E6635C469F25492A-if00"]
    serial_port_idx:int = 0

    baudrate = 1000000

    laccJsonPacket = None
    rvelJsonPacket = None
    rvecJsonPacket = None

    gpsCntr    = 0
    gpsCnt     = 10
    gpsLatYAcc = 0.0
    gpsLonXAcc = 0.0

    minSIV = 3 #5
    
    # lat, lon, alt
    gpsCovariance = [
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01
    ]

    #x, y, z
    imuOrientationCovariance =  [
        0.001, 0.0, 0.0,
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001
    ]

    #x, y, z
    imuAngularVelocityCovariance = [
        0.001, 0.0, 0.0,
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001
    ]

    #x, y, z
    imuLinearVelocityCovariance = [
        0.001, 0.0, 0.0,
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001
    ]

    def __init__(self):
        super().__init__('imu_gps_node')

        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.roll_filter_history = []
        self.pitch_filter_history = []
        self.yaw_filter_history = []

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
            
        self.openSerialPort()

        # self.imu_test_publisher = self.create_publisher(String, 'imu_test', 10)
        self.imu_msg_publisher = self.create_publisher(Imu, 'imu', 10)
        self.imu_cal_publisher = self.create_publisher(ImuCal, 'imu/cal', 10)
        self.gps_nav_publisher = self.create_publisher(NavSatFix, 'gps_nav', 10)
        self.gps_pose_publisher = self.create_publisher(Pose, 'gps_pose', 10)
        self.cmp_azi_publisher = self.create_publisher(Float32, 'cmp_azi', 10)

        self.gps_nav_subscription = self.create_subscription(NavSatFix,"gps_nav"
                                        , self.gps_nav_subscription_callback, 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback
                                       , callback_group=self.cb_group)
        
        # configure interface
        self.send_json_cmd({"cfg":{"imu":True, "gps":True, "cmp":False}})

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("IMU and GPS Node Started")               
        self.get_logger().info(f"ImuGpsNode Started")


    def median5_filter(self, value: float, history: list[float]) -> float:
        """Apply a simple median-5 filter to a single value."""
        history.append(value)
        if len(history) > 5:
            history.pop(0)

        if len(history) < 5:
            return value

        sorted_values = sorted(history)
        return sorted_values[2]

    def tts(self, tts) -> None:
        """
        Send text to speaker 
        """
        json_msg = {"speaker":{"tts":tts}}
        self.sendJsonMsg(json_msg)

    def sendJsonMsg(self, json_msg) -> None :
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
                self.ser = serial.Serial(self.serial_port[self.serial_port_idx], self.baudrate, timeout=1)
                self.get_logger().info(f"openSerialPort: Serial port {self.serial_port[self.serial_port_idx]} opened.")
                serialOpen = True

            except serial.SerialException as e :
                self.get_logger().info(f"openSerialPort: Failed to open serial port: {e}")
                self.get_logger().info("openSerialPort: Try opening serial port again using other port name")
                self.serial_port_idx +=1
                if self.serial_port_idx>1 : self.serial_port_idx=0

    # get data from serial port, returns a line of text
    def getSerialData(self) -> str :
        # Check if a line has been received on the serial port
        err:bool=False
        try :
            if self.ser.in_waiting > 0 :
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

        try :
            unknown = True
            packet = json.loads(received_data)
            if "imu" in packet :
                self.imuPublish(packet.get("imu"))
                unknown = False
            if "gps" in packet :
                self.gpsPublish(packet.get("gps"))
                unknown = False
            if "cmp" in packet :
                self.cmpPublish(packet.get("cmp"))
                unknown = False
            if unknown :
                self.get_logger().info(f"IMU GPS serial json unknown : {received_data}")
                return  
        except Exception as ex:
            self.get_logger().error(f"IMU GPS serial json Exception {ex} : {received_data}")
            return

    # GPS pose
    def gps_nav_subscription_callback(self, msg: NavSatFix) -> None:
        
        lon = msg.longitude
        lat = msg.latitude
        
        latY = (lat/360)*40007863
        #lonX = ((lon/360)*(math.cos((lat/360)*2*math.pi)))*40075017
        lonX = ((lon/360)*(1.0))*40075017
        
        # Offset relative lat lon to zero at start
        if self.gpsCntr < self.gpsCnt :
            self.gpsCntr +=1
            self.gpsLatYAcc += latY
            self.gpsLonXAcc += lonX
            
        latY = latY - self.gpsLatYAcc/self.gpsCntr
        lonX = lonX - self.gpsLonXAcc/self.gpsCntr
        
        pmsg = Pose()
        pmsg.position.x = lonX
        pmsg.position.y = latY

        self.gps_pose_publisher.publish(pmsg)
        
        #self.get_logger().info(f"gps_msg_subscription_callback : {latY=} {lonX=}")
        
    # Process Compass data
    def cmpPublish(self, cmpJsonPacket) -> None:        
        #self.get_logger().info(f"cmpPublish : {cmpJsonPacket=}")
        # msg = Int32()
        # msg.data = cmpJsonPacket.get("azi")
        # self.cmp_azi_publisher.publish(msg)
        pass

    # Process GPS data
    def gpsPublish(self, gpsJsonPacket) -> None:        
        # self.get_logger().info(f"gpsPublish : {gpsJsonPacket=}")

        msg = NavSatFix()

        # NOTE: should we use the imu timestamp to get better accuracy?
        msg.header.stamp = self.get_clock().now().to_msg()
        # The IMU is located close to the rear differential 
        # and aligned XY so no offest is needed
        msg.header.frame_id = "gps_link"

        # TODO: status fields
        # put num sat in view into status field service
        status = gpsJsonPacket.get("status")
        siv = gpsJsonPacket.get("siv")
        # num satelites considered stable
        if (status==True) and (siv>=self.minSIV):
            # Satelite fix is OK
            msg.status.status = NavSatStatus.STATUS_FIX
            # setting servive to satelites in view since we dont know which sat are used
            # msg.status.service = NavSatStatus.SERVICE_GPS \
            #     + NavSatStatus.SERVICE_GLONASS + NavSatStatus.SERVICE_GALILEO
            msg.status.service = siv
            msg.latitude  = 1e-7*gpsJsonPacket.get("lat")
            msg.longitude = 1e-7*gpsJsonPacket.get("lon")
            msg.altitude  = 1e-3*gpsJsonPacket.get("alt") # mm to Meters
        else :
            # Not a valid satelite fix
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.status.service = siv
            msg.latitude  = math.nan
            msg.longitude = math.nan
            msg.altitude  = math.nan

        msg.position_covariance = self.gpsCovariance
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_nav_publisher.publish(msg)

    # Process IMU data
    imuCalRvel = -1
    imuCalRvec = -1
    imuCalLacc = -1
    imuCalTimer = 0
    imuCalTimerInterval = 1.0

    def imuPublish(self, imuJsonPacket) :        
        # self.get_logger().info(f"imuPublish : {imuJsonPacket=}")

        timeSec = time.monotonic()

        # TODO: collect lacc, rvel, rvec packets with same seq# and publish imu message
        try :

            if "lacc" in imuJsonPacket :
                self.laccJsonPacket = imuJsonPacket.get("lacc")
            elif "rvel" in imuJsonPacket :
                self.rvelJsonPacket = imuJsonPacket.get("rvel")
            elif "rvec" in imuJsonPacket :
                self.rvecJsonPacket = imuJsonPacket.get("rvec")
            else :
                self.get_logger().error("IMU json does not have lacc, rvel or rvec")

            # process all 3 imu data sets when next set is available
            if self.laccJsonPacket!=None and  self.rvelJsonPacket!=None and  self.rvecJsonPacket!=None :
                # laccSeq = self.laccJsonPacket.get("seq")
                # rvelSeq = self.rvelJsonPacket.get("seq")
                # rvecSeq = self.rvecJsonPacket.get("seq")

                # seq numbers are not clustered properly to have the same seq# for each data type
                # This seems to work better when this is the only node running
                if True : #laccSeq==rvelSeq and rvelSeq==rvecSeq :
                    #self.get_logger().info(f"{laccSeq=} {rvelSeq=} {rvecSeq=}")

                    ############################################################
                    # IMU
                    msg = Imu()
                    # NOTE: should we use the imu timestamp to get better accuracy?
                    t = self.get_clock().now()
                    # Offset the perceived imu delay - doesnt seem to work
                    # t = t - Duration(seconds=2.0)
                    msg.header.stamp = t.to_msg()
                    # The IMU is located at the centroid of the rear differential 
                    # and aligned XY so no offest is needed
                    msg.header.frame_id = "imu_link"

                    ######################
                    # IMU Rotation vectors - compass
                    #self.get_logger().info(f"imuPublish : {self.rvecJsonPacket=}")
                    i = float(self.rvecJsonPacket.get("i"))
                    j = float(self.rvecJsonPacket.get("j"))
                    k = float(self.rvecJsonPacket.get("k"))
                    real = float(self.rvecJsonPacket.get("real"))

                    # adjust by 90 deg = pi/2 (1.5708)
                    (ex,ey,ez) =  tf_transformations.euler_from_quaternion([i,j,k,real])
                    ex = self.median5_filter(ex, self.roll_filter_history)
                    ey = self.median5_filter(ey, self.pitch_filter_history)
                    ez += math.pi/2
                    ez = self.median5_filter(ez, self.yaw_filter_history)
                    # limit yaw to +-pi
                    if ez>+math.pi : ez -= 2*math.pi
                    if ez<-math.pi : ez += 2*math.pi
                    # swap roll and pitch to match IMU orientation
                    # invert pitch polarity
                    ey, ex = -ex, ey
                    (qx,qy,qz,qw) = tf_transformations.quaternion_from_euler(ex,ey,ez)

                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    msg.orientation.w = qw
                    msg.orientation_covariance = self.imuOrientationCovariance

                    #########################
                    # IMU Rotational velocities
                    #self.get_logger().info(f"imuPublish : {self.rvelJsonPacket=}")
                    msg.angular_velocity.x =  float(self.rvelJsonPacket.get("x"))
                    msg.angular_velocity.y =  float(self.rvelJsonPacket.get("y"))
                    msg.angular_velocity.z =  float(self.rvelJsonPacket.get("z"))
                    msg.angular_velocity_covariance = self.imuAngularVelocityCovariance

                    ###########################
                    # IMU Linear accelerations
                    #self.get_logger().info(f"imuPublish : {self.laccJsonPacket=}")
                    msg.linear_acceleration.x = float(self.laccJsonPacket.get("x"))
                    msg.linear_acceleration.y = float(self.laccJsonPacket.get("y"))
                    msg.linear_acceleration.z = float(self.laccJsonPacket.get("z"))
                    msg.linear_acceleration_covariance = self.imuLinearVelocityCovariance

                    self.imu_msg_publisher.publish(msg)


                    ######################################################################
                    # get calibration statuses
                    imuCalRvel = int(self.rvelJsonPacket.get("stat"))
                    imuCalRvec = int(self.rvecJsonPacket.get("stat"))
                    imuCalLacc = int(self.laccJsonPacket.get("stat"))

                    imuCalMsg = ImuCal()
                    imuCalMsg.rvel = imuCalRvel
                    imuCalMsg.rvec = imuCalRvec
                    imuCalMsg.lacc = imuCalLacc
                    self.imu_cal_publisher.publish(imuCalMsg)

                    if (self.imuCalRvec != imuCalRvec) :
                        self.imuCalRvec = imuCalRvec
                        self.tts(f"IMU calibration status {imuCalRvec}")
                        jsonMsg = {"nav":{"imu_cal_status":imuCalRvec}}
                        self.sendJsonMsg(jsonMsg)

                    if (timeSec > self.imuCalTimer) :
                        self.imuCalTimer = timeSec + self.imuCalTimerInterval
                        jsonMsg = {"nav":{"imu_cal_status":imuCalRvec}}
                        self.sendJsonMsg(jsonMsg)

                    # wait for 3 new packets
                    self.rvecJsonPacket = None
                    self.rvelJsonPacket = None
                    self.laccJsonPacket = None

                    #######################################################
                    #publish /cmp_azi = yaw (azimuth)
                    yaw = Float32()
                    yaw.data = ez
                    self.cmp_azi_publisher.publish(yaw)

        except Exception as ex:
            self.get_logger().error(f"imuPublish json exception : {ex}")
            return

    
    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = ImuGpsNode()
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