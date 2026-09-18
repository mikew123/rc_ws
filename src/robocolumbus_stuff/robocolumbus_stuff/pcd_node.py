"""
Robocolumbus26 pointcloud node.

Creates TOF pointclouds for all 6 sensors from distance data

Publish Combined Lidar and TOF point clouds
Publish number of points in combined point cloud within a xyz region
    Ignore distance = 0 and Inf
    Ignore Intensity
"""

import rclpy
import json
import time
import math
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from robocolumbus_interfaces.msg import Float32X8, TofDist

class PcdNode(Node):
    '''
    '''

    def __init__(self):
        super().__init__('pcd_node')
            
        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "json_msg"
                                        , self.json_msg_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan,"/scan" 
                                            , self.lidar_subscription_callback, 10)
        self.tof_dist_subscriber = self.create_subscription(TofDist, 'tof_dist'
                                            , self.tof_dist_callback, 10)

        # publish a topic for each TOF sensor fc=front_center etc
        self.tof_fc_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fc', 10)
        self.tof_fl_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fl', 10)
        self.tof_fr_pcd_publisher = self.create_publisher(PointCloud2, 'tof_fr', 10)
        self.tof_rc_pcd_publisher = self.create_publisher(PointCloud2, 'tof_rc', 10)
        self.tof_rl_pcd_publisher = self.create_publisher(PointCloud2, 'tof_rl', 10)
        self.tof_rr_pcd_publisher = self.create_publisher(PointCloud2, 'tof_rr', 10)
        self.tof_fc_mid_publisher = self.create_publisher(Float32X8, 'tof_fc_mid', 10) 

        self.combined_pcd_publisher = self.create_publisher(PointCloud2, "combined_pcd", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time.sleep(2) # wait for speaker node to be ready for json message!!??

        self.init_variables()

        self.tts("Point Cloud Node Started")
        self.get_logger().info(f"PcdNode Started")

    def init_variables(self) -> None:

        self.tof_fc_pcd = PointCloud2()
        self.tof_fl_pcd = PointCloud2()
        self.tof_fr_pcd = PointCloud2()
        self.tof_rc_pcd = PointCloud2()
        self.tof_rl_pcd = PointCloud2()
        self.tof_rr_pcd = PointCloud2()

    def lidar_subscription_callback(self, msg: LaserScan) -> None:
        """
        The "combined_pcd" point cloud is created every Lidar LaserScan topic
        by combining the 6 TOF pointclouds with the Lidar data
        adjust all xyz distances relative to base_footprint which is at z=0
        use frame xyz offsets and rpt angles to determine distances
        """
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = (np.float32(msg.angle_min)
                  + np.arange(ranges.size, dtype=np.float32)
                  * np.float32(msg.angle_increment))

        valid = (np.isfinite(ranges)
                 & (ranges >= np.float32(msg.range_min))
                 & (ranges <= np.float32(msg.range_max)))
        ranges = ranges[valid]
        angles = angles[valid]

        # All xyz points relative to "lidar_link" frame
        points = np.empty((ranges.size, 3), dtype=np.float32)
        points[:, 0] = ranges * np.cos(angles)
        points[:, 1] = ranges * np.sin(angles)
        points[:, 2] = 0.0

        itemsize = np.dtype(np.float32).itemsize
        fields = [PointField(name=name, offset=index * itemsize,
                             datatype=PointField.FLOAT32, count=1)
                  for index, name in enumerate(("x", "y", "z"))]
        
        lidar_pcd = PointCloud2(
            header=msg.header,
            height=1,
            width=points.shape[0],
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=points.strides[0],
            row_step=points.nbytes,
            data=points.tobytes())

       
        combined_pcd = lidar_pcd

        self.combined_pcd_publisher.publish(combined_pcd)


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

        # publish tof point clouds for each sensor
        # and save point cloud for creation of combined point cloud
        if tof_ab == "tof_fc" :
            pcd = self.point_cloud(xyz0, 'tof_fc_link')
            self.tof_fc_pcd_publisher.publish(pcd)
            self.tof_fc_pcd = pcd
        elif tof_ab == "tof_fl" :
            pcd = self.point_cloud(xyz0, 'tof_fl_link')
            self.tof_fl_pcd_publisher.publish(pcd)
            self.tof_fl_pcd = pcd
        elif tof_ab == "tof_fr" :
            pcd = self.point_cloud(xyz0, 'tof_fr_link')
            self.tof_fr_pcd_publisher.publish(pcd)
            self.tof_fr_pcd = pcd
        elif tof_ab == "tof_rc" :
            pcd = self.point_cloud(xyz0, 'tof_rc_link')
            self.tof_rc_pcd_publisher.publish(pcd)
            self.tof_rc_pcd = pcd
        elif tof_ab == "tof_rl" :
            pcd = self.point_cloud(xyz0, 'tof_rl_link')
            self.tof_rl_pcd_publisher.publish(pcd)
            self.tof_rl_pcd = pcd
        elif tof_ab == "tof_rr" :
            pcd = self.point_cloud(xyz0, 'tof_rr_link')
            self.tof_rr_pcd_publisher.publish(pcd)
            self.tof_rr_pcd = pcd

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

    def json_msg_callback(self, msg:String) -> None :
        """
        Get point cloud cubic region information for number of points
        {"pcd":{"xyz":{"x":x, "xlen":xlen, "y":y,"ylen":ylen, "z":z,"zlen":zlen}}}
        """
        # self.get_logger().info(f"json_msg_callback: {msg=}")
        data = json.loads(msg.data)

        if 'pcd' in data:
            pcd = data['pcd']
        else : return
        
        if "xyz" in pcd:
            xyz = pcd.xyz
            self.get_logger().info(f"json_msg_callback: {xyz=}")


    def destroy_node(self):
        self.get_logger().info("destroy_node")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PcdNode()
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