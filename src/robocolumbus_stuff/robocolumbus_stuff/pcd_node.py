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
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
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

        self.tof_fc_pcd_subscription = self.create_subscription(PointCloud2, 'tof_fc'
                                        , self.tof_fc_subscription_callback, 10)
        self.tof_fl_pcd_subscription = self.create_subscription(PointCloud2, 'tof_fl'
                                        , self.tof_fl_subscription_callback, 10)
        self.tof_fr_pcd_subscription = self.create_subscription(PointCloud2, 'tof_fr'
                                        , self.tof_fr_subscription_callback, 10)
        self.tof_rc_pcd_subscription = self.create_subscription(PointCloud2, 'tof_rc'
                                        , self.tof_rc_subscription_callback, 10)
        self.tof_rl_pcd_subscription = self.create_subscription(PointCloud2, 'tof_rl'
                                        , self.tof_rl_subscription_callback, 10)
        self.tof_rr_pcd_subscription = self.create_subscription(PointCloud2, 'tof_rr'
                                        , self.tof_rr_subscription_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan,"/scan" 
                                            , self.lidar_subscription_callback, 10)

        
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

    def tof_fc_subscription_callback(self, msg: PointCloud2) -> None:
        self.tof_fc_pcd = msg

    def tof_fl_subscription_callback(self, msg: PointCloud2) -> None:
        self.tof_fl_pcd = msg

    def tof_fr_subscription_callback(self, msg: PointCloud2) -> None:
        self.tof_fr_pcd = msg

    def tof_rc_subscription_callback(self, msg: PointCloud2) -> None:
        self.tof_rc_pcd = msg

    def tof_rl_subscription_callback(self, msg: PointCloud2) -> None:
        self.tof_rl_pcd = msg

    def tof_rr_subscription_callback(self, msg: PointCloud2) -> None:
        self.tof_rr_pcd = msg

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