"""Robocolumbus25 Cone detection node.

Processes camera detections and LIDAR scans to detect and publish cone
positions. Listens for `vision_msgs/Detection3DArray` from an OAK-D-Lite
detector and `sensor_msgs/LaserScan` from a LIDAR, validates detections,
applies simple filtering, and publishes `geometry_msgs/PointStamped`
positions for both camera (`/cone_point_cam`) and lidar
(`/cone_point_lidar`). The node also uses `json_msg` for status/TTS and
publishes informational TTS at startup and on sensor state changes.

Topics (partial):
- Subscribes: `/color/spatial_detections` (`vision_msgs/Detection3DArray`),
    `/scan` (`sensor_msgs/LaserScan`), `/json_msg` (`std_msgs/String`).
- Publishes: `/cone_point_cam` (`geometry_msgs/PointStamped`),
    `/cone_point_lidar` (`geometry_msgs/PointStamped`), `/json_msg`
    (`std_msgs/String`).

Behavior summary:
- Validates camera detections with bounding-box shape/size checks and a
    median-7 distance filter to reduce spikes, then translates camera
    coordinates to ROS frames and publishes `/cone_point_cam`.
- Scans LIDAR ranges within a forward FOV, looks for distance jumps to
    find contiguous cone-like clusters, validates candidate width/range
    criteria, computes an (x,y) position and publishes `/cone_point_lidar`.
"""

import rclpy
import math
import json
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import LaserScan

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ConeNode(Node):
    '''
    Cone locations etc
    '''

    # median 5 filter memory for tof_foc data
    # list of tupples [7X(x,y,z)]
    m7_filter:list = [(0.0,0.0,0.0)]*7

    def __init__(self):
        super().__init__('cone_node')
            

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "/json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "/json_msg"
                                        , self.json_msg_callback, 10)

        self.cone_point_cam_publisher = self.create_publisher(PointStamped, '/cone_point_cam', 10)
        self.cone_point_lidar_publisher = self.create_publisher(PointStamped, '/cone_point_lidar', 10)

        self.cone_det_cam_subscription = self.create_subscription(Detection3DArray,"/color/spatial_detections", 
                                            self.cone_det_cam_subscription_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan,"/scan" 
                                            , self.lidar_subscription_callback, 10)

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("Cone Node Started")               
        self.get_logger().info(f"ConeNode Started")

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

    ######################################################################################################
    # Camera cone detection parameters
    cameraMsgDetected = False
    cameraConeDetection = False
    coneCounter = 0
    coneLastDetXYZ = (0.0,0.0,0.0)

    # Cone detection from oak-D-Lite camera AI - publish /cone_point/cam
    def cone_det_cam_subscription_callback(self, msg: Detection3DArray) -> None:
        # self.get_logger().info(f"{msg=}")

        if not self.cameraMsgDetected :
            # speak once at startup
            self.tts("Camera is active")
            self.cameraMsgDetected = True

        # default cone at 0,0,0 - Invalid
        x = 0.0
        y = 0.0
        z = 0.0

        detections = msg.detections
        num_detections = 0

        # TODO: filter out for best cone detection when multiple occur
        for detection in detections :
            #header = detection.header
            results = detection.results
            bbox = detection.bbox
            bboxSize = bbox.size
            for result in results :
                pose = result.pose.pose
                position = pose.position
                cx = position.x
                cy = position.y
                cz = position.z
                if cz==0 : continue

                # use bounding box to "filter" non-cones
                bx0 = bboxSize.x
                by0 = bboxSize.y
                bx1 = 33*(2.8/cz) # z is distance
                by1 = 59*(2.8/cz)
                xx = math.fabs(1.0 - bx0/bx1)
                yy = math.fabs(1.0 - by0/by1)
                if z>1.2 : 
                    pctx = 0.35 # height when more than 1.2 meter
                    pcty = 0.25 # width
                else : 
                    pctx = 0.55 # top or bottom of cone may be out of FOV
                    pcty = 0.55 # width
                if (xx < pctx) and (yy < pcty) :
                    # Cone validated using bbox shape and size
                    num_detections +=1
                    # Use median-7 filter to remove points with distance spikes
                    m7 = self.m7_filter
                    # add new sample to filter list memory
                    m7 = [(cx,cy,cz),m7[0],m7[1],m7[2],m7[3],m7[4],m7[5]]
                    self.m7_filter = m7
                    # sort tupple[2] z is distance (m7 is not modified)
                    m7s = sorted(m7, key=lambda xyz: xyz[2])

                    # pick the middle sample after sorting
                    (mx,my,mz) = m7s[3]

                    # translate to ROS coordinates
                    (x,y,z) = (mz,-mx,my) # (x,y) and z=distance

                    # self.get_logger().info(f"cone detect: {x=:.2f}, {y=:.2f}, {z=:.2f}")
                    break # exit detections loop after 1st validated cone

        if self.cameraConeDetection :
            # change to not detected after N consecutive no cone
            if num_detections == 0 :
                self.coneCounter +=1
                if self.coneCounter > 10 :
                    # 10 consecutive cone not detected
                    self.cameraConeDetection = False
                    self.tts("Cam lost cone")

                else :
                    # Send last detected cone coordinates
                    (x,y,z) = self.coneLastDetXYZ
                
            else : # cone detection
                # reset counter when cone is detected
                self.coneCounter = 0
                self.coneLastDetXYZ = (x,y,z)

        else : # Cone not detected
            if num_detections>0 and x>0:
                self.coneCounter +=1
                if self.coneCounter > 4 :
                    # 4 consecutive cones detected
                    self.cameraConeDetection = True
                    self.tts(f"Cam cone detected {x=:.2f} {y=:.2f}")
                    self.coneLastDetXYZ =(x,y,z)

            else :
                # reset cone counter when cone not detected
                self.coneCounter = 0

        self.pointPublish("Camera",x,y,z)


    ######################################################################################################
    # Lidar cone detection parameters
    lidarActive = False
    # Field of view pointing forward from Lidar 36 degree scan data 
    fovRad_lidar = 0.758 # 45 deg, +-22.5 degrees
    coneMinRatioLimit = np.float32(0.250*1.5)
    coneMinMaxDiffMax = np.float32(0.060)
    coneMinMaxDiffMin = np.float32(0.005)
    coneRadius        = np.float32(0.05) # 100mm diameter at Lidar scan height     
    diffJump          = np.float32(0.15)
    coneRayMax        = np.float32(1.0)

    # Cone detection from Lidar LaserScan data - publish /cone_point/cam
    def lidar_subscription_callback(self, msg: LaserScan) -> None:
        pass
    #     #self.get_logger().info(f"lidar_subscription_callback: {msg=}")

        if not self.lidarActive :
            self.lidarActive = True
            self.tts("Lidar is active")
   
        angle_min       = np.float32(msg.angle_min)
        angle_max       = np.float32(msg.angle_max)
        angle_increment = np.float32(msg.angle_increment)
        # self.get_logger().info(f"lidar_callback: {angle_min=:.3f} {angle_max:.3f} {angle_increment=:.3f}")

        # get Lidar scan data within determined field of view for cone detection
        len = np.int32((angle_max-angle_min)/angle_increment)
        pfov = np.int32(self.fovRad_lidar/angle_increment)
        dmin = np.int32(len/2 - pfov/2)
        dmax = np.int32(len/2 + pfov/2)
        coneRanges =  np.float32(msg.ranges[dmin:dmax])
        # self.get_logger().info(f"{coneRanges=}")


        begin = np.int32(0)
        end = np.int32(0)
        lastRay = np.float32(coneRanges[0])
        rayNum = np.int32(1)
 
        coneMinDet:float = math.inf
        coneIdxDet:int = 0
        coneRaysDet:list = []

        # NOTE: 1st ray cant be start of cone
        for ray in coneRanges[1:] :
            # if np.math.isinf(ray) : ray = 100 
            if begin==0 :
                # look for dist jump hi to lo to indicate possible start
                if (ray<self.coneRayMax) :
                    if ((lastRay-ray)>self.diffJump) :
                        # possible start of cone detect
                        begin = rayNum
                        # self.get_logger().info(f" possible start of cone detect {begin=}")

            elif (lastRay<self.coneRayMax) :
                # look for dist jump lo to hi to indicate possible end
                if((ray-lastRay)>self.diffJump) :
                    # possible end of cone detect
                    end = rayNum-1
                    # self.get_logger().info(f" possible end of cone detect {end=}")
                    if False : #(end-begin) < rayMinCnt :
                        # number of rays too small for a cone, look for another cone
                        begin = np.int32(0)
                        end = np.int32(0)

                    elif coneRanges.size > 3 :
                        coneRays = coneRanges[begin:end]
                        coneMin = np.min(coneRays)
                        coneMax = np.max(coneRays)
                        # find ray number at center of minimums
                        idx = begin
                        cnt = np.int32(0)
                        coneRayIdxAtMin = np.int32(0)
                        for ray in coneRays :
                            if ray == coneMin :
                                coneRayIdxAtMin += idx
                                cnt +=1
                            idx +=1

                        # get min position as avg of all min positions
                        coneRayIdxAtMin = np.int32(coneRayIdxAtMin/cnt)

                        # validate if cone is near center of range of rays
                        numRays = end - begin
                        minCtr = coneRayIdxAtMin - begin
                        minRatio = math.fabs(0.5 - minCtr/numRays)
                        minRatioValid:bool = minRatio < self.coneMinRatioLimit

                        # Validate min - max distance and cone radius
                        minMaxValid:bool =   ((coneMax - coneMin) < self.coneMinMaxDiffMax) \
                                    and ((coneMax - coneMin) > self.coneMinMaxDiffMin) 

                        # Validate the width of the detected object
                        objWidth:float = 2*coneMin*math.sin((numRays*angle_increment)/2)
                        objWidthValid:bool = math.fabs(1-(objWidth/0.100)) < 0.5

                        if (minMaxValid and minRatioValid and objWidthValid) :
                            # validated cone detection
                            # self.get_logger().info(f" possible cone {coneMin=} {coneMax=} {coneRayIdxAtMin=} {coneRays=}")
                            
                            # keep the closest cone candidate 
                            if coneMin<coneMinDet :
                                coneMinDet = coneMin
                                coneIdxDet = coneRayIdxAtMin
                                coneRaysDet = coneRays


                        # keep looking for more cone candidates
                        begin = np.int32(0)
                        end = np.int32(0)
                    else :
                        begin = np.int32(0)
                        end = np.int32(0)

            else :
                # cone ray distance was too far, look for another cone
                begin = np.int32(0)
                end = np.int32(0)

            lastRay = ray
            rayNum +=1
 
        # self.get_logger().info(f"{coneMinDet=} {coneIdxDet=} {coneRaysDet=}")

        
        coneMin = coneMinDet
        coneIdx = coneIdxDet
        coneRays = coneRaysDet

        # self.get_logger().info(f" {coneMin=} {coneIdx=} {coneRays=}")

        # find angle of detected cone in FOV in front of robot where cone is searched
        # middle of coneRanges[] data is 0 degrees
        # a = np.float32(angle_increment*(coneRayIdxAtMin-(np.size(coneRanges)/2)))
        a = np.float32(angle_increment*(coneIdx-(np.size(coneRanges)/2)))

        # convert to x,y coordinates relative to lidar
        d = np.float32(coneMin)
        x = float(d*np.cos(a))
        y = float(d*np.sin(a))
        z = float(0.0)
        self.pointPublish("Lidar",x,y,z)

        # self.get_logger().info(f"lidar_callback: {x=} {y=} {a=} {d=} {coneMax=} {minRatio=}")

  
    def pointPublish(self, sensor:String, x:float,y:float,z:float) -> None :
        """
        Publish a PointStamped msg topic
        The sensor name chooses the reference frame and publisher to use
        """
        # self.get_logger().info(f"pointPublish: {sensor}, {x=:.2f}, {y=:.2f}, {z=:.2f}")
        stamp = self.get_clock().now().to_msg()
        pmsg = PointStamped()
        pmsg.header.stamp = stamp
        pmsg.point.x = x # Forward meters from sensor
        pmsg.point.y = y # Side meters from sensor
        pmsg.point.z = z # Elevation of center of the cone from sensor level

        if sensor == "Camera" :
            pmsg.header.frame_id="oak-d_frame"
            self.cone_point_cam_publisher.publish(pmsg)

        if sensor == "Lidar" :
            pmsg.header.frame_id="lidar_link"
            self.cone_point_lidar_publisher.publish(pmsg)

    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = ConeNode()

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