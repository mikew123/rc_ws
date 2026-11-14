
#
"""
Navigation node for RoboColumbus25.
Manages autonomous navigation to waypoints and cones using GPS, IMU, camera,
LIDAR, and TOF sensors. Implements a state machine to calibrate sensors, set
initial pose, navigate to waypoints, detect and approach cones, and handle
backup/stop actions. Publishes velocity and pose commands, configures navigation
parameters, and coordinates with other nodes via JSON messages.
"""
# RC25 navigation node

from sympy import Float, false

import rclpy
import math
import time
import tf_transformations
import json
import numpy as np
import utm

import yaml
from pathlib import Path
from pprint import pprint, pformat

from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Pose
from geographic_msgs.msg import GeoPose
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Range
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from robocolumbus_interfaces.msg import Float32X8, TofDist

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.client import Client

class NavNode(Node):
    '''
    Navigate to cones
    '''

    # parameters

    # Timer for state machine
    smTimerRateHz:float = 10.0

    T_INIT_WAIT, T_CAL_IMU, T_WAIT_GO, T_SETUP_NAV, \
        T_GET_WP, T_NAV_WP, T_NAV_WP_BACKUP, T_NAV_WP_AGAIN, \
        T_GOTO_CONE, T_DONE = range(10)
    tc_state = -1
    tc_next_state = T_INIT_WAIT

    # stop when this distance from mid_link during goto wp or cone
    # sm_xy_goal_wp   = 0.5 # 3 was set for outdoors? 3.0 #2.0
    # sm_xy_goal_cone = 0.25

    CAL_IMU_WAIT_BUTT, CAL_IMU, CAL_IMU_DONE = range(3)
    calImuState = -1
    next_calImuState = CAL_IMU_WAIT_BUTT

    NAV_GO_WAIT_BUTT, NAV_GO_BUTT_PRESSED = range(2)
    navGoState = -1
    next_navGoState = NAV_GO_WAIT_BUTT
    
    DP_FWD_RIGHT, DP_REV_LEFT, DP_PAUSE = range(3)
    dpState = -1
    next_dpState = DP_FWD_RIGHT
    dpStopTime = 0.0
    dpPauseNextState = -1
    dpCount = 0

    cd_timer:float = 0.0
    cd_state:int = 0
    cd_last_state = -1

    cd_sub_state = -1

    # state=1 use /cone_point to get location to drive close to cone using /cmd_vel
    cd_close_dist = 1.5 # distance from camera to cone
    cd_close_lvel = 1.0
    cd_close_avel = (9.5*cd_close_lvel)/(2*math.pi)
    cd_close_too_far_dist = 20.0

    # state=2 use camera /cone_point to drive closer to cone using /cmd_vel
    cd_closer_dist = 0.5
    cd_closer_lvel = 0.10
    cd_closer_avel = (9.5*cd_closer_lvel)/(2*math.pi)
    cd_closer_too_far_dist = 2.0

    # state=3 use Lidar and /tof_fc_mid and fr and fl to "touch" cone using /cmd_vel
    cd_touch_c_dist = 0.035
    cd_touch_rl_dist = 0.080
    cd_touch_lin_vel = 0.05
    cd_touch_ang_vel = cd_touch_lin_vel

    # wait while touching cone for observation
    cd_touch_wait = 2.0

    # backup after touching cone using /cmd_vel
    # manual backup then nav2 backup
    cd_backup_dist = 1.5
    cd_backup_vel = 0.5
    # cd_man_backup_dist = 0.5
    # cd_man_backup_vel = 0.25

    # # Field of view pointing forward from Lidar 36 degree scan data 
    tof_fc_dist_max = 1.5
    tof_fov = 0.785 # fovRad = 0.758 # 45 deg, +-22.5 degrees

    # GLOBAL variables 

    # for cone navigation and approach (state=1,2)
    # using /cone_point_cam
    cone_at_x_cam:float = 0.0
    cone_at_y_cam:float = 0.0
    cone_at_d_cam:float = 0.0
    cone_at_a_cam:float = 0.0
    cone_det_time_cam:int = 0
    cone_det_time_out_cam:int = 2.0

    cone_at_y_cam_last_det:float = 0.0

    # for touch (state=3)
    # using /cone_point_lidar
    cone_at_x_lidar:float = 0.0
    cone_at_y_lidar:float = 0.0
    cone_at_d_lidar:float = 0.0
    cone_at_a_lidar:float = 0.0
    cone_det_time_lidar:int = 0
    cone_det_time_out_lidar:int = 2.0

    # using /tof_fc_mid (NOTE: distance from front center TOF sensor)
    cone_at_x_tof_fc:float = 0.0
    cone_at_y_tof_fc:float = 0.0
    cone_at_d_tof_fc:float = 0.0
    cone_at_a_tof_fc:float = 0.0

    # Heading / YAW from IMU/Compass
    cmpAzi:float = 0.0

    # Number of GPS satelites in view
    gpsSiv:int = 0

    # median 5 filter memory
    # list of tupples [5X(x,y,z)]
    m5_filter:list = [(0.0,0.0,0.0)]*5
    
    # True when killSw is released to stop the robot motion
    killSw:bool = False
    killSwChange:bool = False
    # killSwEn:bool = False

    # YAML file with cone locations
    waypointsFileName = "~/sambashare/nav_files/RoboColumbus.yml"
    waypointsFile:dict = None

    wpConfig          = None
    wpGps:bool        = False
    wpCompass:bool    = False
    wpSetPose:dict    = None
    wpSetDatum:dict   = None
    wpWaypoints:dict  = None
    wpWaypoint:dict   = None
    wpWaypointNum:int = 1 # First waypoint number

    wpConeTrue:bool    = False
    wpCone:dict        = None
    wpConeTouched:dict = {}

    wpStopTime:float = 0.0

    # sm_last_state = -1

    gpsCurrent = None

    # EFK filter node sensor configurations
    #    [x_pos   , y_pos    , z_pos,
    #     roll    , pitch    , yaw,
    #     x_vel   , y_vel    , z_vel,
    #     roll_vel, pitch_vel, yaw_vel,
    #     x_accel , y_accel  , z_accel]
    # Enables GPS signals for map->odom tf
    efk_global_odom1_config_gpsEn = [
        True,  True,  False, # lat,lon are used for x_pos,y_pos
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False 
        ]
    # Disables GPS signals for map->odom tf
    efk_global_odom1_config_gpsDis = [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False 
        ]
    # Enables IMU yaw (compass)
    efk_global_imu0_config_yawEn = [
        False, False, False,
        False, False, True, # IMU yaw magnetic (compass)
        False, False, False,
        False, False, False,
        False, False, False 
        ]
    # Disables IMU yaw (compass)
    efk_global_imu0_config_yawDis = [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, False 
        ]


    # Velocity when going to way point
    velocity_smoother_max_velocity_wp = [1.5, 0.0, 2.0]

    # Velocity when approaching cone
    velocity_smoother_max_velocity_cone = [0.5, 0.0, 1.0]

    def  __init__(self, nav: BasicNavigator):
        super().__init__('rc25_nav_node')
        self.nav = nav

        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.readWaypointsFile(self.waypointsFileName)

        self.set_pose_msg_publisher = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 10)

        self.navsat_transform_server_set_param_svc = self.create_client(SetParameters, '/navsat_transform/set_parameters')
        while not self.navsat_transform_server_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/navsat_transform/set_parameters service not available, waiting again...')

        # self.controller_server_set_param_svc = self.create_client(SetParameters, '/controller_server/set_parameters')
        # while not self.controller_server_set_param_svc.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('/controller_server/set_parameters service not available, waiting again...')

        self.efk_global_set_param_svc = self.create_client(SetParameters, '/efk_global/set_parameters')
        while not self.efk_global_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/efk_global/set_parameters service not available, waiting again...')

        self.efk_local_set_param_svc = self.create_client(SetParameters, '/efk_local/set_parameters')
        while not self.efk_local_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/efk_local/set_parameters service not available, waiting again...')

        self.velocity_smoother_set_param_svc = self.create_client(SetParameters, '/velocity_smoother/set_parameters')
        while not self.velocity_smoother_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/velocity_smoother/set_parameters service not available, waiting again...')

        self.planner_server_set_param_svc = self.create_client(SetParameters, '/planner_server/set_parameters')
        while not self.planner_server_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/planner_server_set_param_svc/set_parameters service not available, waiting again...')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cone_point_cam_subscription = self.create_subscription(PointStamped, '/cone_point_cam'
                                            , self.cone_point_cam_subscription_callback, 10)
        self.cone_point_lidar_subscription = self.create_subscription(PointStamped, '/cone_point_lidar'
                                            , self.cone_point_lidar_subscription_callback, 10)

        self.tof_fc_mid_subscription = self.create_subscription(Float32X8, '/tof_fc_mid'
                                            , self.tof_fc_mid_subscription_callback, 10)
        self.tof_dist_subscription = self.create_subscription(TofDist, '/tof_dist'
                                            , self.tof_dist_subscription_callback, 10)
        self.range_r_subscription = self.create_subscription(Range, '/range_r'
                                            , self.range_r_subscription_callback, 10)
        self.range_l_subscription = self.create_subscription(Range, '/range_l'
                                            , self.range_l_subscription_callback, 10)

        self.gps_nav_subscription = self.create_subscription(NavSatFix, '/gps_nav'
                                            , self.gps_nav_subscription_callback, 10)
        self.gps_siv_subscription = self.create_subscription(Int32, '/gps_siv'
                                            , self.gps_siv_subscription_callback, 10)

        self.gps_nav_subscription = self.create_subscription(Float32, '/cmp_azi'
                                            , self.cmp_azi_subscription_callback, 10)

        # Message topic to/from all nodes for general messaging Json formated string
        self.json_msg_publisher = self.create_publisher(String, "/json_msg", 10)
        self.json_msg_subscription = self.create_subscription(String, "/json_msg"
                                            , self.json_msg_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # wait for navigator before turning on timer
        # This acts oddly and never returns - do I need to be lifecycle? can it be in init?
        # self.nav.waitUntilNav2Active()

        # The state machine timer runs a state machine, some functions take a lot of time (>50ms)
        # It gets its own callback group, all others use the default group
        self.sm_timer = self.create_timer((1.0/self.smTimerRateHz), self.sm_timer_callback
                                       , callback_group=self.cb_group)

        time.sleep(2) # wait for json_msg_publisher to be ready!!??
        self.tts("RC25 Navigation Node Started")
        self.get_logger().info(f"NavNode Started")

    def readWaypointsFile(self, file) :
        '''
        Executed in init
        Read yaml file with waypoint locations and if gps is enable for localization
        Parse for config, datum/pose and waypoints
        Each waypoint has a selection to look for cone or not (intermediate waypoint)
        Examples :
        # waypoint with cone location in ROS map related meters x,y
        # dead recogning mode - compass and gps are not used
            config :
                compass : false
                gps : false
            set_pose :
                x : 0.0
                y : 0.0
                deg : 180
            waypoints :
                1 :
                    cone : true
                    x : 10.0
                    y : 0.0

        # waypoint location in absolute lat,lon
        # gps is used for localization, compass for direction
            config :
                gps : true
            waypoints :
                1 :
                    cone : false
                    lat : 12345678
                    lon : 567890
        '''

        self.get_logger().info(f"readWaypointsFile: {file=}")

        path = Path(file).expanduser()
        if not path.exists():
            self.get_logger().info(f"readWaypointsFile: file not found: {path}")
            return
        try:
            with path.open("r", encoding="utf-8") as f:
                doc = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().info(f"readWaypointsFile: Bad format: {e}")
            return

        if not doc:
            self.get_logger().info("readWaypointsFile: file is empty")
            return

        self.get_logger().info("readWaypointsFile: \n"+pformat(doc))

        # Parse file
        if "config" in doc :
            config = doc["config"]
            if "gps" in config :
                self.wpGps = config["gps"] 
            elif "compass" in config :
                self.wpCompass = config["compass"]
        if "set_pose" in doc :
            self.wpSetPose = doc["set_pose"]
        if "set_datum" in doc :
            self.wpSetDatum = doc["set_datum"]
        if "waypoints" in doc :
            self.wpWaypoints = doc["waypoints"]

        # convert lat lon waypoints to x y
        wpoints = self.wpWaypoints
        if self.wpSetPose != None :
            if ("lat" in self.wpSetPose) and ("lon" in self.wpSetPose) :
                # pose lat lon is the starting point 0,0
                poseLat = self.wpSetPose["lat"]
                poseLon = self.wpSetPose["lon"]
                # set pose x,y to origin of map 0,0
                self.wpSetPose["x"] = 0.0
                self.wpSetPose["y"] = 0.0    
            else :
                poseLat = None
                poseLon = None
                self.get_logger().error(f"readWaypointsFile: pose does not have lat+lon")
        else :
            poseLat = None
            poseLon = None
            self.get_logger().error(f"readWaypointsFile: there is no pose")

        if wpoints != None :
            length = len(wpoints) # num of waypoints in file
            self.get_logger().info(f"{poseLat=} {poseLon=} wp {length=} {wpoints=}")
            for n0 in range(length) :
                # waypoints 1 to length
                n = n0+1
                if n in wpoints :
                    wp = wpoints[n]
                    self.get_logger().info(f"{n=} {wp=}")
                    if ("lat" in wp) and ("lon" in wp) :
                        lat = wp["lat"]
                        lon = wp["lon"]
                        (x,y) = self.latLon_to_XY(poseLat, poseLon, lat, lon)
                        wp["x"] = x
                        wp["y"] = y
                        self.wpWaypoints[n] = wp
                    # else :
                    #     self.get_logger().error(f"readWaypointsFile: no lat lon in {wp=}")


            self.get_logger().info(f"readWaypointsFile: {self.wpSetPose=} {self.wpWaypoints=}")

    def latLon_to_XY(self, datum_lat:float, datum_lon:float, lat:float, lon:float) -> tuple :
        '''
        Convert GPS latitude,logitude to map X,Y 
        X,Y are relative to the datum lat,lon
        returns tuple (x,y)
        '''
        # convert lat,lon to map x,y
        (deast, dnorth, dn, ds) = utm.from_latlon(datum_lat, datum_lon)
        (peast, pnorth, pn, ps) = utm.from_latlon(lat, lon)
        x = peast - deast
        y = pnorth - dnorth
        if dn != pn :
            self.logger().error(f"latLon_to_XY: UTM zone number mismatch {dn=} {pn=}")
        return (x,y)
    
    def setupNav(self, stateChange:bool) -> bool :
        '''
        Executed when nav button pressed
        Set up initial pose and/or datum using info in waypoints file

        If gps == True 
            The set_pose information in the file is ignored
            The lat,lon datum and x,y pose are set
            If there is no set_datum in the file then the current gps lat/lon signal is used
            If there is no set_pose the pose is 0,0 yaw=compass
            If set_pose is lat,lon then it is converted to map x,y relative to datum lat,lon
            (The datum orientation is set within navsat_transform from compass)
        If gps == False or doesnt exist
            The datum is not set
            If there is a set_pose it sets the pose using x,y (ignores any lat,lon)
                If there is no set_pose x/y then pose x,y = 0,0 origin
                If there is no set_pose deg or yaw 
                    If compass == True pose yaw = compass yaw
                        otherwise pose yaw=0

        returns True if setup OK
        '''

        # get pose and datum from waypoints file
        # wpDatum:dict = self.wpSetDatum
        wpPose:dict  = self.wpSetPose

        if stateChange :
            # self.get_logger().info(f"setupNav: {wpPose=} {wpDatum=}")
            self.get_logger().info(f"setupNav: {wpPose=}")
            
        if self.wpGps == True :
            # must use the compass while using gps
            self.wpCompass = True
        # else :
        #     # No gps so datum is not used
        #     wpDatum = None

        # datum:list[float] = None
        # if self.wpGps == True :
        #     # get the starting location datum
        #     if wpDatum != None :
        #         if not (("lat" in wpDatum) and ("lon" in wpDatum)) :
        #             # get datum lat,lon current location from gps signal
        #             if stateChange :
        #                 self.get_logger().info(f"setupNav: Get current gps for the datum")
        #                 self.tts("Get current gps for the datum")

        #             datum = self.getCurrentGps()
        #             if datum == None :
        #                 # wait for filtered GPS datum
        #                 return False
                    
        #             self.get_logger().info(f"setupNav: Use current gps as {datum=}")
        #             self.tts("use current gps datum lat={datum[0]:.0f} lon={datum[1]:.0f} yaw={datum[2]:.2f}")

        #         else :
        #             # use datum in wp file
        #             if stateChange :
        #                 self.get_logger().info(f"setupNav: Get datum from file {wpDatum=}")
        #             datum = [
        #                 wpDatum["lat"],
        #                 wpDatum["lon"],
        #                 0.0 # yaw 0=East
        #             ] 
        #     else : # no datum data in waypoint file
        #         # get datum lat,lon current location from gps signal
        #         if stateChange :
        #             self.get_logger().info(f"setupNav: Get current gps for the datum")
        #             self.tts("Get current gps for the datum")

        #         datum = self.getCurrentGps()
        #         if datum == None :
        #             # wait for filtered GPS datum
        #             return False

        #         self.get_logger().info(f"setupNav: Use current gps as {datum=}")
        #         self.tts("use current gps datum lat={datum[0]:.0f} lon={datum[1]:.0f} yaw={datum[2]:.2f}")

            # create pose x,y,yaw from datum lat/lon and compass
            #TODO: get compass yaw, should we use a datum yaw if given?
            x, y, yaw = 0.0, 0.0, None # defaults

            if wpPose != None :
                if ("x" in wpPose) and ("y" in wpPose) :
                    x = wpPose["x"]
                    y = wpPose["y"]
                else :
                    x = 0.0
                    y = 0.0
                    
                # get yaw to use in pose
                if "rad" in wpPose :
                    yaw = wpPose["rad"]
                elif "deg" in wpPose :
                    yaw = wpPose["deg"]/180.0 * math.pi

            # alternate sources of pose yaw
            # if yaw == None :
            #     if wpDatum != None :
            #         if "rad" in wpDatum :
            #             yaw = wpDatum["rad"]
            #         elif "deg" in wpDatum :
            #             yaw = wpDatum["deg"]/180.0 * math.pi
            if yaw == None :
                if self.wpCompass == True :
                    # TODO: get yaw from compass
                    # yaw = 0.0
                    yaw = self.cmpAzi
            else :
                yaw = 0.0

            wpPose = {"x":x,"y":y,"rad":yaw}

        if wpPose == None :
            wpPose = {"x":0.0,"y":0.0,"rad":0.0}

        # if pose orientation in degrees convert to radians
        if (not ("rad" in wpPose)) and ("deg" in wpPose):
            wpPose["rad"] = wpPose["deg"]/180.0 * math.pi

        # if datum != None :
        #     self.send_set_param_request(self.navsat_transform_server_set_param_svc,
        #                             "datum",  datum, "navsat_transform_server")           

        if wpPose != None :
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id="map"
            pose.pose.pose = self.createPose(wpPose["x"],wpPose["y"],wpPose["rad"])
            self.set_pose_msg_publisher.publish(pose)

        # self.get_logger().info(f"setupNav: {pose=} {datum=}")
        self.get_logger().info(f"setupNav: {wpPose["x"]=} {wpPose["y"]=} {wpPose["rad"]=} {pose=}")
        return True

    def cmp_azi_subscription_callback(self, msg:Float32) -> None :
        """
        Cache the orientation direction
        """
        self.cmpAzi = msg.data

    def gps_siv_subscription_callback(self, siv:Int32) -> None :
        """
        Number of satelites in view detected
        """
        self.gpsSiv = siv.data

    gpsCurrentLen = 9

    def gps_nav_subscription_callback(self, msg:NavSatFix) -> None :
        '''
        Save last N valid gps [lat,lon,yaw]
        N should be odd number for median filtering
        NOTE: yaw is from imu/compass
        Clear list when gps invalid (low sat count) occurs
        '''

        lowSatCnt:int = 5
        maxLen = self.gpsCurrentLen

        # Get the number of satelites in view
        # NOTE: status.service is not defined for siv, but currently used
        # siv = msg.status.service # service used for num sat in view
        siv = self.gpsSiv
        if siv < lowSatCnt :
            self.gpsCurrent = None
            return

        # initialize list when gps data is starting to be collected
        if self.gpsCurrent == None :
            self.gpsCurrent = []

        lat = msg.latitude
        lon = msg.longitude
        # alt = msg.altitude
        # YAW from IMU/compass cache
        yaw = 0.0 #self.cmpAzi

        # limit the list length before saving in cache
        if len(self.gpsCurrent) > maxLen :
            self.gpsCurrent.pop(0)

        self.gpsCurrent.append([lat,lon,yaw])

    def getCurrentGps(self) -> list[float] :
        '''
        Use median filter for gps data
        Return filtered gps [lat, lon, yaw] when available
        Else return None
        '''

        curGps:list = self.gpsCurrent

        if curGps == None : 
            return None
        
        # Filter the last N GPS values
        lats:list[float] = []
        lons:list[float] = []
        yaws:list[float] = []
        gps:list[float]
        for gps in curGps :
            lats.append(gps[0])
            lons.append(gps[1])
            yaws.append(gps[2])
            
        curGpsLen = len(curGps)
        if curGpsLen < self.gpsCurrentLen :
            return None
        
        # Median filter by selecting the sorted mid value
        lats.sort()
        lons.sort()
        yaws.sort()
        mid = int(curGpsLen/2) +1
        gps = [lats[mid ],lons[mid],yaws[mid]]

        return gps
    
    def send_set_param_request(self, svc: Client, name, value, sys: str = "") -> None:
        """
        Set a parameter using the given param service
        command line example:
        cli -> ros2 param set /local_costmap/local_costmap obstacle_layer.enabled False
        """
    
        while not svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('send_set_param_request: service not available, waiting again...')

        req = SetParameters.Request()
        self.callback_set_param_done = False

        param = Parameter()
        param.name = name
        param.value.type = None
        if isinstance(value, bool) :
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        elif isinstance(value, float) :
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
        elif isinstance(value, list) :
            if isinstance(value[0], bool) :
                param.value.type = ParameterType.PARAMETER_BOOL_ARRAY
                param.value.bool_array_value = value
            elif isinstance(value[0], float) :
                param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param.value.double_array_value = value
                
        if param.value.type == None :
            self.get_logger().info(f"send_set_param_request: unsupported variable type {value=}")
            return

        req.parameters.append(param)

        self.get_logger().info(f"send_set_param_request: Sending {sys=} {name=} {value=}")
        future = svc.call_async(req)

        # delay a tiny bit since checking future complete does not seem to work!!!
        time.sleep(0.1)

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
        #self.get_logger().info(f"json_msg_callback: {msg=}")
        data = json.loads(msg.data)

        if 'nav' in data :
            nav = data['nav']
            self.processNavMsg(nav)

    def getNextWaypoint(self) -> bool :
        '''
        Get the next waypoint from the read waypoints file
        returns True if there is a new waypoint
        '''
        waypoints = self.wpWaypoints
        n = self.wpWaypointNum
        if not (n in waypoints) :
            self.wpWaypoint = False
            return False
        
        self.wpWaypoint = waypoints[n]
        self.wpWaypointNum +=1
        return True

    buttonDo = False
    # buttonKill = False
    buttonDoTrig = False
    # buttonKillTrig = False

    def processNavMsg(self, nav:dict) -> None :
        # self.get_logger().info(f"processNavMsg: {nav=}")

        if "buttonDo" in nav :
            buttonDo = nav["buttonDo"]
            if (self.buttonDo==False) and (buttonDo==True):
                self.buttonDoTrig = True
            self.buttonDo = buttonDo

        if "imu_cal_status" in nav :
            self.imuCalStatus = nav["imu_cal_status"]

        if "kill" in nav :
            kill = nav["kill"]
            self.processKill(kill)

    def processKill(self, kill:bool) -> None :
        if(kill != self.killSw) :
            self.killSwChange = True
            # self.get_logger().info(f"processKill: kill switch is {kill}")
        self.killSw = kill

    # Timer based state machine for cone navigation

    def smTimerNav2Config(self, state:int) -> None :
        '''
        Configure nav2 parameters for nav to waypoint or goto cone
        '''
        
        if state == self.T_NAV_WP :
            # configure navigation nodes parameters

            # Set which sensors are fused in EFK modules
            if self.wpGps == True :
                # configure for both compass and gps
                # self.send_set_param_request(self.efk_global_set_param_svc, 
                #             'publish_tf', True, "efk_global")
                # self.send_set_param_request(self.efk_local_set_param_svc, 
                #             'publish_tf', False, "efk_local")
                self.send_set_param_request(self.planner_server_set_param_svc, 
                            'GridBased.tolerance', 3.0, "planner_server") 
                self.send_set_param_request(self.efk_global_set_param_svc, # GPS
                            'odom1_config', self.efk_global_odom1_config_gpsEn, "efk_global")
                self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                            'imu0_config', self.efk_global_imu0_config_yawEn, "efk_global")
                # self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                #             'odom0_config', self.efk_global_odom0_config_yawEn, "efk_global")
                # self.send_set_param_request(self.efk_local_set_param_svc, # Compass
                #             'imu0_config', self.efk_local_imu0_config_yawEn, "efk_local")
                self.send_set_param_request(self.velocity_smoother_set_param_svc,
                            'max_velocity', self.velocity_smoother_max_velocity_wp, "velocity_smoother")
            else :
                self.send_set_param_request(self.planner_server_set_param_svc, 
                            'GridBased.tolerance', 0.5, "planner_server") 
                self.send_set_param_request(self.efk_global_set_param_svc, # GPS
                            'odom1_config', self.efk_global_odom1_config_gpsDis, "efk_global")
                self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                            'imu0_config', self.efk_global_imu0_config_yawDis, "efk_global")
                self.send_set_param_request(self.velocity_smoother_set_param_svc,
                            'max_velocity', self.velocity_smoother_max_velocity_wp, "velocity_smoother")

            if self.wpCompass == True:
                self.send_set_param_request(self.efk_global_set_param_svc, # Compass
                            'imu0_config', self.efk_global_imu0_config_yawEn, "efk_global")
                    
        elif state == self.T_GOTO_CONE :
            # configure navigation nodes parameters for goto cone
            self.send_set_param_request(self.velocity_smoother_set_param_svc,
                        'max_velocity', self.velocity_smoother_max_velocity_cone, "velocity_smoother")
            
    last_touched_cone_close_true:bool = False
    sm_timeout:float = 0.0 # Timeout for states

    def sm_timer_callback(self):

        # need a main state machine
        next_state = self.tc_next_state

        stateChange = False
        if self.tc_state!= next_state :
            stateChange = True
            self.get_logger().info(f"sm_timer_callback: state change to {next_state}")
            self.tts(f"Nav timer {next_state=}")

        state = next_state
        self.tc_state = state

        # T_INIT_WAIT
        if state == self.T_INIT_WAIT :
            self.tts("wait for nav 2")
            time.sleep(20)

            # calibrate compass if used
            if (self.wpCompass == True) or (self.wpGps == True) :
                next_state = self.T_CAL_IMU
            else :
                next_state = self.T_WAIT_GO

        # T_CAL_IMU
        if state == self.T_CAL_IMU :
            # Calibrate IMU when compass is used
            status = self.calImu()
            if status :
                #  calibration complete
                next_state = self.T_WAIT_GO

        # T_WAIT_GO
        if state == self.T_WAIT_GO :
            # Wait for go command from button
            if self.waitGo() :
                next_state = self.T_SETUP_NAV

        # T_SETUP_NAV
        if state == self.T_SETUP_NAV :
            # Continue setup until True is returned
            if self.setupNav(stateChange) :
                next_state = self.T_GET_WP

        # T_GET_WP
        elif state == self.T_GET_WP :        
            if self.getNextWaypoint() :
                next_state = self.T_NAV_WP
            else :
                next_state = self.T_DONE

        # T_NAV_WP_AGAIN
        elif state == self.T_NAV_WP_AGAIN :
            next_state = self.T_NAV_WP

        # T_NAV_WP
        elif state == self.T_NAV_WP :
            if stateChange :
                self.get_logger().info(f"T_NAV_WP: next {self.wpWaypoint=}")
                x=self.wpWaypoint["x"]
                y=self.wpWaypoint["y"]
                self.tts(f"Goto waypoint x {x:.1f}, y {y:.1f}")
                self.smTimerNav2Config(state)
            
                # execute goto waypoint once
                x = 0.0 # default
                y = 0.0
                self.wpConeTrue = false # default
                if "cone" in self.wpWaypoint :
                    self.wpConeTrue = self.wpWaypoint["cone"]
                    self.wpCone     = self.wpWaypoint

                if ("x" in self.wpWaypoint) and ("y" in self.wpWaypoint) :
                    # map based XY location
                    x = self.wpWaypoint["x"]
                    y = self.wpWaypoint["y"]
                    
                    #TODO: what angle?
                    # compute angle to waypoint xy
                    (wd,wa,_,_,_) = self.get_dist_from_robot(x,y)
                    a = wa

                    self.get_logger().info(f"T_NAV_WP: Go to way point location dir {wa=:.2f} {wd=:.1f} at {x=:.1f} {y=:.1f}")
                    self.tts(f"Go to waypoint location {x=:.0f} {y=:.0f}")

                    goto_pose = self.createPoseStamped(x,y,a,"map")
                    self.nav.goToPose(goto_pose)

                else :
                    # invalid waypoint, get next
                    next_state = self.T_GET_WP
                    return
                
            #TODO: timeout + kill switch


            # quit nav to cone waypoint if a cone is detected and it is not close to the last touched cone
            if self.wpConeTrue and self.cone_det_cam_true and (not self.last_touched_cone_close_true) :
                if self.cancelNav2Task() :
                    self.tts("Cone detected while navigating to waypoint")
                    next_state = self.T_GOTO_CONE

            elif self.nav.isTaskComplete() :
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED :
                    self.tts("Navigate to way point location is successfull")
                    if self.wpConeTrue :
                        # Find cone close to the way point
                        next_state = self.T_GOTO_CONE
                    else :
                        # No cone is at the way point
                        # TODO: try finding cone at waypoint again
                        next_state = self.T_GET_WP
                else :
                    # try again
                    self.tts("Failed to navigate to way point")
                    # backup a bit on failure before trying again (stuck in bush)
                    next_state = self.T_NAV_WP_BACKUP

        # T_NAV_WP_BACKUP
        elif state == self.T_NAV_WP_BACKUP :
            msg = Twist()
            if stateChange==True :
                self.tts("Backup before navigating to way point location again")
                driveT = 2.5
                self.wpStopTime = time.monotonic() + driveT

            if time.monotonic() >= self.wpStopTime :
                (msg.linear.x, msg.angular.z) = (0.0, 0.0)
                next_state = self.T_NAV_WP_AGAIN
                self.tts("Attempt to navigate to way point location again")
            else :
                speed = 0.1
                (msg.linear.x, msg.angular.z) = self.driveManObs(speed, -2*speed)

            # drive robot command
            self.cmd_vel_publisher.publish(msg)

        # T_GOTO_CONE
        # These states do not use the navigator package 
        elif state == self.T_GOTO_CONE :
            if stateChange :
                self.tts(f"Go to the cone")
                self.smTimerNav2Config(state)
                timeout = 45.0
                self.sm_timeout = time.monotonic() + timeout

            if time.monotonic() >= self.sm_timeout :
                # timed out - backup and try going to waypoint again
                self.tts(f"Timeout going to the cone")
                next_state = self.T_NAV_WP_BACKUP

            # find cone and "touch" it
            done = self.cd_sm()
            if done : 
                # request next cone location
                self.get_logger().info(f"tc: done - touch cone succeeded")
                self.tts("cone was touched")
                # save touched cone actual XY location not waypoint
                current_pose:PoseStamped = None
                (tf_OK, current_pose) = self.getPoseFromTF('map','tof_fc_link')
                if tf_OK :
                    self.wpConeTouched["x"] = current_pose.pose.position.x
                    self.wpConeTouched["y"] = current_pose.pose.position.y
                next_state = self.T_GET_WP

        # T_DONE
        elif state == self.T_DONE :
            if stateChange :
                self.get_logger().info(f"T_DONE: Finished")
                self.tts(f"Finished")
            

        self.tc_next_state = next_state

    def calImu(self) -> bool :
        '''
        Calibrate the IMU by moving robot
        Wait for cal button to be pressed
        Return True when button released
        '''
        #TODO: avoid obsticals
        # check for calibration status 3 (complete)
        # if self.imuCalStatus==3 : return True

        state = self.next_calImuState
        stateChange = False
        next_state = state # default
        returnVal = False

        if state != self.calImuState :
            stateChange = True
        self.calImuState = state

        if state == self.CAL_IMU_WAIT_BUTT :
            if stateChange :
                self.buttonDoTrig = False
                #TODO: message periodically till pressed
                self.tts(f"Press button to start IMU calbration")

            if self.buttonDoTrig == True :
                next_state = self.CAL_IMU

        elif state == self.CAL_IMU :
            if stateChange :
                self.tts(f"Release button when IMU is calibrated")

            status = self.drivePattern(stateChange, 0, 0.5, 2.0, 0.5)
            if status==True :
                next_state = self.CAL_IMU_DONE

        elif state == self.CAL_IMU_DONE :
            # if stateChange :
            #     self.tts(f"IMU calbration is complete")
            
            next_state = self.CAL_IMU_WAIT_BUTT
            returnVal = True

        else :
            #TODO: invalid state ???
            next_state = self.CAL_IMU_WAIT_BUTT
            returnVal = True

        self.next_calImuState = next_state
        
        return returnVal

    def waitGo(self) -> bool :
        '''
        Wait to start navigation 
        Wait for navigate button to be pressed
        Return True when button is pressed
        '''

        state = self.next_navGoState
        stateChange = False
        next_state = state # default
        returnVal = False

        if state != self.navGoState :
            stateChange = True
        self.navGoState = state

        if state == self.NAV_GO_WAIT_BUTT :
            if stateChange :
                self.buttonDoTrig = False
                #TODO: message periodically till pressed
                self.tts(f"Press button to start robot")

            if self.buttonDoTrig==True :
                next_state = self.NAV_GO_BUTT_PRESSED

        if state == self.NAV_GO_BUTT_PRESSED :
            if stateChange :
                self.tts(f"Navigation is starting")

            if self.buttonDoTrig==True :
                next_state = self.NAV_GO_WAIT_BUTT
                returnVal = True

        self.next_navGoState = next_state

        return returnVal


    def driveManObs(self, lx:float, az:float=0.0) -> tuple :
        """
        drive at speed and angle
        Avoid obsticals
        returns adjusted (lx, az)
        """

        # Max distance to object from TOF sensors to avoid/slow
        tofObjMax = 0.50
        # Min distance to object from TOF sensors to stop
        tofObjMin = 0.20
        # Max distance to object from Range sensors to avoid/slow
        rngObjMax = 0.50
        # Min distance to object from Range sensors to stop
        rngObjMin = 0.30

        # Get front left and right range sensor data
        # this is currently using Lidar to emulate a range sensor
        rngr_ob_dist = self.range_r_obstacle_dist
        rngl_ob_dist = self.range_l_obstacle_dist

        # get object distances from TOF sensors
        if lx <0 :
            # get rear sensors when going backward
            c_ob_dist = self.tof_rc_obstacle_dist
            l_ob_dist = self.tof_rl_obstacle_dist
            r_ob_dist = self.tof_rr_obstacle_dist
        else :
            # get front sensors when going forward
            c_ob_dist = self.tof_fc_obstacle_dist
            l_ob_dist = self.tof_fl_obstacle_dist
            r_ob_dist = self.tof_fr_obstacle_dist

        # linear velocity reduces when any rear sensor detects close
        xc = 1.0
        xr = 1.0
        xl = 1.0
        xrr = 1.0
        xrl = 1.0

        # angular velocity steers away from right or left obstical detection
        ac = 0.0
        ar = 0.0
        al = 0.0
        arr = 0.0
        arl = 0.0

        if c_ob_dist < tofObjMin :
            xc = 0.0
        elif c_ob_dist < tofObjMax :
            xc = 1.0 - (tofObjMax - c_ob_dist)/tofObjMax

        if r_ob_dist < tofObjMin :
            xr = 0.0
            ar = -1.0
        elif r_ob_dist < tofObjMax :
            xr = 1.0 - (tofObjMax - r_ob_dist)/tofObjMax
            ar = -0.5

        if l_ob_dist < tofObjMin :
            xl = 0.0
            al = +1.0
        elif l_ob_dist < tofObjMax :
            xl = 1.0 - (tofObjMax - r_ob_dist)/tofObjMax
            al = +0.5

        if rngr_ob_dist < rngObjMin :
            xrr = 0.0
            arr = -1.0
        elif rngr_ob_dist < rngObjMax :
            xrr = 1.0 - (rngObjMax - rngr_ob_dist)/rngObjMax
            arr = -0.5

        if rngl_ob_dist < rngObjMin :
            xrl = 0.0
            arl = +1.0
        elif rngl_ob_dist < rngObjMax :
            xrl = 1.0 - (rngObjMax - r_ob_dist)/rngObjMax
            arl = +0.5

        # add all angular velocities to maximally avoid the obstical
        # NOTE: if L and R are equal they cancel out and no rotation
        # select the minimum linear velocity adjustment
        l = lx
        a = az
        if l < 0 :
            # avoid obstacles while going backward
            a += 2*(ac + ar + al - arr - arl)
            # l *= np.min((xc,xr,xl,xrr,xrl))
            l *= np.min((xc,xr,xl))
        else :
            # avoid obstacles while going forward
            a -= 2*(ac + ar + al - arr - arl)
            l *= np.min((xc,xr,xl,xrr,xrl))

        if l == 0.0 :
            self.get_logger().info(f"driveManObs: Stopped {lx=} {az=} {l=} {a=} {c_ob_dist=} {r_ob_dist=} {l_ob_dist=} {rngr_ob_dist=} {rngl_ob_dist=}")
            self.tts(f"Obstacle stop while manual driving")

        return (l, a)


    def drivePattern(self, init:bool, numMoves:int=0, speed:float=0.5, driveT:float=2.0, pauseT:float=0.5) -> bool :
        '''
        Drive in a "star" like pattern fwd-left/rev-right or similar
            init resets drive pattern count
            numMoves>0 is the number of FWD_RIGHT,REV_LEFT movements (cone search)
            numMoves=0 moves until the IMU is calibrated AND cal button is released 
            speed (m/s) is the speed while moving (NOTE: speed<0 reverses FWD/REV)
            driveT (sec) is the time while moving
            pauseT (sec) is the time paused between movements
            return=True when finished
        '''
        
        # /cmd_vel message to drive robot
        # Twist msg default is velocities = 0
        msg = Twist()

        # Stop when Kill switch active 
        if self.killSw == True :
            self.cmd_vel_publisher.publish(msg)
            return False

        state = self.next_dpState
        stateChange = False
        next_state = state # default
        returnVal = False

        if state != self.dpState :
            stateChange = True
        self.dpState = state

        if init :
            self.dpCount = 0

        if state == self.DP_FWD_RIGHT :
            if stateChange==True :
                self.dpStopTime = time.monotonic() + math.fabs(driveT)

            (msg.linear.x, msg.angular.z) = self.driveManObs(speed, -2*speed)

            if (time.monotonic() >= self.dpStopTime) or  (msg.linear.x==0.0) :
                self.dpPauseNextState = self.DP_REV_LEFT
                next_state = self.DP_PAUSE

        elif state == self.DP_REV_LEFT :
            if stateChange==True :
                self.dpStopTime = time.monotonic() + math.fabs(driveT)

            (msg.linear.x, msg.angular.z) = self.driveManObs(-speed, -2*speed)

            if (time.monotonic() >= self.dpStopTime) or  (msg.linear.x==0.0) :
                self.dpPauseNextState = self.DP_FWD_RIGHT
                next_state = self.DP_PAUSE

        elif state == self.DP_PAUSE :
            if stateChange==True :
                self.dpCount +=1
                self.dpStopTime = time.monotonic() + math.fabs(pauseT)

            msg.linear.x  = 0.0
            msg.angular.z = 0.0

            if time.monotonic() >= self.dpStopTime :
                next_state = self.dpPauseNextState

        # determine when to stop driving pattern
        if numMoves>0 :
            if self.dpCount>numMoves:
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                returnVal = True
        else :
            # continue calibration pattern until button is released
            # if (self.buttonDo==False) and (self.imuCalStatus==3) :
            if self.buttonDo==False :
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
                returnVal = True

        # drive robot command
        self.cmd_vel_publisher.publish(msg)

        self.next_dpState = next_state

        return returnVal

    # Executes in timer callback group, sensor callbacks are in parallel
    # These states do not use the navigator package
    def cd_sm(self) -> bool:
        func = "cone_sm:"
        done = False

        state_change:bool = False
        if self.cd_state != self.cd_last_state : 
            state_change = True

        ks = self.killSw
        ksc = self.killSwChange
        if ksc : 
            self.killSwChange = False

        state:int = self.cd_state
        next_state:int = state

        if state_change :
            self.cd_timer = time.time_ns()*1e-9

        # Attempt search for cone when timeout occurs
        if (time.time_ns()*1e-9 - self.cd_timer) > 15 :
            self.cd_timer = time.time_ns()*1e-9
            next_state = 0
            state = 0
            state_change = True
            
        if state == 0 :
            next_state = self.search_for_cone(func,state,state_change,ks,ksc)

        elif state == 1 :
            next_state = self.drive_to_cone(func,state,state_change,ks,ksc)
                
        elif state == 2 :
            next_state = self.get_closer_to_cone(func,state,state_change,ks,ksc)

        elif state == 3 :
            next_state = self.touch_cone(func,state,state_change,ks,ksc)

        elif state == 4 :
            next_state = self.wait_after_touch(func,state,state_change,ks,ksc)

        elif state == 5 :
            next_state = self.backup_after_touch(func,state,state_change,ks,ksc)

        elif state == 6 :
            next_state = self.stop_after_touch(func,state,state_change,ks,ksc)
            # touch cone success - get ready for next cone
            done = True
            next_state = 0

        self.cd_last_state = state
        self.cd_state = next_state

        return done

    wcScanInit = False

    # state 0 - search for cone using camera  detection 
    # drive in fwd/rev star pattern looking for the cone   
    def search_for_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} wait for cone detection {state=}")
            self.tts("Searching for a cone")
            self.wcScanInit = True

        # get cone xy from camera detect
        x:float = self.cone_at_x_cam
        y:float = self.cone_at_y_cam
        yLastDet:float = self.cone_at_y_cam_last_det
        t:int = self.cone_det_time_cam
        to:int = self.cone_det_time_out_cam
        dt:int = (time.time_ns()*1e-9) - t
        # self.get_logger().info(f"{func} {dt=} {t=} {x=} {y=} ")
        if dt>to and t>0:
            self.get_logger().info(f"{func} cone detection is stale {x=} {y=} {dt=}")
            x = 0
            y = 0
            # clear out globals
            self.cone_det_time_cam = 0.0
            self.cone_at_x_cam = 0.0
            self.cone_at_y_cam = 0.0

        killSwitchActive:bool  = ks
        next_state:int = state

        # if x!=0 and y<0.10 and y>-0.10:
        if self.cone_det_cam_true:
            # A cone has been detected
            self.get_logger().info(f"{func} cone detected at {x=:.3f} {y=:.3f}  {state=} {killSwitchActive=}")
            #stop movement
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)
            self.wcScanInit = True

            if not killSwitchActive : 
                next_state = 1
        else :
            # slowly scan for cone
            if not killSwitchActive :

                #NOTE: kill sw processed in drivePattern
                vel = 0.25
                # rotate toward direction of last cone detected
                if yLastDet > 0.0 :
                    vel = -vel
                status = self.drivePattern(self.wcScanInit, 10, vel, 5.0, 1.0)
                self.wcScanInit = False
                if status:
                    # drive pattern completed - init drive pattern on the next cycle
                    #TODO: move before scanning again - add more states?
                    self.wcScanInit = True
                
        return next_state

    #state 1 - use camera to locate cone and get close
    cd_sub_timer = 0
    def drive_to_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        """
        Drive to the cone after detection without using navigator
        Use /cone_point location to drive to it using /cmd_vel
        """
        if state_change :
            self.get_logger().info(f"{func} drive close to cone using cmd_vel and camera {state=}")
            self.tts("Get close to cone")

        a:float = self.cone_at_a_cam
        d:float = self.cone_at_d_cam
        d -= 0.24 # Crude transformation, camera is 0.24 behind tof sensor

        # get TOF obstacle detections
        fl_ob_dist = self.tof_fl_obstacle_dist
        fr_ob_dist = self.tof_fr_obstacle_dist

        killSwitchActive:bool = ks
        next_state = state
        msg = Twist()

        if killSwitchActive :
            # Stop motors and wait for kill switch not active
            # Motors are stopped by leaving velocities to msg defaults as 0
            pass
        elif not self.cone_det_cam_true :            
            self.get_logger().info(f"{func} lost cone {state=}")
            next_state = 0
        elif d <= 0 :
            self.get_logger().info(f"{func} {d=} lost cone temporarily? {state=}")
            # dont change state - wait for cone detect false or distance to be non-zero
        elif d > 2*self.cd_close_too_far_dist :
            self.get_logger().info(f"{func} cone is too far {d=} {state=}")
            next_state = 5 # back up
        elif d > self.cd_close_dist :
            if d > (self.cd_close_dist -1.0) :
                msg.linear.x =  self.cd_close_lvel
            else :
                # lower speed to cd_closer_lvel
                slowdown = (self.cd_close_lvel-self.cd_closer_lvel)*(1-(self.cd_close_dist-d))
                msg.linear.x =  self.cd_close_lvel - slowdown
            # use angle to turn towards the cone
            msg.angular.z =  (a/0.393)*self.cd_close_avel/2
            # steer away from obstacle detected using TOF sensors
            if fl_ob_dist < 0.5 :
                msg.angular.z += (fl_ob_dist - 0.4) * self.cd_close_avel
            if fr_ob_dist < 0.5 :
                msg.angular.z -= (fr_ob_dist - 0.4) * self.cd_close_avel
                
            
        else : 
            self.get_logger().info(f"{func} at the close distance {d=:.3f} {a=:.3f} {state=}")
            next_state = 2

        #self.get_logger().info(f"{func} {d=:.3f} lx={msg.linear.x:.3f} az={msg.angular.z:.3f}")
        self.cmd_vel_publisher.publish(msg)
        # self.get_logger().info(f"{func} {state=} {msg=}")
        
        return next_state

    #state 2 - get closer to cone
    def get_closer_to_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        """
        State 2 - gets closer to cone before touching
        Basically the same as state 1 but slower and stops a smaller distance from cone
        """
        if state_change :
            self.get_logger().info(f"{func} drive closer to cone using cmd_vel and camera {state=}")
            self.tts("Get closer to cone")

        a:float = self.cone_at_a_cam
        d:float = self.cone_at_d_cam
        d -= 0.24 # Crude transformation, camera is 0.24 behind tof sensor

        # get TOF obstacle detections
        fl_ob_dist = self.tof_fl_obstacle_dist
        fr_ob_dist = self.tof_fr_obstacle_dist

        killSwitchActive:bool = ks
        next_state = state
        msg = Twist()

        if killSwitchActive :
            # Stop motors and wait for kill switch not active
            # Motors are stopped by leaving velocities to msg defaults as 0
            pass
        elif d <= 0 :
            self.get_logger().info(f"{func} lost cone {state=}")
            next_state = 0
        elif d > 2*self.cd_closer_too_far_dist :
            self.get_logger().info(f"{func} cone is too far {d=} {state=}")
            next_state = 5 # back up
        elif d > self.cd_closer_dist :
            msg.linear.x =  self.cd_closer_lvel
            # use angle to turn towards the cone
            msg.angular.z =  (a/0.393)*self.cd_closer_avel
            # steer away from obstacle detected using TOF sensors
            if fl_ob_dist < 0.3 :
                msg.angular.z += 4*(fl_ob_dist - 0.3) * self.cd_closer_avel
            if fr_ob_dist < 0.3 :
                msg.angular.z -= 4*(fr_ob_dist - 0.3) * self.cd_closer_avel
            
        else : 
            self.get_logger().info(f"{func} at the closer distance {d=:.3f} {a=:.3f} {state=}")
            next_state = 3

        #self.get_logger().info(f"{func} {d=:.3f} lx={msg.linear.x:.3f} az={msg.angular.z:.3f}")
        self.cmd_vel_publisher.publish(msg)
        # self.get_logger().info(f"{func} {state=} {msg=}")
        
        return next_state


    #state 3 - "touch" cone
    def touch_cone(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} drive slowly to \"touch\" cone using cmd_vel and lidar sensor {state=}")
            self.tts("Slowly touch the cone")
            # self.nav.cancelTask()

        # self.nav.cancelTask()

        killSwitchActive:bool = ks
        next_state = state
        msg = Twist()

        d = self.cone_at_d_lidar
        a = self.cone_at_a_lidar
        dx = 0.42 #Lidar sensor is X mm back from TOF sensors
        # convert to xy relative to TOF/bumper
        x = d*math.cos(a)
        y = d*math.sin(a)
        x -= dx

        if math.isinf(x) :
            # Try using camera cone detection
            d = self.cone_at_d_cam
            a = self.cone_at_a_cam
            dx = 0.28 #Camera sensor is X mm back from TOF sensors
            # convert to xy relative to TOF/bumper
            x = d*math.cos(a)
            y = d*math.sin(a)
            x -= dx
            
        # get TOF obstacle detections
        fc_ob_dist = self.tof_fc_obstacle_dist
        fl_ob_dist = self.tof_fl_obstacle_dist
        fr_ob_dist = self.tof_fr_obstacle_dist

        # use TOF distance to help determine when touched
        # d_tof = np.min((fc_ob_dist,fl_ob_dist,fr_ob_dist))
        d_tof_c = fc_ob_dist
        d_tof_rl = np.min((fl_ob_dist,fr_ob_dist))

        if killSwitchActive :
            # Stop motors and wait for kill switch not active
            # Motors are stopped by leaving velocities as msg default = 0
            pass
        elif math.isinf(x) :
        #   if(state!=3):
            self.get_logger().info(f"{func} lost cone {state=}")
            next_state = 0
        elif x > 1.5*self.cd_closer_dist :
            self.get_logger().info(f"{func} cone is too far {d=:.3f} {d_tof_c=:.3f} {d_tof_rl=:.3f} {a=:.3f} {x=:.3f} {y=:.3f} {state=}")
            self.tts(f"State 3: The cone is too far at distance {d:.1f} meters")
            next_state = 0 # restart by looking for the cone
        elif (d_tof_c > self.cd_touch_c_dist) and (d_tof_rl > self.cd_touch_rl_dist) :
            self.get_logger().info(f"{func} approaching cone to touch {d=:.3f} {d_tof_c=:.3f} {d_tof_rl=:.3f} {a=:.3f} {x=:.3f} {y=:.3f} {fc_ob_dist=:.3f} {fl_ob_dist=:.3f} {fr_ob_dist=:.3f} {state=}")
            msg.linear.x = (x/0.2)*self.cd_touch_lin_vel + 0.010
            # turn towards cone center
            # msg.angular.z =  (a/0.393)*msg.linear.x #self.cd_touch_ang_vel
            msg.angular.z =  (8*y)*msg.linear.x

        else : 
            self.get_logger().info(f"{func} touched {d=:.3f} {d_tof_c=:.3f} {d_tof_rl=:.3f} {a=:.3f} {x=:.3f} {y=:.3f} {fc_ob_dist=:.3f} {fl_ob_dist=:.3f} {fr_ob_dist=:.3f} {state=}")
            self.tts("The cone was touched")
            next_state = 4

        self.cmd_vel_publisher.publish(msg)

        return next_state


    #state 4 - wait a short time for judge to see the "touch"
    def wait_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} wait a short time {state=}")
            # self.tts("State 4")
            # self.nav.cancelTask()
        
        killSwitchActive:bool = ks
        next_state = state

        if killSwitchActive :
            # wait until kill switch is not active
            pass
        elif (time.time_ns()*1e-9 - self.cd_timer) > self.cd_touch_wait :
            next_state = 5
        
        return next_state


    #state 5 - backup after "touch"
    def backup_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} backup {state=}")
            self.tts("Backup")
            self.cd_sub_state = 0

        cur_time = time.time_ns()*1e-9            
        killSwitchActive:bool = ks
        # killSwitchChange = ksc
        next_state = state
        dist = self.cd_backup_dist
        vel = self.cd_backup_vel
        t = 1.5*self.cd_backup_dist/self.cd_backup_vel

        if self.cd_sub_state == 0 :
            if not killSwitchActive :
                # set timer to backup a bit manually
                self.cd_sub_timer = cur_time
                self.cd_sub_state = 1

        elif self.cd_sub_state == 1 :
            # TODO: kill switch
            msg = Twist()
            dist = 2.0
            vel = self.cd_backup_vel
            t = dist/vel

            if (cur_time - self.cd_sub_timer) < t :
                x,a = self.driveManObs(-vel)
                msg.linear.x = x
                msg.angular.z = a
                if x==0.0 :
                    next_state = 6

            else :
                msg.linear.x = 0.0 # stop
                msg.angular.z = 0.0
                next_state = 6

            self.cmd_vel_publisher.publish(msg)

        return next_state

    #state 6 - STOP
    def stop_after_touch(self, func:str, state:int, state_change:bool, ks:bool, ksc:bool) -> int :
        if state_change :
            self.get_logger().info(f"{func} STOP {state=}")
            # self.nav.cancelTask()
            # self.tts("State 6: Stopping now")
        
        # Force velocities to zero
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)

        next_state = state
        
        return next_state
    

    #***************************************************************************

    def cancelNav2Task(self) -> bool:
        if not self.nav.isTaskComplete() :
            self.nav.cancelTask()
            self.get_logger().info(f"cancelNav2Task: canceling")

        return self.nav.isTaskComplete()
    

    def get_dist_from_robot(self, Tx:float, Ty:float) -> tuple:
        """
        return (dist, angle, robot_x, robot_y, robot_angle).
        input Tx,Ty is the target map xy coordinate (waypoint cone etc).
        output dist,angle is relative to robot tof_fc.
        robot x,y,angle on map relative to tof_fc.
        """


        # get current robot pose at tof_fc to determine the angle offset
        (tf_OK, current_pose) = self.getPoseFromTF('map','tof_fc_link')
        if not tf_OK : return -1.0
        rx = current_pose.pose.position.x
        ry = current_pose.pose.position.y
        (_,_,ra) =  tf_transformations.euler_from_quaternion([
                                current_pose.pose.orientation.x,
                                current_pose.pose.orientation.y,
                                current_pose.pose.orientation.z,
                                current_pose.pose.orientation.w])

        # calc distance from robot to Target (not absolute xy but relative to tof_fc)
        dx = Tx - rx
        dy = Ty - ry
        d = math.sqrt(dx*dx + dy*dy)
        a = math.atan2(dy,dx)

        return (d, a, rx, ry, ra)

    def gotoConeXY(self, cx:float, cy:float, stop_dist:float, t) -> float:
        """
        The navigation is sent and is non-blocking, caller needs to check when finished.
        Go to the cone location, but stop at a given distance to the cone.
        The coordinate cx,cy is relative to tof_fc sensor.
        The stop_dist is the distance from the cone to stop.
        The t is the time to drive before returning .
        Returns distance to the cone, -1 if it did not succeed.
        """

        if cx!=0 and cy!=0 :
            # self.get_logger().info(f"gotoConeXY: cone at {cx=:.3f} {cy=:.3f}")
            
            (_, _, rx, ry, ra) = self.get_dist_from_robot(cx, cy)
            cd = math.sqrt(cx*cx + cy*cy)
            ca = math.atan2(cy,cx)

            self.get_logger().info(f"gotoConeXY: robot at {rx=:.3f} {ry=:.3f}, {ra=:.3f} cone at relative {cx=:.3f} {cy=:.3f} {cd=:.3f} {ca=:.3f}")

            if cd < stop_dist : return cd

            # calculate using cone dist and angle since we need to stop navigating before cone location 
            # calc angle from "map" to cone to drive with heading direct to cone
            a = ra + ca
            # adjust the distance stop at the cost map boundary
            sd = cd - stop_dist
            # calc goto coordinates x,y for "map"->cone
            x = rx + sd*math.cos(a)
            y = ry + sd*math.sin(a)
            
            goto_pose = self.createPoseStamped(x,y,a,"map")

            self.get_logger().info(f"gotoConeXY: goto {x=:.3f} {y=:.3f} {sd=:.3f} {a=:.3f} {t=:.3f}")

            # non-blocking
            self.nav.goToPose(goto_pose)

        else :
            self.get_logger().info(f"gotoXY: No cone detected")
            cd=-1.0

        return float(cd)



    # Cone detection from camera AI relative to camera "oak-d_frame"

    # TODO: globals to be moved to begining of file
    cone_det_cam_true_cnt:int = 0
    cone_det_cam_true_cnt_thr:int = 4
    cone_det_cam_true_cnt_max:int = 8

    cone_det_cam_true:bool = False     
    """filtered cone detection is true"""


    def cone_point_cam_subscription_callback(self, msg: PointStamped) -> None:
        """
        Detect cone using the camera
        X Y Dist in msg is relative to the camera position "oak_d_frame"
        """
        
        # if msg.point.x == 0 : self.get_logger().info(f"{msg=}")
        x = msg.point.x
        y = msg.point.y
        # TODO: use header time
        t = time.time_ns() * 1e-9 # seconds
        a = math.atan2(y,x)
        d = math.sqrt(x*x + y*y)

        # adjust angle for camera angle in urdf
        # TODO: get from TF (urdf file)
        urdf_a = 0.07
        a += urdf_a
        # limit +-pi
        if urdf_a>+math.pi : urdf_a -= math.pi
        if urdf_a<-math.pi : urdf_a += math.pi
        
        # recompute x y, dist not changed
        x = d*math.cos(a)
        y = d*math.sin(a)

        # TF cone xy relative to "/tof_fc_link" which is considered the front of the robot
        self.cone_at_x_cam = x
        self.cone_at_y_cam = y
        self.cone_at_d_cam = d
        self.cone_at_a_cam = a
        self.cone_det_time_cam = t

        # create last_touched_cone_close_true flag
        # when the current detected cone is within 5m of touched cone
        if len(self.wpConeTouched) == 2 :
            try :
                # touched cone XY location in map frame 
                tcx = self.wpConeTouched["x"]
                tcy = self.wpConeTouched["y"]

                # Detected cone XY in the camera "oak_d_frame" (not map)
                cx = self.cone_at_x_cam
                cy = self.cone_at_y_cam
                ca = self.cone_at_a_cam
                cd = self.cone_at_a_cam

                # cone distance and angle from robot camera "oak_d_frame"
                # (cd, ca, rx, ry, ra) = self.get_dist_from_robot(cx, cy)

                # get current robot pose at tof_fc to determine the angle offset
                (tf_OK, current_pose) = self.getPoseFromTF('map','tof_fc_link')
                if not tf_OK : return -1.0
                rx = current_pose.pose.position.x
                ry = current_pose.pose.position.y
                (_,_,ra) =  tf_transformations.euler_from_quaternion([
                                        current_pose.pose.orientation.x,
                                        current_pose.pose.orientation.y,
                                        current_pose.pose.orientation.z,
                                        current_pose.pose.orientation.w])

                # calc angle from "map" to cone to drive with heading direct to cone
                a = ra + ca
                # calc detected cone coordinates x,y in map frame
                dcx = rx + cd*math.cos(a)
                dcy = ry + cd*math.sin(a)

                # calc distance from detected cone to last touched cone
                dx = tcx - dcx
                dy = tcy - dcy
                tcd = math.sqrt(dx*dx + dy*dy)
                        
                # self.get_logger().info(f"cone_point_cam_subscription_callback: {self.last_touched_cone_close_true=} {tcd=} {tcx=} {tcy=} {dcx=} {dcy=} {cx=} {cy=} {d=} {a=} {(cd, ca, rx, ry, ra)=}")

                # impliment a 1m hysterisis for close cone detection
                if self.last_touched_cone_close_true :
                    if tcd > 5 : 
                        self.last_touched_cone_close_true = False
                        self.get_logger().info(f"cone_point_cam_subscription_callback: det cone is more than 5 meters away {self.last_touched_cone_close_true=}")
                else :
                    if tcd < 4 : 
                        self.last_touched_cone_close_true = True
                        self.get_logger().info(f"cone_point_cam_subscription_callback: det cone is less than 4 meters away {self.last_touched_cone_close_true=}")

            except :
                # use last calculated self.last_touched_cone_close_true state
                self.get_logger().info(f"cone_point_cam_subscription_callback: cone XY conversion issue {self.wpConeTouched=} {self.last_touched_cone_close_true=}")
            

        # count consecutive cone detections when not close to last touched cone
        if self.last_touched_cone_close_true :
            self.cone_det_cam_true_cnt = 0
        elif (self.cone_at_d_cam > 0) :
            if self.cone_det_cam_true_cnt < self.cone_det_cam_true_cnt_max :
                self.cone_det_cam_true_cnt += 1
        else :
            if self.cone_det_cam_true_cnt > 0 :
                self.cone_det_cam_true_cnt -= 1

        # determine filtered cone detect true
        if self.cone_det_cam_true :
            if self.cone_det_cam_true_cnt < self.cone_det_cam_true_cnt_thr :
                self.cone_det_cam_true = False
        else :
            if self.cone_det_cam_true_cnt > self.cone_det_cam_true_cnt_thr :
                self.cone_det_cam_true = True

        # save last valid cone detection y
        if x>0.0 :
            self.cone_at_y_cam_last_det = y

        # self.get_logger().info(f"cone_point_cam_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")

    # Cone detection from lidar scan relative to lidar "lidar_link"

    # TODO: globals to be moved to begining of file
    cone_det_lidar_true_cnt:int = 0
    cone_det_lidar_false_cnt:int = 0

    def cone_point_lidar_subscription_callback(self, msg: PointStamped) -> None:

        # if msg.point.x == 0 : self.get_logger().info(f"{msg=}")
        x = msg.point.x
        y = msg.point.y
        # TODO: use header time
        t = time.time_ns() * 1e-9 # seconds
        a = math.atan2(y,x)
        d = math.sqrt(x*x + y*y)

        # transform cone xy to "/tof_fc_link" which is considered the front of the robot
        self.cone_at_x_lidar = x
        self.cone_at_y_lidar = y
        self.cone_at_d_lidar = d
        self.cone_at_a_lidar = a
        self.cone_det_time_lidar = t

        # count consecutive cone detections
        if self.cone_at_d_lidar > 0 :
            self.cone_det_lidar_true_cnt += 1
            self.cone_det_lidar_false_cnt = 0
        else :
            self.cone_det_lidar_true_cnt   = 0
            self.cone_det_lidar_false_cnt += 1

        # self.get_logger().info(f"cone_point_lidar_callback: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")

    # TODO: globals to be moved to begining of file
    tof_dist_obstacle_max = 0.300
    tof_fc_obstacle_dist:np.float32 = np.inf
    tof_fl_obstacle_dist:np.float32 = np.inf
    tof_fr_obstacle_dist:np.float32 = np.inf
    tof_rc_obstacle_dist:np.float32 = np.inf
    tof_rl_obstacle_dist:np.float32 = np.inf
    tof_rr_obstacle_dist:np.float32 = np.inf
    # tof_fc_obstacle_angle:np.float32 = 0 #TODO: do we need the angle?
    range_r_obstacle_dist:np.float32 = np.math.inf
    range_l_obstacle_dist:np.float32 = np.math.inf

    # Get right range sensor
    def range_r_subscription_callback(self, msg) -> None :
        range_r_obstacle_dist = msg.range

    # Get left range sensor
    def range_l_subscription_callback(self, msg) -> None :
        range_l_obstacle_dist = msg.range

    # Get TOF sensor data for obstacle detection
    def tof_dist_subscription_callback(self,msg:TofDist) -> None :
        tof = msg.tof
        # array shape as 8x8
        dist = np.float32(msg.dist).reshape(8,8)/np.float32(1000) # meters
        # replace -1mm with inf
        dist = np.where(dist>=0, dist, np.inf)

        # find minimum dist for obstacle detection
        # TODO: Do we need to not use the lower rows to not see ground clutter?
        # TODO: Should we have different max based on row?
        # TODO: Do we need to calc distance based on row angle?
        dist_min = np.min(dist)
        if dist_min > self.tof_dist_obstacle_max : dist_min = np.inf

        # self.get_logger().info(f"tof_dist_callback: {tof} {dist_min=:.3f}")

        if tof == "tof_fc" :
            self.tof_fc_obstacle_dist = dist_min
        if tof == "tof_fl" :
            self.tof_fl_obstacle_dist = dist_min
        if tof == "tof_fr" :
            self.tof_fr_obstacle_dist = dist_min
        if tof == "tof_rc" :
            self.tof_rc_obstacle_dist = dist_min
        if tof == "tof_rl" :
            self.tof_rl_obstacle_dist = dist_min
        if tof == "tof_rr" :
            self.tof_rr_obstacle_dist = dist_min


    # Cone distance and angle relative to front TOF sensors
    def tof_fc_mid_subscription_callback(self, msg: Float32X8) -> None:
        # self.get_logger().info(f"{msg=}")

        dmin:float = math.inf
        dmin_idx:int = -1
        idx:int = 0

        dist_max = self.tof_fc_dist_max
        fov = self.tof_fov 
        pt_angle = fov/8

        for d in msg.data :
            if not math.isinf(d) :
                if d < dmin : 
                    dmin = d
                    dmin_idx = idx
            idx += 1
            # self.get_logger().info(f"tof_fc_callback A: {d=:.3f} {dmin=:.3f} {dmin_idx=}")

        # TODO: validate cone detect using width and maybe shape

        if dmin >= dist_max : dmin = math.inf

        d = dmin
        a = fov/2 - pt_angle/2 - dmin_idx*pt_angle
        x = d * math.cos(a)
        y = d * math.sin(a)

        self.cone_at_x_tof_fc = x
        self.cone_at_y_tof_fc = y


        self.cone_at_d_tof_fc = d
        self.cone_at_a_tof_fc = a

        # self.get_logger().info(f"tof_fc_callback B: {x=:.3f} {y=:.3f} {a=:.3f} {d=:.3f} ")
        

    # def gotoPoseBlocking(self, pose, t):
    #     """
    #     Go to the pose within in the time limit
    #     """
    #     self.nav.goToPose(pose)
    #     (result, _) = self.waitTaskComplete(t)
       
    #     return result


    # def getCurrentPose(self) -> tuple:
    #     """
    #     get map->base_footprint (rear differential at ground level) transform
    #     returns (tf_OK, pose) pose: PoseStamped
    #     """
    #     return self.getPoseFromTF('map', 'base_footprint')


    def getPoseFromTF(self, child_frame:str, target_frame:str) -> tuple:
        """
        get child_frame->target_frame transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        # try getting pose a few times
        cnt = 0
        tf_OK = False
        while tf_OK == False and cnt < 5 :
            try:
                tf = self.tf_buffer.lookup_transform (
                    child_frame,
                    target_frame,
                    #self.nav.get_clock().now().to_msg(),
                    rclpy.time.Time(), # default 0
                    timeout=rclpy.duration.Duration(seconds=0.1) #0.0)
                    )
                tf_OK = True

            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().info(f'getPoseFromTF: Could not find transform {child_frame}->{target_frame}: {ex}')
                tf_OK = False
                cnt += 1
                
        if tf_OK == False or cnt >= 5 :
            self.get_logger().info(f'getPoseFromTF: Failed to find transform {child_frame}->{target_frame} after {cnt} tries {tf_OK=}')
            return (False,None) 
        
        # translate wall points to align with map coordinates
        if tf_OK :
            # get x, y, theta from TF
            x:float = tf.transform.translation.x
            y:float = tf.transform.translation.y
            q:float = tf.transform.rotation
            # convert quaterion to euler, discard xx and yy
            (xx,yy,a) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            pose = self.createPoseStamped(x,y,a,child_frame)
        else :
            pose = None
            
        return (tf_OK,pose)
        

    def createPose(self, x:float, y:float, yaw:float) -> Pose :
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
        ) = tf_transformations.quaternion_from_euler(0.0,0.0,float(yaw))
        
        return pose

    def createPoseStamped(self,x:float,y:float,a:float,frame_id:str) -> PoseStamped:
        stamp = self.nav.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        # TODO: use a given stamp time?
        pose.header.stamp = stamp
        pose.pose = self.createPose(x,y,a)
        # self.get_logger().info(pose)
        return pose


    def waitTaskComplete(self,t) :
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            # self.get_logger().info(f"{feedback=}")
            try :
                nt = feedback.navigation_time.sec
                if nt > t :
                    self.get_logger().info(f"waitTaskComplete: Canceling task {nt=} > {t=}")
                    # self.nav.cancelTask()
            except :
                pass
                
        feedback = self.nav.getFeedback()
        result = self.nav.getResult()
        # self.get_logger().info(f"{feedback=} {result=}")
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('waitTaskComplete: Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('waitTaskComplete: Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('waitTaskComplete: Goal failed!')
        else :
            self.get_logger().info(f"waitTaskComplete: nav.getResult() {result=}")

        return (result, feedback)

    def destroy_node(self):
        self.get_logger().info("destroy_node")
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    nav = BasicNavigator()
    # wait never returns
    # nav.waitUntilNav2Active()

    node = NavNode(nav)

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