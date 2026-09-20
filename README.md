# RoboColumbus2025
Wheeled autonomus robot for 2025 DPRG RoboColumbus competion

This project was started in June 2024 and used in the RoboColumbus competition in Fall of 2025.<br>
The robot worked well during testing at home, but failed because of a "simple" error. During the contest when started at the start line it did not move and repeated an announcement that it will try to go to waypoint again ... and again. The problem was that the global cost map size was 100x100M giving it a range of +-50M of planning area, the palnner failed! My home testing was less than 50M to a waypoint<br>
https://www.dprg.org/robocolumbus-2025/

# RoboColumbus2026
Wheeled autonomus robot for 2026 DPRG RoboColumbus competion

In 2026 the electronics were put in the jeep cabin. The power system was upgraded and a reliable external 12V supply connection added. A Pi5 reset switch added and accessable on the rear panel. The TOF sensors were attached to the front grill and rear panel above the bumpers and sealed for dust and sprinkles. The Lidar was mounted on the roof.<br>


# RoboColumbus competition
The basic objective for RoboColumbus are:
-Three orange cones, designated as home, target, and challenge, are placed
outdoors approximately 100 yards from each other. The course between the home cone and the
target cone is clear of obstacles. The course between the challenge cone to both the home
cone and the target cone contains at least one obstacle that requires the robot to deviate from a
direct path. The challenge is to touch and stop at each cone.
- If multiple contestants find and touch the same number of cones then the total time is used to select the winner.
- Robots run on a field of grasses and some tree/bushes boxes etc for obsticles to avoid. Some pavement is OK.
- GPS coordinates of the 3 traffic cones in the field are provided. There should be obsticals between some of the cones. The owner can create GPS coordinates to avoid obsticles.
- The cones are orange and about 17" high and 7" diameter.
- Robots are NOT allowed on the field before competition.
- Run times are less than 15 Minutes
- Robots checked for weight(65 lbs), size(48x48x60 in), and safety switch compliance.

(https://www.dprg.org/wp-content/uploads/2024/06/robocolumbusplus-20240611-1.pdf)

# Robot platform
The AXIAL RCX6 1/6 scale Jeep rock crawler platform was selected to be used for the robot. This platform allows generous room for sensors and electronics to be added and can be installed into the interior of the Jeep for protection.
https://www.axialadventure.com/product/1-6-scx6-jeep-jlu-wrangler-4x4-rock-crawler-rtr-green/AXI05000T1.html<br>

The motor drive, steering and shift control signals can be switched between the RC receiver and the computer. A module in the engine compartment performs this with a serial interface to the computer in the Jeep cabin. The RC receiver is powered from the ESC motor controller (facrory default) and the connections to the computer is optional for movement, the module will power up to use the RC receiver signals as the default.<br>

The main batteries in the engine compartment are also be used for sensors and computer power. A DC-DC module is in the engine compartment with 2 sets of 9V at 3 Amps power leads to the Jeep cabin. Fusing the power from the main battery is added for protection. The control multiplexor and DC converters are on one board in a sealed box on the chassis.<br>

# Power
Tap from 1 or 2 (in parallel) 3S 11.1V LiPo battery in engine compartment with a fuse. The batteries can be hot swapped.<br>
Two 9V DC-DC converters/regulators for computer and sensors in sealed box in engine compartment.<br>
In the cabin a power jack for external 12v power, hot plugable, is available for powering the cabin electronics. If the chassis battery power is enabled (a switch under the front right wheel well) it powers the RC receiver and wheels and steering, the cabin electronics are powered by the chassis power if the external 12V supply is not connected.<br>
There are two 5V 4A DC regulators in the cabin, one for the Pi5 and the other for everything else.<br>

# Remote connectivity
- RC transmitter for selecting the recevier or computer </br>
  When the computer is selected it is used as a kill switch </br>
  When receiver is selected it is used for manual steering control </br>
- Game controller with drive controls and start and kill buttons.<br>
- WIFI for development and debug and course configuration </br>
- Tiger VNC server on Pi ; Tiger VNC Viewer on PC 192.168.1.155<br>


# Sensors
## GPS
GPS receiver can be used for primary odometry. The wheel odometry and IMU is fused with the GPS.<br>
This is a standard GPS with many meters of in-accuracy.<br>
The GPS receiver is the U-Blox M10Q using the SparkFun_u-blox_GNSS_v3 library.<br>

## IMU
The IMU is the BNO085 which provides a compass heading and angular rotation velocity and uses the Adafruit_BNO08x library.<br>

## Camera
The cones are detected using the depth camera OAK-D-Lite and the Coneslayer AI ROS package.<br>

## LIDAR
Rotating LIDAR SLLIDAR S3 for far obsticle detection and avoidance.<br>

## TOF
Time Of Flight sensors on front and rear for close obstical avoidance and close cone sensor.<br>
There are 3 VL53L8CX on both front and back each with a total FOV of 45x135 deg with an array of 8x24 distance measurements.<br>

# Computer 
- Raspberry Pi5 with OS Ubuntu 24.04 and a 225G SSD.<br>

# Software
## ROS2
- Using ROS2 Jazzy which requires Linux Ubuntu 24.04 OS
- Ubuntu 24.04 is running natively on the SSD
- The rc_ws directory is published on https://github.com/mikew123/rc_ws.git
- The ros2 rc_ws workspace has been modified from rc25_ws to remove date specific files in the repository and branch for 2026 (rc26) was created.

## ROS2 packages
- The navigator package is used for waypoint navigation by launching the nav2_bringup/launch/bringup_launch.py.<br>
- The robot_localization ekf_node package (2 instances) is used for EFK filtering and managing the map->odom and odom->base dynamic transforms.<br>
- The robot_state_publisher uses a urdf file to create the static transforms from the base to the sensors.<br>
- The robot_localization navsat_transform_node package is used to connect the GPS data to the efk_node that manages the map->odom transform.<br>
- Lots of ROS python libraries are used.<br>

## 3rd party ROS2 
These packages are used for the Lidar and cone detection on the OAK-D-Lite camera. They are cloned into the home directory. The ros2 workspace rc_26 accesses them using "../*"
- Cone detection AI
https://github.com/mw46d/ros_coneslayer.git<br>
The launch code has been tweaked to work with the OAK-D Lite camera system.<br>
- Lidar management
https://github.com/Slamtec/sllidar_ros2.git<br>

# Electronic modules
## Microcontroller
Uses 3 Waveshare RP2040-Zero, a Pico-like MCU Board Based on Raspberry Pi MCU RP2040, Mini ver.</br>
<https://www.waveshare.com/rp2040-zero.htm><br>
- Messages are formatted as Json strings messages over serial. Each controller is selected using /dev/serial/by-id/"..." when opening the serial point on a ROS node<br>
- The Arduino IDE code for each module is in the arduino folder.<br>
- Engine controller: selects between the computer in the cabin and the RC receiver. It also relays the steering, throttle and shift signals from the computer and the servos and motor ESC. It also allows the computer to set default ranges etc, and sends statuses back.</br>
- IMU GPS controller: collect the IMU and front and back TOF sensor data and send to the ROS node over serial<br>
- TOF controller: Collects the TOF sensor values and sends them to the ROS node over serial<br>

## Servo switch
This switches the servo and engine ESC signals between the RC receiver and the computer. </br>
Pololu 2807 4-Channel RC Servo Multiplexer (Partial Kit)</br>
<https://www.pololu.com/product/2807>

## Main DC-DC converter
This supplies power for the cabin electronics. They have been modified to supply 9V output to be multiplexed with the development external 12V supply.</br>
Pololu 6V, 2.7A Step-Down Voltage Regulator D36V28F6</br>
<https://www.pololu.com/product/3783>

## Level shifter
This protects the 3.3V input pins on the micro controller from the 6V signals from the RC receiver</br>
Pololu Logic Level Shifter, 4-Channel, Bidirectional 2595</br>
<https://www.pololu.com/product/2595>

## TOF
These time of flight sensors aare used for obstcal avoidance and planning. There is a set of for both the front and back of the robot. The cluster of 3 sensors give a FOV of 45x135 with an array of 8x24 distance measurements. This L8 sensor model has the best outdoor Lux rating since this competition of out doors.<br>
Pololu VL53L8CX Time-of-Flight 8×8-Zone Distance Sensor Carrier with Voltage Regulators, 400cm Max.<br>
<https://www.pololu.com/product/3419>

## TOF module DC-DC converter
There are two converters to supply 3.3V power to the TOF sensors on the front and back. The 3.3v from the RP2040 module does not have the current rating for the sensors. A seperate regulator is used for the 3 front sensors and for the rear sensors and have enable controls to allow I2C address configuration.<br>
Pololu 3.3V, 600mA Step-Down Voltage Regulator D36V6F3.<br>
<https://www.pololu.com/product/3791>

## I2C buffer
The I2C buffer is used for each I2C bus to the front and rear TOF sensor clusters. The I2C operates at 1MHz and was not reliable because of the long cables.<br>
Adafruit TCA4307 Hot-Swap I2C Buffer with Stuck Bus Recovery - STEMMA QT / Qwiic.<br>
<https://www.adafruit.com/product/5159>

## IMU 
The BNO085 IMU is a pretty good inertial measurement sensor with orientation for a compass reading (yaw) as well as pitch and roll if needed.<br>
Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 - STEMMA QT / Qwiic.<br>
<https://www.adafruit.com/product/4646>

## GPS
This module has both a GPS receiver and a Compass. I could never get the compass to work well. The GPS receiver was the newest model SAM-M10Q from uBlox which I presumes works better than older models.<br>
Amazon Matek M10Q-5883 GNSS Compass Module GNSS Ublox SAM-M10Q QMC5883L Magnetic Compass for RC Drone FPV Racing.<br>
<https://www.amazon.com/M10Q-5883-Compass-SAM-M10Q-QMC5883L-Magnetic/dp/B0BZ7VJKHV>

# Development rig
For 2025 the development was performed using the parts mounted on a board which was mounted on the chassis, the components were not in the Jeep cabin but the sensors were place close to the desired final positions in the Jeep cabin.<br> 
<img src="support/Rc25DevelopmentRig.jpg">

In 2026 the electronics were placed in the Jeep cabin and the only components outside are the Lidar mounted on the roof and the front and back TOF sensor arrays which mount on the jeep body above the bumpers. There are long I2C cables fron the tof controller board to the sensors which are attached to the bumpers using blue tape during development.<br>

????The Pi5 with SSD and the USB hub are mounted in the cabin. The Pi5 and USB hub are powered from seperate regulators from the batteries or external 12V supply.<br>
<img src="support/Pi5wSsdAndUsbHub.jpg">


# New electronics in engine compartment
A waterproof box is mounted to the rear battery holder. This box holds the DC-DC converters for the electronics in the cabin, the servo signal mux and a micro controller to manage the servo switch and send some telemetry to the computer in the cabin. The servo switch defaults to connecting the RC receiver to the motor and servos.</br>
I needed to augment the polulu PWM switch module with analog switches for the throttle signal which had better performance using the serial protocol instead of the servo type PWM. If I did it over I would have also used analog switches for the steering and shift PWM signals.<br>
NOTE: This photo is older and does not have the analog switches.<br>
<img src="support/RCX6-engine_compartment_with_new electronics.jpg">

## Method to switch to computer control
The steering wheel and speed switch on the RC transmitter is used to switch to-from computer control. This must be done with throttle at idle.<br>
- Switch to computer control: Turn steering left (CCW) and press speed switch DN(high) then UP (low).<br>
- Switch to receiver control: Turn steering right and press speed switch DN then UP.<br>

## Electronics boards
The engine, imu_gps and tof custom boards are created using 0.1" grid perf boards and 30 AWG wires that are soldered to the pins of modules and components.<br>

### Engine Controller Board
The electronic module is connected using soldered wires and a 0.1" breadboard cut to fit the waterproof box interior.<br>
The RC receiver signals are connected to the MASTER pins of the RC switch module using standard servo cables.<br>
The ESC motor and servo signals are connected to the OUT pins of the RC switch module using standard servo cables.<br>
The ESC throttle signals are switched using analog switches. The throttle signals I used are a bi-directional serial protocol instead of a PWM protocol for better reolution and the get telemetry from the ESC.<br>
The controler pins are hard wired to the SLAVE pins of the RC switch.<br>
The controller USB supplies the power only to the micro controller</br>
The DC-DC converters are inside the waterproof box. The power input from the battery has an in-line fuse which is not in the box to protect the batteries from an electrical short.</br>
<img src="support/engine_compartment_electronics_module.jpg"></br>
<br>
<img src="support/RCX6_engine_electronics_schematic.jpg"></br>


### IMU and GPS Controller Board
<img src="support/rc25_ImuGpsController.jpg"></br>

### TOF Sensors Controller Board
<img src="support/rc25_TofSensorsControler.jpg"></br>
<img src="support/rc25_TofSensorsControllerPicture.jpg"></br>

<!-- ### Battery/external power Board
This board uses Schotky diodes to Isolate the extarnal power from the Regulated power regulators in the engine compartment (connected to the battery)<br>
The power sources can be hot swapped without shutting down the Pi5<br> -->

# Micro controller firmware
The microcontroller firmware is C-code developed using the Arduino IDE. The interface to the controller uses the USB port for a serial communications interface. A simple Json data structure sends data to-from the computer in the cabin over the USB serial interface cables.<br>

### IMU & GPS Controller (RP2040)
Uses a dual-core RP2040 microcontroller to collect IMU (BNO085) and GPS (U-Blox M10Q) data.
- **Core 1:** Polls and processes IMU and GPS sensor data, formats, and sends JSON messages over USB serial to the host computer.
- **Core 2:** Buffers sensor data and passes it to core 1 for transmission.
Data is formatted as JSON and sent over USB serial. The inter-core buffer ensures reliable, low-latency transfer of sensor packets to the host computer for real-time robot localization and navigation.
**Sensor libraries used:**
- Adafruit BNO08x (IMU)
- SparkFun u-blox GNSS v3 (GPS)

### TOF Controller (RP2040)
Uses both cores of the RP2040 microcontroller to manage Time-of-Flight (TOF) sensor data.
- **Core 1:** Polls and processes data from the front cluster of three TOF sensors, formats, and sends JSON messages over USB serial to the host computer.
- **Core 2:** Polls and processes data from the rear cluster of TOF sensors, buffers distance measurements, and passes them to core 1 for transmission.
Front and rear TOF devices are connected to separate I2C interfaces (`Wire` and `Wire1`).
A serial queue is used for inter-core communication, allowing core 2 to send data to core 1 for transmission.
Sensor data validity is filtered using the status message from each TOF sensor, ensuring only reliable measurements are sent. The controller uses the VL53L8CX TOF sensor library for sensor interfacing and data acquisition.
This design enables efficient, parallel acquisition and low-latency transfer of TOF sensor readings for real-time processing.<br>

### Engine Controller Firmware (RCX6-engine-ctrl-SRXL2)
Controls engine, steering, and shift functions for the RCX6 robot using SRXL2 serial protocol. Handles multiplexing between RC receiver and computer control, relays commands, and manages telemetry.<br>

#### Main Firmware (`RCX6-engine-ctrl-SRXL2.ino`)
- Initializes hardware, sets up serial communication, and manages the main control loop.
- Handles switching between RC and computer control modes.
- Processes incoming JSON commands and relays them to the appropriate subsystems.

#### PWM Control (`rcxpwm.cpp`, `rcxpwm.h`)
- Implements PWM signal generation and decoding for motor, steering, and shift channels.
- Provides functions to read and write PWM values for both RC and computer control.
- Manages safe switching and signal integrity.

#### SRXL2 Protocol (`srxl2.cpp`, `srxl2.h`, `srxl2Structs.h`)
- Implements the SRXL2 serial protocol for communication with Spektrum ESC and telemetry devices.
- Defines SRXL2 message structures and parsing logic.
- Handles bidirectional UART communication, message encoding/decoding, and telemetry extraction.

#### Libraries Used
- Arduino core libraries
- SRXL2 protocol library (custom implementation)
- Standard C++ libraries for data structures and serial communication
**Sensor libraries used:**
- Arduino VL53L8CX (TOF)
- Arduino_JSON
## RC switch interface
### Outputs to RC switch
- Slave select
- Steering servo
- Motor speed/fwd-rev
- High-Low speed select servo
### Inputs from RC switch
- Steering
- High-Low speed select
## Decode RC receiver Steer and Shift PWm signals
### Switch to/from computer control
- From receiver<br>
Enable computer control: Steering CW, toggle Shift DOWN then UP<br>
Enable receiver control: Steering CCW, toggle Shift DOWN the UP<br>
- From computer<br>
Send JSON message<br>
### Kill switch
The remote control transmitter can be used as a "kill" switch to stop and resume robot movements. The kill switch operation can be used when the engine controller is controlled by the computer in "cv" mode. Turning off the remote transmitter or switching out of "cv" mode disables the kill switch and it must be re-enabled next time it is in "cv" mode<br>
The engine must be in computer control mode "cv":<br>
The shift action is to press the shift switch up the down.<br>
- Enable kill switch: Pull trigger and shift<br>
- Disable kill switch: Change mode or power transmitter OFF<br>
- Latch the kill switch: While enabled and trigger is pulled and shift then release trigger<br>
- Unlatch the kill switch: Pull the trigger (must re-latch if desired)<br>

While the kill switch is enabled releasing the trigger will stop any movement. Pulling the trigger will resume movement. Changing mode or powering the transmitter OFF will disable the latch.<br>

## JSON messages
### Congfigure messages from the computer
- Switch to/from computer control modes<br>
{"mode":"bypass"}: Use remote Rc transmitter to control the robot directly <br>
{"mode":"passthru"}: Use remote Rc transmitter to control the robot indirectly <br>
{"mode":"cv"}: Control robot using fwd velocity and steering angle<br>
{"mode":"pct"}: Control robot using Steering and throttle percents <br>
### Runtime control messages from computer
When powered on the throttle and steering are from the receiver. They can be switched to the computer with a JSON command or from controls on the receiver<br>
#### Computer control using percent
The mode must be "pct":
- Throttle<br>
{"thr":pct} percent throttle, forward +pct, reverse -pct<br>
- Steer<br>
{"str":pct} percent steering, right +pct, left -pct<br>
- Gear shift<br>
{"sft":bool} Gear shift, high true, low false<br>
#### Computer control using velocity and angle
The mode must be "cv":
-{"cv":[velocity, steer]}
Velocity in meters per second<br>
Steering in radian angles (+-40 degrees)

### Status messages to computer
## Throttle bidir half duplex serial messages
The Throttle interface between the RC receiver and the ESC controller is a bidirectional UART serial interface at 155200 baud. The RC receiver sends control signals to the ESC. The ESC responds with telemetry data to the RC receiver.<br>
The devices power on with the RC <-> ESC Throttle in BYPASS mode, the Zero can disable BYPASS and receive the serial data from both the RC and the ESC. The Zero can enable serial TX to both the RC and ESC between RX messages to impliment the half duplex.<br>

https://github.com/SpektrumRC/SRXL2

https://github.com/SpektrumRC/SRXL2/blob/master/Docs/SRXL2%20Specification.pdf

https://github.com/SpektrumRC/SpektrumDocumentation/blob/master/Telemetry/spektrumTelemetrySensors.h


# Velocity control

The Spektrum ESC motor speed control is in percent throttle. It seems like any Throttle value less than 21 does not activate the motor so the range seems to be about 20 to 100 where 20 is 0 RPM. Reverse seems to be about the same -20 to -100.<br>
An early test plot of Throttle vs Rpm and Velocity(M/S):<br>

<img src="support/Test scripts/test_plot.png"></br>
The sample rate is 10Hz and there are 10 values for each throtthe value 20 to 100<br>
Throttle is not scaled but offset by 20 so the plot range is 0 to 80 for actual values of 20% to 100%. Velocity Mps is scaled by 100 to graph well. Motor Rpm is scaled by 0.055.<br>
The motor RPM extracted from the ESC telemetry as basically linear with Throttle but has steps so is not very usable. The Velocity Meters/Second is very linear with Throttle except at the upper throttle values above 90% (70 on plot) where it tapers off a bit. The velocity has some noise, it might be caused by the "bumps" on the omni wheel of the GoBuilda Pod Wheel encoder that I used measure the rottaion of the drive shaft by pressing against it.<br>
The plotted velocity Mps was calculated as (millisDiff/1000)/(odomDiff/5,615). The scale factor of 5,615 was determined my measuring the circumference of the tire (~570mm) and reading the odom encoder change after the tire rotates 10 times. (The throttle was set to 25% and the time it was running tweaked to get exactly 10 rotations.)<br>
<br>
This is a detail view of the first 200 samples:
<img src="support/Test scripts/test_plot_detail.png"></br>


## Odometery
The odometry for the robot comes from 3 sources:
- There is a drive shaft rotation encoder for linear velocity. 
- The IMU provides angular velocity and a compass heading
- GPS provides absolute position lat,lon (X,Y) for localization.<br>

### Drive shaft roation encoder
The rotation of the driveshaft from the transmission to the wheels uses a GoBuilda. It is pressed against a driveshaft coupling that does not move as the suspensions bounces using the spring and a custom 3D printed mounting bracket. The bracket connects to the chassis with 3 bolts and has adjustment for spring tension onto the drive shaft coupler.<br>
<img src="support\GoBuilda_odometry_pod.jpg"><br>
<img src="support\ShaftEncoderMount.jpg"><br>
<img src="support\ShaftEncoderPicture.jpg"><br>

# ROS code
NOTE: AI generated summaries.<br>
NOTE: Needs to be updated for 2026 as needed.<br>

### Teleoperation Node — `robocolumbus25_teleop_node.py`

- **Purpose:** Converts joystick input into robot motion and simple JSON control messages.
- **Subscribes:** `/joy` (`sensor_msgs/Joy`), `json_msg` (`std_msgs/String`).
- **Publishes:** `/cmd_vel/teleop` (`geometry_msgs/Twist`), `json_msg` (`std_msgs/String`).
- **Behavior:** Maps joystick axes to throttle (axis 1) and steering (axis 3), applies configured
  max linear speed and steering limits, computes `angular.z` from steering using wheelbase geometry
  (angular.z = tan(steer_angle) * linear_x / wheel_base), publishes `cmd_vel` when joystick moved,
  and emits button-change JSON messages for kill/do events. Sends an initial TTS JSON message
  "Teleop Node Started" at startup.
- **Quick run:** Launch as part of your ROS2 workspace or run the script directly with the ROS2
  environment sourced (it contains a `main` so it can be executed with `python3`).

### Speaker Node — `robocolumbus25_speaker_node.py`

- **Purpose:** Listens for JSON `json_msg` and speaks text via `espeak`.
- **Subscribes:** `json_msg` (`std_msgs/String`) — expects `{"speaker":{"tts":"..."}}`.
- **Publishes:** `json_msg` (`std_msgs/String`) — e.g., startup TTS messages.
- **Behavior:** Parses incoming JSON, logs and runs `espeak -a <volume> "<text>"` for `tts` entries; default `volume=200`. Contains a runnable `main` allowing direct execution.

### Cone Node — `robocolumbus25_cone_node.py`

- **Purpose:** Detects cones from camera and LIDAR, validates detections, and publishes cone positions.
- **Subscribes:** `/color/spatial_detections` (`vision_msgs/Detection3DArray`), `/scan` (`sensor_msgs/LaserScan`), `/json_msg` (`std_msgs/String`).
- **Publishes:** `/cone_point_cam` (`geometry_msgs/PointStamped`), `/cone_point_lidar` (`geometry_msgs/PointStamped`), `/json_msg` (`std_msgs/String`).
- **Behavior:** Filters and validates camera detections (bounding-box checks + median-7 distance filter) and processes LIDAR scans to find cone-like clusters using distance-jump detection and width/ratio validation; publishes detected cone coordinates in appropriate TF frames (`oak-d_frame`, `lidar_link`) and emits TTS/status JSON when sensors activate or lose track.

### TOF Node — `robocolumbus25_tof_node.py`

- **Purpose:** Publishes point clouds and raw distance arrays from serial-connected RP2040 TOF sensors (front/rear, left/center/right).
- **Subscribes:** `json_msg` (`std_msgs/String`).
- **Publishes:** `tof_fc`, `tof_fl`, `tof_fr`, `tof_rc`, `tof_rl`, `tof_rr` (`sensor_msgs/PointCloud2`), `tof_fc_mid` (`rc25_interfaces/Float32X8`), `tof_dist` (`rc25_interfaces/TofDist`), `json_msg` (`std_msgs/String`).
- **Behavior:** Reads JSON packets for each TOF sensor, publishes 8x8 point clouds (with curvature correction) and raw distance arrays, and emits TTS/status messages at startup.



### Wheel Controller Node — `robocolumbus25_wheel_controler_node.py`

- **Purpose:** Controls rear wheel velocity and front steering angle for the Jeep robot using Ackermann steering. Arbitrates between `/cmd_vel` (from ROS2 navigator) and `/cmd_vel/teleop` (from teleop node); teleop commands take priority for 1 second after receipt, blocking navigation commands during that time.
- **Subscribes:** `/cmd_vel` (`geometry_msgs/Twist`), `/cmd_vel/teleop` (`geometry_msgs/Twist`), `json_msg` (`std_msgs/String`).
- **Publishes:** `wheel_odom` (`nav_msgs/Odometry`), `json_msg` (`std_msgs/String`).
- **Behavior:** Converts velocity/steering commands to Ackermann steering, arbitrates between teleop and navigation, sends commands to engine controller via serial JSON, processes wheel encoder odometry, and emits TTS/status messages at startup and on state changes.


### Navigation Node — `robocolumbus25_nav_node.py`

- **Purpose:** Autonomous navigation to waypoints and cones using GPS, IMU, camera, LIDAR, and TOF sensors.
- **Subscribes:** `/cone_point_cam`, `/cone_point_lidar`, `/tof_fc_mid`, `/tof_dist`, `/gps_nav`, `/json_msg`.
- **Publishes:** `/cmd_vel`, `/set_pose`, `/json_msg`.
- **Behavior:** State machine for calibration, waypoint navigation, cone detection/approach, and backup. Reads waypoints from YAML, configures navigation parameters, and coordinates with other nodes via JSON messages and TTS.


### Controller Node — `robocolumbus25_controller_node.py`

- **Purpose:** Controls overall robot navigation and status. Listens for engine status JSON messages, manages battery status publishing, and coordinates pose setting. Publishes TTS/status messages and interacts with other nodes via `json_msg`.
- **Subscribes:** `json_msg` (`std_msgs/String`)
- **Publishes:** `battery_status` (`sensor_msgs/BatteryState`), `set_pose` (`geometry_msgs/PoseWithCovarianceStamped`), `json_msg` (`std_msgs/String`)
- **Behavior:** Processes engine status messages, manages battery status, sets robot pose, and coordinates TTS/status communication with other nodes.



### IMU & GPS Node — `robocolumbus25_imu_gps_node.py`

- **Purpose:** Reads JSON sensor packets from a serial-connected RP2040 and publishes standard ROS2 IMU/GPS messages.
- **Subscribes:** `json_msg` (`std_msgs/String`), `gps_nav` (`sensor_msgs/NavSatFix`).
- **Publishes:** `imu` (`sensor_msgs/Imu`), `imu/cal` (`rc25_interfaces/ImuCal`), `gps_nav` (`sensor_msgs/NavSatFix`), `gps_pose` (`geometry_msgs/Pose`), `cmp_azi` (`std_msgs/Float32`), `json_msg` (`std_msgs/String`).
- **Behavior:** Opens the serial device, parses incoming JSON lines for `imu`, `gps`, and `cmp` packets. Publishes `sensor_msgs/Imu` constructed from rotation vectors, angular velocity and linear acceleration; publishes `NavSatFix` for GPS with covariance and local `gps_pose` offsets; publishes calibration `ImuCal` messages and compass azimuth (`cmp_azi`). Sends configuration commands to the serial controller and emits simple JSON status messages on `json_msg`.


### Custom Interface Messages — `rc25_interfaces/msg`

- **Float32X8.msg:** Array of 8 float32 values, used for publishing TOF sensor distance data and similar multi-value sensor outputs.
- **ImuCal.msg:** IMU calibration status and data, typically includes fields for calibration state, progress, and results from the robot's IMU.
- **TofDist.msg:** Structured TOF sensor distance data, includes sensor identifier and a 64-element float32 array for detailed distance measurements from TOF sensors.


## 3rd party ROS packages used
The Lidar is managed using the https://github.com/Slamtec/sllidar_ros2.git package<br>
The Camera cone detection AI uses the https://github.com/mw46d/ros_coneslayer.git package<br>
The 3rd party libraries are used by launching them within the rc25 launch file<br>

## TF link map
The oak-d physical component TF frames are created in ros_coneslayer and connected to in the URDF file<br>
The complete TF frame map (pdftoppm -png rc25_08_17_25.pdf rc25_08_17_25):<br>
<img src="support/rc25_08_17_25-1.png"><br>

## Waypoints
The way points are set using a YAML formated waypoints file. The starting pose and waypoints can be in either X,Y or lat/lon format. The GPS can be enabled or not. The waypoints file are in the /sambashare directory so that the PC can easily acces them remotely<br>

## Using GPS
https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html<br>
<img src="support/Gps_localization.jpg"><br>
I ended up connecting the odometry output of the EFK local node to the EFK global (GPS) node instead of connecting EFK global to IMU and Engine odometry<br>
There are seperate param.yaml files for the EFK_local, EFK_global and navsat_transform nodes<>
The AMCL node is disabled in the nav2 params file, but needed to set some other parameters to make AMCL happy.<br>
The navsat_transform creates the map->odom TF for localization correction using GPS.<br>
### GPS ROS odd stuff
I had to set the GPS datum in the navsat_transform params even though I set wait_for_datum: true!! It must be relatively close to the actual GPS waypoints location. Example: I set the datum in the file to my house in Dallas for testing, but that was not close enough for the competition in Farmersville!!<br>
I got best results updating the dataum param with actual GPS value read from the reciever at the initial position<br>
The nav2 navigation needs the X,Y coordinates relative to the initial position, it does not accept GPS lat,lon which are converted to X,Y meters using UTM conversions.

# TODO for 2026:
## Electrical Mechanical
### Put electronics inside jeep - DONE
- Lidar mounted on roof
- Does camera detect cones through existing windshield? - YES
- NOTE: The camera overheats and needed a fan to cool.

### Other
- Improve external DC power - YES
- Work on transmission and drive chain: noticable grease/oil and odd noises

## Code
- Detect cone while going to waypoint and then approach it before getting to waypoint
- If cone is not detected or lost detetction at waypoint go to points close by (say +-5M) if no cone is found go to next waypoint
- Avoid obstacles while "manually" searching in movement pattern for the cone 
- Convert nodes to LifeCycle
- Manage modified coneslayer ROS package better, maybe add to the rc25 workspace and/or make a fork and clean up mods and submit for inclusion in github.

## Simulation?
- Create simulation model
- Create Gazebo height map for terrain hills and valleys
- Add odd obstacles like grass/weeds/branches to Gazebo
- Sensor sensitivity to rounded objects and small things like weeds?
- Model robot bounce caused my shock obsorbers?
  
## RTK GPS?
- Update to RTK $$

# Rebuild with electronics inside cabin - DONE

Photos of the jeep chassis as well as the electronics mounted in the jeep body and outside the body:

Jeep chassis showing the loose cables that go to the engine controller box<br>
You can see the Lidar mounted on the roof<br>
<img src="support/car_chassis_no_electronics.jpg"><br>

The underside of the electronics showing the power and USB cables to the engine controller box<br>
<img src="support/car_undercarriage.jpg"><br>

Electronics in cabin with the Lidar on the roof<br>
<img src="support/car_with_stuff_inside.jpg"><br>

NOTE: needs updated pics with new WiFi and camera fan and external 12V jack and Pi5 reset switch.<br>

NOTE: The front and rear TOF sensors need to be mounted on the jeep front and rear


Electronics mounted on board front and rear view<br>
Note the WiFi antennas in the back<br>
<img src="support/electronics_fron_view.jpg"><br>
<img src="support/electronics_on _board.jpg"><br>

View through windows<br>
<img src="support/view_back_window.jpg"><br>
<img src="support/view_through_side_window.jpg"><br>
