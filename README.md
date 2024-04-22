# TM ROS Driver

## 1. Overview

Techman Robot is a state-of-the-art production tool that is highly compatible and flexible to collaboration between
human and machine. The Robot Operating System (ROS) provides abundant libraries and tools which can be utilized to
reduce the cost of trivial development software tool and build robot applications without struggling.
Our TM ROS driver provides nodes for communication with Techman Robot controllers, data including robot states,
images from the eye-in-hand camera and URDF models for various robot arms via _TMflow_.

## 2. Feature

This driver is for __ROS2 Foxy, Humble__ and __Rolling__.

To use the driver, make sure your ROS PC is installed correctly.

If the user wants to know how to use the ROS1 driver,
please visit the [TM ROS1 driver](https://github.com/TechmanRobotInc/tmr_ros1) website
or directly click the __TM ROS driver version__ listed in the table below.

More information: TM ROS driver support list
|ROS Distro (ROS Environment Setup)|TM ROS driver version|TM ROS Vision|Remark: switch GitHub branches|
|:---|:---|:---:|:---:|
|[__<font color=#808080ROS Noetic Ninjemys__](http://wiki.ros.org/noetic)|[__<font color=#0000FFTM ROS1 Noetic driver__](https://github.com/TechmanRobotInc/tmr_ros1/tree/noetic)|supported|noetic|
|[__<font color=#808080ROS Melodic Morenia__](http://wiki.ros.org/melodic)|[__<font color=#0000FFTM ROS1 Melodic driver__](https://github.com/TechmanRobotInc/tmr_ros1/)|x|master|
<!-- markdownlint-disable MD013 -->
|[__<font color=#808080ROS 2 Foxy Fitzroy__](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)|[__<font color=#800000TM ROS2 Foxy driver__](https://github.com/MatthijsBurgh/tmr_ros2)|supported|master|
<!-- markdownlint-disable MD013 -->
|[__<font color=#808080ROS 2 Dashing Diademata__](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/)|[__<font color=#800000TM ROS2 Dashing driver__](https://github.com/MatthijsBurgh/tmr_ros2/tree/dashing-devel)|supported|dashing-devel|

Note1: The two current master branches are ROS1 Melodic and ROS2 Foxy.

Note2:
The tutorial that follows mentioned how to build a ROS environment on Ubuntu by sourcing is
to take the ROS installed through the Debian packages as an example.

### ROS2 Driver

The driver for ROS2 publishes identical topics and provides identical services as [TM ROS1 version](https://github.com/TechmanRobotInc/tmr_ros1).

This driver uses _ROS2 composition_, there are two nodes in the identical process:
one node publishes topics while the other node sets up service servers.

### Installation

Clone the TM ROS driver of the git repository into your working directory and then build it.

The user can directly refer to the chapters introduced in the following text:
steps 1 to 4 of __&sect; Usage with demo code & driver__.

## 3. Usage

The TM ROS driver is designed to interface the TM Robot's operating software (__TMflow__) with the
Robot Operating  System (ROS)
so that program developers and researchers can build and reuse their own programs to control the TM robot externally.

After installing the correct ROS2 version of the computer, the next step is to ensure that your hardware,
control computer, and TM Robot are all properly configured to communicate with each other.
See below to make sure the network settings on your computer are correct,
the TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.

### TMflow Listen node setup

The __Listen node__: a socket server can be established and be connected with ROS,
to communicate to an external device according to the [defined protocol](https://assets.omron.eu/downloads/manual/en/v1/i848_tm_expression_editor_and_listen_node_reference_manual_en.pdf).
The user can make the robot communicate with the user's ROS (remote)
computer equipment through a wired network when all the network parameters in the _Network setting_ are set.

1. Create a _Listen task_ of flow project of __TMflow__ software,
   and then drag the __Listen node__ from the _nodes menu_ onto the project flow, as shown below.
   [![1](figures/1.png)](https://www.youtube.com/watch?v=LuKE2wVNn5Y)

2. Set the `Network` settings: mouse-click to enter the page of __System &rArr; Network__ in order.
Example: Set the Subnet mask: to 255.255.255.0 and IP address 192.168.10.2
__Note__: Set the network mask, and the communication with the TM Robot must be in the set domain.
   ![2](figures/2.png)

3. Set the __Ethernet Slave__ `Data Table Setting` item:
   mouse-click to enter the page of __Setting &rArr; Connection &rArr; Ethernet Slave__ in order.
   We recommend _one easy method_[^1]
   to set the __Ethernet Slave__ `Data Table setting` is to directly import the software package.

   Or the previously provided method as follows:
   (Note: TMflow software version changes may have slightly different settings.)
   The user can manually click the `Data Table Setting`[^2] Item
   and check the following boxes as item _predefined_[^3] to receive/send specific data:

      - [x] Robot_Error
      - [x] Project_Run
      - [x] Project_Pause
      - [x] Safeguard_A
      - [x] ESTOP
      - [x] Camera_Light
      - [x] Error_Code
      - [x] Joint_Angle
      - [x] Coord_Robot_Flange
      - [x] Coord_Robot_Tool
      - [x] TCP_Force
      - [x] TCP_Force3D
      - [x] TCP_Speed
      - [x] TCP_Speed3D
      - [x] Joint_Speed
      - [x] Joint_Torque
      - [x] Project_Speed
      - [x] MA_Mode
      - [x] Robot Light
      - [x] Ctrl_DO0~DO7
      - [x] Ctrl_DI0~DI7
      - [x] Ctrl_AO0
      - [x] Ctrl_AI0~AI1
      - [x] END_DO0~DO3
      - [x] END_DI0~DI2
      - [x] END_AI0

   When you need to check more about the maximum, minimum, and average calculation properties of joint torque,
   the _three checked items_[^4] listed below can be checked individually or all of them,
   please leave them unchecked when not in use.

      - [ ] Joint_Torque_Average
      - [ ] Joint_Torque_Min
      - [ ] Joint_Torque_Max

4. Enable the __Ethernet Slave__ settings: mouse-click to enable or disable TM Ethernet Slave.
   Once enabled,
   the robot establishes a Socket server
   to send the robot status and data to the connected clients and permissions to access specific robot data.

   Mouse-click to enable the `Ethernet Slave` setting and let `STATUS:` &rArr; __`Enable`__.
   ![2](figures/3.png)

5. Press the Play/Pause Button on the Robot Stick to start running this _Listen task_ project.

[^1]: See [TM ROS Driver vs TMflow Software Usage: Import Data Table Setting](https://github.com/TechmanRobotInc/TM_Export).
[^2]: <u>Turn off</u> Ethernet Slave.
Let "STATUS:  __Disable__" be displayed on the Ethernet Slave setting page,
then click `Data Table Setting` to enter the next page for related settings.
[^3]: The checked items listed above must <u>all</u> be selected for TM ROS setting.
[^4]: This function requires <u>TMflow 1.84 or later</u> versions to support.

#### Remote connection to TM ROBOT

Static IP of remote connection network settings through the wired network.

1. Set the wired network of the user's (remote) Ubuntu computer by mouse-click on the top right of the desktop &rArr;
   Click on "__Wired Settings__" &rArr; Click on the gear icon &rArr;
   In the IPv4 feature options, click on "Manual" in order.

   ![user_remote_network_settings](figures/user_remote_network_settings.png)
2. Set the Static IP settings: where the IP address is fixed for the first three yards same as the previous setting
   `192.168.10`, last yards 3–254 machine numbers are available.
   (Because _TM ROBOT_, you have been set to `192.168.10.2`)

   Example: Set the Netmask: 255.255.255.0 and IP address 192.168.10.30

   ![user_remote_IP_example](figures/user_remote_IP_example.png)
3. Check Internet connection: start a terminal to test the connectivity with the target host _TM ROBOT_,
   by typing `ping 192.168.10.2`
   ![ping_target_host](figures/ping_target_host.png)

>:bulb: __Tip__: Remember to reconfigure the network settings due to <u>static IP changes</u> or <u>replacement of the
ROS control PC</u>.

As mentioned above, a valuable debugging tool is your operating system's <u>ping</u> command.
If nothing appears to happen or an error is thrown, the robot cannot be accessed from your computer.
Please go back to the top of this chapter and re-operate in the order of instructions.

If you are an experienced user,
you may just need to <u>turn off</u> &rArr; <u>turn on</u> the gear icon of "__Wired Settings__" on your computer or to
<u>turn off</u> &rArr; <u>turn on</u> the "__Ethernet Slave Data Table__" setting of the robot to reconfigure the
hardware settings.

#### TM ROS driver usage

##### ROS2 driver usage

After the user has set up the ROS2 environment
(example: [Debian packages for ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html))
and built the TM driver based on the specific workspace,
please enter your workspace `<workspace` by launching the terminal,
and remember to make the workspace visible to ROS.

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd <workspace>
source ./install/setup.bash
```

:bulb: How do you prepare the __TM Robot__ to be ready?
Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.

Then, run the driver to maintain the connection with TM Robot by typing

```bash
ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
```

Example: `ros2 run tm_driver tm_driver robot_ip:=192.168.10.2`, if the `<robot_ip_address>` is `192.168.10.2`

Now,
the user can use a new terminal
to run each ROS node or command but don't forget to source the correct setup shell files as starting a new terminal.

__Usage with MoveIt2 (Tentative)

See [MoveIt2 tutorial](https://moveit.ros.org/install-moveit2/source/) to install the MoveIt2 packages.

Assuming that the user is ready to build MoveIt2,
and the user wants to apply the MoveIt by TM Robot, please don't forget to source the MoveIt environment,
or you can add  `source <MoveIt_WS/install/setup.bash` to your `.bashrc`.

The `<MoveIt_WS` means the MoveIt2 workspace, for example `COLCON_WS`.

The `<TMDriver_WS` means TM driver workspace, for example `tmdriver_ws`.

Then, to build the TM driver based on the <TMDriver_WS> workspace,
please enter the specific workspace `tmdriver_ws` by launching the terminal,
and remember to make the workspace visible to ROS.

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/COLCON_WS/install/setup.bash
cd ~/tmdriver_ws
colcon build
source ./install/setup.bash
```

>:bulb: If you have built the TM driver before,
you must use `colcon build --cmake-clean-cache` or `colcon build --cmake-force-configure` instead of `colcon build` in
the previous step to force execution CMake configuration step, for example

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/COLCON_WS/install/setup.bash
cd ~/tmdriver_ws
colcon build --cmake-clean-cache
source ./install/setup.bash
```

The demo launches the RViz GUI
and demonstrates planning and execution of a simple collision-free motion plan with TM Robot.

>:bulb: Do you prepare the __TM Robot__ ready?
Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.

To bring up the MoveIt2 demo environment in simulation mode with virtual TM Robot (Example: TM5-900), by typing

```bash
ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py
```

>:bookmark_tabs: Note1:
There are several built-in TM Robot nominal robot model settings, available for tm5-900, tm5-700,
tm12, and tm14 models, as well as the eyeless models tm5x-900, tm5x-700, tm12x and tm14x models.

The user can also manipulate the real TM5-900 Robot (Example: TM5-900) to run by typing

>:warning:[CAUTION] This demo will let the real TM Robot move, please be careful.
If the user is a beginner or unfamiliar with the arm movement path,
it is recommended that the user place his hand on the big red emergency _Stick Stop Button_ at any time,
and press the button appropriately in the event of any accident that may occur.

```bash
ros2 launch tm_moveit_cpp_demo tm5-900_run_moveit_cpp.launch.py robot_ip:=<robot_ip_address
```

The parameter `<robot_ip_address>` means the IP address of the TM Robot.

>:bookmark_tabs: Note2:
If your real Robot is a TM12, in the above example,
you should type `tm12_run_moveit_cpp.launch.py` to instead of `tm5-900_run_moveit_cpp.launch.py`.
>:bookmark_tabs: Note3: If your real Robot is the eyeless model as a TM12x, in the above example,
you should type `tm12x_run_moveit_cpp.launch.py` to instead of `tm5-900_run_moveit_cpp.launch.py`.

## 4. Vision

### TM ROS Vision usage

This chapter describes that the user can get image data through TMvision&trade; of TM Robot. __(Built-in Vision System)__

#### Dependencies

- Python packages:
   1. flask
   2. waitress
   3. opencv-python==3.4.13.47 (Minimum)
   4. numpy
   5. datetime

For example, install Python3 packages:

```bash
  pip3 install flask
  pip3 install waitress
  pip3 install opencv-python
  pip3 install datetime
```

#### Techman Robot Vision

- type: sensor_msgs::msg::Image
  - message name: techman_image

#### Build TM ROS Vision driver node on your (remote) computer

Under the environment settings have been finished with your workspace `<workspace>`, then type

```bash
cd ~/<workspace> && source ./install/setup.bash
ros2 run tm_get_status image_talker
```

> :bulb: The user can check whether the connection succeeds or not.
> When you proceed to the following steps introduced in the following text: steps 6 of § TMflow Vision node setup.

#### TMflow Vision node setup

The __Vision node__ provides the creation of a plane with fixed-point type,
servo type, and object type as well as a variety of AOI identification functions.

:bulb: Before going through the following steps,
please build the TM ROS Vision driver node on your (remote)
computer and then connect this (remote) computer to the local TM Robot computer.

1. Create a _Vision task_ project of __TMflow__ software,
   and then drag the __Vision node__ from the _nodes menu_ onto the project flow, as shown below.

   ![create_a_vision_task](figures/create_a_vision_task.png)
2. Click the __AOI -only__ icon, and then follow the steps below to handle some settings related to accessing TM Robot HMI.

   ![choose_aoi_only](figures/choose_aoi_only.png)

   TMflow 1.76 second version only:

   If no suitable dongle is detected, warning alerts will be displayed in the window.

   ![open_need_dongle_key](figures/open_need_dongle_key.png)
   TMflow 1.80 version:

   The user doesn't need a dongle to activate this function.

3. Click the __Find__ icon.
   ![select_find](figures/select_find.png)

4. In TMflow 1.76 second version, click the __AI_Detection__ icon.

   ![choose_ai_detection_only](figures/choose_ai_detection_only.png)
   In TMflow 1.80 version, click the __External Detection__ icon.
   ![change1](figures/change1.png)

5. In TMflow 1.76 second version, click the __+ Add Parameters__ button.
   ![choose_add_parameters](figures/choose_add_parameters.png)
   In TMflow 1.80 version, click the __Setting__ button.
   ![change2](figures/change2.png)

6. To check whether the connection succeeds or not, please enter ```<user_pc_ip_address>:6189/api``` in the
   __HTTP Parameters__ blank text and click the __Send__ button to get the information of the (remote) computer for ROS.

   The `<user_pc_ip_address>` means the IP address of the user's (remote) ROS computer, for example, 192.168.2.12

   ![check_connect_success](figures/check_connect_success.png)

   If the connection fails, a __TIMEOUT__ error will be displayed in the window
   ![wrong_ip_address](figures/wrong_ip_address.png)

   If the IP address of the user's (remote) ROS computer doesn't exist, __ERROR_CODE_7__ will be displayed in the window.
   ![wrong_port](figures/wrong_port.png)
7. Enter ```<user_pc_ip_address>:6189/api/DET``` in the URL blank text and type arbitrary letters in the __Value__ blank
   text; the __Key__ will be generated automatically.
   ![add_model](figures/add_model.png)
8. Assign a name to the model in the __Model name__ blank text and click the __Save__ button.
   ![save_model](figures/save_model.png)

9. Press the Play/Pause Button on the Robot Stick to start running this _Vision task_ project.

   Note: TMflow software version changes may have slightly different settings.
   ([SW1.76_Rev2.00](https://www.tm-robot.com/zh-hant/wpdmdownload/software-manual-tmflow_sw1-76_rev2-00/))
   ([SW1.80_Rev2.00](https://www.tm-robot.com/zh-hant/wpdmdownload/software-manual-tmflow_sw1-80_rev2-00-2/))

#### Receive image data on the user's computer from TMflow Vision node

:bulb: How do you prepare the TM Robot?
Make sure that TM Robot's operating software (TMflow)
relative __HTTP Parameters__ Vision settings are ready and the _Vision task_ project is running.

Now, in a new terminal of your (remote) ROS computer: Source `setup.bash` in the workspace path and run to get image
data from TMvision&trade; by typing:

```bash
source ./install/setup.bash
ros2 run tm_custom_package sub_img
```

Then, the viewer will display image data from _TMflow_.

## 5. Program script demonstration

### Demo package description

This chapter describes the _demo_ package and the code used as a C++ programming example,
showing how to program robot scripts (TM Robot Expressions) through the TM ROS driver connection.

- demo_send_script:

In this demo code, it shows how to send a __Listen node__ script to control the TM Robot.

The user can use a service named "send_script" to send the script.

"id" &rarr; The transaction number expressed in any <u>alphanumeric</u>[^5] characters.

"script" &rarr; the script that the user wants to send.

"ok" &rarr; the correctness of the script.

[^5]: If a non-alphanumeric byte is encountered, a CPERR 04 error is reported. When used as a communication packet response, it is a transaction number and identifies which group of commands to respond.

- demo_ask_item:

In this demo code, the user can use this service to send TMSVR[^6] cmd.

[^6]: For more detailed information, please refer to _defined protocol_: Expression Editor and Listen Node.pdf (Chapter 9.6 TMSVR)

- demo_ask_sta:

In this demo code, the user can use this service to send TMSTA[^7] cmd.

[^7]: For more detailed information, please refer to _defined protocol_ (Chapter7.5 TMSTA)

- demo_connect_tm:

In this demo code, the user can set the connection type.

If the user sets reconnect to true, every time the driver disconnects from the __Listen node__, it will try to reconnect.

There are two kinds of connection settings the user can select,
one is `connect_tmsvr` for Ethernet server connection, and the other is `connect_tmsct` for TMflow connection.

- demo_set_event:

In this demo code, six event types can be selected.

func &rarr; TAG, WAIT_TAG, STOP, PAUSE, RESUME and EXIT

arg0 &rarr; if func is TAG or WAIT_TAG, arg0 is the tag number

arg1 &rarr;  if func is TAG or WAIT_TAG, arg1 is timeout in ms

- demo_set_io:

In this demo code, the user should set the module, type, pin, and state.[^8]

[^8]: For more detailed information, please refer to _defined protocol_ (Chapter6.5 IO)

module &rarr;  MODULE_CONTROLBOX or MODULE_ENDEFFECTOR

type &rarr; TYPE_DIGITAL_IN, TYPE_DIGITAL_OUT, TYPE_INSTANT_DO, TYPE_ANALOG_IN, TYPE_ANALOG_OUT, TYPE_INSTANT_AO

pin &rarr;  pin number

state &rarr; STATE_OFF or STATE_ON value, or other value (if type expressed in a specific control module)

- demo_set_positions:

In this demo code,
the user should pay attention to the parameter definition of the data format setting[^9] and the unit of the parameter
to be operated.

[^9]: For more detailed information, please refer to _defined protocol_ (Chapter8 PTP, Line, Circle, Pline, Move_PTP,
Move_Line, Move_PLine)

motion_type &rarr;  PTP_J , PTP_T , LINE_J , LINE_T , CIRC_J ,CIRC_T , PLINE_J ,PLINE_T

positions &rarr;  motion target position: If expressed in Cartesian coordinate (unit: m),
if expressed in joint angles (unit: rad)

velocity &rarr; motion velocity: if expressed in Cartesian coordinate (unit: m/s)[^10],
if expressed in joint velocity (unit: rad/s, and the maximum value is limited to &pi;)[^10]

[^10]: The unit of the parameters is different, the user can find the conversion in the program of TM ROS driver.

acc_time &rarr; time to reach maximum speed (unit: ms)

blend_percentage &rarr; blending value: expressed as a percentage (unit: %, and the minimum value of 0 means no blending)

fine_goal &rarr; precise position mode: If activated, the amount of error in the final position will converge more,
but it will take a few more milliseconds.

- demo_write_item:

In this demo code, the user can use this service to send TMSVR[^11] cmd.

[^11]: For more detailed information, please refer to _defined protocol_ (Chapter9.3 svr_write())

- demo_leave_listen_node:

In this demo code, the user can use send_script service sending a script to leave the __Listen node__.

:bulb: If the user has sent the demo_leave_listen_node script to leave the __Listen node__,
and you want to run the TM Robot again, please remember that the _Listen task_ project should be resumed to run.
You can press the Stop Button on the Robot Stick and then press the Play/Pause Button to resume operation.

#### __Usage with demo code & driver

Note: If the user has even successfully built a specific code(tmr_ros2),
the user only needs to change to the TM driver workspace path  `cd ~/tmdriver_ws`,
and then directly refer to steps 5~6 below.

1. Type to create a root workspace directory by starting a terminal: For example,  `tmdriver_ws` or `catkin_ws`,
   then type to change the current directory into the workspace directory path.

   ```bash
   mkdir ~/tmdriver_ws
   cd ~/tmdriver_ws
   ```

2. Clone the TM driver of the git repository into the current directory by typing

   ```bash
   git clone https://github.com/MatthijsBurgh/tmr_ros2.git
   ```

3. After the download is done, rename the download folder `tmr_ros2` (or `tmr_ros2-master`) to `src` by typing

   ```bash
   mv tmr_ros2 src
   ```

4. At the workspace directory to build the download packages and source 'setup.bash' in this workspace
   to make the workspace visible to ROS.

   __Note__: Did you set `source /opt/ros/${ROS_DISTRO}/setup.bash` already?
  Make sure to obtain the correct setup file according to your workspace hierarchy,
  and then type the following below to compile.

   ```bash
   colcon build
   source ./install/setup.bash
   ```

5. In a new terminal: Source setup.bash in the workspace path and run the driver to connect to TM Robot by typing

   ```bash
   source ./install/setup.bash
   ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
   ```

   The `<robot_ip_address>` is the IP address of the TM Robot,
   the user can get it through TM Flow, for example, `192.168.10.2`.

6. In another new terminal: Source setup.bash in the workspace path and run the specific demo node function,
   which the user wants to study for applications.
   For example, the user select to run `demo_set_io`, the user can type

   ```bash
   source ./install/setup.bash
   ros2 run demo demo_set_io
   ```

   :warning:[CAUTION] Some demos will let the TM Robot move, please be careful.

## 6. TM GUI debugging and demonstration

This chapter describes a simplified GUI for displaying tm_driver connection status,
sct, sta, svr messages, and robot status.
The user can optionally install the _tm_ui_for_debug_and_demo_ package
to aid in viewing messages between the driver and the robot through the GUI display.
If the driver connection fails, the user can also try to send a reconnect command on this GUI for debugging.

### GUI Debugging description

- If the user forgets to run the TM ROS driver, the user will see all the controlled label items of the GUI displayed as
  `NaN`.

- The user can click the`Quit_GUI` button or click the `x` close button in the upper right corner to close this GUI.

- If `Ethernet` and `Listen Node` connection displays are `on`,
  it means that ROS SvrClient and SctClient are successfully connected.

- If the`Ethernet` connection display is `off`, the user should check whether the TM Robot has been started or whether
  the network settings are correct.

- If the`Listen Node` connection is `off`, the user should check whether the task project is running.

:bulb: If `Listen Node` connection is interrupted as `Project_Run` is stopped, the `Listen Node` connection will be
`off`.

- If both `Ethernet` and `Listen Node` connection displays are `on`,
  but the `Robot_Link` is false or `Robot_Error` is true; this means the robot is working abnormally,
  or maybe the ESTOP button was pressed or some kind of protection or error[^12] occurred.
  Therefore, when the user sends a move script command at this time, it will not work.

    [^12]: For more detailed information, please refer to the TM Robot User Guide.

- The user can use the self-developed script to read/write project data through communication protocols to control the
  TM Robot.
  If it does not work properly,
  the user can quickly determine
  whether there is a communication error code by viewing the `Response ROS Node Status` display.

- When the user sends a command or clicks DO0 Ctrl `H/L` button of Control_Box,
  the user also can see the response message[^13] embedded in the `Robot Response` item view.

    [^13]: For details of this item, please refer to __SctResponse.msg__, __StaResponse.msg__ and __SvrResponse.msg__ of TM ROS driver code.

- The user can click `clear` button to clear the old response message.

:bulb: If the`Ethernet` connection is interrupted,
the display of most controlled label items in the GUI will be displayed as "NaN"
and the robot feedback state will remain the last state and become invalid.

#### Usage with GUI debugging

Note:
If the user has even successfully built a specific code(tmr_ros2),
the user only needs to change to the TM driver workspace path  ```cd ~/tmdriver_ws```,
and then directly refer to steps 5~6 below.

1. Type to create a root workspace directory by starting a terminal: For example,  `tmdriver_ws` or `catkin_ws`,
   then type to change the current directory into the workspace directory path.

   ```bash
   mkdir ~/tmdriver_ws
   cd ~/tmdriver_ws
   ```

2. Clone the TM driver of the git repository into the current directory by typing

   ```bash
   git clone https://github.com/MatthijsBurgh/tmr_ros2.git
   ```

3. After the download done, rename the download folder `tmr_ros2` (or `tmr_ros2-master`) to `src` by typing

   ```bash
   mv tmr_ros2 src
   ```

4. At the workspace directory to build the download packages and source 'setup.bash' in this workspace to make the
   workspace visible to ROS.

   __Note__: Did you set `source /opt/ros/${ROS_DISTRO}/setup.bash` already?
   Make sure to obtain the correct setup file according to your workspace hierarchy,
   and then type the following below to compile.

   ```bash
   colcon build
   source ./install/setup.bash
   ```

5. In a new terminal: Source setup.bash in the workspace path and run the driver to connect to TM Robot by typing

   ```bash
   source ./install/setup.bash
   ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
   ```

   The <robot_ip_address> is the IP address of the TM Robot, the user can get it through TM Flow, for example, 192.168.10.2

6. In another new terminal: Source setup.bash in the workspace path and start GUI debug by typing

   ```bash
   source ./install/setup.bash
   ros2 run tm_ui_for_debug_and_demo robot_ui
   ```

## 7. TM Robot corrected kinematics value loading and robot description file generation

Real kinematic values vary from TM robot to another one as each robot is calibrated at the factory.

The user can use the `tm_mod_urdf` package to extract specific kinematic values from your TM robot,
which are taken into account by a Python script using a specific set of commands to automatically generate a new URDF
or Xacro robot model description file. If the user just wants to use the TM Robot nominal model to control the robot,
the user can skip the rest of this chapter.

### Corrected kinematics value description

The precise kinematic parameters of a robot are useful for improving the end-point accuracy of the robot.

Due to manufacturing tolerances during manufacturing and the installation error in the robot assembly process,
the positioning accuracy and precision of the mechanism will be affected.
The error between the reality and the nominal robot model is significantly reduced by the corrected robot description.
The kinematic parameter compensated deviations of the robot can improve the absolute positioning accuracy of the robot.

If the user needs to improve simulation accuracy or end effector tracking performance,
it is recommended that the user import the corrected calibrated kinematic parameters from the real TM Robot
to replace the nominal set of D-H parameters.
Techman Robot provides a URDF file that configures the TM Robot model with a set of nominal DH parameters,
and one that uses the programming scripts to obtain calibrated kinematic parameters from a parameter server,
which is connected to your TM robot and perform a set of overrides to output a new corrected URDF file.

The common Python script is used as follows:

 ```bash
 python3 <script_name> <urdf_from> <urdf_gen>
 ```

- `<script_name>`: Provide `modify_xacro.py` or `modify_urdf.py` two Python scripts program as options.
- `<urdf_from>`: The first argument represents the original URDF model form of the TM Robot,
  and the file part naming[^14] is `<urdf_from>`.

[^14]: There are several built-in TM Robot nominal robot model settings, available for tm5-900, tm5-700, tm12, and tm14 models, as well as the eyeless models tm5x-900, tm5x-700, tm12x and tm14x models.

 For example, select the tm12 nominal robot model as the input model form, the user can type tm12 as the `<urdf_from>`.
 For details of this item, please refer to the `modify_urdf.py` or `modify_xacro.py` code.

- `<urdf_gen>`: The second argument means the newly generated URDF model form of the TM Robot,
  and the file[^15] name is `<urdf_gen>`.

 [^15]: For example, if the user names it `test` and selects `modify_xacro.py` as script program, a `test.urdf.xacro` robot description file will be generated.

 The Python script for more specific arguments is used as follows:

 ```bash
 python3 <script_name> <urdf_from> <urdf_gen> <specific_param>
 ```

- <specific_param>: The third argument is provided for use in some special cases.
  Please refer to the scripting program[^16] for details of this item.

 [^16]: For a simple third argument example, type the argument `-M` as follows:
        Example: ```python3 modify_xacro.py tm5-900 test -M```

 &rarr; A robot description file "`macro.test.urdf.xacro`" will be generated, and the string 'macro.'
 is prepended to the `<urdf_gen>` name.

#### Create with specific kinematic parameters of the local TM Robot

:bulb: Do you run the driver to maintain the connection with TM Robot,
make sure that TM Robot's operating software (TMflow) network settings are ready and the Listen node is running.

- #### Take generating a new Xacro file as an example

The following steps describe
how to import specific kinematic values using a real TM5-900 Robot following the procedure below
and select the corresponding type tm5-900 as an example of `<urdf_from>`.

1. In a terminal: Source setup.bash in the workspace path and run the driver to connect to TM Robot by typing

   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   cd <workspace
   source ./install/setup.bash
   ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
   ```

   The parameter `<robot_ip_address>` means the IP address of your TM Robot, the user can get it through TM Flow.

2. In another new terminal: source setup.bash in the workspace path,
   change the current directory to the directory path of the python script to correct URDF,
   and then enter the specified command format to generate a new named URDF with arguments,
   for example, named `user_defined`.

   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   cd <workspace
   source ./install/setup.bash
   cd src/tm_mod_urdf/tm_mod_urdf
   python3 modify_xacro.py tm5-900 user_defined
   ```

   When this procedure is completed,
   the user can find that the newly generated named robot description file has been saved,
   e.g. `user_defined.urdf.xacro`.

3. Next, the user must modify the filename part of the default pre-built nominal robot model in `tm5-900.urdf.xacro` to
   a newly generated robot model description naming the file.

   ```bash
   cd src\tm_description\xacro\
   vim tm5-900.urdf.xacro
   ```

   or use `gedit` text editor instead of `vim` to edit the file contents, by typing

   ```bash
   gedit tm5-900.urdf.xacro
   ```

   :bookmark_tabs: Note1:
   If your real Robot is a TM5-700, in the above example,
   you should type `tm5-700` as an example for `<urdf_from>` and modify the `tm5-700.urdf.xacro` file.

   :bookmark_tabs: Note2: If your real Robot is the eyeless model as a TM5X-700,
   in the above example,
   you should type `tm5x-700` as an example for `<urdf_from>` and modify the `tm5x-700.urdf.xacro` file.

   Please refer to the following to modify the content format of the filename line:

   ```xml
   <?xml version="1.0"?>
   <robot xmlns:xacro="https://www.ros.org/wiki/xacro" name="YOUR_ROBOT_NAME">
     <!-- Before modification: (Take the pre-built TM5-900 nominal robot model as an example) -->
     <xacro:include filename="$(find tm_description)/xacro/macro.tm5-900-nominal.urdf.xacro" />
     <!-- After modification: (Replace with your actual newly generated Xacro file) -->
     <xacro:include filename="$(find tm_description)/xacro/user_defined.urdf.xacro" />
   </robot>
   ```

Finally,
the user can launch the modified robot file `tm5-900.urdf.xacro`
to run your TM Robot or simulate the robot more accurately.

:bulb: __Tip__: Remember to recompile since the code has been changed.

Please go back to your specific workspace.
Then you can choose `colcon build --cmake-clean-cache` to rebuild, or you can clean the build,
install and log directories with `rm -r build install log` before executing `colcon build`.

- #### Take generating a new URDF file as an example

The following steps describe
how to import specific kinematic values using a real TM5-900 Robot following the procedure below
and select the corresponding type tm5-900 as an example of `<urdf_from>`.

1. In a terminal: Source setup.bash in the workspace path and run the driver to connect to TM Robot by typing

   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   cd <workspace>
   source ./install/setup.bash
   ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address
   ```

   The parameter `<robot_ip_address` means the IP address of your TM Robot, the user can get it through TM Flow.

2. In another new terminal: source setup.bash in the workspace path,
   change the current directory to the directory path of the python script to correct URDF,
   and then enter the specified command format to generate a new named URDF with arguments,
   for example, named `user_defined`.

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd <workspace>
source ./install/setup.bash
cd src/tm_mod_urdf/tm_mod_urdf
python3 modify_urdf.py tm5-900 user_defined
```

When this procedure is completed,
the user can find that the newly generated named robot description file has been saved, e.g. `user_defined.urdf`.

:bookmark_tabs: Note1: If your real Robot is a TM12, in the above example, you should type tm12 as an example for `<urdf_from>`.

:bookmark_tabs: Note2: If your real Robot is the eyeless model as a TM12X,
in the above example, you should type tm12x as an example for `<urdf_from>`.

Finally, the user can use the new robot file, such as `user_defined.urdf`,
instead of the default nominal URDF model to run your TM Robot or simulate the robot more accurately.

:bulb: __Tip__: Remember to recompile since the code has been changed.

Please go back to your specific workspace. Then you can choose `colcon build --cmake-clean-cache` to rebuild,
or you can clean the build,
install and log directories with `rm -r build install log` before executing `colcon build`.

#### Import information available on the screen

- How can the user confirm that the data conversion process has been completed?

Ans: The user can find the string `File saved with new kinematic values.` displayed on the screen.

- How can the user find the location of the newly generated named robot description file?

Ans: The user can first find the displayed string `[new save file path:]` on the screen,
and the following string is the file save location.

## 8. Contact us/Technical support

More Support & Service, please contact us.
[@TECHMAN ROBOT](https://www.tm-robot.com/en/contact-us/)
