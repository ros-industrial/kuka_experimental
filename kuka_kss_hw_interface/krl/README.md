# Configuring KSS_HW_Interface on the Kuka controller

This guide highlights the steps needed in order to successfully configure the **KSS interface** on the controller to work with the **kuka_kss_hardware_interface** on your PC with ROS.
Note that the robot code distribution is divided by the major *Kuka System Software* versions: *KSS85*, *KSS86*, etc.  The process is mostly the same for these, but watch for minor variations pertaining to your specific release.

**Kuka system requirements:**
 - *Kuka System Software* V8.5 or later.
 - *Ethernet KRL (EKI)* option.
 - *Robot Sensor Interface (RSI)* option.

## 1. Controller network configuration

The ROS interface network will be physically connected to the robot Kuka Line Interface (KLI).  On KRC4 controllers this is typically labelled as either X66 or X67.1.
This  setup reflects a summary of steps outlined in the RSI manual under the section "Configuring Ethernet via KLI".  The ROS interface will connect through a second virtual network interface as RSI typically does.

The addresses in this instruction set are setup as follows:
**Kuka Robot KLI (virtual 5):** 172.31.1.147
**Kuka Robot RSI Net (virtual 6):** 172.32.1.10
**ROS client PC:**  172.32.1.20
**Work Visual Configuration PC:** 172.31.1.20
Note the different subnets 172.31.x.x and 172.32.x.x

### 1.1 Setup ROS Virtual Network Adapter
 1. Log in as **Expert** or **Administrator** on the teach pad.
 2. In the main menu, select **Start-up > Network configuration**. The Network configuration window opens
 3. Press the **Advanced...**. button.  The window for advanced network configuration opens.
 4. Press **Add interface**  button.  A new entry is created automatically in the **Configured interfaces:** area.
 5. Select the newly created entry and enter the network name (e.g. **ROS Interface**) in the **Interface designation:** box.
 6. Select the **Mixed IP address** type in the **Address type:** box.  This will automatically create some Receiving Task entries.
 7. In the boxes underneath, enter the IP address of the robot controller and the subnet mask.  For example:
    * **IP**: 172.32.1.10
    * **Subnet mask**: 255.255.255.0
 8. Press the **Save** button.

Note that the default first network interface (typically called **virtual5 KLI**) is assigned to handle Windows communication, as indicated by the check-box on the lower right.  This is the network address to which the Kuka WorkVisual tool connects.  Make sure that the **virtual5 KLI** and the **ROS Interface** are on different sub-nets.  Make sure that your **ROS client PC** is connected to the *same* subnet as the **ROS Interface**.

 9. Reboot the controller with reload files.  Select menu **Shutdown**.  In the dialog,  check *Force cold start* and *Reload files*.  Touch the  **Reboot Control PC** button.
10. After the robot reboot completes, use a ping command on your **ROS client PC** to verify communication with the new **ROS Interface** address.   For example:
    ping 172.32.1.10



## 2. Setup Robot Files

Select the distribution folder corresponding to the KSS version on the target robot controller.  The KSS version can be displayed on the robot by selecting the menu **Help > Info**.
On the robot controller, a user login level of Expert or Administrator is required.
Files can be transferred to the robot with 2 different methods:
A) Use a USB stick to copy the files onto the controller.
B) Use the *Kuka Work Visual* configuration software.  The online file editor is particularly convenient.

### 2.1 Install Options on the Robot Controller
Use *Kuka Work Visual* software to install Ethernet KRL and Robot Sensor Interface.

### 2.2 Setup Robot Sensor Interface (RSI) Files
File(s) are located in the KSS_xx/SensorInterface distribution folder.
##### ros_rsi_ext_ethernet.xml
1. Edit the `<IP_NUMBER>172.32.1.243</IP_NUMBER>` tag so that it corresponds to the IP address of the **ROS client PC** (172.32.1.20).  For example `<IP_NUMBER>172.32.1.20</IP_NUMBER>`
2. It is recommended to keep the `PORT` tag as it is (49152).  This can be changed but care must be taken to use an acceptable port per the Kuka RSI documentation and also change the configuration yaml file for the ROS node.
3. Copy the file to the RSI config folder on the robot controller: C:\KRC\Roboter\Config\User\Common\SensorInterface.  
This folder should already exist if Robot Sensor Interface is properly installed on the robot.

Note that the `rsi/listen_address` and `rsi/listen_port` parameters of the `kuka_kss_hw_interface` must correspond to the `IP_NUMBER`and `PORT` set in this config file.

##### ros_rsi_ext.rsix
This is the RSI program for real-time control of the robot.  The RSI software uses a graphical block-based programming language.  After the RSI option package is installed in Kuka Work Visual, the graphical editor is available as a tool and should always be used to modify an RSI program.
It should not be necessary to edit the RSI program.  However, there are some parameters that a user might desire to modify.
1. Modify parameters if necessary:
**Ethernet.Timeout** - The RSI Ethernet communication is preconfigured to tolerate up to 100 late packets before error failure.  
**IIRFilter.Cutoff** - Each axis has a 50Hz filter to smooth motion.  The cutoff frequency may be increased to provide faster response or decreased to provide better smoothing.
**AxisCorr Limits** - Limits the adjustment from the starting position.  If this limit is exceeded, further adjustments are ignored (saturation behavior).
**AxisCorrMon Limits** - Limits total adjustment range for each axis from the starting position.  If this limit is exceeded, the program stops with an error.
**Monitor blocks** - The monitor blocks enable debug graphing with the RSI Monitor tool on the robot pendant.  Use menu **Display > RSI Monitor**.  The monitored signals can be identified or changed in the RSI program.
2. Copy the file to the RSI config folder on the robot controller: C:\KRC\Roboter\Config\User\Common\SensorInterface.  
This folder should already exist if Robot Sensor Interface is properly installed on the robot.

### 2.3 Setup Ethernet KRL (EKI) Files
File(s) are located in the KSS_xx/EthernetKRL distribution folder.
##### EkiKSSinterface.xml
This file defines the ROS communication server that will run on the robot.  
1. Edit the `<IP>172.31.1.148</IP>` tag so that it corresponds to the IP address assigned to the **ROS Interface** created above. For example, `<IP>172.32.1.10</IP>`
2. It is recommended to keep the `PORT` tag as it is (54600).  This can be changed but care must be taken to use an acceptable port per the Kuka EKI documentation and also change the configuration file for the ROS node.
3. Copy the file to the EKI config folder on the robot controller: C:\KRC\Roboter\Config\User\Common\EthernetKRL.  
This folder should already exist if Ethernet KRL is properly installed on the robot.  
If using Work Visual online file editing, this folder does not appear.  A common hack is to place the file into the SensorInterface folder and move it after transfer to the robot.

Note that the `kuka/ip_address` and `kuka/server_port` parameters of the `kuka_kss_hw_interface` must correspond to the `IP` and `PORT` set in this EKI config file.

### 2.3 Prepare the Robot for External Control
1. Edit the file STEU/MADA/$Option.DAT.
2. Find the line for `$CHK_MOVENA` and set the value to FALSE. `$CHK_MOVENA=FALSE`
3. Make sure no program is selected for running.  If the R in the top status bar is not gray, touch it and press **Cancel Program** on the drop-down menu.
4. Use the menu **Configuration>Inputs/Outputs>Automatic External**.  Set the input Value column:
`$EXT_START = 1026`
`$MOVE_ENABLE = 1025`
`$CONF_MESS = 1026`
`$DRIVES_OFF = 1025`
5. Touch the X in the left border to save the settings.

### 2.4 Setup the KRL Program Files and Server
File(s) are located in the KSS_xx/Program distribution folder.
1. Copy the folders `ROS` and `KukaUtils` to `KRC:\R1\Program`.
2. Install the server code.  Edit the default SPS.sub program on the robot, located in the R1/System folder.  Place the routine `ROS_SPS_Init()` before the LOOP and place the routine `ROS_SPS_loopCall()` inside the LOOP, as shown below:

>       ;FOLD USER INIT
>       ; Please insert user defined initialization commands
>       ;ENDFOLD (USER INIT)
>       ;ENDFOLD (INI)
>       ROS_SPS_Init()
>       LOOP
>         WAIT FOR NOT($POWER_FAIL)
>         ;FOLD BACKUPMANAGER PLC
>         IF BM_ENABLED THEN
>           BM_OUTPUTSIGNAL = BM_OUTPUTVALUE
>         ENDIF
>         ;ENDFOLD (BACKUPMANAGER PLC)
>         ;FOLD USER PLC
>         ;Make your modifications here
>         ;ENDFOLD (USER PLC)
>         ROS_SPS_loopCall()
>       ENDLOOP

3. After closing the SPS.sub file, editing is complete.  
4. You should see that the `kuka_ros_run` program is automatically selected to run.  
5. Touch the `S` in the top status bar.  You should see that `Kuka_ROS_CONN` is displayed in the SYS process and `ProgCtrlExec` is displayed in the EX3 process.



### 2.5 Consider the Real-Time Patch for Linux
The RSI interface operates at `250 Hz` (4ms cycle). The **kuka_kss_hardware_interface** does not have a period configured and is completely driven by the controller's output. Every controller RSI output has a IPOC timestamp (in millisec) which increments for every cycle. The **kuka_kss_hardware_interface** will answer as quickly as possible. The answer includes the last IPOC received. If the connected **ROS client PC** did not manage to answer within the RSI cycle of **4ms**, the IPOC timestamp of RSI has incremented. The IPOC included in the answer is not matched and the controller will increment a counter. When this counter hits the **Timeout** parameter (**100**), the RSI connection will shut down. If this parameter is lowered, the demand for real-time computation on the **ROS client PC** will increase.
If the **ROS client PC** does not have the real-time patch: 
 - you may have problems with the connection to RSI shutting down now and then.
 - you may hear servo "clunks" during motion.
 - you may have servo over current faults during motion that stop program execution.
Other debugging info and options:
 - monitor the published topic: rsi_delay, which contains the current late packet count.
 - compile the ROS node with the TEST_COMM_TIMING preprocessor option.  This enables publishing of additional timing topics.

 If these indicators occur you should compile and install a [RT-Preempt](https://rt.wiki.kernel.org/index.php/RT_PREEMPT_HOWTO) kernel for your PC.
The **kuka_kss_hardware_interface** node will automatically recognize the availability of the real-time patch and assign the node a real-time priority.


## 3. Configure the kuka_kss_hw_interface
The **kuka_kss_hw_interface** needs to be configured in order to successfully communicate with the robot controller. Inside `/kuka_kss_hw_interface/test` and `/kuka_kss_hw_interface/config` in this repository is a set of `*.yaml` files. These configuration files may be loaded into a launch-file used to start the **kuka_kss_hw_interface** with correct parameters, such as:

* **controller_joint_names** maps the joint value order in the robot to the joint names in the robot description (URDF or .xacro).  See file `config/controller_joint_names.yaml`
* **kuka/ip_address** and **kuka/server_port** specify the address and port of the EKI server on the robot, corresponding to the settings specified in **EkiKSSinterface.xml**  See file `test/test_params.yaml`
* **rsi/listen_address** and **rsi/listen_port** specify the address and port on the **ROS client PC**,  corresponding to the RSI setup specified in ** ros_kss_ext_ethernet.xml**.  See file `test/test_params.yaml`

We recommend that you copy the configuration files, edit the copies for your needs and use these files to create your own launch file.   An example is available at `launch/run_hardware_interface.launch`, `test/test_params.yaml`, `config/controller_joint_names.yaml` and `config/hardware_controllers.yaml`.  This files should achieve a working test-launch.

In order to successfully launch the **kuka_kss_hw_interface** a parameter `robot_description` needs to be present on the ROS parameter server. This parameter can be set manually or by adding this line inside the launch file (replace the robot type support package and .xacro to match your application):
```
<param name="robot_description" command="$(find xacro)/xacro '$(find kuka_kr6_support)/urdf/kr6r900sixx.xacro'"/>
```
Make sure that the line is added before the `kuka_kss_hw_interface` itself is loaded.

## 4. Testing
At this point you are ready to test the interface. 
### Safety Warning:
Make sure the robot is properly contained within safety guarding and that the X11 safety interfaces are properly wired to the **Safety Door Switch** and an **Emergency Stop**.  Follow all safety guidelines outlined in the Kuka operating manuals.  The **kuka_kss_hw_interface** is prepared to run the robot in Automatic External mode, which means that the driver is capable of starting motion from the ROS system.  Be sure all personnel exit the operating area before starting the **kuka_kss_hw_interface**.

There are several ROS packages that are used in this testing:
 - rqt
 - packages in ros_controllers: https://github.com/ros-controls/ros_controllers/
 - urdf
 - joint_state_publisher

Note that the **kuka_kss_hw_interface** initially connects to the robot with Ethernet KRL communication and PTP motion control mode.  The current communication link supports an update cycle of about 20Hz.  This is sufficient for slow motions but attempting faster motions will exhibit jerky control.  For a faster control loop the hardware interface control mode can be switched to RSI control with the service /request_control_type.

### 4.1 Initial Connection Test

1. Edit the parameters in `test/test_params.yaml` to match the configuration on the robot, as described in the section above.

2. The next steps describe how to launch the test file:
In a new terminal:
```
$ roscore
```
3. On the Kuka teach pad, enter mode **Ext**.   For initial testing safety, reduce the run override speed to 10% or lower by touching the override setting area to the right of the Ext in the top status bar.

4. The launch file `launch/run_hardware_interface.launch` has all the commands to run an initial test of the hardware interface.  This is predefined to use the URDF  for an Agilus KR6R900 robot.  If a different robot is to be connected, edit the file to replace the xacro with the correct model.
5. Execute the sample launch file:
```
$ roslaunch kuka_kss_hw_interface run_kss_hw_interface.launch sim:=false
```
The **kuka_kss_hw_interface** will attempt to connect to the robot controller.   
Below is a sample showing important INFO messages from the successful startup of **kuka_kss_hw_interface**:
```
[ INFO] [52.466289927]: Starting hardware interface...
[ INFO] [52.475511483]: controller_joint_names configured for 6 axes.
[ INFO] [52.476239403]: Loaded kuka_kss_hardware_interface
[ INFO] [52.477450305]: Starting Kuka communication link.
[ INFO] [52.479651122]: Kuka ROS EKI interface version V2003ROS.
[ INFO] [52.482346337]: Configuring connection to Kuka TCP Server interface on: 172.32.1.10, 54600
[ INFO] [52.485504889]: Installing RSI communication link.
[ INFO] [52.485639464]: Kuka ROS RSI interface version V2003ROS.
[ INFO] [52.486370178]: Configuring ROS UDP Server interface on: 172.32.1.243, 49152
[ INFO] [52.710306724]: Waiting for robot EKI server...
[ INFO] [52.712041312]: EKI server connected. Initializing communication.
[ INFO] [52.876292835]: EKI Successful connection with robot.
[ INFO] [52.876385857]: EKI Initial position: A1=-6.03 A2=-101.70 A3=93.94 A4=-6.56 A5=47.22 A6=77.66 E1=0.00 E2=0.00  E3=0.00  E4=0.00  E5=0.00  E6=0.00
```
The robot pendant will display the message "ROS client connected."

6. Open a new terminal and run the rqt utility.
```
$ rqt
```

* Use the **Plugin > Services** menu to select and open the **Service Caller** tool.

7. Use the dropdown list of services to select the **/start_motion** service.  Click the Call button to call the service.  You should see on the Kuka teach pad that the program starts running.  
	* If the SIR indicators are not all green, call the service again.  
	* If the program fails to start, check for errors on the  Kuka teach pad or ROS output.
	* Other services are available to call.
		* /stop_motion
		* /reset
		* /enable_robot 
		* /disable_robot
		* /get_robot_info
		* /get_control_type
		* /request_control_type

8. Use the **Plugin > Robot** menu to select and open the **Joint trajectory controller** tool.
	* Choose **controller manager ns** and **controller** and you should be able to move each robot joint.
	* As an alternative to the rqt interface you may choose to run the **Joint trajectory controller** tool directly:
```
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```




