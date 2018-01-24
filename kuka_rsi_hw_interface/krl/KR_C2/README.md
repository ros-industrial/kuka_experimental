# Configuring RSI on the controller

This guide highlights the steps needed in order to successfully configure the **RSI interface** on the KR-C2 controller to work with the **kuka_rsi_hardware_interface** on your PC with ROS.

## 1. Controller network configuration

Controller setup is described in chapter 4 of the **KUKA.Ethernet RSI XML 1.1** manual.
**KUKA.RobotSensorInterface** package must be installed. Depending on the version of the **KUKA.RobotSensorInterface** package you might also need the **KUKA.Ethernet RSI XML**. This can be verified in the *InstallTech* window (`Setup -> 9 Install additional Software` on the KCP).

Make sure not to use the `192.0.1.x` IP range for the controller or PC.

From the Ethernet RSI XML manual:

> The IP address range 192.0.1.x is reserved and is disabled for applications.
> Configuring the VxWorks network connection with this address range results in a system error in the KUKA system software. It is no longer possible to boot the robot controller.

Ping the controller from the PC with ROS to make sure the network configuration is working.


## 2. KRL Files

The files included in this folder specifies the data transferred via RSI. Some of the files needs to be modified to work for your specific configuration.

##### ros_rsi_ethernet.xml

1. Edit the `IP_NUMBER` tag so that it corresponds to the IP address of the PC running `kuka_rsi_hw_interface`.
2. Keep the `PORT` tag as it is (49152) or change it if you want to use another port.

Note that the `rsi/listen_address` and `rsi/listen_port` parameters of the `kuka_rsi_hw_interface` must correspond to the `IP_NUMBER`and `PORT` set in these KRL files.


##### ros_rsi.src

This should only be edited if the start position or max joint movements specified within the file is not desirable for your application. These variables are inside the Configuration fold.

##### Copy files to controller

1. Copy the `ros_rsi.src` file to `KRC:\R1\Program` (Alternatively `C:\KRC\ROBOTER\KRC\R1\Program`)
2. Copy the `ros_rsi_ethernet.xml` file to `C:\KRC\ROBOTER\Init`

Note: You must reload KSS or reboot the controller if `ros_rsi.src` is not created / copied using the HMI.

## 3. Configure the kuka_rsi_hw_interface

The **kuka_rsi_hardware_interface** needs to be configured in order to successfully communicate with RSI. Inside `/kuka_rsi_hw_interface/test` and `/kuka_rsi_hw_interface/config` in this repository is a set of `*.yaml` files. These configuration files may be loaded into a launch-file used to start the **kuka_rsi_hardware_interface** with correct parameters, such as:

* **Joint names** corresponding to the robot description (URDF or .xacro).
* **IP address** and **port** corresponding to the RSI setup specified in **ros_rsi_ethernet.xml**.

We recommend that you copy the configuration files, edit the copies for your needs and use these files to create your own launch file. A template will be provided at a later time. However, at this time, have a look at `test_hardware_interface.launch`, `test_params.yaml`, `controller_joint_names.yaml` and `hardware_controllers.yaml` to achieve a working test-launch.

In order to successfully launch the **kuka_rsi_hardware_interface** a parameter `robot_description` needs to be present on the ROS parameter server. This parameter can be set manually or by adding this line inside the launch file (replace support package and `.xacro` to match your application):

```
<param name="robot_description" command="$(find xacro)/xacro.py '$(find kuka_kr6_support)/urdf/kr6r900sixx.xacro'"/>
```

Make sure that the line is added before the `kuka_hardware_interface` itself is loaded.

## 4. Testing

At this point you are ready to test the RSI interface. Before the test, make sure that:

* You have specified the `rsi/listen_address` and `rsi/listen_port` of the **kuka_rsi_hardware_interface** to correspond with the KRL files on the controller.
* You have a launch-file loading the network parameters, robot description, kuka_hardware_interface, hardware controller and controller joint names.

The next steps describe how to launch the test file:

* In a new terminal:

```
$ roscore
```

* Open a new terminal and roslaunch the previously configured launch-file:

```
$ roslaunch kuka_rsi_hw_interface test_hardware_interface.launch sim:=false
```

The **kuka_rsi_hardware_interface** is now waiting for the robot controller to connect. Follow the next steps to connect RSI:

1. On the teach pad, enter mode **T1** for testing purposes.
2. Navigate to `KRC:\R1\Program` and select `ros_rsi.src`.
3. Press and hold an enabling switch and the run/play-button. The robot will first move to the start position.
   * A message like **Programmed path reached (BCO)** will be shown at the teach pad.
4. Press and hold again. The teach pad will post a warning **!!!Attention –RSI sensor mode active!!!**.
5. Confirm the warning and press and hold the buttons again. This time the terminal where **kuka_rsi_hardware_interface** is running should output **Got connection from robot**. The RSI connection is now up and running.
6. In a new terminal:

```
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Choose **controller manager ns** and **controller** and you should be able to move each robot joint.

* Note that T1-mode limits the robot movement velocity and is intended for testing purposes.

## 5. Troubleshooting

### RSI: Error in function <ST_ETHERNET>

Problems establishing connection between robot or PC. Check IP, port and that the **kuka_rsi_hardware_interface** is running on the PC. May also come from malformed XML in `ros_rsi_ethernet.xml`.


### SEN: RSI execution error <execute> - RSI stopped

Most likely due to late packages. From the Ethernet RSI XML manual:

> A data packet received by the external system must be answered within approx. 10 ms. If the data packet is not received by the robot controller within this period, the response is classified as too late. When the maximum number of external data packets for which a response has been sent too late has been exceeded, the robot interprets this as an error and stops.

If you have problems with the connection to RSI shutting down now and then while moving the robot it is suggested to:

* Compile and install a [RT-Preempt](https://rt.wiki.kernel.org/index.php/RT_PREEMPT_HOWTO) kernel for your PC.
* Give **kuka_rsi_hardware_interface** on your PC real-time priority when the RSI connection is established.
