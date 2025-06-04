/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021 Kuka Robotics Corp
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Kuka Robotics Corp or Kuka GMBH,
 *     nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Pat Duda <Pat.Duda@kuka.com>
 */

#ifndef KUKA_ROBOT_STATE_H
#define KUKA_ROBOT_STATE_H

#include <vector>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <urdf/model.h>
#include <realtime_tools/realtime_clock.h>
#include <std_msgs/Duration.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/GetRobotInfo.h>
#include <kuka_msgs/ControlType.h>
#include <kuka_msgs/ControlState.h>

namespace kuka_kss_hw_interface
{



/**
 * @brief The KukaRobotState class stores the current state of the Kuka robot.
 * The current values and current commands are stored.
 * Robot status like run state is also contained.
 */
class KukaRobotState
{
public:
    KukaRobotState();
    ~KukaRobotState();

    /**
     * @brief The KukaJointType enum contains the joint types from the actual robot.
     */
    enum KukaJointType
    {
        Linear = 1,
        Revolute = 3,
        Endless = 5 // also Revolute
    };

    /**
     * @brief The KukaRunState enum reflects the state of the control program.
     */
    enum KukaRunState
    {
        Unknown,
        NoProgram,
        Ready,
        Running,
        Stopped,
        Done,
        Error
    };

    enum KukaOpMode
    {
        T1 = 1,
        T2 = 2,
        Auto = 3,
        Ext = 4
    };

    // For mapping to a bitfield
    enum KukaStatusID
    {
        Estop = 1,
        GuardStop = 2,
        DrivesPowered = 4,
        MotionPossible = 8,
        InMotion = 16,
        InError = 32
    };

    struct KukaStatus
    {
        unsigned int
            Estop:1,
            GuardStop:1,
            DrivesPowered:1,
            MotionPossible:1,
            InMotion:1,
            InError:1,
            OpMode:3;
        unsigned int:23; // extra bits
    };

    union KukaStatusCode
    {
        unsigned int code;
        KukaStatus stat;
    };

    /**
     * @brief The KukaControlState enum reflects the selected control program running on the robot
     */
    enum KukaControlState
    {
        RosControl_KRL_PTP=kuka_msgs::ControlState::RosControl_KRL_PTP,
        RosControl_RSI4ms_Pos=100,
        RosControl_RSI12ms_Pos=101,
        RosControl_RSI4ms_Vel=150, // future
        RosControl_RSI12ms_Vel=151  // future
    };


    /**
     * @brief The KukaCapability enum reflects possible installed options on the robot
     */
    enum KukaCapability
    {
        KukaEKI,
        KukaRSI,
        KukaMxA  //future
    };

    /**
     * @brief setJointType is used to initialize the Kuka joint types from MADA.
     * @param index - The kuka joint index
     * @param jtype - KukaJointType
     */
    void setJointType(int index, KukaJointType jtype) {joint_types_[index]=jtype;}

    /**
     * @brief getJointType - retrieve the current joint type setting.
     * @param index
     * @return
     */
    KukaJointType getJointType(int index) {return(joint_types_[index]);}

    /**
     * @brief checkJointType - Checks the Kuka joint type against the ROS definition from URDF
     * The Kuka joint types must be properly initialized with /ref setJointType().
     * @param index - The kuka joint index
     * @param checkJoint - The ROS Joint definition from URDF
     * @return
     */
    bool checkJointType(int index, urdf::Joint checkJoint) ;

    /**
     * @brief setJointMotorMaxVel - Set the max motor velocity.  Used for calculating joint vel.
     * @param index - The kuka joint index
     * @param max - The max motor speed in RPM
     */
    void setJointMotorMaxVel(int index, double max) {max_motor_vel_[index]=max;}

    /**
     * @brief setJointGearRatio - Set the axis gear ratio
     * @param index - The kuka joint index
     * @param num - numerator from MADA
     * @param denom - denominator from MADA
     */
    void setJointGearRatio(int index, int num, int denom);
    /**
     * @brief setIPOC - set the current IPOC value from RSI or $ROBTIMER.  Time in ms.
     * Automatically captures the system time stamp.
     * This should always be set when updating robot pos/vel/eff state data.
     * @param i - IPOC value.
     */
    void setIPOC(unsigned long long i);

    /**
     * @brief setIPOC - set the current IPOC value from RSI or $ROBTIMER.  Time in ms.
     * Enables manuallysetting a time stamp from another source.
     * This should always be set when updating robot pos/vel/eff state data.
     * @param i - IPOC value.
     * @param t - The desired time stamp.
     */
    void setIPOC(unsigned long long i, ros::Time t);


    /**
     * @brief getIPOC - Get the IPOC value for most recent data set.
     * @return
     */
    unsigned long long getIPOC() {return(ipoc_);}

    /**
     * @brief getIpocDuration - calculate the update interval duration from IPOC value.
     * @return
     */
    ros::Duration getIpocDuration();

    /**
     * @brief getTimeStamp - return the timestamp for the latest update
     * @return
     */
    ros::Time getTimeStamp() { return time_stamp_;}

    /**
     * @brief getJointPosition - Gets the entire position vector for reference.
     * @return - the values are in ROS standard radians.
     */
    std::vector<double> getJointPositions() const {return joint_position_;}

    /**
     * @brief getJointPositionPtr - gets a pointer for the joint value.
     * To be used when registering ROS Control interface.
     * @param j - The kuka joint index.
     * @return
     */
    double* getJointPositionPtr(int j) {return(&joint_position_[j]);}

    /**
     * @brief setJointPosition - update the joint position state.  Automatically converts to ROS standard radians.
     * @param j - The kuka joint index.
     * @param joint_position - the kuka position value in degrees.
     */
    void setJointPosition(int j, double joint_pos);

    /**
     * @brief getJointPositionKuka - Gets the actual joint position converted to Kuka standards.
     * @return
     */
    double getJointPositionKuka(int j);

    /**
     * @brief getJointVelocities - Get the list of jont velocities in ROS units.
     * @return
     */
    std::vector<double> getJointVelocities() const {return joint_velocity_;}

    /**
     * @brief getJointVelocityPtr - gets a pointer for the joint velocity.
     * To be used when registering ROS Control interface.
     * @param j - The kuka joint index.
     * @return
     */
    double* getJointVelocityPtr(int j) {return(&joint_velocity_[j]);}

    /**
     * @brief setJointVelocity - update the joint velocity state.  Automatically converts to ROS standard radians/sec.
     * Kuka units are % max(rotational) and % max(linear)
     * Values stored as ROS standard: radians/s(rotational) and m/s(linear)
     * @param j - The kuka joint index.
     * @param joint_position - the kuka position value in degrees.
     */
    void setJointVelocity(int j, double joint_vel_percent);

    std::vector<double> getJointEfforts() const {return joint_effort_;}
    /**
     * @brief getJointEffortPtr - gets a pointer for the joint effort.
     * To be used when registering ROS Control interface.
     * @param j - The kuka joint index.
     * @return
     */
    double* getJointEffortPtr(int j) {return(&joint_effort_[j]);}

    /**
     * @brief setJointEffort - update the joint effort state.
     * @param j - The kuka joint index.
     * @param joint_eff_Nm - Kuka and ROS standards are same.  Nm for revolute, N for Linear.
     */
    void setJointEffort(int j, double joint_eff_Nm) {joint_effort_[j] = joint_eff_Nm;}


    /**
     * @brief getCommandJointPositionKuka - Gets the commanded position converted to Kuka standards.
     * @return
     */
    double getCommandJointPositionKuka(int j);

    /**
     * @brief getCommandJointPositionPtr - gets a pointer for the commanded joint velocity.
     * To be used when registering ROS Control interface.
     * @param j - The kuka joint index.
     * @return
     */
    double* getCommandJointPositionPtr(int j) {return(&joint_position_command_[j]);}


    /**
     * @brief getCommandJointPositionKuka - Gets the commanded velocity converted to Kuka standards.
     * @return
     */
    double getCommandJointVelocityPercent(int j);

    /**
     * @brief getCommandJointVelocityPtr - gets a pointer for the commanded joint velocity.
     * To be used when registering ROS Control interface.
     * @param j - The kuka joint index.
     * @return
     */
    double* getCommandJointVelocityPtr(int j) {return(&joint_velocity_command_[j]);}



    /**
     * @brief posKukaToROS - Helper function to convert Kuka to ROS standard.
     * @param joint_pos
     * @param t
     * @return
     */
    double posKukaToROS(double joint_pos, KukaJointType t);

    /**
     * @brief posROSToKuka - Helper function to convert ROS to Kuka standard.
     * @param joint_pos
     * @param t
     * @return
     */
    double posROSToKuka(double joint_pos, KukaJointType t);

    /**
     * @brief velKukaToROS - Helper function to convert Kuka to ROS standard.
     * @param jvel
     * @param j
     * @return
     */
    double velKukaToROS(double jvel, int j);

    /**
     * @brief velROSToKuka - Helper function to convert ROS to Kuka standard.
     * @param jvel
     * @param j
     * @return
     */
    double velROSToKuka(double jvel, int j);



    /**
     * @brief getRobotMutex - For locking when robot data is being changed:
     *        actual pos/vel/eff and command pos/vel/eff as well as IPOC data.
     * @return
     */
    std::mutex& getRobotMutex() {return robot_mutex_;}

    /**
     * @brief getStateMutex - For locking when changing robot run state or control state.
     * @return
     */
    std::mutex& getStatusMutex() {return status_mutex_;}

    unsigned int getDOF() const {return n_dof_;}
    void setDOF(unsigned int dof) {n_dof_ = dof;}

    /**
     * @brief getRobotInfo
     * @return Gets a reference to the robot info
     */
    industrial_msgs::GetRobotInfo::Response& getRobotInfo() {return (rob_info_);}

    int getSerialNum() const {return serial_num_;}
    void setSerialNum(int serial_num) {serial_num_ = serial_num;
                                       rob_info_.robots[0].serial_number = std::to_string(serial_num);
                                       rob_info_.controller.serial_number = std::to_string(serial_num);}

    std::string getRobName() const {return rob_name_;}
    void setRobName(const std::string &rob_name) {rob_name_ = rob_name;}

    std::string getRobModel() const {return rob_model_;}
    void setRobModel(const std::string &rob_model) {rob_model_ = rob_model;
                                                    rob_info_.robots[0].model = rob_model;}

    /**
     * @brief The ControlState enum reflects the selected operation state (control program running on the robot)
     */
    kuka_msgs::ControlState getRequestdRobOpState() const {return requested_op_state_;}
    void setRequestedRobOpState(const kuka_msgs::ControlState &state);
    kuka_msgs::ControlState getRobOpState() const {return current_op_state_;}
    void setRobOpState(const kuka_msgs::ControlState &state);
    void setRobOpState(const std::string &state);
    std::string getOpStateStr(const kuka_msgs::ControlState &state) const;
    kuka_msgs::ControlState getOpStateFromStr(const std::string &stateStr);
    bool checkValidOpState(const kuka_msgs::ControlState &state, std::string &diag_msg);

    /**
     * @brief The ControlType enum reflects the desired mode of controlling the robot
     */
    kuka_msgs::ControlType getRequestedControlType() const {return requested_control_type_;}
    void setRequestedControlType(const kuka_msgs::ControlType &requested_control_type);
    kuka_msgs::ControlType getSelectedControlType() const {return selected_control_type_;}
    void setSelectedControlType(const kuka_msgs::ControlType &selected_control_type);
    void setSelectedControlType(std::string &selected_control_type);
    std::string getControlTypeStr(const kuka_msgs::ControlType &type) const;
    kuka_msgs::ControlType getControlTypeFromStr(std::string &type);
    kuka_msgs::ControlType getControlTypeFromOpState(const kuka_msgs::ControlState &state);
    bool checkValidControlType(const kuka_msgs::ControlType &ctrl_type, std::string &diag_msg);


    KukaRunState getRobRunState() const {return rob_run_state_;}
    void setRobRunState(const KukaRunState &state) {rob_run_state_ = state;}

    void addFeature(KukaCapability feature);
    bool hasFeature(KukaCapability feature);


    KukaOpMode getOpMode() const;
    void setOpMode(const KukaOpMode op_mode);
    void setOpMode(const int op_mode);

    void setErrorCode(const int code);
    int getErrorCode();

    KukaStatus getStatus() const;
    void setStatus(const KukaStatus &status);
    /**
     * @brief setStatus updates the status with a binary value.  Use a binary combination set with KukaStatusID
     * @param status - The bits to turn on.  Unspecified bits are off.
     * @example setStatus(KukaStatusID.Estop | KukaStatusID.GuardStop);
     */
    void setStatus(const unsigned int &status);
    /**
     * @brief setStatusOn updates the bits identified to on.
     * @param status
     * @example setStatusOn(KukaStatusID.Estop);
     */
    void setStatusOn(const KukaStatusID id);
    /**
     * @brief setStatusOff updates the bits identified to off.
     * @param status
     * @example setStatusOff(KukaStatusID.GuardStop);
     */
    void setStatusOff(const KukaStatusID id);




    /**
     * @brief statusChanged indicates that the status value has changed since last checked.
     * The internal tracking is reset so a call to this is one-shot.
     * A second call will return false until the status was changed again with one of the set methods.
     * @return
     */
    bool statusChanged() ;
    /**
     * @brief getRos_status
     * @return The ROS status type for ready use in ROS messages.
     */
    industrial_msgs::RobotStatus getRos_status() const {return (ros_status_);}


protected:
    /**
     * @brief toTriState - Converts an int with 0, 1 to the TriState value
     * @return (0=off, 1=on, other=unknown)
     */
    industrial_msgs::TriState toTriState (unsigned int val);
    /**
     * @brief toTriState - Converts the bool to a TriState value
     * @return
     */
    industrial_msgs::TriState toTriState (bool val);

private:

    void setRosStatus(const KukaRobotState::KukaStatus &status);
    void setRosStatus(const KukaStatusID id);
    void setRosOpMode(const KukaOpMode op_mode);
    // robot info - use robot_mutex_ ===========================================
    std::mutex robot_mutex_;
    // Kuka units are degrees(rotational) and mm(linear)
    // Values stored as ROS standard: radians(rotational) and m(linear)
    std::vector<double> joint_position_;
    // Kuka units are % max(rotational) and % max(linear)
    // Values stored as ROS standard: radians/s(rotational) and m/s(linear)
    std::vector<double> joint_velocity_;
    // Kuka units are Nm(rotational) and Nm(linear) of motor torque
    std::vector<double> joint_effort_;

    // These command values will typically be set directly by ROS Control.
    // Kuka units are degrees(rotational) and mm(linear)
    // Values stored as ROS standard: radians(rotational) and m(linear)
    std::vector<double> joint_position_command_;
    // Kuka units are % max(rotational) and % max(linear)
    // Values stored as ROS standard: radians/s(rotational) and m/s(linear)
    std::vector<double> joint_velocity_command_;

    void IPOC_setVal(unsigned long long i);
    bool ipoc_init_;
    unsigned long long ipoc_;
    unsigned long long ipocPrevious_;
    ros::Time time_stamp_;

    // Robot State info - use state_mutex_ ===========================================
    std::mutex status_mutex_;
    // Handle opState request & handle state transition from request to selected(running)
    kuka_msgs::ControlState requested_op_state_;
    kuka_msgs::ControlState current_op_state_;
    // Handle control type request & handle state transition from request to selected(running)
    kuka_msgs::ControlType requested_control_type_;
    kuka_msgs::ControlType selected_control_type_;

    KukaRunState rob_run_state_;
    KukaOpMode op_mode_;
    KukaStatusCode status_;
    KukaStatusCode status_previous_;
    industrial_msgs::RobotStatus ros_status_;

    // Robot configuration info - one time setup
    industrial_msgs::GetRobotInfo::Response rob_info_;
    int serial_num_;
    std::string rob_name_;
    std::string rob_model_;
    std::set<KukaCapability> features_;
    unsigned int n_dof_;
    std::vector<KukaJointType> joint_types_;
    std::vector<double> max_motor_vel_;
    std::vector<double> gear_ratio_;


    const double RAD2DEG = 57.295779513082323;
    const double DEG2RAD = 0.017453292519943295;
};







}  // namespace kuka_kss_hw_interface

#endif // KUKA_ROBOT_STATE_H
