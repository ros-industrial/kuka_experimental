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



#include <kuka_kss_hw_interface/kuka_robot_state.h>


namespace kuka_kss_hw_interface
{

KukaRobotState::KukaRobotState():
    n_dof_(6), ipoc_init_(false), ipoc_(0), ipocPrevious_(0),
    rob_run_state_(Unknown),
    joint_position_(12,0.0), joint_velocity_(12,0.0), joint_effort_(12,0.0),
    joint_position_command_(12,0.0), joint_velocity_command_(12,0.0),
    joint_types_(12, Revolute), max_motor_vel_(12,1.0), gear_ratio_(12,1.0)
{
    rob_info_.robots.resize(1);

    selected_control_type_.control_type = kuka_msgs::ControlType::KUKA_KRL;
    current_op_state_.control_state = kuka_msgs::ControlState::RosControl_KRL_PTP;
}


KukaRobotState::~KukaRobotState()
{

}


bool KukaRobotState::checkJointType(int index, urdf::Joint checkJoint)
{
    if (index >= n_dof_)
        return(true);

    switch (joint_types_[index])
    {
    case Linear:
        return(checkJoint.type == urdf::Joint::PRISMATIC);
    case Revolute:
        return(checkJoint.type == urdf::Joint::REVOLUTE);
    case Endless:
        return(checkJoint.type == urdf::Joint::CONTINUOUS);
    }

    return (false);
}


void KukaRobotState::setJointGearRatio(int index, int num, int denom)
{
    if (denom == 0)
    {
        throw std::logic_error("setJointGearRatio denom=0 causes divide by zero.");
    }
    gear_ratio_[index]=(double)num/(double)denom;
}

void KukaRobotState::IPOC_setVal(unsigned long long i)
{
    if (ipoc_init_)
    {
        ipocPrevious_ = ipoc_;
    }
    else
    {
        ipoc_init_ = true;
        ipocPrevious_ = i;
    }
    ipoc_ = i;
}

void KukaRobotState::setRequestedRobOpState(const kuka_msgs::ControlState &state)
{
    std::string msg = "setRequestedRobOpState init";
    bool isValid = checkValidOpState(state, msg);
    if (!isValid)
    {
        throw std::invalid_argument(msg);
    }
    requested_op_state_ = state;
}

/**
 * @brief setRobOpState Sets the operation state (robot program) that is active.
 * Note that this is intended to be called (set) from feedback reported by the robot.
 * setRequestedRobOpState() is to be used to request a state change.
 * Note that this also sets the Control Type with setSelectedControlType()
 *
 * @param state
 */
void KukaRobotState::setRobOpState(const kuka_msgs::ControlState &state)
{
    std::string msg = "setRobOpState init";
    bool isValid = checkValidOpState(state, msg);
    if (!isValid)
    {
        throw std::invalid_argument(msg);
    }
    setSelectedControlType(getControlTypeFromOpState(state));
    current_op_state_ = state;
}


void  KukaRobotState::setRobOpState(const std::string &state)
{
    setRobOpState(getOpStateFromStr(state));
}

/**
 * @brief checkValidOpState validates the opState against the capabilities reported by the robot.
 * @param state - The Op State to check for validity.
 * @param diag_msg - reason string used for exception reporting.
 * @return
 */
bool KukaRobotState::checkValidOpState(const kuka_msgs::ControlState &state, std::string &diag_msg)
{
    bool valid = false;
    switch (state.control_state)
    {
    case kuka_msgs::ControlState::RosControl_KRL_PTP:
        diag_msg = "OK";
        valid = true;
        break;
    case kuka_msgs::ControlState::RosControl_RSI4ms_Pos:
    case kuka_msgs::ControlState::RosControl_RSI4ms_Vel:
    case kuka_msgs::ControlState::RosControl_RSI12ms_Pos:
    case kuka_msgs::ControlState::RosControl_RSI12ms_Vel:
        if (hasFeature(KukaCapability::KukaRSI))
        {
            diag_msg = "OK";
            valid = true;
        }
        else
        {
            valid = false;
            diag_msg = "Unsupported feature: RSI not installed on robot.";
        }
        break;
    default:
        valid = false;
        diag_msg = "Unsupported feature: unknown state.";
        break;
    }
    return(valid);
}

kuka_msgs::ControlType KukaRobotState::getControlTypeFromOpState(const kuka_msgs::ControlState &state)
{
    kuka_msgs::ControlType ctrlType;
    ctrlType.control_type = kuka_msgs::ControlType::KUKA_UNKNOWN;

    switch (state.control_state)
    {
    case kuka_msgs::ControlState::RosControl_KRL_PTP:
        ctrlType.control_type = kuka_msgs::ControlType::KUKA_KRL;
        break;
    case kuka_msgs::ControlState::RosControl_RSI4ms_Pos:
    case kuka_msgs::ControlState::RosControl_RSI4ms_Vel:
        ctrlType.control_type = kuka_msgs::ControlType::KUKA_RSI4ms;
        break;
    case kuka_msgs::ControlState::RosControl_RSI12ms_Pos:
    case kuka_msgs::ControlState::RosControl_RSI12ms_Vel:
        ctrlType.control_type = kuka_msgs::ControlType::KUKA_RSI12ms;
        break;
    }
    return(ctrlType);
}

void KukaRobotState::setRequestedControlType(const kuka_msgs::ControlType &request_control_type)
{
    std::string msg = "init";
    bool isValid = checkValidControlType(request_control_type, msg);
    if (!isValid)
        throw std::invalid_argument(msg);
    else
        requested_control_type_ = request_control_type;
}


void KukaRobotState::setSelectedControlType(const kuka_msgs::ControlType &selected_control_type)
{
    std::string msg = "init";
    bool isValid = checkValidControlType(selected_control_type, msg);
    if (!isValid)
        throw std::invalid_argument(msg);
    else
        selected_control_type_ = selected_control_type;
}

void KukaRobotState::setSelectedControlType(std::string &selected_control_type)
{
    setSelectedControlType(getControlTypeFromStr(selected_control_type));
}

bool KukaRobotState::checkValidControlType(const kuka_msgs::ControlType &ctrl_type, std::string &diag_msg)
{
    bool valid = false;
    switch(ctrl_type.control_type)
    {
    case kuka_msgs::ControlType::KUKA_KRL:
        diag_msg = "OK";
        valid = true;
        break;
    case kuka_msgs::ControlType::KUKA_RSI4ms:
    case kuka_msgs::ControlType::KUKA_RSI12ms:
        if (hasFeature(KukaCapability::KukaRSI))
        {
            diag_msg = "OK";
            valid = true;
        }
        else
        {
            diag_msg = "Unsupported feature: RSI not installed on robot.";
            valid = false;
        }

        break;
    default:
        diag_msg = "Unsupported feature: unknown type.";
        valid = false;
        break;
    }
    return(valid);
}


void KukaRobotState::setIPOC(unsigned long long i)
{
    IPOC_setVal(i);
    time_stamp_ = ros::Time::now();
}

void KukaRobotState::setIPOC(unsigned long long i, ros::Time t)
{
    IPOC_setVal(i);
    time_stamp_ = t;
}


ros::Duration KukaRobotState::getIpocDuration()
{
    ros::Duration d;
    double t_sec;
    if (!ipoc_init_)
    {
        d.fromSec(0.0);
        return(d);
    }

    //IPOC is in millisec (special check in case messages are out of time order)
    if (ipoc_ > ipocPrevious_)
        t_sec = (double)(ipoc_ - ipocPrevious_)*0.001;
    else
        t_sec = (double)(ipocPrevious_ - ipoc_)*0.001;

    d.fromSec(t_sec);
    return(d);
}


void KukaRobotState::setJointPosition(int j, double joint_pos)
{
    joint_position_[j] = posKukaToROS(joint_pos, joint_types_[j]);
}

double KukaRobotState::getJointPositionKuka(int j)
{
    return(posROSToKuka(joint_position_[j], joint_types_[j]));
}


double KukaRobotState::posKukaToROS(double joint_pos, KukaJointType t)
{
    double jval;

    switch(t)
    {
    case Linear:
        // Convert mm to meter
        jval = joint_pos/1000.0;
        break;
    case Revolute:
    case Endless:
        // Convert degrees to radians
        jval = DEG2RAD * joint_pos;
        break;
    }
    return(jval);
}


double KukaRobotState::posROSToKuka(double joint_pos, KukaJointType t)
{
    double jval;

    switch(t)
    {
    case Linear:
        // Convert mm to meter
        jval = joint_pos * 1000.0;
        break;
    case Revolute:
    case Endless:
        // Convert degrees to radians
        jval = RAD2DEG * joint_pos;
        break;
    }
    return(jval);
}



void KukaRobotState::setJointVelocity(int j, double joint_vel_percent)
{
    joint_velocity_[j] = velKukaToROS(joint_vel_percent, j);
}

double KukaRobotState::velKukaToROS(double jvel, int j)
{
    double vel;

    switch(joint_types_[j])
    {
    case Linear:
        // Convert % to motor RPM =>(joint_vel_percent/100.0)*max_motor_vel_
        // Convert to mm and then meters ==> /(gear_ratio_ * 1000.0)
        vel = (jvel/100.0)*max_motor_vel_[j]/(gear_ratio_[j] * 1000.0);
        break;
    case Revolute:
    case Endless:
        // Convert % to motor RPM =>(joint_vel_percent/100.0)*max_motor_vel_
        // Convert to degrees/s ==> *6.0/gear_ratio_  (6.0=360deg/rev*1min/60sec)
        // Convert degrees to radians
        vel = (jvel/100.0)*max_motor_vel_[j]*6.0/gear_ratio_[j] * DEG2RAD;
        break;
    }
    return(vel);
}

double KukaRobotState::velROSToKuka(double jvel, int j)
{
    double vel;

    switch(joint_types_[j])
    {
    case Linear:
        // Convert meters to mm => *1000.0
        // Convert to motor RPM => *gear_ratio_[j]
        // Convert motor RPM to % =>/max_motor_vel_*100.0
        vel = jvel*1000.0*gear_ratio_[j]/max_motor_vel_[j]*100.0;
        break;
    case Revolute:
    case Endless:
        // Convert radians to degrees
        // Convert degrees to motor RPM ==> *gear_ratio_/6.0  (6.0=360deg/rev*1min/60sec)
        // Convert motor RPM to %=> /max_motor_vel_*100.0
        vel = RAD2DEG*(jvel*gear_ratio_[j]/6.0)/max_motor_vel_[j]*100.0;
        break;
    }
    return(vel);
}


std::string KukaRobotState::getOpStateStr(const kuka_msgs::ControlState &state) const
{
    switch (state.control_state)
    {
        case kuka_msgs::ControlState::RosControl_KRL_PTP:
            return("KRL_PTP");
            break;
        case kuka_msgs::ControlState::RosControl_RSI4ms_Pos:
            return("RSI4ms_Pos");
            break;
        case kuka_msgs::ControlState::RosControl_RSI12ms_Pos:
            return("RSI12ms_Pos");
            break;
        case kuka_msgs::ControlState::RosControl_RSI4ms_Vel:
            return("RSI4ms_Vel");
            break;
        case kuka_msgs::ControlState::RosControl_RSI12ms_Vel:
            return("RSI12ms_Vel");
            break;
    }
    return("UNKNOWN");
}


kuka_msgs::ControlState KukaRobotState::getOpStateFromStr(const std::string &stateStr)
{
    kuka_msgs::ControlState state;

    if (stateStr=="KRL_PTP")
    {
        state.control_state = kuka_msgs::ControlState::RosControl_KRL_PTP;
    }
    else if (stateStr=="RSI4ms_Pos")
    {
        state.control_state = kuka_msgs::ControlState::RosControl_RSI4ms_Pos;
    }
    else if (stateStr=="RSI12ms_Pos")
    {
        state.control_state = kuka_msgs::ControlState::RosControl_RSI12ms_Pos;
    }
    else if (stateStr=="RSI4ms_Vel")
    {
        state.control_state = kuka_msgs::ControlState::RosControl_RSI4ms_Vel;
    }
    else if (stateStr=="RSI12ms_Vel")
    {
        state.control_state = kuka_msgs::ControlState::RosControl_RSI12ms_Vel;
    }
    else state.control_state = kuka_msgs::ControlState::RosControl_UNKNOWN;

    return(state);
}


std::string KukaRobotState::getControlTypeStr(const kuka_msgs::ControlType &type) const
{
    switch(type.control_type)
    {
    case kuka_msgs::ControlType::KUKA_KRL:
        return("KUKA_KRL");
        break;
    case kuka_msgs::ControlType::KUKA_RSI4ms:
        return("KUKA_RSI4ms");
        break;
    case kuka_msgs::ControlType::KUKA_RSI12ms:
        return("KUKA_RSI12ms");
        break;
    default:
        return("UNKNOWN");
        break;
    }

    return("UNKNOWN");
}

kuka_msgs::ControlType KukaRobotState::getControlTypeFromStr(std::string &type)
{
    kuka_msgs::ControlType ctrType;

    if (type=="KUKA_KRL")
    {
        ctrType.control_type = kuka_msgs::ControlType::KUKA_KRL;
    }
    else if (type=="KUKA_RSI4ms")
    {
        ctrType.control_type = kuka_msgs::ControlType::KUKA_RSI4ms;
    }
    else if (type=="KUKA_RSI12ms")
    {
        ctrType.control_type = kuka_msgs::ControlType::KUKA_RSI12ms;
    }
    else ctrType.control_type = kuka_msgs::ControlType::KUKA_UNKNOWN;

    return(ctrType);
}


double KukaRobotState::getCommandJointPositionKuka(int j)
{
    return(posROSToKuka(joint_position_command_[j], joint_types_[j]) );
}

double KukaRobotState::getCommandJointVelocityPercent(int j)
{
    return(velROSToKuka(joint_velocity_command_[j], joint_types_[j]) );
}


void KukaRobotState::addFeature(KukaCapability feature)
{
    features_.insert(feature);
}

bool KukaRobotState::hasFeature(KukaCapability feature)
{
    auto result = features_.find(feature);
    if (result == features_.end())
        return(false);
    else
        return(true);
}

KukaRobotState::KukaStatus KukaRobotState::getStatus() const
{
    return status_.stat;
}

void KukaRobotState::setStatus(const KukaRobotState::KukaStatus &status)
{
    status_previous_ = status_;
    status_.stat = status;
    setRosStatus(status_.stat);
}

void KukaRobotState::setStatus(const unsigned int &status)
{
    status_previous_ = status_;
    status_.code = status;
    setRosStatus(status_.stat);
}

void KukaRobotState::setStatusOn(const KukaStatusID id)
{
    status_previous_ = status_;
    status_.code |= id;
    setRosStatus(id);
}

void KukaRobotState::setStatusOff(const KukaStatusID id)
{
    status_previous_ = status_;
    status_.code &= ~id;
    setRosStatus(id);
}

bool KukaRobotState::statusChanged()
{
    bool chg = (status_.code != status_previous_.code);
    status_previous_ = status_;
    return(chg);
}

void KukaRobotState::setRosStatus(const KukaRobotState::KukaStatus &status)
{
    ros_status_.drives_powered = toTriState(status.DrivesPowered);
    ros_status_.e_stopped = toTriState(status.Estop || status.GuardStop);
    ros_status_.in_error = toTriState(status.InError);
    ros_status_.in_motion = toTriState(status.InMotion);
    ros_status_.motion_possible = toTriState(status.MotionPossible);
}

// Transfers the status of a single ID from status_ into ros_status_
// Note: always set internal status_ value before using this call
void KukaRobotState::setRosStatus(const KukaStatusID id)
{
    switch(id)
    {
    case DrivesPowered :
        ros_status_.drives_powered = toTriState(status_.stat.DrivesPowered);
        break;
    case Estop:
    case GuardStop:
        ros_status_.e_stopped = toTriState(status_.stat.Estop || status_.stat.GuardStop);
        break;
    case MotionPossible :
        ros_status_.motion_possible = toTriState(status_.stat.MotionPossible);
        break;
    case InMotion:
        ros_status_.in_motion = toTriState(status_.stat.InMotion);
        break;
    case InError:
        ros_status_.in_error = toTriState(status_.stat.InError);
        break;
    }
}

industrial_msgs::TriState KukaRobotState::toTriState(unsigned int val)
{
    industrial_msgs::TriState ts_val;
    switch (val)
    {
    case 0:
        ts_val.val = industrial_msgs::TriState::OFF;
        break;
    case 1:
        ts_val.val = industrial_msgs::TriState::ON;
        break;
    default:
        ts_val.val = industrial_msgs::TriState::UNKNOWN;
        break;
    }

    return(ts_val);
}


industrial_msgs::TriState KukaRobotState::toTriState(bool val)
{
    industrial_msgs::TriState ts_val;
    if (val)
        ts_val.val = industrial_msgs::TriState::ON;
    else
        ts_val.val = industrial_msgs::TriState::OFF;
    return(ts_val);
}


KukaRobotState::KukaOpMode KukaRobotState::getOpMode() const
{
    return op_mode_;
}

void KukaRobotState::setOpMode(const KukaRobotState::KukaOpMode op_mode)
{
    op_mode_ = op_mode;
    status_.stat.OpMode = (unsigned int)op_mode_;
    setRosOpMode(op_mode_);
}

void KukaRobotState::setOpMode(const int op_mode)
{
    if (op_mode < 1 || op_mode > 4)
        throw std::out_of_range("OpMode must be 1-4.");
    op_mode_ = (KukaRobotState::KukaOpMode)op_mode;
    status_.stat.OpMode = (unsigned int)op_mode_;
    setRosOpMode(op_mode_);
}

void KukaRobotState::setRosOpMode(const KukaRobotState::KukaOpMode op_mode)
{
    switch(op_mode)
    {
    case KukaOpMode::T1:
    case KukaOpMode::T2:
        ros_status_.mode.val = industrial_msgs::RobotMode::MANUAL;
        break;
    case KukaOpMode::Auto:
    case KukaOpMode::Ext:
        ros_status_.mode.val = industrial_msgs::RobotMode::AUTO;
        break;
    default:
        ros_status_.mode.val = industrial_msgs::RobotMode::UNKNOWN;
    }
}


void KukaRobotState::setErrorCode(const int code)
{
    ros_status_.error_code = code;
}



int KukaRobotState::getErrorCode()
{
    return(ros_status_.error_code);
}



} // namespace kuka_kss_hw_interface
