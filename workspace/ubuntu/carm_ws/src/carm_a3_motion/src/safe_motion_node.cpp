#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include "arm_control_sdk/carm_cobot.h"
#include "arm_control_sdk/data_type_def.h"
#include "carm_a3_motion/GetCartesianSnapshot.h"
#include "carm_a3_motion/GetExtendedState.h"
#include "carm_a3_motion/GetJointSnapshot.h"
#include "carm_a3_motion/GetToolInfo.h"
#include "carm_a3_motion/JogJoint.h"
#include "carm_a3_motion/MoveFlowPose.h"
#include "carm_a3_motion/MoveJoint.h"
#include "carm_a3_motion/MoveJointTrajectory.h"
#include "carm_a3_motion/MoveLineJoint.h"
#include "carm_a3_motion/MoveLinePose.h"
#include "carm_a3_motion/MovePose.h"
#include "carm_a3_motion/SetCollisionConfig.h"
#include "carm_a3_motion/SetControlMode.h"
#include "carm_a3_motion/SetGripper.h"
#include "carm_a3_motion/SetSpeed.h"
#include "carm_a3_motion/SetToolIndex.h"
#include "carm_a3_motion/SolveFK.h"
#include "carm_a3_motion/SolveFKArray.h"
#include "carm_a3_motion/SolveIK.h"
#include "carm_a3_motion/SolveIKArray.h"

namespace {

std::string statusToString(const carm::ArmStatus& status) {
    std::ostringstream ss;
    ss << "connected=" << (status.arm_is_connected ? "true" : "false")
       << ",servo=" << (status.servo_status ? "true" : "false")
       << ",state=" << status.state
       << ",fsm_state=" << status.fsm_state
       << ",speed_percentage=" << status.speed_percentage
       << ",debug=" << (status.on_debug_mode ? "true" : "false")
       << ",dof=" << status.arm_dof
       << ",name=" << status.arm_name;
    return ss.str();
}

std::string vectorToString(const std::vector<double>& values) {
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            ss << ", ";
        }
        ss << values[i];
    }
    ss << "]";
    return ss.str();
}

}  // namespace

class SafeMotionNode {
public:
    SafeMotionNode() : nh_(), pnh_("~") {
        loadParams();

        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        flange_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/maxhub_a3/flange_pose", 10);
        diagnostics_pub_ = nh_.advertise<std_msgs::String>("/maxhub_a3/diagnostics", 10, true);

        status_srv_ = nh_.advertiseService("/carm_a3/motion/status",
                                           &SafeMotionNode::handleStatus,
                                           this);
        set_ready_srv_ = nh_.advertiseService("/carm_a3/motion/set_ready",
                                              &SafeMotionNode::handleSetReady,
                                              this);
        set_servo_srv_ = nh_.advertiseService("/carm_a3/motion/set_servo_enable",
                                              &SafeMotionNode::handleSetServoEnable,
                                              this);
        emergency_stop_srv_ = nh_.advertiseService("/carm_a3/motion/emergency_stop",
                                                   &SafeMotionNode::handleEmergencyStop,
                                                   this);
        jog_joint_srv_ = nh_.advertiseService("/carm_a3/motion/jog_joint",
                                              &SafeMotionNode::handleJogJoint,
                                              this);
        move_joint_srv_ = nh_.advertiseService("/carm_a3/motion/move_joint",
                                               &SafeMotionNode::handleMoveJoint,
                                               this);
        move_joint_traj_srv_ = nh_.advertiseService("/carm_a3/motion/move_joint_trajectory",
                                                    &SafeMotionNode::handleMoveJointTrajectory,
                                                    this);
        move_pose_srv_ = nh_.advertiseService("/carm_a3/motion/move_pose",
                                              &SafeMotionNode::handleMovePose,
                                              this);
        move_line_joint_srv_ = nh_.advertiseService("/carm_a3/motion/move_line_joint",
                                                    &SafeMotionNode::handleMoveLineJoint,
                                                    this);
        move_line_pose_srv_ = nh_.advertiseService("/carm_a3/motion/move_line_pose",
                                                   &SafeMotionNode::handleMoveLinePose,
                                                   this);
        move_flow_pose_srv_ = nh_.advertiseService("/carm_a3/motion/move_flow_pose",
                                                   &SafeMotionNode::handleMoveFlowPose,
                                                   this);
        get_joint_snapshot_srv_ = nh_.advertiseService("/carm_a3/motion/get_joint_snapshot",
                                                       &SafeMotionNode::handleGetJointSnapshot,
                                                       this);
        get_cartesian_snapshot_srv_ = nh_.advertiseService("/carm_a3/motion/get_cartesian_snapshot",
                                                           &SafeMotionNode::handleGetCartesianSnapshot,
                                                           this);
        get_extended_state_srv_ = nh_.advertiseService("/carm_a3/motion/get_extended_state",
                                                       &SafeMotionNode::handleGetExtendedState,
                                                       this);
        get_tool_info_srv_ = nh_.advertiseService("/carm_a3/motion/get_tool_info",
                                                  &SafeMotionNode::handleGetToolInfo,
                                                  this);
        set_speed_srv_ = nh_.advertiseService("/carm_a3/motion/set_speed",
                                              &SafeMotionNode::handleSetSpeed,
                                              this);
        set_control_mode_srv_ = nh_.advertiseService("/carm_a3/motion/set_control_mode",
                                                     &SafeMotionNode::handleSetControlMode,
                                                     this);
        set_collision_config_srv_ = nh_.advertiseService("/carm_a3/motion/set_collision_config",
                                                         &SafeMotionNode::handleSetCollisionConfig,
                                                         this);
        set_tool_index_srv_ = nh_.advertiseService("/carm_a3/motion/set_tool_index",
                                                   &SafeMotionNode::handleSetToolIndex,
                                                   this);
        set_gripper_srv_ = nh_.advertiseService("/carm_a3/motion/set_gripper",
                                                &SafeMotionNode::handleSetGripper,
                                                this);
        solve_ik_srv_ = nh_.advertiseService("/carm_a3/motion/solve_ik",
                                             &SafeMotionNode::handleSolveIK,
                                             this);
        solve_fk_srv_ = nh_.advertiseService("/carm_a3/motion/solve_fk",
                                             &SafeMotionNode::handleSolveFK,
                                             this);
        solve_ik_array_srv_ = nh_.advertiseService("/carm_a3/motion/solve_ik_array",
                                                   &SafeMotionNode::handleSolveIKArray,
                                                   this);
        solve_fk_array_srv_ = nh_.advertiseService("/carm_a3/motion/solve_fk_array",
                                                   &SafeMotionNode::handleSolveFKArray,
                                                   this);

        ROS_WARN("carm_a3_motion started with allow_motion=%s, dry_run=%s",
                 allow_motion_ ? "true" : "false",
                 dry_run_ ? "true" : "false");
        ROS_WARN("set_ready gate=%s, servo gate=%s",
                 allow_ready_ ? "open" : "closed",
                 allow_servo_enable_ ? "open" : "closed");
        ROS_INFO("Target controller: %s:%d", robot_host_.c_str(), robot_port_);

        if (connect_on_start_) {
            connectSdk();
        }
        if (publish_state_) {
            const double period_s = state_publish_rate_hz_ > 0.0 ?
                                            1.0 / state_publish_rate_hz_ :
                                            0.2;
            state_timer_ = nh_.createTimer(ros::Duration(period_s),
                                           &SafeMotionNode::onStateTimer,
                                           this);
        }
    }

    ~SafeMotionNode() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (arm_ && arm_->is_connected()) {
            try {
                if (callbacks_registered_) {
                    arm_->release_joint_cbk();
                    arm_->release_pose_cbk();
                    arm_->release_error_cbk("safe_motion_error");
                    arm_->release_completion_cbk("safe_motion_completion");
                }
                arm_->disconnect();
            } catch (const std::exception& e) {
                ROS_WARN("disconnect threw: %s", e.what());
            } catch (...) {
                ROS_WARN("disconnect threw an unknown exception");
            }
        }
    }

private:
    void loadParams() {
        pnh_.param<std::string>("robot_host", robot_host_, "192.168.31.60");
        pnh_.param<int>("robot_port", robot_port_, 8090);
        pnh_.param<double>("connect_timeout_s", connect_timeout_s_, 1.0);
        pnh_.param<bool>("connect_on_start", connect_on_start_, true);
        pnh_.param<bool>("auto_ready_on_connect", auto_ready_on_connect_, false);
        pnh_.param<bool>("register_callbacks_on_connect", register_callbacks_on_connect_, false);
        pnh_.param<double>("pre_ready_delay_s", pre_ready_delay_s_, 1.0);
        pnh_.param<bool>("publish_state", publish_state_, true);
        pnh_.param<double>("state_publish_rate_hz", state_publish_rate_hz_, 5.0);
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<std::string>("joint_state_frame_id", joint_state_frame_id_, base_frame_id_);
        pnh_.param<std::string>("flange_frame_id", flange_frame_id_, "flange");
        pnh_.param<bool>("publish_flange_tf", publish_flange_tf_, false);
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        pnh_.getParam("joint_names", joint_names_);
        pnh_.param<bool>("publish_gripper_joints", publish_gripper_joints_, true);
        pnh_.param<std::string>("gripper_right_joint_name", gripper_right_joint_name_, "joint7");
        pnh_.param<std::string>("gripper_left_joint_name", gripper_left_joint_name_, "joint8");
        pnh_.param<double>("gripper_joint_scale", gripper_joint_scale_, 0.5);
        pnh_.param<double>("gripper_joint_max_m", gripper_joint_max_m_, 0.037);

        pnh_.param<bool>("allow_ready", allow_ready_, false);
        pnh_.param<bool>("allow_servo_enable", allow_servo_enable_, false);
        pnh_.param<bool>("allow_motion", allow_motion_, false);
        pnh_.param<bool>("allow_settings", allow_settings_, false);
        pnh_.param<bool>("allow_gripper", allow_gripper_, false);
        pnh_.param<bool>("dry_run", dry_run_, true);

        pnh_.param<bool>("require_connected", require_connected_, true);
        pnh_.param<bool>("require_servo_enabled", require_servo_enabled_, true);
        pnh_.param<bool>("require_position_mode", require_position_mode_, true);
        pnh_.param<bool>("reject_controller_error", reject_controller_error_, true);

        pnh_.param<int>("joint_count", joint_count_, 6);
        pnh_.param<double>("max_jog_delta_rad", max_jog_delta_rad_, 0.03);
        pnh_.param<double>("max_move_delta_rad", max_move_delta_rad_, 0.15);
        pnh_.param<double>("max_pose_position_delta_m", max_pose_position_delta_m_, 0.05);
        pnh_.param<double>("max_gripper_pos_m", max_gripper_pos_m_, 0.08);
        pnh_.param<double>("max_gripper_tau_n", max_gripper_tau_n_, 100.0);
        pnh_.param<double>("default_duration_s", default_duration_s_, 2.0);
        pnh_.param<double>("min_duration_s", min_duration_s_, 0.5);
        pnh_.param<double>("max_duration_s", max_duration_s_, 10.0);
        pnh_.param<bool>("use_duration", use_duration_, false);
        pnh_.param<bool>("wait_for_motion", wait_for_motion_, false);
        pnh_.param<double>("speed_level", speed_level_, 1.0);
        pnh_.param<int>("speed_response_level", speed_response_level_, 20);
        pnh_.param<bool>("set_speed_before_motion", set_speed_before_motion_, false);
        pnh_.param<bool>("verify_after_motion", verify_after_motion_, true);
        pnh_.param<double>("verify_timeout_s", verify_timeout_s_, 3.0);
        pnh_.param<double>("verify_poll_s", verify_poll_s_, 0.1);
        pnh_.param<double>("verify_joint_tolerance_rad", verify_joint_tolerance_rad_, 0.003);
        pnh_.param<double>("verify_pose_position_tolerance_m",
                           verify_pose_position_tolerance_m_,
                           0.005);
    }

    bool connectSdk(std::string* message = nullptr) {
        try {
            if (!arm_) {
                arm_.reset(new carm::CArmSingleCol(robot_host_, robot_port_, connect_timeout_s_));
            }
            if (!arm_->is_connected()) {
                const int ret = arm_->connect(robot_host_, robot_port_, connect_timeout_s_);
                if (message) {
                    std::ostringstream ss;
                    ss << "connect ret=" << ret;
                    *message = ss.str();
                }
            }
            if (!arm_->is_connected()) {
                return false;
            }
            publishDiagnostics("connected=true");
            initializeSdkSessionLocked();
            return true;
        } catch (const std::exception& e) {
            if (message) {
                *message = std::string("connect exception: ") + e.what();
            }
            ROS_ERROR("SDK connect failed: %s", e.what());
            return false;
        } catch (...) {
            if (message) {
                *message = "connect exception: unknown";
            }
            ROS_ERROR("SDK connect failed: unknown exception");
            return false;
        }
    }

    void initializeSdkSessionLocked() {
        if (sdk_session_initialized_ || !arm_ || !arm_->is_connected()) {
            return;
        }

        if (pre_ready_delay_s_ > 0.0) {
            ROS_INFO("SDK session init: sleeping %.3f s before optional set_ready",
                     pre_ready_delay_s_);
            ros::Duration(pre_ready_delay_s_).sleep();
        }

        if (auto_ready_on_connect_) {
            const int ret = arm_->set_ready();
            ready_called_on_connect_ = ret >= 1;
            ROS_WARN("SDK session init: set_ready ret=%d", ret);
        } else {
            ROS_WARN("SDK session init: auto_ready_on_connect=false; not calling set_ready()");
        }

        if (register_callbacks_on_connect_) {
            registerSdkCallbacksLocked();
        } else {
            ROS_WARN("SDK session init: register_callbacks_on_connect=false; not registering callbacks");
        }

        sdk_session_initialized_ = true;
    }

    void registerSdkCallbacksLocked() {
        if (callbacks_registered_) {
            return;
        }
        arm_->register_joint_cbk(std::bind(&SafeMotionNode::handleJointCallback,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2,
                                           std::placeholders::_3,
                                           std::placeholders::_4));
        arm_->register_pose_cbk(std::bind(&SafeMotionNode::handlePoseCallback,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
        arm_->register_error_cbk("safe_motion_error",
                                 std::bind(&SafeMotionNode::handleErrorCallback,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
        arm_->register_completion_cbk("safe_motion_completion",
                                      std::bind(&SafeMotionNode::handleCompletionCallback,
                                                this,
                                                std::placeholders::_1));
        callbacks_registered_ = true;
        ROS_WARN("SDK session init: registered joint/pose/error/completion callbacks");
    }

    bool ensureConnected(std::string* message) {
        if (arm_ && arm_->is_connected()) {
            return true;
        }
        if (!connectSdk(message)) {
            if (message && message->empty()) {
                *message = "not connected";
            }
            return false;
        }
        return true;
    }

    carm::ArmStatus getStatusLocked() {
        if (!arm_) {
            throw std::runtime_error("SDK object is not initialized");
        }
        return arm_->get_status();
    }

    bool checkMotionPreconditions(const carm::ArmStatus& status, std::string* message) const {
        if (require_connected_ && !status.arm_is_connected) {
            *message = "controller is not connected";
            return false;
        }
        if (reject_controller_error_ && status.state < 0) {
            *message = "controller state is error";
            return false;
        }
        if (require_servo_enabled_ && !status.servo_status) {
            *message = "servo is not enabled; this node will not enable it automatically";
            return false;
        }
        if (require_position_mode_ && status.fsm_state != 1) {
            *message = "controller is not in POSITION mode; this node will not change mode automatically";
            return false;
        }
        return true;
    }

    bool normalizeDuration(double requested, double* duration, std::string* message) const {
        *duration = requested > 0.0 ? requested : default_duration_s_;
        if (*duration < min_duration_s_ || *duration > max_duration_s_) {
            std::ostringstream ss;
            ss << "duration_s out of range [" << min_duration_s_ << ", " << max_duration_s_ << "]";
            *message = ss.str();
            return false;
        }
        return true;
    }

    double maxJointError(const std::vector<double>& actual,
                         const std::vector<double>& target) const {
        double max_error = 0.0;
        const size_t count = std::min(actual.size(), target.size());
        for (size_t i = 0; i < count; ++i) {
            max_error = std::max(max_error, std::abs(actual[i] - target[i]));
        }
        return max_error;
    }

    double maxPositionDelta(const std::array<double, 7>& a,
                            const std::array<double, 7>& b) const {
        double max_delta = 0.0;
        for (size_t i = 0; i < 3U; ++i) {
            max_delta = std::max(max_delta, std::abs(a[i] - b[i]));
        }
        return max_delta;
    }

    bool checkPoseDeltaLocked(const std::array<double, 7>& target,
                              const std::string& label,
                              std::string* message) {
        const std::array<double, 7> current = arm_->get_cart_pose();
        const double delta = maxPositionDelta(current, target);
        if (delta > max_pose_position_delta_m_) {
            std::ostringstream ss;
            ss << label << " position delta " << delta
               << " exceeds max_pose_position_delta_m=" << max_pose_position_delta_m_;
            *message = ss.str();
            return false;
        }
        return true;
    }

    bool waitForPoseTargetLocked(const std::array<double, 7>& target,
                                 std::array<double, 7>* actual,
                                 double* max_error,
                                 std::string* message) {
        if (!verify_after_motion_) {
            *actual = arm_->get_cart_pose();
            *max_error = maxPositionDelta(*actual, target);
            *message = "verification disabled";
            return true;
        }
        const ros::Time deadline = ros::Time::now() + ros::Duration(verify_timeout_s_);
        const ros::Duration poll_duration(verify_poll_s_);
        while (ros::ok()) {
            *actual = arm_->get_cart_pose();
            *max_error = maxPositionDelta(*actual, target);
            if (*max_error <= verify_pose_position_tolerance_m_) {
                *message = "pose target verified";
                return true;
            }
            if (ros::Time::now() >= deadline) {
                std::ostringstream ss;
                ss << "pose verification timeout, max_position_error=" << *max_error
                   << ", tolerance=" << verify_pose_position_tolerance_m_;
                *message = ss.str();
                return false;
            }
            poll_duration.sleep();
        }
        *message = "ROS shutdown during pose verification";
        return false;
    }

    bool waitForJointTargetLocked(const std::vector<double>& target,
                                  std::vector<double>* actual,
                                  double* max_error,
                                  std::string* message) {
        if (!verify_after_motion_) {
            *actual = arm_->get_joint_pos();
            *max_error = maxJointError(*actual, target);
            *message = "verification disabled";
            return true;
        }

        const ros::Time deadline = ros::Time::now() + ros::Duration(verify_timeout_s_);
        ros::Duration poll_duration(std::max(0.01, verify_poll_s_));
        while (ros::ok()) {
            *actual = arm_->get_joint_pos();
            if (static_cast<int>(actual->size()) < joint_count_) {
                *message = "SDK returned too few joints during verification: " +
                           std::to_string(actual->size());
                return false;
            }
            actual->resize(static_cast<size_t>(joint_count_));
            *max_error = maxJointError(*actual, target);
            if (*max_error <= verify_joint_tolerance_rad_) {
                *message = "target verified";
                return true;
            }
            if (ros::Time::now() >= deadline) {
                std::ostringstream ss;
                ss << "verification timeout, max_error=" << *max_error
                   << ", tolerance=" << verify_joint_tolerance_rad_;
                *message = ss.str();
                return false;
            }
            poll_duration.sleep();
        }
        *message = "ROS shutdown during verification";
        return false;
    }

    bool vectorToPoseArray(const std::vector<double>& values,
                           std::array<double, 7>* pose,
                           std::string* message) const {
        if (values.size() != 7) {
            *message = "pose must contain exactly 7 values: x,y,z,qx,qy,qz,qw";
            return false;
        }
        std::copy(values.begin(), values.end(), pose->begin());
        return true;
    }

    std::vector<double> poseArrayToVector(const std::array<double, 7>& pose) const {
        return std::vector<double>(pose.begin(), pose.end());
    }

    std::vector<double> trimToJointNameCount(const std::vector<double>& values) const {
        const size_t n = std::min(values.size(), joint_names_.size());
        return std::vector<double>(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(n));
    }

    std::vector<std::string> trimJointNames(const size_t value_count) const {
        const size_t n = std::min(value_count, joint_names_.size());
        return std::vector<std::string>(joint_names_.begin(),
                                        joint_names_.begin() + static_cast<std::ptrdiff_t>(n));
    }

    void publishDiagnostics(const std::string& text) {
        if (!diagnostics_pub_) {
            return;
        }
        std_msgs::String msg;
        msg.data = text;
        diagnostics_pub_.publish(msg);
    }

    void onStateTimer(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            publishDiagnostics(message);
            return;
        }
        try {
            publishArmStatusLocked();
            publishJointStateLocked();
            publishFlangePoseLocked();
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(2.0, "state publishing failed: %s", e.what());
            publishDiagnostics(std::string("state_publish_failed: ") + e.what());
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "state publishing failed: unknown exception");
            publishDiagnostics("state_publish_failed: unknown exception");
        }
    }

    void publishArmStatusLocked() {
        const carm::ArmStatus status = getStatusLocked();
        publishDiagnostics(statusToString(status));
    }

    void publishJointStateLocked() {
        const std::vector<double> positions = arm_->get_joint_pos();
        if (positions.empty()) {
            ROS_WARN_THROTTLE(2.0, "SDK returned empty joint positions");
            return;
        }
        const std::vector<double> velocities = arm_->get_joint_vel();
        const std::vector<double> efforts = arm_->get_joint_tau();

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = joint_state_frame_id_;
        msg.name = trimJointNames(positions.size());
        msg.position = trimToJointNameCount(positions);
        msg.velocity = trimToJointNameCount(velocities);
        msg.effort = trimToJointNameCount(efforts);
        appendGripperJointStateLocked(&msg);
        joint_pub_.publish(msg);
    }

    void appendGripperJointStateLocked(sensor_msgs::JointState* msg) {
        if (!publish_gripper_joints_ || msg == nullptr) {
            return;
        }

        double gripper_pos = 0.0;
        double gripper_vel = 0.0;
        double gripper_tau = 0.0;
        try {
            gripper_pos = arm_->get_gripper_pos();
            gripper_vel = arm_->get_gripper_vel();
            gripper_tau = arm_->get_gripper_tau();
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(2.0, "gripper joint state read failed: %s", e.what());
            return;
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "gripper joint state read failed: unknown exception");
            return;
        }

        const double scaled = gripper_pos * gripper_joint_scale_;
        const double joint_pos = std::max(0.0, std::min(gripper_joint_max_m_, scaled));
        const double joint_vel = gripper_vel * gripper_joint_scale_;

        msg->name.push_back(gripper_right_joint_name_);
        msg->name.push_back(gripper_left_joint_name_);
        msg->position.push_back(joint_pos);
        msg->position.push_back(joint_pos);
        if (!msg->velocity.empty()) {
            msg->velocity.push_back(joint_vel);
            msg->velocity.push_back(joint_vel);
        }
        if (!msg->effort.empty()) {
            msg->effort.push_back(gripper_tau);
            msg->effort.push_back(gripper_tau);
        }
    }

    void publishFlangePoseLocked() {
        const std::array<double, 7> pose = arm_->get_cart_pose();

        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = base_frame_id_;
        msg.pose.position.x = pose[0];
        msg.pose.position.y = pose[1];
        msg.pose.position.z = pose[2];
        msg.pose.orientation.x = pose[3];
        msg.pose.orientation.y = pose[4];
        msg.pose.orientation.z = pose[5];
        msg.pose.orientation.w = pose[6];
        flange_pose_pub_.publish(msg);

        if (publish_flange_tf_) {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header = msg.header;
            tf_msg.child_frame_id = flange_frame_id_;
            tf_msg.transform.translation.x = pose[0];
            tf_msg.transform.translation.y = pose[1];
            tf_msg.transform.translation.z = pose[2];
            tf_msg.transform.rotation = msg.pose.orientation;
            tf_broadcaster_.sendTransform(tf_msg);
        }
    }

    std::vector<double> flattenJointArray(const std::vector<std::vector<double>>& points,
                                          size_t point_count) const {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        std::vector<double> values(point_count * static_cast<size_t>(joint_count_), nan);
        for (size_t i = 0; i < std::min(point_count, points.size()); ++i) {
            const size_t count = std::min(points[i].size(), static_cast<size_t>(joint_count_));
            std::copy(points[i].begin(),
                      points[i].begin() + static_cast<std::ptrdiff_t>(count),
                      values.begin() +
                              static_cast<std::ptrdiff_t>(i * static_cast<size_t>(joint_count_)));
        }
        return values;
    }

    std::vector<double> flattenPoseArray(const std::vector<std::array<double, 7>>& points,
                                         size_t point_count) const {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        std::vector<double> values(point_count * 7U, nan);
        for (size_t i = 0; i < std::min(point_count, points.size()); ++i) {
            std::copy(points[i].begin(),
                      points[i].end(),
                      values.begin() + static_cast<std::ptrdiff_t>(i * 7U));
        }
        return values;
    }

    bool handleStatus(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            const carm::ArmStatus status = getStatusLocked();
            res.success = true;
            std::ostringstream ss;
            ss << statusToString(status)
               << ",sdk_initialized=" << (sdk_session_initialized_ ? "true" : "false")
               << ",ready_on_connect=" << (ready_called_on_connect_ ? "true" : "false")
               << ",callbacks=" << (callbacks_registered_ ? "true" : "false")
               << ",last_joint_count=" << last_joint_count_.load();
            res.message = ss.str();
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleGetJointSnapshot(carm_a3_motion::GetJointSnapshot::Request&,
                                carm_a3_motion::GetJointSnapshot::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            res.positions = arm_->get_joint_pos();
            res.velocities = arm_->get_joint_vel();
            res.efforts = arm_->get_joint_tau();
            if (static_cast<int>(res.positions.size()) < joint_count_) {
                res.success = false;
                res.message = "SDK returned too few joints: " +
                              std::to_string(res.positions.size());
                return true;
            }
            res.positions.resize(static_cast<size_t>(joint_count_));
            res.velocities.resize(std::min(res.velocities.size(), static_cast<size_t>(joint_count_)));
            res.efforts.resize(std::min(res.efforts.size(), static_cast<size_t>(joint_count_)));
            res.success = true;
            res.message = "joint snapshot ok";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleGetCartesianSnapshot(carm_a3_motion::GetCartesianSnapshot::Request&,
                                    carm_a3_motion::GetCartesianSnapshot::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            res.pose = poseArrayToVector(arm_->get_cart_pose());
            res.plan_pose = poseArrayToVector(arm_->get_plan_cart_pose());
            res.success = true;
            res.message = "cartesian snapshot ok, pose order=x,y,z,qx,qy,qz,qw";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleGetExtendedState(carm_a3_motion::GetExtendedState::Request&,
                                carm_a3_motion::GetExtendedState::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            const carm::ArmConfig config = arm_->get_config();
            res.version = arm_->get_version();
            res.dof = config.dof;
            res.limit_upper = config.limit_upper;
            res.limit_lower = config.limit_lower;
            res.joint_vel_limits = config.joint_vel;
            res.joint_acc_limits = config.joint_acc;
            res.joint_dec_limits = config.joint_dec;
            res.joint_jerk_limits = config.joint_jerk;
            res.plan_positions = arm_->get_plan_joint_pos();
            res.plan_velocities = arm_->get_plan_joint_vel();
            res.plan_efforts = arm_->get_plan_joint_tau();
            res.joint_external_tau = arm_->get_joint_external_tau();
            res.cart_external_force = arm_->get_cart_external_force();
            res.gripper_state = arm_->get_gripper_state();
            res.gripper_pos = arm_->get_gripper_pos();
            res.gripper_vel = arm_->get_gripper_vel();
            res.gripper_tau = arm_->get_gripper_tau();
            res.plan_gripper_pos = arm_->get_plan_gripper_pos();
            res.plan_gripper_tau = arm_->get_plan_gripper_tau();
            res.tool_index = arm_->get_tool_index();
            res.success = true;
            res.message = "extended SDK state ok";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleGetToolInfo(carm_a3_motion::GetToolInfo::Request& req,
                           carm_a3_motion::GetToolInfo::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            res.current_index = arm_->get_tool_index();
            res.coordinate = poseArrayToVector(arm_->get_tool_coordinate(req.index));
            res.success = true;
            res.message = "tool coordinate order=x,y,z,qx,qy,qz,qw";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleSolveIK(carm_a3_motion::SolveIK::Request& req,
                       carm_a3_motion::SolveIK::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        std::array<double, 7> pose{};
        if (!vectorToPoseArray(req.pose, &pose, &message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            std::vector<double> seed = req.seed_positions;
            if (seed.empty()) {
                seed = arm_->get_joint_pos();
            }
            if (static_cast<int>(seed.size()) < joint_count_) {
                res.success = false;
                res.message = "seed_positions must be empty or contain at least " +
                              std::to_string(joint_count_) + " values";
                return true;
            }
            seed.resize(static_cast<size_t>(joint_count_));
            std::vector<double> solution;
            const int ret = arm_->inverse_kine(req.tool_index, pose, seed, solution);
            res.success = ret >= 1 && static_cast<int>(solution.size()) >= joint_count_;
            if (static_cast<int>(solution.size()) >= joint_count_) {
                solution.resize(static_cast<size_t>(joint_count_));
            }
            res.positions = solution;
            res.message = "inverse_kine ret=" + std::to_string(ret) +
                          ", seed=" + vectorToString(seed) +
                          ", solution=" + vectorToString(solution);
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleSolveFK(carm_a3_motion::SolveFK::Request& req,
                       carm_a3_motion::SolveFK::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        if (static_cast<int>(req.positions.size()) != joint_count_) {
            res.success = false;
            res.message = "positions must contain exactly " + std::to_string(joint_count_) + " values";
            return true;
        }
        try {
            std::array<double, 7> pose{};
            const int ret = arm_->forward_kine(req.tool_index, req.positions, pose);
            res.pose = poseArrayToVector(pose);
            res.success = ret >= 1;
            res.message = "forward_kine ret=" + std::to_string(ret) +
                          ", pose=[x,y,z,qx,qy,qz,qw]";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleSolveIKArray(carm_a3_motion::SolveIKArray::Request& req,
                            carm_a3_motion::SolveIKArray::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        if (req.poses.empty() || req.poses.size() % 7U != 0U) {
            res.success = false;
            res.message = "poses must contain one or more 7-value poses";
            return true;
        }

        try {
            const size_t point_count = req.poses.size() / 7U;
            std::vector<std::array<double, 7>> poses;
            poses.reserve(point_count);
            for (size_t i = 0; i < point_count; ++i) {
                std::array<double, 7> pose{};
                std::copy(req.poses.begin() + static_cast<std::ptrdiff_t>(i * 7U),
                          req.poses.begin() + static_cast<std::ptrdiff_t>((i + 1U) * 7U),
                          pose.begin());
                poses.push_back(pose);
            }

            std::vector<std::vector<double>> seeds;
            seeds.reserve(point_count);
            if (req.seed_positions.empty()) {
                std::vector<double> seed = arm_->get_joint_pos();
                if (static_cast<int>(seed.size()) < joint_count_) {
                    res.success = false;
                    res.message = "SDK returned too few joints for default seed: " +
                                  std::to_string(seed.size());
                    return true;
                }
                seed.resize(static_cast<size_t>(joint_count_));
                for (size_t i = 0; i < point_count; ++i) {
                    seeds.push_back(seed);
                }
            } else {
                if (req.seed_positions.size() != point_count * static_cast<size_t>(joint_count_)) {
                    res.success = false;
                    res.message = "seed_positions must be empty or contain point_count * " +
                                  std::to_string(joint_count_) + " values";
                    return true;
                }
                for (size_t i = 0; i < point_count; ++i) {
                    const auto begin = req.seed_positions.begin() +
                                       static_cast<std::ptrdiff_t>(i * static_cast<size_t>(joint_count_));
                    seeds.emplace_back(begin, begin + joint_count_);
                }
            }

            std::vector<std::vector<double>> solutions;
            const int ret = arm_->inverse_kine_array(req.tool_index, poses, seeds, solutions);
            res.point_success.assign(point_count, false);
            bool all_success = ret >= 1 && solutions.size() == point_count;
            for (size_t i = 0; i < std::min(point_count, solutions.size()); ++i) {
                res.point_success[i] = static_cast<int>(solutions[i].size()) >= joint_count_;
                all_success = all_success && res.point_success[i];
                if (static_cast<int>(solutions[i].size()) > joint_count_) {
                    solutions[i].resize(static_cast<size_t>(joint_count_));
                }
            }
            res.positions = flattenJointArray(solutions, point_count);
            res.success = all_success;
            res.message = "inverse_kine_array ret=" + std::to_string(ret) +
                          ", points=" + std::to_string(point_count) +
                          ", solved=" + std::to_string(solutions.size()) +
                          ", positions layout=point_count*joint_count, failed points contain NaN";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleSolveFKArray(carm_a3_motion::SolveFKArray::Request& req,
                            carm_a3_motion::SolveFKArray::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        if (req.positions.empty() ||
            req.positions.size() % static_cast<size_t>(joint_count_) != 0U) {
            res.success = false;
            res.message = "positions must contain one or more " +
                          std::to_string(joint_count_) + "-value joint points";
            return true;
        }

        try {
            const size_t point_count = req.positions.size() / static_cast<size_t>(joint_count_);
            std::vector<std::vector<double>> joints;
            joints.reserve(point_count);
            for (size_t i = 0; i < point_count; ++i) {
                const auto begin = req.positions.begin() +
                                   static_cast<std::ptrdiff_t>(i * static_cast<size_t>(joint_count_));
                joints.emplace_back(begin, begin + joint_count_);
            }

            std::vector<std::array<double, 7>> poses;
            const int ret = arm_->forward_kine_array(req.tool_index, joints, poses);
            res.point_success.assign(point_count, false);
            bool all_success = ret >= 1 && poses.size() == point_count;
            for (size_t i = 0; i < std::min(point_count, poses.size()); ++i) {
                res.point_success[i] = true;
            }
            res.poses = flattenPoseArray(poses, point_count);
            res.success = all_success;
            res.message = "forward_kine_array ret=" + std::to_string(ret) +
                          ", points=" + std::to_string(point_count) +
                          ", solved=" + std::to_string(poses.size()) +
                          ", pose order=x,y,z,qx,qy,qz,qw, failed points contain NaN";
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleSetSpeed(carm_a3_motion::SetSpeed::Request& req,
                        carm_a3_motion::SetSpeed::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_settings_) {
            res.success = false;
            res.message = "blocked: allow_settings is false";
            return true;
        }
        if (req.level < 0.0 || req.level > 10.0 ||
            req.response_level < 1 || req.response_level > 100) {
            res.success = false;
            res.message = "level must be [0,10], response_level must be [1,100]";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = dry_run_ ? 1 : arm_->set_speed_level(req.level, req.response_level);
        res.success = ret >= 1;
        res.message = std::string(dry_run_ ? "dry_run " : "") +
                      "set_speed_level ret=" + std::to_string(ret);
        return true;
    }

    bool handleSetControlMode(carm_a3_motion::SetControlMode::Request& req,
                              carm_a3_motion::SetControlMode::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_settings_) {
            res.success = false;
            res.message = "blocked: allow_settings is false";
            return true;
        }
        if (req.mode < 0 || req.mode > 4) {
            res.success = false;
            res.message = "mode must be in [0,4]";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = dry_run_ ? 1 : arm_->set_control_mode(req.mode);
        res.success = ret >= 1;
        res.message = std::string(dry_run_ ? "dry_run " : "") +
                      "set_control_mode ret=" + std::to_string(ret);
        return true;
    }

    bool handleSetCollisionConfig(carm_a3_motion::SetCollisionConfig::Request& req,
                                  carm_a3_motion::SetCollisionConfig::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_settings_) {
            res.success = false;
            res.message = "blocked: allow_settings is false";
            return true;
        }
        if (req.sensitivity_level < 0 || req.sensitivity_level > 2) {
            res.success = false;
            res.message = "sensitivity_level must be in [0,2]";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = dry_run_ ? 1 : arm_->set_collision_config(req.enable, req.sensitivity_level);
        res.success = ret >= 1;
        res.message = std::string(dry_run_ ? "dry_run " : "") +
                      "set_collision_config ret=" + std::to_string(ret);
        return true;
    }

    bool handleSetToolIndex(carm_a3_motion::SetToolIndex::Request& req,
                            carm_a3_motion::SetToolIndex::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_settings_) {
            res.success = false;
            res.message = "blocked: allow_settings is false";
            return true;
        }
        if (req.index < 0) {
            res.success = false;
            res.message = "index must be >= 0";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = dry_run_ ? 1 : arm_->set_tool_index(req.index);
        res.success = ret >= 1;
        res.message = std::string(dry_run_ ? "dry_run " : "") +
                      "set_tool_index ret=" + std::to_string(ret);
        return true;
    }

    bool handleSetGripper(carm_a3_motion::SetGripper::Request& req,
                          carm_a3_motion::SetGripper::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_ || !allow_gripper_) {
            res.success = false;
            res.message = "blocked: allow_motion or allow_gripper is false";
            return true;
        }
        if (req.pos < 0.0 || req.pos > max_gripper_pos_m_ ||
            req.tau < 0.0 || req.tau > max_gripper_tau_n_) {
            res.success = false;
            res.message = "gripper pos/tau out of configured range";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = dry_run_ ? 1 : arm_->set_gripper(req.pos, req.tau);
        res.success = ret >= 1;
        res.message = std::string(dry_run_ ? "dry_run " : "") +
                      "set_gripper ret=" + std::to_string(ret);
        return true;
    }

    bool handleSetReady(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_ready_) {
            res.success = false;
            res.message = "blocked: allow_ready is false";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = arm_->set_ready();
        ready_called_on_connect_ = ret >= 1;
        res.success = ret >= 1;
        res.message = "set_ready ret=" + std::to_string(ret);
        return true;
    }

    bool handleSetServoEnable(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_servo_enable_) {
            res.success = false;
            res.message = "blocked: allow_servo_enable is false";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = arm_->set_servo_enable(req.data);
        res.success = ret >= 1;
        res.message = std::string("set_servo_enable(") + (req.data ? "true" : "false") +
                      ") ret=" + std::to_string(ret);
        return true;
    }

    void handleJointCallback(double stamp,
                             std::vector<double> positions,
                             std::vector<double>,
                             std::vector<double>) {
        last_joint_stamp_.store(stamp);
        last_joint_count_.store(positions.size());
    }

    void handlePoseCallback(double stamp, std::array<double, 7>) {
        last_pose_stamp_.store(stamp);
    }

    void handleErrorCallback(int code, const std::string message) {
        ROS_ERROR("carm_a3_motion SDK error callback code=%d message=%s",
                  code,
                  message.c_str());
    }

    void handleCompletionCallback(const std::string task_key) {
        ROS_WARN("carm_a3_motion SDK completion callback task_key=%s",
                 task_key.c_str());
    }

    bool handleEmergencyStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        const int ret = arm_->emergency_stop();
        res.success = ret >= 1;
        res.message = "emergency_stop ret=" + std::to_string(ret);
        return true;
    }

    bool handleJogJoint(carm_a3_motion::JogJoint::Request& req,
                        carm_a3_motion::JogJoint::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        if (req.joint_index < 1 || req.joint_index > joint_count_) {
            res.success = false;
            res.message = "joint_index must be in [1, " + std::to_string(joint_count_) + "]";
            return true;
        }
        if (std::abs(req.delta_rad) > max_jog_delta_rad_) {
            res.success = false;
            res.message = "abs(delta_rad) exceeds max_jog_delta_rad";
            return true;
        }
        double duration = 0.0;
        if (!normalizeDuration(req.duration_s, &duration, &res.message)) {
            res.success = false;
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }

        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }

            ROS_INFO("jog_joint: reading current joint positions");
            std::vector<double> current = arm_->get_joint_pos();
            if (static_cast<int>(current.size()) < joint_count_) {
                res.success = false;
                res.message = "SDK returned too few joints: " + std::to_string(current.size());
                return true;
            }
            std::vector<double> target(current.begin(), current.begin() + joint_count_);
            target[static_cast<size_t>(req.joint_index - 1)] += req.delta_rad;

            if (dry_run_) {
                res.success = true;
                res.message = "dry_run jog accepted: current=" + vectorToString(current) +
                              ", target=" + vectorToString(target);
                return true;
            }

            int speed_ret = 0;
            if (set_speed_before_motion_) {
                speed_ret = arm_->set_speed_level(speed_level_, speed_response_level_);
            }
            const double sdk_duration = use_duration_ ? duration : -1.0;
            ROS_INFO("jog_joint: calling move_joint duration=%.3f wait=%s target=%s",
                     sdk_duration,
                     wait_for_motion_ ? "true" : "false",
                     vectorToString(target).c_str());
            const int move_ret = arm_->move_joint(target, sdk_duration, wait_for_motion_);
            ROS_INFO("jog_joint: move_joint returned %d", move_ret);
            std::vector<double> actual;
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForJointTargetLocked(target,
                                                           &actual,
                                                           &max_error,
                                                           &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "jog_joint speed_ret=" + std::to_string(speed_ret) +
                          ", move_ret=" + std::to_string(move_ret) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_error=" + std::to_string(max_error) +
                          ", target=" + vectorToString(target) +
                          ", actual=" + vectorToString(actual);
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleMoveJoint(carm_a3_motion::MoveJoint::Request& req,
                         carm_a3_motion::MoveJoint::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        if (static_cast<int>(req.positions.size()) != joint_count_) {
            res.success = false;
            res.message = "positions must contain exactly " + std::to_string(joint_count_) + " values";
            return true;
        }
        double duration = 0.0;
        if (!normalizeDuration(req.duration_s, &duration, &res.message)) {
            res.success = false;
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }

        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }

            ROS_INFO("move_joint: reading current joint positions");
            const std::vector<double> current = arm_->get_joint_pos();
            if (static_cast<int>(current.size()) < joint_count_) {
                res.success = false;
                res.message = "SDK returned too few joints: " + std::to_string(current.size());
                return true;
            }
            for (int i = 0; i < joint_count_; ++i) {
                const double delta = std::abs(req.positions[static_cast<size_t>(i)] -
                                              current[static_cast<size_t>(i)]);
                if (delta > max_move_delta_rad_) {
                    res.success = false;
                    res.message = "joint " + std::to_string(i + 1) +
                                  " delta exceeds max_move_delta_rad";
                    return true;
                }
            }

            if (dry_run_) {
                res.success = true;
                res.message = "dry_run move accepted: current=" + vectorToString(current) +
                              ", target=" + vectorToString(req.positions);
                return true;
            }

            int speed_ret = 0;
            if (set_speed_before_motion_) {
                speed_ret = arm_->set_speed_level(speed_level_, speed_response_level_);
            }
            const double sdk_duration = use_duration_ ? duration : -1.0;
            const bool wait = wait_for_motion_ && req.wait;
            ROS_INFO("move_joint: calling move_joint duration=%.3f wait=%s target=%s",
                     sdk_duration,
                     wait ? "true" : "false",
                     vectorToString(req.positions).c_str());
            const int move_ret = arm_->move_joint(req.positions, sdk_duration, wait);
            ROS_INFO("move_joint: move_joint returned %d", move_ret);
            std::vector<double> actual;
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForJointTargetLocked(req.positions,
                                                           &actual,
                                                           &max_error,
                                                           &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "move_joint speed_ret=" + std::to_string(speed_ret) +
                          ", move_ret=" + std::to_string(move_ret) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_error=" + std::to_string(max_error) +
                          ", target=" + vectorToString(req.positions) +
                          ", actual=" + vectorToString(actual);
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleMoveJointTrajectory(carm_a3_motion::MoveJointTrajectory::Request& req,
                                   carm_a3_motion::MoveJointTrajectory::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        if (req.point_count <= 0) {
            res.success = false;
            res.message = "point_count must be > 0";
            return true;
        }
        if (static_cast<int>(req.positions_flat.size()) != req.point_count * joint_count_) {
            res.success = false;
            res.message = "positions_flat size must equal point_count * joint_count";
            return true;
        }
        if (!req.stamps.empty() && static_cast<int>(req.stamps.size()) != req.point_count) {
            res.success = false;
            res.message = "stamps must be empty or contain point_count values";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }

        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }

            const std::vector<double> current = arm_->get_joint_pos();
            if (static_cast<int>(current.size()) < joint_count_) {
                res.success = false;
                res.message = "SDK returned too few joints: " + std::to_string(current.size());
                return true;
            }

            std::vector<std::vector<double>> trajectory;
            trajectory.reserve(static_cast<size_t>(req.point_count));
            for (int point = 0; point < req.point_count; ++point) {
                std::vector<double> target;
                target.reserve(static_cast<size_t>(joint_count_));
                for (int joint = 0; joint < joint_count_; ++joint) {
                    target.push_back(req.positions_flat[static_cast<size_t>(point * joint_count_ + joint)]);
                }
                trajectory.push_back(target);
            }

            std::vector<double> previous(current.begin(), current.begin() + joint_count_);
            for (size_t point = 0; point < trajectory.size(); ++point) {
                for (int joint = 0; joint < joint_count_; ++joint) {
                    const double delta = std::abs(trajectory[point][static_cast<size_t>(joint)] -
                                                  previous[static_cast<size_t>(joint)]);
                    if (delta > max_move_delta_rad_) {
                        res.success = false;
                        res.message = "trajectory point " + std::to_string(point + 1) +
                                      " joint " + std::to_string(joint + 1) +
                                      " delta exceeds max_move_delta_rad";
                        return true;
                    }
                }
                previous = trajectory[point];
            }

            if (dry_run_) {
                res.success = true;
                res.message = "dry_run trajectory accepted: points=" + std::to_string(req.point_count);
                return true;
            }

            int speed_ret = 0;
            if (set_speed_before_motion_) {
                speed_ret = arm_->set_speed_level(speed_level_, speed_response_level_);
            }
            const bool wait = wait_for_motion_ && req.wait;
            ROS_INFO("move_joint_trajectory: calling move_joint_traj points=%d wait=%s",
                     req.point_count,
                     wait ? "true" : "false");
            const int move_ret = arm_->move_joint_traj(trajectory, {}, req.stamps, wait);
            ROS_INFO("move_joint_trajectory: move_joint_traj returned %d", move_ret);

            const std::vector<double>& final_target = trajectory.back();
            std::vector<double> actual;
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForJointTargetLocked(final_target,
                                                           &actual,
                                                           &max_error,
                                                           &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "move_joint_trajectory speed_ret=" + std::to_string(speed_ret) +
                          ", move_ret=" + std::to_string(move_ret) +
                          ", points=" + std::to_string(req.point_count) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_error=" + std::to_string(max_error) +
                          ", final_target=" + vectorToString(final_target) +
                          ", actual=" + vectorToString(actual);
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleMovePose(carm_a3_motion::MovePose::Request& req,
                        carm_a3_motion::MovePose::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        std::array<double, 7> target{};
        if (!vectorToPoseArray(req.pose, &target, &res.message)) {
            res.success = false;
            return true;
        }
        double duration = 0.0;
        if (!normalizeDuration(req.duration_s, &duration, &res.message)) {
            res.success = false;
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }
            if (!checkPoseDeltaLocked(target, "move_pose", &message)) {
                res.success = false;
                res.message = message;
                return true;
            }
            if (dry_run_) {
                res.success = true;
                res.message = "dry_run move_pose accepted: target=" + vectorToString(poseArrayToVector(target));
                return true;
            }
            const double sdk_duration = use_duration_ ? duration : -1.0;
            const bool wait = wait_for_motion_ && req.wait;
            const int move_ret = arm_->move_pose(target, sdk_duration, wait);
            std::array<double, 7> actual{};
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForPoseTargetLocked(target, &actual, &max_error, &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "move_pose ret=" + std::to_string(move_ret) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_position_error=" + std::to_string(max_error) +
                          ", actual=" + vectorToString(poseArrayToVector(actual));
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleMoveLineJoint(carm_a3_motion::MoveLineJoint::Request& req,
                             carm_a3_motion::MoveLineJoint::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        if (static_cast<int>(req.positions.size()) != joint_count_) {
            res.success = false;
            res.message = "positions must contain exactly " + std::to_string(joint_count_) + " values";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }
            const std::vector<double> current = arm_->get_joint_pos();
            if (static_cast<int>(current.size()) < joint_count_) {
                res.success = false;
                res.message = "SDK returned too few joints: " + std::to_string(current.size());
                return true;
            }
            for (int i = 0; i < joint_count_; ++i) {
                const double delta = std::abs(req.positions[static_cast<size_t>(i)] -
                                              current[static_cast<size_t>(i)]);
                if (delta > max_move_delta_rad_) {
                    res.success = false;
                    res.message = "joint " + std::to_string(i + 1) +
                                  " delta exceeds max_move_delta_rad";
                    return true;
                }
            }
            if (dry_run_) {
                res.success = true;
                res.message = "dry_run move_line_joint accepted: target=" +
                              vectorToString(req.positions);
                return true;
            }
            const bool wait = wait_for_motion_ && req.wait;
            const int move_ret = arm_->move_line_joint(req.positions, wait);
            std::vector<double> actual;
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForJointTargetLocked(req.positions,
                                                           &actual,
                                                           &max_error,
                                                           &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "move_line_joint ret=" + std::to_string(move_ret) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_error=" + std::to_string(max_error) +
                          ", actual=" + vectorToString(actual);
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleMoveLinePose(carm_a3_motion::MoveLinePose::Request& req,
                            carm_a3_motion::MoveLinePose::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        std::array<double, 7> target{};
        if (!vectorToPoseArray(req.pose, &target, &res.message)) {
            res.success = false;
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }
            if (!checkPoseDeltaLocked(target, "move_line_pose", &message)) {
                res.success = false;
                res.message = message;
                return true;
            }
            if (dry_run_) {
                res.success = true;
                res.message = "dry_run move_line_pose accepted: target=" +
                              vectorToString(poseArrayToVector(target));
                return true;
            }
            const bool wait = wait_for_motion_ && req.wait;
            const int move_ret = arm_->move_line_pose(target, wait);
            std::array<double, 7> actual{};
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForPoseTargetLocked(target, &actual, &max_error, &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "move_line_pose ret=" + std::to_string(move_ret) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_position_error=" + std::to_string(max_error) +
                          ", actual=" + vectorToString(poseArrayToVector(actual));
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    bool handleMoveFlowPose(carm_a3_motion::MoveFlowPose::Request& req,
                            carm_a3_motion::MoveFlowPose::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!allow_motion_) {
            res.success = false;
            res.message = "blocked: allow_motion is false";
            return true;
        }
        std::array<double, 7> target{};
        if (!vectorToPoseArray(req.pose, &target, &res.message)) {
            res.success = false;
            return true;
        }
        if (req.line_theta_weight < 0.0 || req.line_theta_weight > 1.0 ||
            req.accuracy <= 0.0 || req.accuracy > 0.01) {
            res.success = false;
            res.message = "line_theta_weight must be [0,1], accuracy must be (0,0.01]";
            return true;
        }
        std::string message;
        if (!ensureConnected(&message)) {
            res.success = false;
            res.message = message;
            return true;
        }
        try {
            const carm::ArmStatus status = getStatusLocked();
            if (!checkMotionPreconditions(status, &message)) {
                res.success = false;
                res.message = message + "; " + statusToString(status);
                return true;
            }
            if (!checkPoseDeltaLocked(target, "move_flow_pose", &message)) {
                res.success = false;
                res.message = message;
                return true;
            }
            if (dry_run_) {
                res.success = true;
                res.message = "dry_run move_flow_pose accepted: target=" +
                              vectorToString(poseArrayToVector(target));
                return true;
            }
            const bool wait = wait_for_motion_ && req.wait;
            const int move_ret = arm_->move_flow_pose(
                    target, req.line_theta_weight, req.accuracy, wait);
            std::array<double, 7> actual{};
            double max_error = 0.0;
            std::string verify_message;
            const bool verified = move_ret >= 1 &&
                                  waitForPoseTargetLocked(target, &actual, &max_error, &verify_message);
            res.success = move_ret >= 1 && verified;
            res.message = "move_flow_pose ret=" + std::to_string(move_ret) +
                          ", verified=" + (verified ? "true" : "false") +
                          ", verify_message=" + verify_message +
                          ", max_position_error=" + std::to_string(max_error) +
                          ", actual=" + vectorToString(poseArrayToVector(actual));
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::mutex mutex_;
    std::unique_ptr<carm::CArmSingleCol> arm_;

    ros::Publisher joint_pub_;
    ros::Publisher flange_pose_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Timer state_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer status_srv_;
    ros::ServiceServer set_ready_srv_;
    ros::ServiceServer set_servo_srv_;
    ros::ServiceServer emergency_stop_srv_;
    ros::ServiceServer jog_joint_srv_;
    ros::ServiceServer move_joint_srv_;
    ros::ServiceServer move_joint_traj_srv_;
    ros::ServiceServer move_pose_srv_;
    ros::ServiceServer move_line_joint_srv_;
    ros::ServiceServer move_line_pose_srv_;
    ros::ServiceServer move_flow_pose_srv_;
    ros::ServiceServer get_joint_snapshot_srv_;
    ros::ServiceServer get_cartesian_snapshot_srv_;
    ros::ServiceServer get_extended_state_srv_;
    ros::ServiceServer get_tool_info_srv_;
    ros::ServiceServer set_speed_srv_;
    ros::ServiceServer set_control_mode_srv_;
    ros::ServiceServer set_collision_config_srv_;
    ros::ServiceServer set_tool_index_srv_;
    ros::ServiceServer set_gripper_srv_;
    ros::ServiceServer solve_ik_srv_;
    ros::ServiceServer solve_fk_srv_;
    ros::ServiceServer solve_ik_array_srv_;
    ros::ServiceServer solve_fk_array_srv_;

    std::string robot_host_;
    int robot_port_ = 8090;
    double connect_timeout_s_ = 1.0;
    bool connect_on_start_ = true;
    bool publish_state_ = true;
    double state_publish_rate_hz_ = 5.0;
    std::string base_frame_id_;
    std::string joint_state_frame_id_;
    std::string flange_frame_id_;
    bool publish_flange_tf_ = false;
    std::vector<std::string> joint_names_;
    bool publish_gripper_joints_ = true;
    std::string gripper_right_joint_name_ = "joint7";
    std::string gripper_left_joint_name_ = "joint8";
    double gripper_joint_scale_ = 0.5;
    double gripper_joint_max_m_ = 0.037;
    bool auto_ready_on_connect_ = false;
    bool register_callbacks_on_connect_ = false;
    double pre_ready_delay_s_ = 1.0;
    bool sdk_session_initialized_ = false;
    bool ready_called_on_connect_ = false;
    bool callbacks_registered_ = false;
    std::atomic<double> last_joint_stamp_{0.0};
    std::atomic<double> last_pose_stamp_{0.0};
    std::atomic<size_t> last_joint_count_{0};

    bool allow_ready_ = false;
    bool allow_servo_enable_ = false;
    bool allow_motion_ = false;
    bool allow_settings_ = false;
    bool allow_gripper_ = false;
    bool dry_run_ = true;

    bool require_connected_ = true;
    bool require_servo_enabled_ = true;
    bool require_position_mode_ = true;
    bool reject_controller_error_ = true;

    int joint_count_ = 6;
    double max_jog_delta_rad_ = 0.03;
    double max_move_delta_rad_ = 0.15;
    double max_pose_position_delta_m_ = 0.05;
    double max_gripper_pos_m_ = 0.08;
    double max_gripper_tau_n_ = 100.0;
    double default_duration_s_ = 2.0;
    double min_duration_s_ = 0.5;
    double max_duration_s_ = 10.0;
    bool use_duration_ = false;
    bool wait_for_motion_ = false;
    double speed_level_ = 1.0;
    int speed_response_level_ = 20;
    bool set_speed_before_motion_ = false;
    bool verify_after_motion_ = true;
    double verify_timeout_s_ = 3.0;
    double verify_poll_s_ = 0.1;
    double verify_joint_tolerance_rad_ = 0.003;
    double verify_pose_position_tolerance_m_ = 0.005;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "carm_a3_safe_motion");
    SafeMotionNode node;
    ros::spin();
    return 0;
}
