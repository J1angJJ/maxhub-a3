#include <array>
#include <algorithm>
#include <atomic>
#include <functional>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "arm_control_sdk/carm_cobot.h"
#include "arm_control_sdk/data_type_def.h"
#include "carm_a3_motion/JogJoint.h"
#include "carm_a3_motion/MoveJoint.h"

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

        pnh_.param<bool>("allow_ready", allow_ready_, false);
        pnh_.param<bool>("allow_servo_enable", allow_servo_enable_, false);
        pnh_.param<bool>("allow_motion", allow_motion_, false);
        pnh_.param<bool>("dry_run", dry_run_, true);

        pnh_.param<bool>("require_connected", require_connected_, true);
        pnh_.param<bool>("require_servo_enabled", require_servo_enabled_, true);
        pnh_.param<bool>("require_position_mode", require_position_mode_, true);
        pnh_.param<bool>("reject_controller_error", reject_controller_error_, true);

        pnh_.param<int>("joint_count", joint_count_, 6);
        pnh_.param<double>("max_jog_delta_rad", max_jog_delta_rad_, 0.03);
        pnh_.param<double>("max_move_delta_rad", max_move_delta_rad_, 0.15);
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::mutex mutex_;
    std::unique_ptr<carm::CArmSingleCol> arm_;

    ros::ServiceServer status_srv_;
    ros::ServiceServer set_ready_srv_;
    ros::ServiceServer set_servo_srv_;
    ros::ServiceServer emergency_stop_srv_;
    ros::ServiceServer jog_joint_srv_;
    ros::ServiceServer move_joint_srv_;

    std::string robot_host_;
    int robot_port_ = 8090;
    double connect_timeout_s_ = 1.0;
    bool connect_on_start_ = true;
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
    bool dry_run_ = true;

    bool require_connected_ = true;
    bool require_servo_enabled_ = true;
    bool require_position_mode_ = true;
    bool reject_controller_error_ = true;

    int joint_count_ = 6;
    double max_jog_delta_rad_ = 0.03;
    double max_move_delta_rad_ = 0.15;
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "carm_a3_safe_motion");
    SafeMotionNode node;
    ros::spin();
    return 0;
}
