#include <array>
#include <algorithm>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include "arm_control_sdk/carm_cobot.h"
#include "arm_control_sdk/data_type_def.h"

class ReadonlyStateNode {
public:
    ReadonlyStateNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("robot_host", robot_host_, "192.168.31.60");
        pnh_.param<int>("robot_port", robot_port_, 8090);
        pnh_.param<double>("connect_timeout_s", connect_timeout_s_, 1.0);
        pnh_.param<double>("publish_rate_hz", publish_rate_hz_, 5.0);
        pnh_.param<bool>("connect_on_start", connect_on_start_, true);
        pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        pnh_.param<std::string>("joint_state_frame_id", joint_state_frame_id_, base_frame_id_);
        pnh_.param<std::string>("flange_frame_id", flange_frame_id_, "flange");
        pnh_.param<bool>("publish_flange_tf", publish_flange_tf_, true);
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        pnh_.getParam("joint_names", joint_names_);

        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        flange_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/maxhub_a3/flange_pose", 10);
        diagnostics_pub_ = nh_.advertise<std_msgs::String>("/maxhub_a3/diagnostics", 10, true);

        ROS_INFO("MAXHUB A3 read-only ROS node starting.");
        ROS_INFO("Target controller: %s:%d", robot_host_.c_str(), robot_port_);
        ROS_INFO("This node does not call ready, enable, motion, stop, or gripper APIs.");

        if (connect_on_start_) {
            connectReadonly();
        } else {
            publishDiagnostics("connect_on_start=false; node is idle");
        }

        const double period_s = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.2;
        timer_ = nh_.createTimer(ros::Duration(period_s), &ReadonlyStateNode::onTimer, this);
    }

    ~ReadonlyStateNode() {
        if (arm_) {
            try {
                arm_->disconnect();
            } catch (const std::exception& e) {
                ROS_WARN("disconnect threw: %s", e.what());
            } catch (...) {
                ROS_WARN("disconnect threw an unknown exception");
            }
        }
    }

private:
    void connectReadonly() {
        try {
            arm_.reset(new carm::CArmSingleCol(robot_host_, robot_port_, connect_timeout_s_));
            if (!arm_->is_connected()) {
                const int ret = arm_->connect(robot_host_, robot_port_, connect_timeout_s_);
                ROS_INFO("connect returned: %d", ret);
            }

            std::ostringstream ss;
            ss << "connected=" << (arm_->is_connected() ? "true" : "false");
            publishDiagnostics(ss.str());
        } catch (const std::exception& e) {
            arm_.reset();
            ROS_ERROR("Failed to initialize/connect SDK: %s", e.what());
            publishDiagnostics(std::string("connect_failed: ") + e.what());
        } catch (...) {
            arm_.reset();
            ROS_ERROR("Failed to initialize/connect SDK: unknown exception");
            publishDiagnostics("connect_failed: unknown exception");
        }
    }

    void onTimer(const ros::TimerEvent&) {
        if (!arm_) {
            return;
        }

        try {
            publishArmStatus();
            publishJointState();
            publishFlangePose();
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(2.0, "Read-only polling failed: %s", e.what());
            publishDiagnostics(std::string("poll_failed: ") + e.what());
        } catch (...) {
            ROS_WARN_THROTTLE(2.0, "Read-only polling failed: unknown exception");
            publishDiagnostics("poll_failed: unknown exception");
        }
    }

    void publishArmStatus() {
        const auto status = arm_->get_status();

        std::ostringstream ss;
        ss << "arm_index=" << status.arm_index
           << ",arm_name=" << status.arm_name
           << ",connected=" << (status.arm_is_connected ? "true" : "false")
           << ",dof=" << status.arm_dof
           << ",servo=" << (status.servo_status ? "true" : "false")
           << ",state=" << status.state
           << ",fsm_state=" << status.fsm_state
           << ",speed_percentage=" << status.speed_percentage
           << ",debug=" << (status.on_debug_mode ? "true" : "false");

        publishDiagnostics(ss.str());
    }

    void publishJointState() {
        const auto positions = arm_->get_joint_pos();
        const auto velocities = arm_->get_joint_vel();
        const auto efforts = arm_->get_joint_tau();

        if (positions.empty()) {
            ROS_WARN_THROTTLE(2.0, "SDK returned empty joint positions");
            return;
        }

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = joint_state_frame_id_;
        msg.name = trimJointNames(positions.size());
        msg.position = trimToJointCount(positions);
        msg.velocity = trimToJointCount(velocities);
        msg.effort = trimToJointCount(efforts);
        joint_pub_.publish(msg);
    }

    void publishFlangePose() {
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

    std::vector<double> trimToJointCount(const std::vector<double>& values) const {
        const std::size_t n = std::min(values.size(), joint_names_.size());
        return std::vector<double>(values.begin(), values.begin() + static_cast<long>(n));
    }

    std::vector<std::string> trimJointNames(const std::size_t value_count) const {
        const std::size_t n = std::min(value_count, joint_names_.size());
        return std::vector<std::string>(joint_names_.begin(), joint_names_.begin() + static_cast<long>(n));
    }

    void publishDiagnostics(const std::string& text) {
        std_msgs::String msg;
        msg.data = text;
        diagnostics_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher joint_pub_;
    ros::Publisher flange_pose_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Timer timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::unique_ptr<carm::CArmSingleCol> arm_;
    std::string robot_host_;
    int robot_port_ = 8090;
    double connect_timeout_s_ = 1.0;
    double publish_rate_hz_ = 5.0;
    bool connect_on_start_ = true;
    std::string base_frame_id_;
    std::string joint_state_frame_id_;
    std::string flange_frame_id_;
    bool publish_flange_tf_ = true;
    std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "maxhub_a3_readonly_state_publisher");
    ReadonlyStateNode node;
    ros::spin();
    return 0;
}
