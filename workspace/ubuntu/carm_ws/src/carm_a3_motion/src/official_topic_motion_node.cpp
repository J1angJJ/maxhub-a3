#include <array>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include "arm_control_sdk/carm_cobot.h"
#include "arm_control_sdk/data_type_def.h"

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

}  // namespace

class OfficialTopicMotionNode {
public:
    OfficialTopicMotionNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("robot_host", robot_host_, "192.168.31.60");
        pnh_.param<int>("robot_port", robot_port_, 8090);
        pnh_.param<double>("connect_timeout_s", connect_timeout_s_, 1.0);
        pnh_.param<bool>("auto_ready", auto_ready_, false);
        pnh_.param<bool>("register_callbacks", register_callbacks_, false);
        pnh_.param<double>("pre_ready_delay_s", pre_ready_delay_s_, 1.0);
        pnh_.param<bool>("allow_move_joint", allow_move_joint_, false);

        arm_.reset(new carm::CArmSingleCol(robot_host_, robot_port_, connect_timeout_s_));
        if (!arm_->is_connected()) {
            const int ret = arm_->connect(robot_host_, robot_port_, connect_timeout_s_);
            ROS_INFO("official_topic_motion connect ret=%d", ret);
        }

        if (pre_ready_delay_s_ > 0.0) {
            ROS_INFO("official_topic_motion sleeping %.3f s before optional set_ready", pre_ready_delay_s_);
            ros::Duration(pre_ready_delay_s_).sleep();
        }

        if (auto_ready_) {
            const int ret = arm_->set_ready();
            ROS_WARN("official_topic_motion auto set_ready ret=%d", ret);
        } else {
            ROS_WARN("official_topic_motion auto_ready=false; not calling set_ready()");
        }

        if (register_callbacks_) {
            registerOfficialCallbacks();
        } else {
            ROS_WARN("official_topic_motion register_callbacks=false; not registering SDK callbacks");
        }

        status_srv_ = nh_.advertiseService("/carm_a3/official_motion/status",
                                           &OfficialTopicMotionNode::handleStatus,
                                           this);
        ready_sub_ = nh_.subscribe("/carm_a3/official_motion/ready",
                                   1,
                                   &OfficialTopicMotionNode::handleReady,
                                   this);
        emergency_stop_sub_ = nh_.subscribe("/carm_a3/official_motion/emergency_stop",
                                            1,
                                            &OfficialTopicMotionNode::handleEmergencyStop,
                                            this);
        move_joint_sub_ = nh_.subscribe("/carm_a3/official_motion/move_joint",
                                        1,
                                        &OfficialTopicMotionNode::handleMoveJoint,
                                        this);

        ROS_WARN("official_topic_motion started with allow_move_joint=%s",
                 allow_move_joint_ ? "true" : "false");
        ROS_WARN("move_joint callback intentionally matches official ROS1 demo: move_joint(msg->position, -1, false)");
    }

    ~OfficialTopicMotionNode() {
        if (arm_ && arm_->is_connected()) {
            if (register_callbacks_) {
                arm_->release_joint_cbk();
                arm_->release_pose_cbk();
                arm_->release_error_cbk("official_error");
                arm_->release_completion_cbk("official_completion");
            }
            arm_->disconnect();
        }
    }

private:
    void registerOfficialCallbacks() {
        arm_->register_joint_cbk(std::bind(&OfficialTopicMotionNode::handleJointCallback,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2,
                                           std::placeholders::_3,
                                           std::placeholders::_4));
        arm_->register_pose_cbk(std::bind(&OfficialTopicMotionNode::handlePoseCallback,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
        arm_->register_error_cbk("official_error",
                                 std::bind(&OfficialTopicMotionNode::handleErrorCallback,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
        arm_->register_completion_cbk("official_completion",
                                      std::bind(&OfficialTopicMotionNode::handleCompletionCallback,
                                                this,
                                                std::placeholders::_1));
        ROS_WARN("official_topic_motion registered SDK joint/pose/error/completion callbacks");
    }

    bool handleStatus(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        if (!arm_ || !arm_->is_connected()) {
            res.success = false;
            res.message = "not connected";
            return true;
        }
        try {
            res.success = true;
            res.message = statusToString(arm_->get_status());
        } catch (const std::exception& e) {
            res.success = false;
            res.message = e.what();
        }
        return true;
    }

    void handleReady(const std_msgs::BoolConstPtr& msg) {
        if (!msg->data) {
            ROS_WARN("official_topic_motion ready=false ignored");
            return;
        }
        const int ret = arm_->set_ready();
        ROS_WARN("official_topic_motion set_ready ret=%d", ret);
    }

    void handleEmergencyStop(const std_msgs::BoolConstPtr& msg) {
        if (!msg->data) {
            ROS_WARN("official_topic_motion emergency_stop=false ignored");
            return;
        }
        const int ret = arm_->emergency_stop();
        ROS_ERROR("official_topic_motion emergency_stop ret=%d", ret);
    }

    void handleMoveJoint(const sensor_msgs::JointStateConstPtr& msg) {
        if (!allow_move_joint_) {
            ROS_ERROR("official_topic_motion blocked: allow_move_joint=false");
            return;
        }
        if (msg->position.size() != 6) {
            ROS_ERROR("official_topic_motion expected 6 joint positions, got %zu",
                      msg->position.size());
            return;
        }
        ROS_WARN("official_topic_motion calling move_joint(position, -1, false)");
        const int ret = arm_->move_joint(msg->position, -1, false);
        ROS_WARN("official_topic_motion move_joint ret=%d", ret);
    }

    void handleJointCallback(double stamp,
                             std::vector<double> positions,
                             std::vector<double>,
                             std::vector<double>) {
        last_joint_stamp_ = stamp;
        last_joint_count_ = positions.size();
    }

    void handlePoseCallback(double stamp, std::array<double, 7>) {
        last_pose_stamp_ = stamp;
    }

    void handleErrorCallback(int code, const std::string message) {
        ROS_ERROR("official_topic_motion SDK error callback code=%d message=%s",
                  code,
                  message.c_str());
    }

    void handleCompletionCallback(const std::string task_key) {
        ROS_WARN("official_topic_motion SDK completion callback task_key=%s",
                 task_key.c_str());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<carm::CArmSingleCol> arm_;

    ros::ServiceServer status_srv_;
    ros::Subscriber ready_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Subscriber move_joint_sub_;

    std::string robot_host_;
    int robot_port_ = 8090;
    double connect_timeout_s_ = 1.0;
    bool auto_ready_ = false;
    bool register_callbacks_ = false;
    double pre_ready_delay_s_ = 1.0;
    bool allow_move_joint_ = false;
    double last_joint_stamp_ = 0.0;
    double last_pose_stamp_ = 0.0;
    size_t last_joint_count_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "carm_a3_official_topic_motion");
    OfficialTopicMotionNode node;
    ros::spin();
    return 0;
}
