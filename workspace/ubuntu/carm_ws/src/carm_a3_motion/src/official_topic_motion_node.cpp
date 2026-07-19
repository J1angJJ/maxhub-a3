#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

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
        pnh_.param<bool>("allow_move_joint", allow_move_joint_, false);

        arm_.reset(new carm::CArmSingleCol(robot_host_, robot_port_, connect_timeout_s_));
        if (!arm_->is_connected()) {
            const int ret = arm_->connect(robot_host_, robot_port_, connect_timeout_s_);
            ROS_INFO("official_topic_motion connect ret=%d", ret);
        }

        if (auto_ready_) {
            const int ret = arm_->set_ready();
            ROS_WARN("official_topic_motion auto set_ready ret=%d", ret);
        } else {
            ROS_WARN("official_topic_motion auto_ready=false; not calling set_ready()");
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
            arm_->disconnect();
        }
    }

private:
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
    bool allow_move_joint_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "carm_a3_official_topic_motion");
    OfficialTopicMotionNode node;
    ros::spin();
    return 0;
}
