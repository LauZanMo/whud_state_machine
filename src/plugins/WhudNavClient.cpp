#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {

class WhudNavClient : public PluginBase {
public:
  WhudNavClient()
      : PluginBase(), nh_("~whud_nav_client"), nav_client_("nav_client") {}
  ~WhudNavClient() {}

  virtual void OnInit(MavRosPublisher &mavros_pub) override {
    PluginBase::OnInit(mavros_pub);

    nh_.param<string>("map_frame_id", map_frame_id_, "map");
    nh_.param<string>("body_frame_id", body_frame_id_, "nav_link");

    nav_vel_sub_ =
        nh_.subscribe("cmd_vel", 1, &WhudNavClient::nav_vel_cb, this);

    // nav_client_.waitForServer();
    // tf_listener_.waitForTransform("/" + map_frame_id_, "/" + body_frame_id_,
    //                               ros::Time(0), ros::Duration(0, 0));
  }

  void nav_vel_cb(const geometry_msgs::Twist::ConstPtr &req) {
    if (mavros_pub_ != nullptr && control_flag_)
      mavros_pub_->cmd_vel_pub.publish(req);
  }

  virtual bool SetTask(ros::V_string param) override {
    PluginBase::SetTask(param);
    geometry_msgs::Pose set_pose;
    set_pose.position.x = (atof(param[0].c_str()));
    set_pose.position.y = (atof(param[1].c_str()));
    set_pose.position.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    set_pose.orientation.x = q.getX();
    set_pose.orientation.y = q.getY();
    set_pose.orientation.z = q.getZ();
    set_pose.orientation.w = q.getW();

    // goal serialization
    // while (!TransformDetector())
    //   ROS_WARN("Can not find trasform from body frame to map");

    if (!ConsensusDetector(set_pose)) {
      ROS_WARN("You set the pose which is the same as last set");
      task_status_ = TaskStatus::DONE;
    } else {
      move_base_msgs::MoveBaseGoal goal = GoalSerialization(set_pose);
      nav_client_.sendGoal(
          goal,
          std::bind(&WhudNavClient::DoneCb, this, std::placeholders::_1,
                    std::placeholders::_2),
          std::bind(&WhudNavClient::ActiveCb, this),
          std::bind(&WhudNavClient::FeedbackCb, this, std::placeholders::_1));
    }
  }

  virtual void TaskSpin() override {}

  virtual void StopTask() override {
    nav_client_.cancelGoal();
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_client_;
  ros::Subscriber nav_vel_sub_;

  tf::TransformListener tf_listener_;
  string map_frame_id_, body_frame_id_;

  geometry_msgs::Pose last_set_pose_;

  void ActiveCb() {}

  void FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {}

  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      task_status_ = TaskStatus::DONE;
  }

  bool TransformDetector() const {
    return tf_listener_.canTransform("/" + map_frame_id_, "/" + body_frame_id_,
                                     ros::Time(0));
  }

  bool ConsensusDetector(geometry_msgs::Pose set_pose) {
    if (last_set_pose_ == set_pose)
      return false;
    else {
      last_set_pose_ = set_pose;
      return true;
    }
  }

  move_base_msgs::MoveBaseGoal GoalSerialization(geometry_msgs::Pose set_pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = map_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = set_pose;
    return goal;
  }
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudNavClient,
                       whud_state_machine::PluginBase)