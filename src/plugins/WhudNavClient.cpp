/**
 * @file WhudNavClient.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief whud_nav_clent plugin
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {

class WhudNavClient : public PluginBase {
public:
  /**
   * @brief Construct a new Whud Nav Client object
   *
   * @note In this function, private node handle and navigation client(move base
   * action client) will be initialized
   */
  WhudNavClient()
      : PluginBase(), nh_("~whud_nav_client"), nav_client_("move_base") {}

  /**
   * @brief Destroy the Whud Nav Client object
   */
  ~WhudNavClient() {}

  /**
   * @brief Plugin init
   *
   * @note In this function, map frame and body frame will be assigned by ros
   * parameters, navigation velocity subscriber will be initialized, and plugin
   * will wait for action server and transform set up
   *
   * @warning You need to set up move base action server and transform between
   * map frame and body frame, unless the state machine will be stucked in this
   * plugin init
   *
   * @param mavros_pub mavros publisher passed by state machine
   */
  virtual void OnInit(MavRosPublisher &mavros_pub) override {
    PluginBase::OnInit(mavros_pub);

    nh_.param<string>("map_frame_id", map_frame_id_, "map");
    nh_.param<string>("body_frame_id", body_frame_id_, "nav_link");

    nav_vel_sub_ =
        nh_.subscribe("cmd_vel", 1, &WhudNavClient::nav_vel_cb, this);

    nav_client_.waitForServer();
    tf_listener_.waitForTransform("/" + map_frame_id_, "/" + body_frame_id_,
                                  ros::Time(0), ros::Duration(0, 0));
  }

  /**
   * @brief Set task with given parameters
   *
   * @note In this function, parameters passed by state machine will be parsed,
   * the result will be transformed into move base goal, and the goal will sent
   * to move base server after consensus detect
   *
   * @param param Parameter vector passed by state machine, plugins will set
   * task according to the paramter parse:
   * param[0]: position x
   * param[1]: position y
   *
   * @retval true: Parameters are parsed correctly
   * @retval false: Parameters are parsed wrong
   */
  virtual bool SetTask(ros::V_string param) override {
    PluginBase::SetTask(param);
    // parse param passed by state machine
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

    // detect consensus with last set pose
    if (!ConsensusDetector(set_pose)) {
      ROS_WARN("You set the pose which is the same as last set");
      task_status_ = TaskStatus::DONE;
    } else {
      // goal serialization
      move_base_msgs::MoveBaseGoal goal = GoalSerialization(set_pose);
      nav_client_.sendGoal(
          goal,
          std::bind(&WhudNavClient::DoneCb, this, std::placeholders::_1,
                    std::placeholders::_2),
          std::bind(&WhudNavClient::ActiveCb, this),
          std::bind(&WhudNavClient::FeedbackCb, this, std::placeholders::_1));
    }
  }

  /**
   * @brief Spin task in regular frequency
   *
   * @note Because navigation state will be checked in done callback, this
   * function is empty
   */
  virtual void TaskSpin() override {}

  /**
   * @brief Stop task if task is timeout
   *
   * @note In this function, goal will be cancelled and convertion messages will
   * be sent to mavros to stop navigation command velocity
   */
  virtual void StopTask() override {
    nav_client_.cancelGoal();

    // send conversion messages to stop cmd vel
    std_msgs::Bool conversion;
    conversion.data = true;
    mavros_pub_->conversion_pub.publish(conversion);
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_client_;
  ros::Subscriber nav_vel_sub_;

  tf::TransformListener tf_listener_;
  string map_frame_id_, body_frame_id_;

  geometry_msgs::Pose last_set_pose_;

  /**
   * @brief Action active callback
   */
  void ActiveCb() {}

  /**
   * @brief Action feedback callback
   *
   * @param feedback Current local position
   */
  void FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {}

  /**
   * @brief Action done callback
   *
   * @param state Goal state
   * @param result Navigation result
   */
  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      task_status_ = TaskStatus::DONE;

      std_msgs::Bool conversion;
      conversion.data = true;
      mavros_pub_->conversion_pub.publish(conversion);
    }
  }

  /**
   * @brief Navigation velocity subscriber callback
   *
   * @note Send twist to mavros if control flag is true
   *
   * @param req Command velocity sent by navigation server
   */
  void nav_vel_cb(const geometry_msgs::Twist::ConstPtr &req) {
    if (mavros_pub_ != nullptr && control_flag_)
      mavros_pub_->cmd_vel_pub.publish(req);
  }

  /**
   * @brief Detect consensus between last set pose and set pose
   *
   * @param set_pose Set pose
   * @retval true: Set pose is different from last set pose
   * @retval false: Set pose is the same with last set pose
   */
  bool ConsensusDetector(geometry_msgs::Pose set_pose) {
    if (last_set_pose_ == set_pose)
      return false;
    else {
      last_set_pose_ = set_pose;
      return true;
    }
  }

  /**
   * @brief Serialize goal from pose to move base goal
   *
   * @param set_pose Set pose
   * @return move_base_msgs::MoveBaseGoal: Move base goal
   */
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