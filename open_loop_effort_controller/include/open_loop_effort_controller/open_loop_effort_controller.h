#ifndef open_loop_effort_controller__20188101642
#define open_loop_effort_controller__20188101642

# include <controller_interface/controller.h>
# include <hardware_interface/joint_command_interface.h>
# include <thread>
# include <mutex>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>

# include <subscription_notifier/subscription_notifier.h>
# include <ros/callback_queue.h>


namespace control
{

class OpenLoopEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);


  std::string getJointName()
  {
    return m_joint_name;
  };
  bool isWellInit(){return m_well_init;};

protected:

  hardware_interface::EffortJointInterface* m_hw;
  hardware_interface::JointHandle m_jh;

  bool m_well_init;

  ros::CallbackQueue m_queue;
  boost::shared_ptr<ros::AsyncSpinner> m_spinner;

  hardware_interface::JointHandle m_joint_handles;

  std::string m_joint_name;
  double m_eff_cmd;
  double m_max_effort;


  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_target_js_rec;

  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  bool m_configured;

  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::string name, double& eff);

  void timerCallback(const ros::TimerEvent& event);
  void stopThreads();
};


}

# endif
