#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ignition/math/Pose3.hh>
#include <mutex>

namespace gazebo
{
class HumanActorPlugin : public ModelPlugin
{
public:
  HumanActorPlugin() : ModelPlugin() {}

  // Called once when the plugin is loaded.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model_ = _model;

    // Initialise ROS if necessary.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "human_actor_plugin",
                ros::init_options::NoSigintHandler);
    }

    this->rosNode_ = std::make_unique<ros::NodeHandle>("~");

    // Get topic name from SDF (defaults to /actor_pose).
    std::string topicName = "/actor_pose";
    std::string namespace_ = "";
    if (_sdf->HasElement("ros_topic"))
      topicName = _sdf->Get<std::string>("ros_topic");

    if (_sdf->HasElement("ros_namespace"))
      namespace_ = _sdf->Get<std::string>("ros_namespace");

    std::string fullTopic = namespace_ + topicName;
    std::cout <<  "Full topic: " << fullTopic << std::endl;

    // Subscribe to desired pose commands.
    this->poseSub_ = this->rosNode_->subscribe(
        fullTopic, 1, &HumanActorPlugin::PoseCallback, this);


    // Start from current pose.
    this->targetPose_ = this->model_->WorldPose();

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HumanActorPlugin::OnUpdate, this, std::placeholders::_1));

    ROS_INFO_STREAM("[HumanActorPlugin] Loaded. Subscribing to " << fullTopic);
  }

private:
  // Callback for PoseStamped messages.
  void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    const auto &p = msg->pose.position;
    const auto &o = msg->pose.orientation;
    this->targetPose_.Pos().Set(p.x, p.y, p.z);
    this->targetPose_.Rot().Set(o.w, o.x, o.y, o.z);
  }

  // Called by the world update start event.
  void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    // Instantly set pose (teleport). Replace with interpolation for smooth motion.
    this->model_->SetWorldPose(this->targetPose_);
  }

  // Pointer to the model.
  physics::ModelPtr model_;

  // Target pose received from ROS.
  ignition::math::Pose3d targetPose_;

  // ROS components.
  std::unique_ptr<ros::NodeHandle> rosNode_;
  ros::Subscriber poseSub_;

  // Mutex to protect pose updates.
  std::mutex mutex_;

  // Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
};

// Register this plugin with Gazebo.
GZ_REGISTER_MODEL_PLUGIN(HumanActorPlugin)
} // namespace gazebo

