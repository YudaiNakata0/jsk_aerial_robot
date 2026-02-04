#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <mutex>
#include <vector>

namespace gazebo
{
class TrajectoryVisualPlugin : public VisualPlugin
{
public:
  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
  {
    std::cout << "TrajectoryVisualPlugin LOADED" << std::endl;

    visual_ = _visual;
    scene_ = visual_->GetScene();
    if (!scene_) return;

    line_ = visual_->CreateDynamicLine(
      rendering::RENDERING_LINE_STRIP);
    line_->setMaterial("Gazebo/Red");

    update_connection_ =
      event::Events::ConnectPreRender(
        std::bind(&TrajectoryVisualPlugin::OnUpdate, this));

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "trajectory_visual_plugin",
                ros::init_options::NoSigintHandler);
    }

    ros_node_.reset(new ros::NodeHandle);
    sub_ = ros_node_->subscribe(
      "gimbalrotor/endeffector_pose", 10,
      &TrajectoryVisualPlugin::PoseCallback, this);
  }

  void PoseCallback(const geometry_msgs::PoseConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.emplace_back(
      msg->position.x,
      msg->position.y,
      msg->position.z);
  }

  void OnUpdate()
  {
    if (!visual_ || !scene_ || !line_)
      return;

    if (!scene_->Initialized())
      return;

    std::lock_guard<std::mutex> lock(mutex_);

    line_->Clear();
    for (const auto& p : points_)
      line_->AddPoint(p);
  }

private:
  rendering::VisualPtr visual_;
  rendering::ScenePtr scene_;
  rendering::DynamicLines* line_;

  event::ConnectionPtr update_connection_;

  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Subscriber sub_;

  std::vector<ignition::math::Vector3d> points_;
  std::mutex mutex_;
};

GZ_REGISTER_VISUAL_PLUGIN(TrajectoryVisualPlugin)
}