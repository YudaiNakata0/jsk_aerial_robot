#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>
#include <std_msgs/Bool.h>

namespace gazebo
{
  class TrajectoryVisualizer : public VisualPlugin
  {
  private: bool isDrawing;
  private: ros::Subscriber controlSub;
    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) override
    {
      this->visual = _parent;

      // 1. ROSの初期化
      if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_trajectory_visualizer", ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // 2. 設定の読み込み
      std::string topic_name = "/robot_pose";
      if (_sdf->HasElement("pose_topic"))
        topic_name = _sdf->Get<std::string>("pose_topic");
      std::string control_topic = "/drawing_state";
      if (_sdf->HasElement("control_topic"))
	control_topic = _sdf->Get<std::string>("control_topic");

      // 3. 描画用ラインの初期設定
      this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
      this->line->AddPoint(ignition::math::Vector3d(0, 0, 0)); // 初期点
      this->line->setMaterial("Gazebo/Yellow");
      this->line->setVisibilityFlags(GZ_VISIBILITY_ALL);

      // 4. ROS Subscriberの作成
      this->rosSub = this->rosNode->subscribe(topic_name, 10, &TrajectoryVisualizer::OnPose, this);
      this->controlSub = this->rosNode->subscribe(control_topic, 1, &TrajectoryVisualizer::OnControl, this);

      // 更新イベントへの接続
      this->updateConnection = event::Events::ConnectPreRender(
          std::bind(&TrajectoryVisualizer::OnUpdate, this));
      
      ROS_INFO("Trajectory Visualizer Loaded. Listening to: %s", topic_name.c_str());
    }

    private: void OnPose(const geometry_msgs::Pose::ConstPtr &_msg)
    {
      if(!this->isDrawing) return;
        std::lock_guard<std::mutex> lock(this->mutex);
        this->newPoints.push_back(ignition::math::Vector3d(
            _msg->position.x+1, _msg->position.y, _msg->position.z-0.01));
    }
    private: void OnControl(const std_msgs::Bool::ConstPtr &_msg)
    {
      this->isDrawing = _msg->data;
    }
    private: void OnUpdate()
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (this->newPoints.empty()) return;
	// ignition::math::Pose3d current_robot_pose = this->visual->WorldPose();

        for (const auto &p : this->newPoints) {
	  // ignition::math::Vector3d p_local = current_robot_pose.Rot().Inverse() * (p_world-current_robot_pose.Pos());
            this->line->AddPoint(p);
        }
        this->newPoints.clear();
    }

    private: rendering::VisualPtr visual;
    private: rendering::DynamicLines *line;
    private: event::ConnectionPtr updateConnection;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: std::vector<ignition::math::Vector3d> newPoints;
    private: std::mutex mutex;
  };

  GZ_REGISTER_VISUAL_PLUGIN(TrajectoryVisualizer)
}
