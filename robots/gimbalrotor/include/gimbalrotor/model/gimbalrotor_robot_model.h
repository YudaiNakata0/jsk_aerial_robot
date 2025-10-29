// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class GimbalrotorRobotModel : public transformable::RobotModel
{
public:
  GimbalrotorRobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_t_min_thre = 0,
                        double epsilon = 10);
  virtual ~GimbalrotorRobotModel() = default;

  template <class T>
  std::vector<T> getLinksRotationFromCog();
  template <class T>
  std::vector<T> getThrustCoordRot();
  Eigen::MatrixXd calcDualAdjointFromKDLFrame(const KDL::Frame& T) const;
  Eigen::MatrixXd getDualAdjointMatrix(const std::string& target_frame,
                                               const std::string& ref_frame) const;

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  KDL::JntArray gimbal_processed_joint_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> thrust_coords_rot_;
  std::mutex links_rotation_mutex_;
  std::mutex thrust_rotation_mutex_;
};

template <>
inline std::vector<KDL::Rotation> GimbalrotorRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template <>
inline std::vector<KDL::Rotation> GimbalrotorRobotModel::getThrustCoordRot()
{
  std::lock_guard<std::mutex> lock(thrust_rotation_mutex_);
  return thrust_coords_rot_;
}

inline Eigen::MatrixXd GimbalrotorRobotModel::calcDualAdjointFromKDLFrame(const KDL::Frame& T) const
{
  Eigen::Matrix3d R = aerial_robot_model::kdlToEigen(T.M);
  Eigen::Vector3d p(T.p.x(), T.p.y(), T.p.z());
  Eigen::Matrix3d p_skew = aerial_robot_model::skew(p);

  Eigen::MatrixXd Ad_dual(6, 6);
  Ad_dual.setZero();
  Ad_dual.block<3,3>(0,0) = R;
  Ad_dual.block<3,3>(0,3) = p_skew * R;
  Ad_dual.block<3,3>(3,3) = R;
  return Ad_dual;
}

inline Eigen::MatrixXd GimbalrotorRobotModel::getDualAdjointMatrix(const std::string& target_frame, const std::string& ref_frame) const
{
  auto& tf_map = const_cast<GimbalrotorRobotModel*>(this)->getSegmentsTf();
  if (tf_map.find(target_frame) == tf_map.end() || tf_map.find(ref_frame) == tf_map.end())
    {
      ROS_ERROR_STREAM("[GimbalrotorRobotModel] invalid frame name in calcDualAdjointBetweenFrames: "
                       << ref_frame << " or " << target_frame);
      return Eigen::MatrixXd::Zero(6,6);
    }

  // transform from ref_frame to target_frame
  KDL::Frame ref_to_target = tf_map.at(ref_frame).Inverse() * tf_map.at(target_frame);
  return calcDualAdjointFromKDLFrame(ref_to_target);
}
