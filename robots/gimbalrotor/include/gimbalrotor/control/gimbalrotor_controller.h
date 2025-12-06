// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <thread>

namespace aerial_robot_control
{
class GimbalrotorController : public PoseLinearController
{
public:
  GimbalrotorController();
  ~GimbalrotorController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

private:
  ros::Publisher flight_cmd_pub_;
  ros::Publisher gimbal_control_pub_;
  ros::Publisher gimbal_state_pub_;
  ros::Publisher target_vectoring_force_pub_;
  ros::Publisher rpy_gain_pub_;                      // for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher gimbal_dof_pub_;                    // for spinal
  ros::Publisher feedforward_acc_cog_pub_;
  ros::Publisher feedforward_ang_acc_cog_pub_;
  ros::Publisher wrench_error_cog_pub_;
  ros::Publisher filtered_est_external_wrench_pub_;  // for wrenchcomp
  ros::Subscriber desire_wrench_sub_;                // for wrenchcomp
  ros::Subscriber attaching_flag_sub_;
  ros::Subscriber send_feedforward_switch_flag_sub_;
  ros::Subscriber xyz_wrench_control_flag_sub_;
  ros::Subscriber cog_x_vel_mode_sub_;  
  ros::Time time_hover_;

  boost::shared_ptr<GimbalrotorRobotModel> gimbalrotor_robot_model_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_full_thrust_;
  std::vector<double> target_gimbal_angles_;
  bool hovering_approximate_;
  Eigen::VectorXd target_vectoring_f_;
  Eigen::VectorXd target_vectoring_f_trans_;
  Eigen::VectorXd target_vectoring_f_rot_;
  Eigen::MatrixXd integrated_map_inv_trans_;
  Eigen::MatrixXd integrated_map_inv_rot_;
  Eigen::VectorXd estimated_external_wrench_in_cog_;
  Eigen::VectorXd desire_wrench_;
  Eigen::VectorXd desire_wrench_from_pos_;
  Eigen::VectorXd target_wrench_cog_;
  Eigen::VectorXd p_wrench_stamp_;
  Eigen::VectorXd feedforward_sum_;
  Eigen::VectorXd desire_pos_;
  Eigen::VectorXd filtered_ftsensor_wrench_;
  Eigen::VectorXd offset_external_wrench_;
  std::vector<Eigen::VectorXd> offset_sample_;

  double candidate_yaw_term_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;
  bool underactuate_;
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  bool send_feedforward_switch_flag_;
  bool offset_record_flag_;
  bool attaching_flag_, const_err_i_flag_, first_flag_;
  bool xyz_wrench_control_flag_;
  bool if_cog_x_vel_mode_;
  double err_i_x_, err_i_y_, err_i_z_, err_i_yaw_, err_p_y_;
  double wrench_diff_gain_;
  double acc_shock_thres_;
  double x_p_gain_, y_p_gain_;
  IirFilter lpf_est_external_wrench_;
  int flight_state_;
  double target_acc_gain_;
  double recording_start_time_;
  double recording_end_time_;

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
  void DesireWrenchCallback(geometry_msgs::WrenchStamped msg);
  void ExtWrenchControl();
  void AttachingFlagCallBack(std_msgs::Bool msg);
  void SendFeedforwardSwitchFlagCallBack(std_msgs::Bool msg);
  void FlightStateCallback(std_msgs::UInt8 msg);
  void XYZWrenchControlFlagCallBack(std_msgs::Bool msg);
  void COGXVelModeCallBack(std_msgs::Bool msg);
};
};  // namespace aerial_robot_control
