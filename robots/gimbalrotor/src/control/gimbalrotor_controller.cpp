#include <gimbalrotor/control/gimbalrotor_controller.h>

using namespace std;

namespace aerial_robot_control
{
GimbalrotorController::GimbalrotorController() : PoseLinearController()
{
}

void GimbalrotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  gimbalrotor_robot_model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);

  GimbalrotorController::rosParamInit();

  rotor_coef_ = gimbal_dof_ + 1;  // number of virtual rotors in each rotor arm

  target_base_thrust_.resize(motor_num_ * rotor_coef_);
  target_full_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
  //  for wrench comp
  feedforward_acc_cog_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("feedforward_acc_world", 1);
  feedforward_ang_acc_cog_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("feedforward_ang_acc_cog", 1);
  wrench_error_cog_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("wrench_error_cog", 1);
  attaching_flag_sub_ = nh_.subscribe("attaching_flag", 1, &GimbalrotorController::AttachingFlagCallBack, this);
  send_feedforward_switch_flag_sub_ = nh_.subscribe("send_feedforward_switch_flag", 1, &GimbalrotorController::SendFeedforwardSwitchFlagCallBack, this);
  xyz_wrench_control_flag_sub_ = nh_.subscribe("xyz_wrench_control_flag", 1, &GimbalrotorController::XYZWrenchControlFlagCallBack, this);
  filtered_est_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("filtered_est_external_wrench",1);
  desire_wrench_sub_ = nh_.subscribe("desire_wrench", 1, &GimbalrotorController::DesireWrenchCallback, this);
  // body_x_vel_mode_sub_ = nh_.subscribe("body_x_vel_mode", 1, &GimbalrotorController::BodyXVelModeCallBack, this);
  desire_pos_for_impedance_sub_ = nh_.subscribe("desire_pos_for_impedance", 1, &GimbalrotorController::DesirePosImpedanceCallback, this);
  // impedance: debug output of impedance force (world frame, [N])
  impedance_force_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("debug/impedance_force", 1);
  // impedance: direction of impedance (e.g. surface normal, world frame)
  impedance_direction_sub_ = nh_.subscribe("impedance_direction", 1, &GimbalrotorController::ImpedanceDirectionCallback, this);
  // impedance: on/off of impedance control
  impedance_flag_sub_ = nh_.subscribe("impedance_flag", 1, &GimbalrotorController::ImpedanceFlagCallback, this);
  estimated_external_wrench_in_cog_ = Eigen::VectorXd::Zero(6);
  desire_wrench_ = Eigen::VectorXd::Zero(6);
  filtered_ftsensor_wrench_ = Eigen::VectorXd::Zero(6);
  desire_wrench_from_pos_ = Eigen::VectorXd::Zero(6);
  target_wrench_cog_ = Eigen::VectorXd::Zero(6);
  p_wrench_stamp_ = Eigen::VectorXd::Zero(6);
  feedforward_sum_ = Eigen::VectorXd::Zero(6);
  attaching_flag_ = false;
  xyz_wrench_control_flag_ = false;
  if_body_x_vel_mode_ = false;
  const_err_i_flag_ = false;
  first_flag_ = true;
  offset_record_flag_ = false;
  offset_external_wrench_ = Eigen::VectorXd::Zero(6);
  prev_p_term_ = Eigen::VectorXd::Zero(6);
  offset_p_term_bx_ = Eigen::VectorXd::Zero(3);
  offset_p_term_by_ = Eigen::VectorXd::Zero(3);
  offset_p_term_bz_ = Eigen::VectorXd::Zero(3);
  desire_pos_for_impedance_ = Eigen::VectorXd::Zero(3);
  desire_pos_for_impedance_received_ = false;
  impedance_flag_ = false;
  prev_impedance_flag_ = false;
  prev_impedance_active_ = false;
  impedance_dir_ = Eigen::Vector3d::UnitX();
  impedance_dir_received_ = false;
  
  flight_state_ = 0;
  target_acc_gain_ = 1.0;
}

void GimbalrotorController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();

  // impedance: turn off impedance control (e.g. after landing)
  impedance_flag_ = false;
  prev_impedance_flag_ = false;
  prev_impedance_active_ = false;
  use_fixed_body_x_dir_ = false;
}

void GimbalrotorController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
  getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "underactuate", underactuate_, false);
  //  for wrench comp
  getParam<double>(control_nh, "wrench_diff_gain", wrench_diff_gain_, 1.0);
  getParam<bool>(control_nh, "send_feedforward_switch_flag", send_feedforward_switch_flag_, false);
  getParam<double>(control_nh, "acc_shock_thres", acc_shock_thres_, 20.0);
  double cutoff_freq, sample_freq;
  getParam<double>(control_nh, "cutoff_freq", cutoff_freq, 25.0);
  getParam<double>(control_nh, "sample_freq", sample_freq, 100.0);
  lpf_est_external_wrench_ = IirFilter(sample_freq, cutoff_freq, 6);

  double recording_time;
  getParam<double>(control_nh, "wrench_record/start_time", recording_start_time_, 1.0);
  getParam<double>(control_nh, "wrench_record/recording_time", recording_time, 1.0);
  recording_end_time_ = recording_start_time_ + recording_time;

  x_p_gain_ = pid_controllers_.at(X).getPGain();
  y_p_gain_ = pid_controllers_.at(Y).getPGain();

  /* impedance */
  getParam<double>(control_nh, "impedance/spring_gain", K_imp_, 1.0);
  getParam<double>(control_nh, "impedance/force_limit", limit_F_imp_, 5.0);
  // impedance: additional damping along body x (the PID D term also acts as damping in body x velocity mode)
  getParam<double>(control_nh, "impedance/damping_gain", D_imp_, 0.0);
}

bool GimbalrotorController::update()
{
  sendGimbalCommand();
  if (gimbal_calc_in_fc_)
  {
    std_msgs::UInt8 msg;
    msg.data = gimbal_dof_;
    gimbal_dof_pub_.publish(msg);
  }

  return PoseLinearController::update();
}

void GimbalrotorController::controlCore()
{
  ExtWrenchControl();
  PoseLinearController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

  // if(if_body_x_vel_mode_)
  //   {
  //     // tf::Vector3 p_term_w(pid_controllers_.at(X).getPTerm(),
  //     // 			   pid_controllers_.at(Y).getPTerm(),
  //     // 			   pid_controllers_.at(Z).getPTerm());
  //     // tf::Vector3 p_term_cog = uav_rot.inverse() * p_term_w;
  //     // target_acc_cog[0] -= p_term_cog[0];
  //     target_acc_cog[0] = pid_controllers_body_.at(X).result();
  //     target_acc_cog[1] = pid_controllers_body_.at(Y).result();
  //     target_acc_cog[2] = pid_controllers_body_.at(Z).result();
  //     target_acc_cog = uav_rot.inverse() * target_acc_cog;
  //   }

  if (underactuate_)
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
  else
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  if (gimbal_calc_in_fc_)
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  else
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + gyro;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

  double mass_inv = 1 / gimbalrotor_robot_model_->getMass();

  Eigen::Matrix3d inertia_inv = inertia.inverse();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
  const double m_f_rate = gimbalrotor_robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;

  /* calculate normal allocation */
  for (int i = 0; i < motor_num_; i++)
  {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  /* calculate masked rotation matrix */
  std::vector<KDL::Rotation> thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();
  std::vector<Eigen::MatrixXd> masked_rot;
  for (int i = 0; i < motor_num_; i++)
  {
    tf::Quaternion r;
    tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
    Eigen::Matrix3d conv_cog_from_thrust;
    tf::matrixTFToEigen(tf::Matrix3x3(r), conv_cog_from_thrust);
    if (gimbal_dof_ == 1)
    {
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
    else if (gimbal_dof_ == 2)
    {
      Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
  }

  /* mask integrated allocation */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
  Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, (gimbal_dof_ + 1) * motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    integrated_rot.block(3 * i, rotor_coef_ * i, 3, rotor_coef_) = masked_rot[i];
  }
  integrated_map = full_q_mat * integrated_rot;

  /* extract controlled axis  */
  if (underactuate_)
  {
    target_wrench_acc_cog = target_wrench_acc_cog.tail(4);  // z, roll, pitch, yaw
    integrated_map = integrated_map.bottomRows(4);          // z, roll, pitch, yaw
  }

  /* vectoring force mapping */
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
  integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
  integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
  if (underactuate_)
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
  else
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
  target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);  // debug
  last_col = 0;

  /* under actuated axis  */
  if (underactuate_)
  {
    if (hovering_approximate_)
    {
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
    else
    {
      target_roll_ = atan2(-target_acc_dash.y(),
                           sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
  }

  /*  calculate target base thrust (considering only translational components)*/
  double max_yaw_scale = 0;  // for reconstruct yaw control term in spinal
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    if (gimbal_dof_ == 1)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
    }
    else if (gimbal_dof_ == 2)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
      target_base_thrust_.at(rotor_coef_ * i + 2) = f_i[2];
    }
    if (integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW)) > max_yaw_scale)
      max_yaw_scale = integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW));  // underactuated: yaw col is shifted

    last_col += rotor_coef_;
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  /* calculate target full thrusts and gimbal angles (considering full components)*/
  last_col = 0;
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) +
                                     target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    target_full_thrust_.at(i) = f_i_integrated.norm();
    if (gimbal_dof_ == 1)
    {
      target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
    }
    else if (gimbal_dof_ == 2)
    {
      if (f_i_integrated[0] == 0 || f_i_integrated[2] == 0)
        continue;

      double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
      double gimbal_pitch =
          atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2] * cos(gimbal_roll));
      target_gimbal_angles_.at(2 * i) = gimbal_roll;
      target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
    }
    last_col += rotor_coef_;
  }
}

void GimbalrotorController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();

  if (gimbal_calc_in_fc_)
  {
    sendTorqueAllocationMatrixInv();
 }
  else
  {
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < motor_num_; i++)
    {
      if (gimbal_dof_ == 1)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
      }
      else if (gimbal_dof_ == 2)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i));
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      }
    }
    gimbal_control_pub_.publish(gimbal_control_msg);

    std_msgs::Float32MultiArray target_vectoring_force_msg;
    target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
    for (int i = 0; i < target_vectoring_f_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
    }
    target_vectoring_force_pub_.publish(target_vectoring_force_msg);
  }
}

void GimbalrotorController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;

  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;

  if (gimbal_calc_in_fc_)
  {
    flight_command_data.base_thrust = target_base_thrust_;
    flight_command_data.angles[2] = candidate_yaw_term_;
  }
  else
  {
    flight_command_data.base_thrust = target_full_thrust_;
  }

  flight_cmd_pub_.publish(flight_command_data);
}

void GimbalrotorController::sendGimbalCommand()
{
  sensor_msgs::JointState gimbal_state_msg;
  gimbal_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_; i++)
  {
    if (gimbal_dof_ == 1)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
      std::string gimbal_name = "gimbal" + std::to_string(i + 1);
      gimbal_state_msg.name.push_back(gimbal_name);
    }
    else if (gimbal_dof_ == 2)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i));
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
      std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";
      gimbal_state_msg.name.push_back(gimbal_roll_name);
      gimbal_state_msg.name.push_back(gimbal_pitch_name);
    }
  }
  // gimbal_state_pub_.publish(gimbal_state_msg);
}

void GimbalrotorController::sendTorqueAllocationMatrixInv()
{
  spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
  torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
  Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
  if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
    ROS_ERROR("Torque Allocation Matrix overflow");
  for (unsigned int i = 0; i < motor_num_ * rotor_coef_; i++)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
  }
  torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
}

void GimbalrotorController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void GimbalrotorController::DesireWrenchCallback(geometry_msgs::WrenchStamped msg)
{
  const std::string src_frame = !msg.header.frame_id.empty() ? msg.header.frame_id : "fc";
  KDL::Wrench w_end(
                    KDL::Vector(msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z),
                    KDL::Vector(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z)
                    );

  KDL::Frame end_to_cog = gimbalrotor_robot_model_->getKdlFrameFromCog(src_frame);
  KDL::Wrench w_cog = end_to_cog * w_end;
  desire_wrench_.head<3>() = aerial_robot_model::kdlToEigen(w_cog.force);
  desire_wrench_.tail<3>() = aerial_robot_model::kdlToEigen(w_cog.torque);
  // // apply wrench offset
  // if(std::isfinite(offset_external_wrench_(2)))
  //   {
  //     desire_wrench_ -= offset_external_wrench_;
  //   }
}

void GimbalrotorController::ExtWrenchControl(){
  if(first_flag_)
  {
    lpf_est_external_wrench_.setInitValues(est_external_wrench_);
    first_flag_ = false;
  }
  Eigen::VectorXd filtered_est_external_wrench;
  filtered_est_external_wrench = lpf_est_external_wrench_.filterFunction(est_external_wrench_);

  // // record offset external wrench
  // if(!offset_record_flag_ && navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE)
  // {
  //   if(time_hover_.isZero())
  //     {
  // 	time_hover_ = ros::Time::now();
  // 	offset_sample_.clear();
  // 	ROS_INFO("[gimbalrotor_controller]Start offset recording...");
  //     }
  //   ros::Duration duration = ros::Time::now() - time_hover_;
  //   if(duration.toSec() > recording_start_time_ && duration.toSec() <= recording_end_time_)
  //     {
  // 	offset_sample_.push_back(filtered_est_external_wrench);
  //     }
  //   else if(duration.toSec() > recording_end_time_)
  //     {
  // 	Eigen::VectorXd avg = Eigen::VectorXd::Zero(6);
  // 	for(const auto& w : offset_sample_){avg += w;}
  // 	avg /= (double) offset_sample_.size();
  // 	offset_external_wrench_ = avg;
  // 	if(std::isfinite(offset_external_wrench_(2)))
  // 	  {
  // 	    ROS_INFO("[gimbalrotor_contorller]Recorded external wrench for offset: "
  // 		     "Force: [%.6f, %.6f, %.6f], Torque: [%.6f, %.6f, %.6f]",
  // 		     offset_external_wrench_(0),
  // 		     offset_external_wrench_(1),
  // 		     offset_external_wrench_(2),
  // 		     offset_external_wrench_(3),
  // 		     offset_external_wrench_(4),
  // 		     offset_external_wrench_(5));
  // 	    offset_record_flag_ = true;
  // 	    desire_wrench_ -= offset_external_wrench_;
  // 	  }
  // 	else{ROS_INFO("[gimbalrotor_controller]Could not record external wrench for offset.");}
  //     }
  // }
  // // reset offset external wrench (when landed or stopped)
  // if(navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE ||
  //    navigator_->getNaviState() == aerial_robot_navigation::STOP_STATE)
  //   {
  //     if(offset_record_flag_)
  // 	{
  // 	  ROS_INFO("[gimbalrotor_contorller]Reset offset wrench");
  // 	  offset_sample_.clear();
  // 	  offset_record_flag_ = false;
  // 	  offset_external_wrench_ = Eigen::VectorXd::Zero(6);
  // 	  time_hover_ = ros::Time(0);
  // 	}
  //   }

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  //double target_ang_acc_z = pid_controllers_.at(YAW).result();
  double target_ang_acc_z = candidate_yaw_term_;
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  /* feedforward */
  double mass_inv = 1/ gimbalrotor_robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

  Eigen::Vector3d force_error, torque_error;
  force_error = desire_wrench_.head(3); // + cog_rot.inverse() * filtered_est_external_wrench.head(3);
  torque_error = desire_wrench_.tail(3); //+ cog_rot.inverse() * filtered_est_external_wrench.tail(3);

  Eigen::Vector3d target_acc = target_acc_gain_ * mass_inv * force_error;

  /* impedance on/off: switch body x control mode along with impedance_flag_ */
  // done here (control loop thread) instead of the callback, to avoid race with navigator/controller states
  if(impedance_flag_ != prev_impedance_flag_){
    if(impedance_flag_){
      // on: body x velocity mode (same process as BaseNavigator::bodyXControlModeCallback with 1)
      navigator_->setTargetZeroVel();
      navigator_->setTargetZeroAcc();
      navigator_->setBodyXControlMode(aerial_robot_navigation::VEL_CONTROL_MODE);
      ROS_INFO("[gimbalrotor_controller] impedance: on, (body)x velocity control mode");
    }
    else{
      // off: body x position mode (same process as BaseNavigator::bodyXControlModeCallback with 0)
      tf::Vector3 pos_cog = estimator_->getPos(Frame::COG, estimate_mode_);
      navigator_->setTargetPos(pos_cog);
      navigator_->setXControlMode(aerial_robot_navigation::POS_CONTROL_MODE);
      navigator_->setYControlMode(aerial_robot_navigation::POS_CONTROL_MODE);
      navigator_->setTargetZeroVel();
      navigator_->setTargetZeroAcc();
      navigator_->setBodyXControlMode(aerial_robot_navigation::POS_CONTROL_MODE);
      ROS_INFO("[gimbalrotor_controller] impedance: off, (body)x position control mode");
    }
    prev_impedance_flag_ = impedance_flag_;
  }

  /* impedance (spring-damper along fixed direction n, active when impedance_flag_ is on in body x velocity mode) */
  // F_imp = K_imp * (x_d - x).n + D_imp * (0 - v).n, applied along n in world frame
  // - n is fixed during impedance mode (not the current body x axis), to avoid the coupling between
  //   attitude fluctuation and position control (the same n is used for projection in PoseLinearController)
  // - n is given by impedance_direction topic (e.g. surface normal), otherwise the body x axis at the switch
  // - desire_pos_for_impedance_ is the target CoG position in world frame
  // - pos/vel/orientation are taken from the estimator here, because pos_, vel_ and body_orientation_
  //   are updated in PoseLinearController::controlCore(), which is called after this function
  // - the result is sent as feedforward acc via navigator, so it takes effect in the next control cycle
  // body x velocity mode without impedance_flag_ is a plain velocity mode (no impedance force)
  bool impedance_active = impedance_flag_ &&
    navigator_->getBodyXControlMode() == aerial_robot_navigation::VEL_CONTROL_MODE;
  Eigen::Vector3d F_imp_w = Eigen::Vector3d::Zero();
  if(impedance_active){
    tf::Vector3 pos_tf = estimator_->getPos(Frame::COG, estimate_mode_);
    tf::Vector3 vel_tf = estimator_->getVel(Frame::COG, estimate_mode_);
    Eigen::Vector3d pos_cur(pos_tf.x(), pos_tf.y(), pos_tf.z());
    Eigen::Vector3d vel_cur(vel_tf.x(), vel_tf.y(), vel_tf.z());

    // initialize target position with current position when switched into impedance mode
    // (unless a target was given in advance), to avoid a force step at the switch
    if(!prev_impedance_active_ && !desire_pos_for_impedance_received_){
      desire_pos_for_impedance_ = pos_cur;
      ROS_INFO("[gimbalrotor_controller] impedance: target pos is initialized by current pos [%.3f, %.3f, %.3f]",
               pos_cur(0), pos_cur(1), pos_cur(2));
    }

    // latch the impedance direction when switched into impedance mode
    if(!prev_impedance_active_ && !impedance_dir_received_){
      // body x axis in world frame at the switch
      Eigen::Matrix3d baselink_rot;
      tf::matrixTFToEigen(estimator_->getOrientation(Frame::BASELINK, estimate_mode_), baselink_rot);
      impedance_dir_ = baselink_rot.col(0);
      ROS_INFO("[gimbalrotor_controller] impedance: direction is fixed to current body x [%.3f, %.3f, %.3f]",
               impedance_dir_(0), impedance_dir_(1), impedance_dir_(2));
    }
    const Eigen::Vector3d& bx = impedance_dir_;

    // use the same fixed direction for projection of position error in PoseLinearController::controlCore()
    use_fixed_body_x_dir_ = true;
    fixed_body_x_dir_.setValue(bx(0), bx(1), bx(2));

    // project world frame errors onto n (previously (R * pos).x() was used, which is not the body frame value)
    double err_pos_bx = (desire_pos_for_impedance_ - pos_cur).dot(bx);
    double err_vel_bx = -vel_cur.dot(bx);
    double F_imp_bx = K_imp_ * err_pos_bx + D_imp_ * err_vel_bx;
    F_imp_bx = clamp(F_imp_bx, -limit_F_imp_, limit_F_imp_);

    // apply as a world frame vector along n (previously added to force_error(0), i.e., world x)
    F_imp_w = F_imp_bx * bx;
    target_acc += target_acc_gain_ * mass_inv * F_imp_w;
  }
  else{
    // target received outside impedance mode is kept for the next switch; reset it once used
    if(prev_impedance_active_){
      desire_pos_for_impedance_received_ = false;
      impedance_dir_received_ = false;
    }
    // return to the current body x axis for projection in PoseLinearController
    use_fixed_body_x_dir_ = false;
  }
  prev_impedance_active_ = impedance_active;
  Eigen::Vector3d target_ang_acc = target_acc_gain_ * inertia_inv * torque_error;
  Eigen::Vector3d feedforward_acc = cog_rot * (target_acc + feedforward_sum_.head(3));
  Eigen::Vector3d feedforward_ang_acc = cog_rot * (target_ang_acc + feedforward_sum_.tail(3));

  if(send_feedforward_switch_flag_ && attaching_flag_)
  {
    // target_pitch_ += target_acc[0];
    // target_roll_ += target_acc[1];
    navigator_->setXControlMode(1);
    navigator_->setYControlMode(1);
    navigator_->setTargetAccX(target_acc[0]);
    navigator_->setTargetAccY(target_acc[0]);
    // navigator_->setTargetAccY(feedforward_acc[1]);
    // navigator_->setTargetAngAccZ(feedforward_ang_acc[2]);
    // target_wrench_acc_cog[0] += feedforward_acc[0];
    // target_wrench_acc_cog[1] += feedforward_acc[1];
    // target_wrench_acc_cog[5] += feedforward_ang_acc[2];

    // feedforward_sum_.head(3) += target_acc * wrench_diff_gain_;
    // feedforward_sum_.tail(3) += target_ang_acc * wrench_diff_gain_;

    // std::cout << "send_feedforward" << std::endl;
  }
  // if(!attaching_flag_)
  //   {
  //   navigator_->setTargetAccX(0);
  //   // navigator_->setTargetAccY(0);
  //   // navigator_->setTargetAngAccZ(0);
  //   feedforward_sum_ = Eigen::VectorXd::Zero(6);
  // }

  // record offset(p term)
  if(navigator_->getXControlMode() == 0){prev_p_term_[0] = pid_controllers_.at(X).getPTerm();}
  else if(navigator_->getXControlMode() == 1){target_acc[0] += prev_p_term_[0];}
  if(navigator_->getYControlMode() == 0){prev_p_term_[1] = pid_controllers_.at(Y).getPTerm();}
  else if(navigator_->getYControlMode() == 1){target_acc[1] += prev_p_term_[1];}
  if(navigator_->getZControlMode() == 0){prev_p_term_[2] = pid_controllers_.at(Z).getPTerm();}
  else if(navigator_->getZControlMode() == 1){target_acc[2] += prev_p_term_[2];}

  if(navigator_->getBodyXControlMode() == 0){offset_p_term_bx_ = prev_p_term_.head(3).dot(w_base_bx_) * w_base_bx_;}
  else if(navigator_->getBodyXControlMode() == 1){target_acc += offset_p_term_bx_;}
  if(navigator_->getBodyYControlMode() == 0){offset_p_term_by_ = prev_p_term_.head(3).dot(w_base_by_) * w_base_by_;}
  else if(navigator_->getBodyYControlMode() == 1){target_acc += offset_p_term_by_;}
  if(navigator_->getBodyZControlMode() == 0){offset_p_term_bz_ = prev_p_term_.head(3).dot(w_base_bz_) * w_base_bz_;}
  else if(navigator_->getBodyZControlMode() == 1){target_acc += offset_p_term_bz_;}

  navigator_->setTargetAccX(target_acc[0]);
  navigator_->setTargetAccY(target_acc[1]);
  navigator_->setTargetAccZ(target_acc[2]);

  if(xyz_wrench_control_flag_ || if_body_x_vel_mode_){
    // Eigen::VectorXd external_acc =  mass_inv * offset_external_wrench_;
    // external_acc[2] -= 9.80665;
    // double x_i_term = pid_controllers_.at(X).getITerm();
    // double y_i_term = pid_controllers_.at(Y).getITerm();
    // double z_i_term = pid_controllers_.at(Z).getITerm();
    // Eigen::Vector3d adjust_acc;
    // adjust_acc[0] = -external_acc[0] - x_i_term;
    // adjust_acc[1] = -external_acc[1] - y_i_term;
    // adjust_acc[2] = -external_acc[2] - z_i_term;
    // ROS_INFO("[gimbalrotor_contorller]adjust acc: [%.6f, %.6f, %.6f]",
    // 	     adjust_acc[0],
    // 	     adjust_acc[1],
    // 	     adjust_acc[2]);
    // if(std::isfinite(adjust_acc[2]))
    //   {
    // 	target_acc[0] = target_acc[0] + adjust_acc[0];
    // 	target_acc[1] = target_acc[1] + adjust_acc[1];
    // 	target_acc[2] = target_acc[2] + adjust_acc[2];
    //   }
    // target_acc[0] = target_acc[0] - x_i_term;
    // target_acc[1] = target_acc[1] - y_i_term;
    // target_acc[2] = target_acc[2] - z_i_term + 9.80665;
    navigator_->setTargetAccX(target_acc[0]);
    navigator_->setTargetAccY(target_acc[1]);
    navigator_->setTargetAccZ(target_acc[2]);
  }
  
  if(pid_controllers_.at(X).result()<0.0)
  {
    //attaching_flag_ = false;
  }
    
  // during attaching
  //   if(attaching_flag_)
  //   {
  //     if(!const_err_i_flag_)
  //       {
  //         err_i_x_ = pid_controllers_.at(X).getErrI();
  //         err_i_y_ = pid_controllers_.at(Y).getErrI();
  //         err_i_z_ = pid_controllers_.at(Z).getErrI();
  //         // err_i_yaw_ = pid_controllers_.at(YAW).getErrI();
  //         x_p_gain_ = pid_controllers_.at(X).getPGain();
  //         y_p_gain_ = pid_controllers_.at(Y).getPGain();
  //         //err_p_y_ = pid_controllers_.at(Y).getErrP();
  //         const_err_i_flag_ = true;
  //       }
  //     pid_controllers_.at(X).setErrI(err_i_x_);
  //     pid_controllers_.at(Y).setErrI(err_i_y_);
  //     pid_controllers_.at(Z).setErrI(err_i_z_);
  //     // pid_controllers_.at(YAW).setErrI(err_i_yaw_);
  //     //pid_controllers_.at(Y).setErrP(0);
  //     // pid_controllers_.at(X).setPGain(0.0);
  //     pid_controllers_.at(Y).setPGain(0.0);
  //   }
  // if(!attaching_flag_)
  // {
  //   // pid_controllers_.at(X).setPGain(x_p_gain_);
  //   pid_controllers_.at(Y).setPGain(y_p_gain_);
  //   const_err_i_flag_ = false;
  // }
  geometry_msgs::Vector3Stamped feedforward_acc_cog_msg;
  geometry_msgs::Vector3Stamped feedforward_ang_acc_cog_msg;
  geometry_msgs::WrenchStamped wrench_error_cog_msg;
  geometry_msgs::WrenchStamped filtered_est_external_wrench_msg;
  feedforward_acc_cog_msg.vector.x = feedforward_acc[0];
  feedforward_acc_cog_msg.vector.y = feedforward_acc[1];
  feedforward_acc_cog_msg.vector.z = feedforward_acc[2];
  feedforward_ang_acc_cog_msg.vector.x = feedforward_ang_acc[0];
  feedforward_ang_acc_cog_msg.vector.y = feedforward_ang_acc[1];
  feedforward_ang_acc_cog_msg.vector.z = feedforward_ang_acc[2];
  wrench_error_cog_msg.wrench.force.x = force_error[0];
  wrench_error_cog_msg.wrench.force.y = force_error[1];
  wrench_error_cog_msg.wrench.force.z = force_error[2];
  wrench_error_cog_msg.wrench.torque.x = torque_error[0];
  wrench_error_cog_msg.wrench.torque.y = torque_error[1];
  wrench_error_cog_msg.wrench.torque.z = torque_error[2];
  filtered_est_external_wrench_msg.wrench.force.x = filtered_est_external_wrench[0];
  filtered_est_external_wrench_msg.wrench.force.y = filtered_est_external_wrench[1];
  filtered_est_external_wrench_msg.wrench.force.z = filtered_est_external_wrench[2];
  filtered_est_external_wrench_msg.wrench.torque.x = filtered_est_external_wrench[3];
  filtered_est_external_wrench_msg.wrench.torque.y = filtered_est_external_wrench[4];
  filtered_est_external_wrench_msg.wrench.torque.z = filtered_est_external_wrench[5];

  feedforward_acc_cog_pub_.publish (feedforward_acc_cog_msg);
  feedforward_ang_acc_cog_pub_.publish(feedforward_ang_acc_cog_msg);
  wrench_error_cog_pub_.publish(wrench_error_cog_msg);
  filtered_est_external_wrench_pub_.publish(filtered_est_external_wrench_msg);
  // impedance: debug
  geometry_msgs::Vector3Stamped impedance_force_msg;
  impedance_force_msg.header.stamp = ros::Time::now();
  impedance_force_msg.vector.x = F_imp_w(0);
  impedance_force_msg.vector.y = F_imp_w(1);
  impedance_force_msg.vector.z = F_imp_w(2);
  impedance_force_pub_.publish(impedance_force_msg);
  setTargetWrenchAccCog(target_wrench_acc_cog);

}

void GimbalrotorController::AttachingFlagCallBack(std_msgs::Bool msg)
{
  attaching_flag_ = msg.data;
}

void GimbalrotorController::SendFeedforwardSwitchFlagCallBack(std_msgs::Bool msg)
{
  send_feedforward_switch_flag_ = msg.data;
}

void GimbalrotorController::XYZWrenchControlFlagCallBack(std_msgs::Bool msg)
{
  xyz_wrench_control_flag_ = msg.data;
  if(!msg.data)
    {
      tf::Vector3 pos_cog = estimator_->getPos(Frame::COG, estimate_mode_);
      navigator_->setTargetPosX(pos_cog.x());
      navigator_->setTargetPosY(pos_cog.y());
      navigator_->setTargetPosZ(pos_cog.z());
      navigator_->setXControlMode(0);
      navigator_->setYControlMode(0);
      navigator_->setZControlMode(0);
    }
}

void GimbalrotorController::BodyXVelModeCallBack(std_msgs::Bool msg)
{
  if_body_x_vel_mode_ = msg.data;
}

void GimbalrotorController::DesirePosImpedanceCallback(geometry_msgs::Vector3 msg)
{
  // impedance: target CoG position in world frame
  desire_pos_for_impedance_ << msg.x, msg.y, msg.z;
  desire_pos_for_impedance_received_ = true;
}

void GimbalrotorController::ImpedanceFlagCallback(std_msgs::Bool msg)
{
  // impedance: only store the request here; body x control mode is switched in ExtWrenchControl()
  if(msg.data && navigator_->getNaviState() <= aerial_robot_navigation::START_STATE)
    {
      ROS_WARN("[gimbalrotor_controller] impedance: ignore on request before takeoff");
      return;
    }
  impedance_flag_ = msg.data;
}

void GimbalrotorController::ImpedanceDirectionCallback(geometry_msgs::Vector3 msg)
{
  // impedance: direction of impedance in world frame (e.g. surface normal), normalized here
  Eigen::Vector3d dir(msg.x, msg.y, msg.z);
  if(dir.norm() < 1e-6)
    {
      ROS_WARN("[gimbalrotor_controller] impedance: ignore zero direction");
      return;
    }
  impedance_dir_ = dir.normalized();
  impedance_dir_received_ = true;
}

}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorController, aerial_robot_control::ControlBase);
