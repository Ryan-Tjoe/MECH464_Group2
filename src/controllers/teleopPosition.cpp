#include <teleop/teleopPosition.h>
#include <hardware_interface/hardware_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace teleop {
  bool TeleopPosition::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
    if (!initBase(node_handle)) return false;

    // These are the robot's maximum rated values, but they're unnecessarily (and uncomfortably) fast
    // dqMax << 2.1, 2.1, 2.1, 2.1, 2.5, 2.5, 2.5;
    // ddqMax << 15, 7.5, 10, 12.5, 15, 20, 20;
    // dddqMax << 7500, 3750, 5000, 6250, 7500, 10000, 10000;

    //dqMax << 1.8, 1.8, 1.8, 1.8, 2.0, 2.0, 2.0;
    dqMax << 1.2, 1.2, 1.2, 1.2, 1.6, 1.6, 1.6;
    ddqMax << 6, 3, 4, 6, 6, 12, 12;
    dddqMax << 4000, 2000, 2000, 3000, 4000, 5000, 5000;

    try {
      k_p = get7dParam("p_gains", node_handle).asDiagonal();
      k_d = get7dParam("d_gains", node_handle).asDiagonal();
      k_dq_ = get7dParam("drift_comp_gains", node_handle).asDiagonal();
      dq_max_lower_ = get7dParam("dq_max_lower", node_handle);
      dq_max_upper_ = get7dParam("dq_max_upper", node_handle);
      ddq_max_lower_ = get7dParam("ddq_max_lower", node_handle);
      ddq_max_upper_ = get7dParam("ddq_max_upper", node_handle);
      const std::vector<std::string> joint_names = getJointParams<std::string>("joint_names", node_handle);
    
      // Get joint torque interface
      auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
      if (not effort_joint_interface) {
        throw std::invalid_argument("TeleopPosition: Error getting effort joint interface from hardware of " + arm_id_ + ".");
      }
      joint_handles_.clear();
      for (const auto& name : joint_names) {
        try {
          joint_handles_.push_back(effort_joint_interface->getHandle(name));
        } catch (const hardware_interface::HardwareInterfaceException& e) {
          throw std::invalid_argument("TeleopPosition: Exception getting joint handle: " + std::string(e.what()));
        }
      }

#ifndef CUSTOM_IK
      // Get Cartesian pose interface
      auto* cartesian_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
      if (not cartesian_interface)
        throw std::invalid_argument("TeleopPosition: Error getting cartesian interface from hardware of " + arm_id_ + ".");
      try
      {
        for (const auto& name : cartesian_interface->getNames())
          LOG << name;
        cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
              cartesian_interface->getHandle(arm_id_ + "_robot"));
      } catch (const hardware_interface::HardwareInterfaceException& e)
      {
        throw std::invalid_argument("TeleopPosition: Exception getting joint handle: " + std::string(e.what()));
      }
#endif

      // Get state interface.
      auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
      if (not state_interface) {
        throw std::invalid_argument("TeleopPosition: Error getting state interface from hardware.");
      }
      try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id_ + "_robot"));
      } catch (hardware_interface::HardwareInterfaceException& ex) {
        throw std::invalid_argument("TeleopPosition: Exception getting state handle from interface: " + std::string(ex.what()));
      }

      // And the model interface as well
      auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
      if (model_interface == nullptr) {
        LOG << "TeleopPosition: Error getting model interface from hardware";
      } else
      {
        try {
          model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id_ + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
          LOG << "TeleopPosition: Exception getting model handle from interface: " << ex.what();
        }
      }

      // Setup joint walls
      // Virtual joint position wall parameters
      const std::array<double, 7> kPDZoneWidth = {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
      const std::array<double, 7> kDZoneWidth = {{0.12, 0.09, 0.09, 0.09, 0.0349, 0.0349, 0.0349}};
      const std::array<double, 7> kPDZoneStiffness = {{2000.0, 2000.0, 1000.0, 1000.0, 500.0, 200.0, 200.0}};
      const std::array<double, 7> kPDZoneDamping = {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};
      const std::array<double, 7> kDZoneDamping = {{30.0, 30.0, 30.0, 10.0, 5.0, 5.0, 5.0}};
      std::array<double, 7> upper_joint_soft_limit{};
      std::array<double, 7> lower_joint_soft_limit{};
      getJointLimits(node_handle, joint_names, upper_joint_soft_limit, lower_joint_soft_limit);
      virtual_joint_wall_ = std::make_unique<JointWallContainer<7>>(
          upper_joint_soft_limit, lower_joint_soft_limit, kPDZoneWidth, kDZoneWidth, kPDZoneStiffness,
          kPDZoneDamping, kDZoneDamping);
    } catch (const std::invalid_argument& ex) {
      LOG << ex.what();
      return false;
    }

    return true;
  }

  void TeleopPosition::starting(const ros::Time& /*time*/) {
    // Reset joint walls to start from the current q, dq
    virtual_joint_wall_->reset();

    // Set joint parameters to the current states.
    franka::RobotState initial_state = state_handle_->getRobotState();
    q_d_last_ = Eigen::Map<Vector7d>(initial_state.q.data());
    q_d_ = q_d_last_;
    q_init_ = q_d_last_;
    q_d_target_ = q_d_;
    dq_d_last_.setZero();
    tau_d_last_.setZero();
    velocity_m_.setZero();
    
    ruckig_input_.current_position << q_d_;
    ruckig_input_.current_velocity.setZero();
    ruckig_input_.current_acceleration.setZero();

    // Set Cartesian pose to current state
    auto transform = Isometry3d(Matrix4d::Map(initial_state.O_T_EE.data()));
    const Isometry3d eTf = Isometry3d(array2eigen<4,4>(initial_state.F_T_EE)).inverse();
    initial_pose_ = transform * eTf;

    position_d_ << initial_pose_.translation();
    orientation_d_ = Quaterniond(initial_pose_.rotation());
    position_d_target_ << initial_pose_.translation();
    orientation_d_target_ = Quaterniond(initial_pose_.rotation());
    initial_position_ << initial_pose_.translation();
    initial_orientation_ = Quaterniond(initial_pose_.rotation());
    position_m_ << initial_pose_.translation();
    orientation_m_ = Quaterniond(initial_pose_.rotation());

    teleop_state_ = TeleopState::ALIGN;

#ifdef CUSTOM_IK
    if (!ikThread.joinable())
      ikThread = std::thread([this](){ikThreadLoop();});
#endif
  }

#ifdef CUSTOM_IK
  void TeleopPosition::ikThreadLoop() {
    // Run inverse kinematics on a separate thread because
    // (1) It only has to run when a new command arrives, not every control loop
    // (2) It slows down the control loop and causes reflexes
    // (3) Using Franka's internal inverse kinematics solver requires compliance with their jerk limits which are way too low
    LOG << "Created inverse kinematics thread";
    while (runLoop)
    {
      // Get next setpoint pose
      if (updateBase(0.001)) {
        Isometry3d pose = Isometry3d::Identity();
        pose = pose.rotate(orientation_d_).pretranslate(position_d_);
        
        // Get current joints
        franka::RobotState robot_state = state_handle_->getRobotState();
        auto q = array2eigen<7,1>(robot_state.q);
        
        // Perform inverse kinematics
        Vector7d jointsDes;
        if (!ikSolver.inverseKin(q, pose, jointsDes)) {
          LOG << "Inverse kinematics failed";
          jointsDes << q;
        }

        jointQ.emplace(jointsDes);
      }   
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
#endif

  void TeleopPosition::update(const ros::Time& /*time*/, const ros::Duration& period) {
    // Get the slave robot's state
    franka::RobotState robot_state = state_handle_->getRobotState();
    q_m_ = Eigen::Map<Vector7d>(robot_state.q.data());
    dq_m_ = Eigen::Map<Vector7d>(robot_state.dq.data());
    auto transform = Isometry3d(Matrix4d::Map(robot_state.O_T_EE.data()));
    const Isometry3d eTf = Isometry3d(array2eigen<4,4>(robot_state.F_T_EE)).inverse();
    transform = transform * eTf;
    position_m_ << transform.translation();
    orientation_m_ = Quaterniond(transform.rotation());

    // Only update the master's desired state if we have received a new one
#ifndef CUSTOM_IK
    if (updateBase(period.toSec()) ) {
      // Update the new ones
      Isometry3d T_d = Isometry3d::Identity();
      T_d.rotate(orientation_d_);
      T_d.translate(position_d_);
      
      // Pass this to Franka to do the inverse kinematics
      std::array<double,16> poseArr = eigen2array<4,4>(T_d.matrix());
      cartesian_pose_handle_->setCommand(poseArr);

      // This is one frame delayed from the Cartesian setCommand
      auto cart_state = cartesian_pose_handle_->getRobotState();
      q_d_ = array2eigen<7,1>(cart_state.q_d);
      dq_d_ = array2eigen<7,1>(cart_state.dq_d);
    }
#else
    // Try to get a new joint command
    Vector7d q_new;
    if (jointQ.try_dequeue(q_new)) {
      doneRuckig = false;
      q_d_target_ << q_new;
      dq_d_target_ = (q_d_target_ - q_d_last_) / period.toSec();
    }
    
    // At a higher rate, update Ruckig. This sets q_d_ and dq_d_
    if (!doneRuckig) doneRuckig = computeRuckig();
#endif

    // Get force from sensor (done in updateBase) or estimate from joint torques
    if (!m_comms.hasForceSensor())
      wrench_m_ = Eigen::Map<Vector6d>(robot_state.K_F_ext_hat_K.data());
    f_ext_norm_ = wrench_m_.head(3).norm();

    // Correct for robot drift
    dq_unsaturated_ = k_dq_ * (q_d_ - q_d_last_) + dq_d_;
    
    // Impose a deadband to stop small vibrations
    for (int i = 0; i < 7; i++)
      if (abs(dq_unsaturated_(i)) < 0.0025)
        dq_unsaturated_(i) = 0;

    // Calculate target postions and velocities for slave arm with saturation
    dq_d_ = saturateAndLimit(dq_unsaturated_, dq_d_last_, dqMax, ddqMax, period.toSec());
    dq_d_last_ = dq_d_;
    q_d_ = q_d_last_ + (dq_d_ * period.toSec());
    q_d_last_ = q_d_;

    // Force zero velocity if frozen
    if (teleop_state_ == TeleopState::ALIGN || teleop_state_ == TeleopState::PAUSE) 
    {
      dq_d_.setZero();
      dq_d_last_.setZero();
      q_d_ = q_init_;
      q_d_last_ = q_d_;
    }

    // Compute the joint torques to achieve the desired motion
    if (!robot_state.current_errors) {
      // Compute PD control for the slave arm to track the leader's motions.
      tau_d_ =
          slave_stiffness_scaling_ * k_p * (q_d_ - q_m_) +
          slave_stiffness_scaling_* k_d * (dq_d_ - dq_m_);
    } else {
      // Control target torques to zero if any arm is in error state.
      tau_d_ = decrease_factor_ * tau_d_last_;
    }

    // Add the Coriolis force as well. Gravity and friction compensation is done internally
    Vector7d coriolis = array2eigen<7,1>(model_handle_->getCoriolis());
    tau_d_ += coriolis;
    
    // Add torques from joint walls to the torque commands.
    const std::array<double, 7> virtual_wall_tau_slave =
        virtual_joint_wall_->computeTorque(eigen2array<7,1>(q_m_), eigen2array<7,1>(dq_m_));
    tau_d_ += array2eigen<7,1>(virtual_wall_tau_slave);
    
    // Store torques for next time step
    tau_d_last_ = tau_d_;
    
    // Send the computed values to the arm
    for (int i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d_[i]);
    }
    
    // Send to master as well
    if (publish_rate_()) {
        Vector6d velocity = array2eigen<6,1>(robot_state.O_dP_EE_c);
        Isometry3d transform(Matrix4d::Map(robot_state.O_T_EE.data()));
        const Isometry3d eTf = Isometry3d(array2eigen<4,4>(robot_state.F_T_EE)).inverse();
        transform = transform * eTf;
        
        const Isometry3d Tf = transform * eTf;
        m_comms.sendState(transform, Tf, velocity, wrench_m_, q_m_, angle_error_);
    }
  }

  bool TeleopPosition::computeRuckig() 
  {
    // Maximum values from Franka with a bit of safety buffer
    ruckig_input_.max_velocity << dqMax;
    ruckig_input_.max_acceleration << ddqMax;
    ruckig_input_.max_jerk << dddqMax;

    // Set the desired joints in Ruckig
    ruckig_input_.target_position << q_d_target_;
    //ruckig_input_.target_velocity << dq_d_target_; // Causes invalid input error

    // Perform the update
    auto res = rkg.update(ruckig_input_, ruckig_output_);
    if (res == ruckig::Result::Working) 
    {
      q_d_ << ruckig_output_.new_position;
      dq_d_ << ruckig_output_.new_velocity;
      ruckig_output_.pass_to_input(ruckig_input_);
    } 
    else if (res == ruckig::Result::Finished)
      return true;
    else if (res == ruckig::Result::ErrorInvalidInput)
      LOG << "Ruckig failed: invalid input";
    else if (res == ruckig::Result::ErrorTrajectoryDuration)
      LOG << "Ruckig failed: trajectory duration error";
    else if (res == ruckig::Result::ErrorExecutionTimeCalculation)
      LOG << "Ruckig failed: execution time too long";
    else if (res == ruckig::Result::ErrorSynchronizationCalculation)
      LOG << "Ruckig synchronization failed";
    else 
      LOG << "Ruckig update failed";

    return false; // Means unfinished, not necessarily failed
  }

  Matrix<double, 7, 1> TeleopPosition::saturateAndLimit(const Vector7d& x_calc,
                                                       const Vector7d& x_last,
                                                       const Vector7d& x_max,
                                                       const Vector7d& dx_max,
                                                       const double delta_t) {
    Vector7d x_limited;
    for (int i = 0; i < 7; i++) {
      double delta_x_max = dx_max[i] * delta_t;
      double diff = x_calc[i] - x_last[i];
      double x_saturated = x_last[i] + std::max(std::min(diff, delta_x_max), -delta_x_max);
      x_limited[i] = std::max(std::min(x_saturated, x_max[i]), -x_max[i]);
    }
    return x_limited;
  }

  double TeleopPosition::rampParameter(const double x,
                               const double neg_x_asymptote,
                               const double pos_x_asymptote,
                               const double shift_along_x,
                               const double increase_factor) {
    return 0.5 * (pos_x_asymptote + neg_x_asymptote - (pos_x_asymptote - neg_x_asymptote) * tanh(increase_factor * (x - shift_along_x)));
  }
  
  void TeleopPosition::getJointLimits(ros::NodeHandle& nh,
                              const std::vector<std::string>& joint_names,
                              std::array<double, 7>& upper_joint_soft_limit,
                              std::array<double, 7>& lower_joint_soft_limit) {
    const std::string& node_namespace = nh.getNamespace();
    std::size_t found = node_namespace.find_last_of('/');
    std::string parent_namespace = node_namespace.substr(0, found);
    if (!nh.hasParam(parent_namespace + "/robot_description")) {
      throw std::invalid_argument("TeleopPosition: No parameter robot_description (namespace: " + parent_namespace + ")found to set joint limits!");
    }
    urdf::Model urdf_model;
    if (!urdf_model.initParamWithNodeHandle(parent_namespace + "/robot_description", nh)) {
      throw std::invalid_argument("TeleopPosition: Could not initialize urdf model from robot_description (namespace: " + parent_namespace + ").");
    }
    joint_limits_interface::SoftJointLimits soft_limits;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      const std::string& joint_name = joint_names.at(i);
      auto urdf_joint = urdf_model.getJoint(joint_name);
      if (!urdf_joint) {
        LOG << "TeleopPosition: Could not get joint " << joint_name << " from urdf";
      }
      if (!urdf_joint->safety) {
        LOG << "TeleopPosition: Joint " << joint_name << " has no limits";
      }
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
        upper_joint_soft_limit[i] = soft_limits.max_position;
        lower_joint_soft_limit[i] = soft_limits.min_position;
      } else {
        LOG << "TeleopPosition: Could not parse joint limit for joint " << joint_name << " for joint limit interfaces";
      }
    }
  }

  Vector7d TeleopPosition::get7dParam(const std::string& param_name, ros::NodeHandle& nh) {
    auto buffer = getJointParams<double>(param_name, nh);
    return Vector7d(Eigen::Map<Vector7d>(buffer.data()));
  }
}  // namespace teleop

PLUGINLIB_EXPORT_CLASS(teleop::TeleopPosition,
                       controller_interface::ControllerBase)