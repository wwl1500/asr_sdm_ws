#include "asr_sdm_controller/front_unit_following_controller.hpp"
#include <stdexcept>
#include <iostream>

namespace asr_sdm_controller
{

FrontUnitFollowingController::FrontUnitFollowingController(
  double segment_length,
  int num_segments)
: segment_length_(segment_length),
  num_segments_(num_segments),
  pinocchio_wrapper_(nullptr),
  use_pinocchio_(false)
{
  if (segment_length <= 0.0) {
    throw std::invalid_argument("段长度必须为正数");
  }
  if (num_segments <= 0) {
    throw std::invalid_argument("段数量必须为正数");
  }
}

FrontUnitFollowingController::FrontUnitFollowingController(
  double segment_length,
  int num_segments,
  std::shared_ptr<PinocchioKinematicsWrapper> pinocchio_wrapper)
: segment_length_(segment_length),
  num_segments_(num_segments),
  pinocchio_wrapper_(pinocchio_wrapper),
  use_pinocchio_(pinocchio_wrapper != nullptr && pinocchio_wrapper->isInitialized())
{
  if (segment_length <= 0.0) {
    throw std::invalid_argument("段长度必须为正数");
  }
  if (num_segments <= 0) {
    throw std::invalid_argument("段数量必须为正数");
  }
  
  if (use_pinocchio_) {
    std::cout << "[FrontUnitFollowingController] 使用 Pinocchio 进行运动学计算" << std::endl;
  } else {
    std::cout << "[FrontUnitFollowingController] 使用简化运动学模型" << std::endl;
  }
}

void FrontUnitFollowingController::setPinocchioWrapper(
  std::shared_ptr<PinocchioKinematicsWrapper> wrapper)
{
  pinocchio_wrapper_ = wrapper;
  use_pinocchio_ = (wrapper != nullptr && wrapper->isInitialized());
  
  if (use_pinocchio_) {
    std::cout << "[FrontUnitFollowingController] 已启用 Pinocchio" << std::endl;
  } else {
    std::cout << "[FrontUnitFollowingController] 已禁用 Pinocchio，使用简化模型" << std::endl;
  }
}

bool FrontUnitFollowingController::isPinocchioEnabled() const
{
  return use_pinocchio_;
}

std::string FrontUnitFollowingController::getComputationMethod() const
{
  return use_pinocchio_ ? "pinocchio" : "simplified";
}

std::vector<double> FrontUnitFollowingController::computeJointVelocities(
  double v1,
  double omega1,
  const std::vector<double> & joint_angles)
{
  if (static_cast<int>(joint_angles.size()) < num_segments_) {
    throw std::invalid_argument(
            "Joint angles size must be at least num_segments");
  }

  std::vector<double> joint_velocities(num_segments_);

  // Compute phi_dot1 using the first formula from the paper:
  // φ̇₁ = -(2/L) v₁ sin(φ₁) - ω₁ (2 cos(φ₁) + 1)
  if (num_segments_ > 0) {
    double phi1 = joint_angles[0];
    joint_velocities[0] = -(2.0 / segment_length_) * v1 * std::sin(phi1) -
      omega1 * (2.0 * std::cos(phi1) + 1.0);
  }

  // Recursively compute phi_dot_i for i > 1:
  // φ̇ᵢ = -(2/L) v̄ᵢ sin(φᵢ) - (ω₁ + Σⱼ₌₁ᵢ⁻¹ φ̇ⱼ)(cos(φᵢ) + 1)
  for (int i = 1; i < num_segments_; ++i) {
    double phi_i = joint_angles[i];

    // Compute cumulative angular velocity: ω₁ + Σⱼ₌₁ᵢ⁻¹ φ̇ⱼ
    double cumulative_omega = omega1;
    for (int j = 0; j < i; ++j) {
      cumulative_omega += joint_velocities[j];
    }

    // 计算段 i 的速度 (v̄ᵢ)
    // 如果启用了 Pinocchio，使用雅可比矩阵计算；否则使用简化模型
    double v_bar_i;
    if (use_pinocchio_ && pinocchio_wrapper_) {
      v_bar_i = computeSegmentVelocityWithPinocchio(
        v1, omega1, joint_angles, joint_velocities, i);
    } else {
      v_bar_i = computeSegmentVelocity(
        v1, omega1, joint_angles, joint_velocities, i);
    }

    // Compute phi_dot_i
    joint_velocities[i] = -(2.0 / segment_length_) * v_bar_i * std::sin(phi_i) -
      cumulative_omega * (std::cos(phi_i) + 1.0);
  }

  return joint_velocities;
}

double FrontUnitFollowingController::computeSegmentVelocity(
  double v1,
  double omega1,
  const std::vector<double> & joint_angles,
  const std::vector<double> & joint_velocities,
  int segment_index)
{
  // For the second segment (index 1), we can use a simplified model
  // based on the kinematic constraints from the paper
  if (segment_index == 1) {
    // v̄₂ is the velocity of the second unit
    // Based on the kinematic model, it depends on v1, omega1, and phi1
    double phi1 = joint_angles[0];
    // phi_dot1 is available but not used in this simplified model
    // double phi_dot1 = joint_velocities[0];

    // Approximate v̄₂ using the velocity constraint
    // This is a simplified approximation - a full implementation would
    // solve the complete kinematic model
    double v_bar_2 = v1 * std::cos(phi1) - segment_length_ * omega1 * std::sin(phi1);
    return v_bar_2;
  }

  // For subsequent segments, propagate the velocity recursively
  // This is a simplified model - a full implementation would use
  // the complete kinematic equations from the paper
  double prev_v = computeSegmentVelocity(
    v1, omega1, joint_angles, joint_velocities, segment_index - 1);
  double phi_prev = joint_angles[segment_index - 1];
  double phi_dot_prev = joint_velocities[segment_index - 1];

  // Velocity propagation based on joint motion
  double v_bar = prev_v * std::cos(phi_prev) -
    segment_length_ * phi_dot_prev * std::sin(phi_prev);

  return v_bar;
}

double FrontUnitFollowingController::computeSegmentVelocityWithPinocchio(
  double v1,
  double omega1,
  const std::vector<double> & joint_angles,
  const std::vector<double> & joint_velocities,
  int segment_index)
{
  if (!pinocchio_wrapper_ || !pinocchio_wrapper_->isInitialized()) {
    // 回退到简化模型
    return computeSegmentVelocity(v1, omega1, joint_angles, joint_velocities, segment_index);
  }
  
  try {
    // 使用 Pinocchio 计算段速度
    double velocity = pinocchio_wrapper_->computeSegmentLinearVelocity(
      segment_index, joint_angles, joint_velocities);
    
    // 如果计算结果无效，回退到简化模型
    if (std::isnan(velocity) || std::isinf(velocity)) {
      return computeSegmentVelocity(v1, omega1, joint_angles, joint_velocities, segment_index);
    }
    
    return velocity;
  } catch (const std::exception & e) {
    std::cerr << "[FrontUnitFollowingController] Pinocchio 计算失败: " << e.what() 
              << "，回退到简化模型" << std::endl;
    return computeSegmentVelocity(v1, omega1, joint_angles, joint_velocities, segment_index);
  }
}

}  // namespace asr_sdm_controller

