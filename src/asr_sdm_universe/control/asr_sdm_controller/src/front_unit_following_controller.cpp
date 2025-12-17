#include "asr_sdm_controller/front_unit_following_controller.hpp"
#include <stdexcept>

namespace asr_sdm_controller
{

FrontUnitFollowingController::FrontUnitFollowingController(
  double segment_length,
  int num_segments)
: segment_length_(segment_length), num_segments_(num_segments)
{
  if (segment_length <= 0.0) {
    throw std::invalid_argument("Segment length must be positive");
  }
  if (num_segments <= 0) {
    throw std::invalid_argument("Number of segments must be positive");
  }
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

    // Compute velocity of segment i (v̄ᵢ)
    // This is approximated based on the kinematic model
    // For simplicity, we use the velocity propagation from the front unit
    double v_bar_i = computeSegmentVelocity(
      v1, omega1, joint_angles, joint_velocities, i);

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

}  // namespace asr_sdm_controller

