#ifndef ASR_SDM_CONTROLLER__FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_
#define ASR_SDM_CONTROLLER__FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_

#include <vector>
#include <cmath>

namespace asr_sdm_controller
{

/**
 * @brief Front-Unit-Following Controller for Snake-like Robot
 * 
 * Implements the control method from the paper:
 * "Modeling and Control of a Snake-Like Robot Using the Screw-Drive Mechanism"
 * 
 * The controller computes joint angular velocities recursively so that
 * all joints follow the path of the front unit.
 */
class FrontUnitFollowingController
{
public:
  /**
   * @brief Constructor
   * @param segment_length Length of each segment (L in the paper)
   * @param num_segments Number of segments (excluding base_link)
   */
  FrontUnitFollowingController(double segment_length, int num_segments);

  /**
   * @brief Compute joint angular velocities
   * @param v1 Linear velocity of the front unit
   * @param omega1 Angular velocity of the front unit
   * @param joint_angles Current joint angles [phi1, phi2, ..., phiN]
   * @return Joint angular velocities [phi_dot1, phi_dot2, ..., phi_dotN]
   */
  std::vector<double> computeJointVelocities(
    double v1,
    double omega1,
    const std::vector<double> & joint_angles);

  /**
   * @brief Set segment length
   * @param length Segment length L
   */
  void setSegmentLength(double length) {segment_length_ = length;}

  /**
   * @brief Get segment length
   * @return Segment length L
   */
  double getSegmentLength() const {return segment_length_;}

  /**
   * @brief Set number of segments
   * @param num Number of segments
   */
  void setNumSegments(int num) {num_segments_ = num;}

  /**
   * @brief Get number of segments
   * @return Number of segments
   */
  int getNumSegments() const {return num_segments_;}

private:
  /**
   * @brief Compute the velocity of segment i
   * @param v1 Front unit linear velocity
   * @param omega1 Front unit angular velocity
   * @param joint_angles Current joint angles
   * @param joint_velocities Computed joint angular velocities (up to i-1)
   * @param segment_index Segment index (0-based, 0 is front unit)
   * @return Linear velocity of segment i
   */
  double computeSegmentVelocity(
    double v1,
    double omega1,
    const std::vector<double> & joint_angles,
    const std::vector<double> & joint_velocities,
    int segment_index);

  double segment_length_;  // L in the paper
  int num_segments_;        // Number of segments
};

}  // namespace asr_sdm_controller

#endif  // ASR_SDM_CONTROLLER__FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_

