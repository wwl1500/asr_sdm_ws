#ifndef ASR_SDM_CONTROLLER__FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_
#define ASR_SDM_CONTROLLER__FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_

#include <vector>
#include <cmath>
#include <memory>
#include <string>

#include "asr_sdm_controller/pinocchio_kinematics_wrapper.hpp"

namespace asr_sdm_controller
{

/**
 * @brief 蛇形机器人前端单元跟随控制器
 * 
 * 实现论文中的控制方法：
 * "Modeling and Control of a Snake-Like Robot Using the Screw-Drive Mechanism"
 * 
 * 控制器递归计算关节角速度，使所有关节跟随前端单元的路径。
 * 支持可选的 Pinocchio 集成以获得更精确的运动学计算。
 */
class FrontUnitFollowingController
{
public:
  /**
   * @brief 构造函数
   * @param segment_length 每段的长度 L（论文中的 L）
   * @param num_segments 段数量（不包括 base_link）
   */
  FrontUnitFollowingController(double segment_length, int num_segments);

  /**
   * @brief 带 Pinocchio 封装器的构造函数
   * @param segment_length 每段的长度 L
   * @param num_segments 段数量
   * @param pinocchio_wrapper Pinocchio 运动学封装器
   */
  FrontUnitFollowingController(
    double segment_length,
    int num_segments,
    std::shared_ptr<PinocchioKinematicsWrapper> pinocchio_wrapper);

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
   * @brief 获取段数量
   * @return 段数量
   */
  int getNumSegments() const {return num_segments_;}

  /**
   * @brief 设置 Pinocchio 封装器
   * @param wrapper Pinocchio 运动学封装器
   */
  void setPinocchioWrapper(std::shared_ptr<PinocchioKinematicsWrapper> wrapper);

  /**
   * @brief 检查 Pinocchio 是否启用
   * @return 是否启用 Pinocchio
   */
  bool isPinocchioEnabled() const;

  /**
   * @brief 获取计算方法
   * @return "pinocchio" 或 "simplified"
   */
  std::string getComputationMethod() const;

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

  /**
   * @brief 使用 Pinocchio 计算段速度
   * @param v1 前端单元线速度
   * @param omega1 前端单元角速度
   * @param joint_angles 当前关节角度
   * @param joint_velocities 已计算的关节角速度
   * @param segment_index 段索引
   * @return 段的线速度
   */
  double computeSegmentVelocityWithPinocchio(
    double v1,
    double omega1,
    const std::vector<double> & joint_angles,
    const std::vector<double> & joint_velocities,
    int segment_index);

  double segment_length_;  ///< 段长度 L
  int num_segments_;       ///< 段数量
  std::shared_ptr<PinocchioKinematicsWrapper> pinocchio_wrapper_;  ///< Pinocchio 封装器
  bool use_pinocchio_;     ///< 是否使用 Pinocchio
};

}  // namespace asr_sdm_controller

#endif  // ASR_SDM_CONTROLLER__FRONT_UNIT_FOLLOWING_CONTROLLER_HPP_

