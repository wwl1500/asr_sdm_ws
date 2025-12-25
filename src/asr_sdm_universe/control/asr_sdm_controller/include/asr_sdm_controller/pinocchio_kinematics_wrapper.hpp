/**
 * @file pinocchio_kinematics_wrapper.hpp
 * @brief Pinocchio 运动学封装类
 * 
 * 封装 Pinocchio 库的运动学计算功能，提供 URDF 加载、前向运动学、
 * 雅可比矩阵计算等接口。
 */

#ifndef ASR_SDM_CONTROLLER__PINOCCHIO_KINEMATICS_WRAPPER_HPP_
#define ASR_SDM_CONTROLLER__PINOCCHIO_KINEMATICS_WRAPPER_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace asr_sdm_controller
{

/**
 * @brief Pinocchio 运动学封装类
 * 
 * 提供基于 Pinocchio 的运动学计算功能，包括：
 * - URDF 模型加载
 * - 前向运动学计算
 * - 雅可比矩阵计算
 * - 段速度计算
 */
class PinocchioKinematicsWrapper
{
public:
  /**
   * @brief 默认构造函数
   */
  PinocchioKinematicsWrapper();

  /**
   * @brief 从 URDF 文件路径构造
   * @param urdf_path URDF 文件路径
   */
  explicit PinocchioKinematicsWrapper(const std::string & urdf_path);

  /**
   * @brief 析构函数
   */
  ~PinocchioKinematicsWrapper() = default;

  /**
   * @brief 从 URDF 字符串初始化
   * @param urdf_string URDF 内容字符串
   * @return 初始化是否成功
   */
  bool initializeFromUrdfString(const std::string & urdf_string);

  /**
   * @brief 从 URDF 文件初始化
   * @param urdf_path URDF 文件路径
   * @return 初始化是否成功
   */
  bool initializeFromUrdfFile(const std::string & urdf_path);

  /**
   * @brief 计算前向运动学
   * @param joint_positions 关节位置向量
   * @return 计算是否成功
   */
  bool computeForwardKinematics(const std::vector<double> & joint_positions);

  /**
   * @brief 获取指定坐标系的位置
   * @param frame_name 坐标系名称
   * @return 位置向量 (x, y, z)
   */
  Eigen::Vector3d getFramePosition(const std::string & frame_name) const;

  /**
   * @brief 获取指定坐标系的姿态（旋转矩阵）
   * @param frame_name 坐标系名称
   * @return 3x3 旋转矩阵
   */
  Eigen::Matrix3d getFrameOrientation(const std::string & frame_name) const;

  /**
   * @brief 计算指定坐标系的雅可比矩阵
   * @param frame_name 坐标系名称
   * @param joint_positions 关节位置向量
   * @return 6xN 雅可比矩阵
   */
  Eigen::MatrixXd computeFrameJacobian(
    const std::string & frame_name,
    const std::vector<double> & joint_positions);

  /**
   * @brief 使用雅可比矩阵计算段的线速度
   * @param segment_index 段索引 (从 0 开始)
   * @param joint_positions 关节位置向量
   * @param joint_velocities 关节速度向量
   * @return 段的线速度大小
   */
  double computeSegmentLinearVelocity(
    int segment_index,
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_velocities);

  /**
   * @brief 获取段长度
   * @param segment_index 段索引
   * @return 段长度 (m)
   */
  double getSegmentLength(int segment_index) const;

  /**
   * @brief 获取关节数量
   * @return 关节数量
   */
  int getNumJoints() const;

  /**
   * @brief 检查是否已初始化
   * @return 是否已初始化
   */
  bool isInitialized() const;

  /**
   * @brief 获取所有坐标系名称
   * @return 坐标系名称列表
   */
  std::vector<std::string> getFrameNames() const;

  /**
   * @brief 获取所有关节名称
   * @return 关节名称列表
   */
  std::vector<std::string> getJointNames() const;

private:
  /**
   * @brief 构建坐标系索引映射
   */
  void buildFrameIndices();

  /**
   * @brief 验证关节位置向量
   * @param joint_positions 关节位置向量
   * @return 是否有效
   */
  bool validateJointPositions(const std::vector<double> & joint_positions) const;

  pinocchio::Model model_;                              ///< Pinocchio 模型
  std::unique_ptr<pinocchio::Data> data_;               ///< Pinocchio 数据
  bool initialized_;                                    ///< 初始化标志
  std::map<std::string, pinocchio::FrameIndex> frame_indices_;  ///< 坐标系索引映射
  double default_segment_length_;                       ///< 默认段长度
};

}  // namespace asr_sdm_controller

#endif  // ASR_SDM_CONTROLLER__PINOCCHIO_KINEMATICS_WRAPPER_HPP_
