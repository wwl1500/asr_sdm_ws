/**
 * @file pinocchio_kinematics_wrapper.cpp
 * @brief Pinocchio 运动学封装类实现
 */

#include "asr_sdm_controller/pinocchio_kinematics_wrapper.hpp"

#include <cmath>
#include <stdexcept>
#include <iostream>
#include <sstream>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace asr_sdm_controller
{

PinocchioKinematicsWrapper::PinocchioKinematicsWrapper()
: initialized_(false), default_segment_length_(0.18)
{
}

PinocchioKinematicsWrapper::PinocchioKinematicsWrapper(const std::string & urdf_path)
: initialized_(false), default_segment_length_(0.18)
{
  initializeFromUrdfFile(urdf_path);
}

bool PinocchioKinematicsWrapper::initializeFromUrdfString(const std::string & urdf_string)
{
  try {
    // 从 URDF 字符串构建模型
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    
    // 创建数据结构
    data_ = std::make_unique<pinocchio::Data>(model_);
    
    // 构建坐标系索引映射
    buildFrameIndices();
    
    initialized_ = true;
    
    std::cout << "[PinocchioKinematicsWrapper] 成功从 URDF 字符串初始化" << std::endl;
    std::cout << "  - 关节数量: " << model_.nq << std::endl;
    std::cout << "  - 坐标系数量: " << model_.nframes << std::endl;
    
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[PinocchioKinematicsWrapper] 从 URDF 字符串初始化失败: " << e.what() << std::endl;
    initialized_ = false;
    return false;
  }
}

bool PinocchioKinematicsWrapper::initializeFromUrdfFile(const std::string & urdf_path)
{
  try {
    // 从 URDF 文件构建模型
    pinocchio::urdf::buildModel(urdf_path, model_);
    
    // 创建数据结构
    data_ = std::make_unique<pinocchio::Data>(model_);
    
    // 构建坐标系索引映射
    buildFrameIndices();
    
    initialized_ = true;
    
    std::cout << "[PinocchioKinematicsWrapper] 成功从 URDF 文件初始化: " << urdf_path << std::endl;
    std::cout << "  - 关节数量: " << model_.nq << std::endl;
    std::cout << "  - 坐标系数量: " << model_.nframes << std::endl;
    
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[PinocchioKinematicsWrapper] 从 URDF 文件初始化失败: " << e.what() << std::endl;
    initialized_ = false;
    return false;
  }
}

bool PinocchioKinematicsWrapper::computeForwardKinematics(
  const std::vector<double> & joint_positions)
{
  if (!initialized_) {
    std::cerr << "[PinocchioKinematicsWrapper] 未初始化，无法计算前向运动学" << std::endl;
    return false;
  }
  
  if (!validateJointPositions(joint_positions)) {
    std::cerr << "[PinocchioKinematicsWrapper] 关节位置无效" << std::endl;
    return false;
  }
  
  try {
    // 将 std::vector 转换为 Eigen 向量
    Eigen::VectorXd q(model_.nq);
    size_t num_joints = std::min(joint_positions.size(), static_cast<size_t>(model_.nq));
    for (size_t i = 0; i < num_joints; ++i) {
      q(i) = joint_positions[i];
    }
    // 填充剩余关节为 0
    for (int i = static_cast<int>(num_joints); i < model_.nq; ++i) {
      q(i) = 0.0;
    }
    
    // 计算前向运动学
    pinocchio::forwardKinematics(model_, *data_, q);
    
    // 更新坐标系位置
    pinocchio::updateFramePlacements(model_, *data_);
    
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[PinocchioKinematicsWrapper] 前向运动学计算失败: " << e.what() << std::endl;
    return false;
  }
}

Eigen::Vector3d PinocchioKinematicsWrapper::getFramePosition(const std::string & frame_name) const
{
  if (!initialized_) {
    std::cerr << "[PinocchioKinematicsWrapper] 未初始化" << std::endl;
    return Eigen::Vector3d::Zero();
  }
  
  auto it = frame_indices_.find(frame_name);
  if (it == frame_indices_.end()) {
    std::cerr << "[PinocchioKinematicsWrapper] 未找到坐标系: " << frame_name << std::endl;
    return Eigen::Vector3d::Zero();
  }
  
  return data_->oMf[it->second].translation();
}

Eigen::Matrix3d PinocchioKinematicsWrapper::getFrameOrientation(const std::string & frame_name) const
{
  if (!initialized_) {
    std::cerr << "[PinocchioKinematicsWrapper] 未初始化" << std::endl;
    return Eigen::Matrix3d::Identity();
  }
  
  auto it = frame_indices_.find(frame_name);
  if (it == frame_indices_.end()) {
    std::cerr << "[PinocchioKinematicsWrapper] 未找到坐标系: " << frame_name << std::endl;
    return Eigen::Matrix3d::Identity();
  }
  
  return data_->oMf[it->second].rotation();
}

Eigen::MatrixXd PinocchioKinematicsWrapper::computeFrameJacobian(
  const std::string & frame_name,
  const std::vector<double> & joint_positions)
{
  if (!initialized_) {
    std::cerr << "[PinocchioKinematicsWrapper] 未初始化" << std::endl;
    return Eigen::MatrixXd::Zero(6, 1);
  }
  
  auto it = frame_indices_.find(frame_name);
  if (it == frame_indices_.end()) {
    std::cerr << "[PinocchioKinematicsWrapper] 未找到坐标系: " << frame_name << std::endl;
    return Eigen::MatrixXd::Zero(6, model_.nv);
  }
  
  try {
    // 将 std::vector 转换为 Eigen 向量
    Eigen::VectorXd q(model_.nq);
    size_t num_joints = std::min(joint_positions.size(), static_cast<size_t>(model_.nq));
    for (size_t i = 0; i < num_joints; ++i) {
      q(i) = joint_positions[i];
    }
    for (int i = static_cast<int>(num_joints); i < model_.nq; ++i) {
      q(i) = 0.0;
    }
    
    // 计算雅可比矩阵
    Eigen::MatrixXd J(6, model_.nv);
    J.setZero();
    
    pinocchio::computeFrameJacobian(
      model_, *data_, q, it->second,
      pinocchio::LOCAL_WORLD_ALIGNED, J);
    
    return J;
  } catch (const std::exception & e) {
    std::cerr << "[PinocchioKinematicsWrapper] 雅可比计算失败: " << e.what() << std::endl;
    return Eigen::MatrixXd::Zero(6, model_.nv);
  }
}

double PinocchioKinematicsWrapper::computeSegmentLinearVelocity(
  int segment_index,
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_velocities)
{
  if (!initialized_) {
    return 0.0;
  }
  
  // 构建段的坐标系名称
  std::string frame_name;
  if (segment_index == 0) {
    frame_name = "base_link";
  } else {
    frame_name = "link_" + std::to_string(segment_index + 1);
  }
  
  // 计算雅可比矩阵
  Eigen::MatrixXd J = computeFrameJacobian(frame_name, joint_positions);
  
  if (J.cols() == 0) {
    return 0.0;
  }
  
  // 将关节速度转换为 Eigen 向量
  Eigen::VectorXd q_dot(model_.nv);
  q_dot.setZero();
  size_t num_joints = std::min(joint_velocities.size(), static_cast<size_t>(model_.nv));
  for (size_t i = 0; i < num_joints; ++i) {
    q_dot(i) = joint_velocities[i];
  }
  
  // 计算线速度 (取雅可比矩阵的前 3 行)
  Eigen::Vector3d linear_velocity = J.topRows(3) * q_dot;
  
  // 返回线速度的大小
  return linear_velocity.norm();
}

double PinocchioKinematicsWrapper::getSegmentLength(int segment_index) const
{
  (void)segment_index;  // 暂时忽略索引，返回默认值
  return default_segment_length_;
}

int PinocchioKinematicsWrapper::getNumJoints() const
{
  if (!initialized_) {
    return 0;
  }
  return model_.nq;
}

bool PinocchioKinematicsWrapper::isInitialized() const
{
  return initialized_;
}

std::vector<std::string> PinocchioKinematicsWrapper::getFrameNames() const
{
  std::vector<std::string> names;
  for (const auto & pair : frame_indices_) {
    names.push_back(pair.first);
  }
  return names;
}

std::vector<std::string> PinocchioKinematicsWrapper::getJointNames() const
{
  std::vector<std::string> names;
  if (!initialized_) {
    return names;
  }
  for (int i = 1; i < model_.njoints; ++i) {  // 跳过 universe joint
    names.push_back(model_.names[i]);
  }
  return names;
}

void PinocchioKinematicsWrapper::buildFrameIndices()
{
  frame_indices_.clear();
  for (size_t i = 0; i < model_.frames.size(); ++i) {
    frame_indices_[model_.frames[i].name] = static_cast<pinocchio::FrameIndex>(i);
  }
}

bool PinocchioKinematicsWrapper::validateJointPositions(
  const std::vector<double> & joint_positions) const
{
  for (const auto & pos : joint_positions) {
    if (std::isnan(pos) || std::isinf(pos)) {
      return false;
    }
  }
  return true;
}

}  // namespace asr_sdm_controller
