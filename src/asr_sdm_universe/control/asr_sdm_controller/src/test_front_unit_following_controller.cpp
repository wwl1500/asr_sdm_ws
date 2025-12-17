#include <gtest/gtest.h>
#include "asr_sdm_controller/front_unit_following_controller.hpp"
#include <cmath>

class FrontUnitFollowingControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_unique<asr_sdm_controller::FrontUnitFollowingController>(
      0.18,  // segment_length
      4      // num_segments
    );
  }

  std::unique_ptr<asr_sdm_controller::FrontUnitFollowingController> controller_;
};

// Test basic initialization
TEST_F(FrontUnitFollowingControllerTest, Initialization)
{
  EXPECT_DOUBLE_EQ(controller_->getSegmentLength(), 0.18);
  EXPECT_EQ(controller_->getNumSegments(), 4);
}

// Test with zero velocities (robot at rest)
TEST_F(FrontUnitFollowingControllerTest, ZeroVelocities)
{
  double v1 = 0.0;
  double omega1 = 0.0;
  std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0};

  auto joint_velocities = controller_->computeJointVelocities(v1, omega1, joint_angles);

  EXPECT_EQ(joint_velocities.size(), 4);
  // All joint velocities should be zero when robot is at rest
  for (const auto & vel : joint_velocities) {
    EXPECT_NEAR(vel, 0.0, 1e-6);
  }
}

// Test with forward motion only (v1 > 0, omega1 = 0)
TEST_F(FrontUnitFollowingControllerTest, ForwardMotion)
{
  double v1 = 0.1;  // 0.1 m/s forward
  double omega1 = 0.0;
  std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0};

  auto joint_velocities = controller_->computeJointVelocities(v1, omega1, joint_angles);

  EXPECT_EQ(joint_velocities.size(), 4);
  // When joints are straight and moving forward, velocities should be small
  // First joint velocity should be zero (sin(0) = 0)
  EXPECT_NEAR(joint_velocities[0], 0.0, 1e-6);
}

// Test with rotation only (v1 = 0, omega1 > 0)
TEST_F(FrontUnitFollowingControllerTest, RotationOnly)
{
  double v1 = 0.0;
  double omega1 = 0.1;  // 0.1 rad/s rotation
  std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0};

  auto joint_velocities = controller_->computeJointVelocities(v1, omega1, joint_angles);

  EXPECT_EQ(joint_velocities.size(), 4);
  // When rotating with straight joints, first joint should have negative velocity
  // phi_dot1 = -omega1 * (2*cos(0) + 1) = -omega1 * 3
  EXPECT_NEAR(joint_velocities[0], -omega1 * 3.0, 1e-3);
}

// Test with curved configuration
TEST_F(FrontUnitFollowingControllerTest, CurvedConfiguration)
{
  double v1 = 0.1;
  double omega1 = 0.05;
  std::vector<double> joint_angles = {0.5, 0.3, 0.2, 0.1};  // Curved shape

  auto joint_velocities = controller_->computeJointVelocities(v1, omega1, joint_angles);

  EXPECT_EQ(joint_velocities.size(), 4);
  // All velocities should be finite
  for (const auto & vel : joint_velocities) {
    EXPECT_TRUE(std::isfinite(vel));
  }
}

// Test parameter modification
TEST_F(FrontUnitFollowingControllerTest, ParameterModification)
{
  controller_->setSegmentLength(0.2);
  EXPECT_DOUBLE_EQ(controller_->getSegmentLength(), 0.2);

  controller_->setNumSegments(5);
  EXPECT_EQ(controller_->getNumSegments(), 5);
}

// Test error handling - insufficient joint angles
TEST_F(FrontUnitFollowingControllerTest, InsufficientJointAngles)
{
  double v1 = 0.1;
  double omega1 = 0.0;
  std::vector<double> joint_angles = {0.0, 0.0};  // Only 2 angles, need 4

  EXPECT_THROW(
    controller_->computeJointVelocities(v1, omega1, joint_angles),
    std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

