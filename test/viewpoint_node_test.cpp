#include "simplecp/viewpoint_node.h"

#include <cmath>

#include <geometry_msgs/Point.h>
#include <gtest/gtest.h>

#include "simplecp/MarkerEvent.h"

namespace simplecp {
TEST(TestViewpointNode, XControl) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::X;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x, result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y + sqrt(2), result_point.y);
  EXPECT_FLOAT_EQ(current_position.z, result_point.z);
}

TEST(TestViewpointNode, MoveToRightIfCurrentlyRightOfX) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y - 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::X;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x, result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y - sqrt(2), result_point.y);
  EXPECT_FLOAT_EQ(current_position.z, result_point.z);
}

TEST(TestViewpointNode, YControl) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::Y;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x + sqrt(2), result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y, result_point.y);
  EXPECT_FLOAT_EQ(current_position.z, result_point.z);
}

TEST(TestViewpointNode, MoveToBackIfCurrentlyBehindY) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x - 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::Y;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x - sqrt(2), result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y, result_point.y);
  EXPECT_FLOAT_EQ(current_position.z, result_point.z);
}

TEST(TestViewpointNode, ZControl) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::Z;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x + sqrt(1.5), result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y + sqrt(1.5), result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z, result_point.z);
}

TEST(TestViewpointNode, Pitch) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::PITCH;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x, result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y + sqrt(3), result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z, result_point.z);
}

TEST(TestViewpointNode, MoveToRightIfCurrentlyRightOfPitch) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y - 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::PITCH;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x, result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y - sqrt(3), result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z, result_point.z);
}

TEST(TestViewpointNode, Roll) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::ROLL;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x + sqrt(3), result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y, result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z, result_point.z);
}

TEST(TestViewpointNode, MoveToBackIfBehindRoll) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x - 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::ROLL;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x - sqrt(3), result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y, result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z, result_point.z);
}

TEST(TestViewpointNode, Yaw) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z + 1;
  auto control_type = MarkerEvent::YAW;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x, result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y, result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z + sqrt(3), result_point.z);
}

TEST(TestViewpointNode, AlwaysGoUpOnYaw) {
  geometry_msgs::Point marker_position;
  marker_position.x = 1;
  marker_position.y = 2;
  marker_position.z = 3;
  geometry_msgs::Point current_position;
  current_position.x = marker_position.x + 1;
  current_position.y = marker_position.y + 1;
  current_position.z = marker_position.z - 1;
  auto control_type = MarkerEvent::YAW;
  geometry_msgs::Point result_point;
  ComputeOrthogonalPoint(current_position, marker_position, control_type,
                         &result_point);
  EXPECT_FLOAT_EQ(marker_position.x, result_point.x);
  EXPECT_FLOAT_EQ(marker_position.y, result_point.y);
  EXPECT_FLOAT_EQ(marker_position.z + sqrt(3), result_point.z);
}

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
