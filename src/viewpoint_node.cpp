#include "simplecp/viewpoint_node.h"

#include <cmath>

#include <OGRE/OgreVector3.h>
#include <ros/console.h>

namespace simplecp {

void ComputeOrthogonalPoint(
    const geometry_msgs::Point& current_position,
    const geometry_msgs::Point& marker_position,
    const std::string& control_type,
    geometry_msgs::Point* orthogonal_position) {
  auto current =
      Ogre::Vector3(current_position.x, current_position.y, current_position.z);
  auto marker =
      Ogre::Vector3(marker_position.x, marker_position.y, marker_position.z);
  auto difference = current - marker;
  Ogre::Vector3 orthogonal_difference(0, 0, 0);
  if (control_type == MarkerEvent::X) {
    int sign = 1;
    if (difference.y < 0) {
      sign = -1;
    }
    orthogonal_difference.x = 0;
    orthogonal_difference.y = sign * sqrt(difference.x * difference.x
                                          + difference.y * difference.y);
    orthogonal_difference.z = difference.z;
  } else if (control_type == MarkerEvent::Y) {
    int sign = 1;
    if (difference.x < 0) {
      sign = -1;
    }
    orthogonal_difference.x = sign * sqrt(difference.x * difference.x
                                          + difference.y * difference.y);
    orthogonal_difference.y = 0;
    orthogonal_difference.z = difference.z;
  } else if (control_type == MarkerEvent::Z) {
    orthogonal_difference.x = difference.x;
    orthogonal_difference.y = difference.y;
    orthogonal_difference.z = 0;
    orthogonal_difference *=
        difference.length() / orthogonal_difference.length();
  } else if (control_type == MarkerEvent::PITCH) {
    int sign = 1;
    if (difference.y < 0) {
      sign = -1;
    }
    orthogonal_difference.x = 0;
    orthogonal_difference.y = sign * difference.length();
    orthogonal_difference.z = 0;
  } else if (control_type == MarkerEvent::ROLL) {
    int sign = 1;
    if (difference.x < 0) {
      sign = -1;
    }
    orthogonal_difference.x = sign * difference.length();
    orthogonal_difference.y = 0;
    orthogonal_difference.z = 0;
  } else if (control_type == MarkerEvent::YAW) {
    // Special case: always go above the yaw ring.
    orthogonal_difference.x = 0;
    orthogonal_difference.y = 0;
    orthogonal_difference.z = difference.length();
    // Special case: add a tiny component in the original direction.
    orthogonal_difference += 0.01 / difference.length() * difference;
  } else {
    ROS_ERROR("Unknown control type %s", control_type.c_str());
  }
  auto ogre_orthogonal_position = marker + orthogonal_difference;
  orthogonal_position->x = ogre_orthogonal_position.x;
  orthogonal_position->y = ogre_orthogonal_position.y;
  orthogonal_position->z = ogre_orthogonal_position.z;
}

ViewpointNode::ViewpointNode(const std::string& pose_topic,
                             const std::string& event_topic,
                             const std::string& camera_topic)
  : node_handle_(),
    position_subscriber_(
        node_handle_.subscribe(pose_topic, 5,
                               &ViewpointNode::PoseCallback, this)),
    event_subscriber_(
        node_handle_.subscribe(event_topic, 5,
                               &ViewpointNode::EventCallback, this)),
    camera_publisher_(
        node_handle_.advertise<view_controller_msgs::CameraPlacement>(
            camera_topic, 5)),
    current_position_() {
}

void ViewpointNode::PoseCallback(const geometry_msgs::Pose& pose) {
  current_position_ = pose.position;
}

void ViewpointNode::EventCallback(const MarkerEvent& event) {
  if (event.type == MarkerEvent::CLICK) {
    geometry_msgs::Point orthogonal_position;
    ComputeOrthogonalPoint(current_position_, event.position,
                           event.control_type, &orthogonal_position);
    view_controller_msgs::CameraPlacement message;
    MakeCameraPlacementMessage(orthogonal_position, event.position, &message);
    camera_publisher_.publish(message);
  }
}

void ViewpointNode::MakeCameraPlacementMessage(
    const geometry_msgs::Point& position,
    const geometry_msgs::Point& focus,
    view_controller_msgs::CameraPlacement* message) {
  message->target_frame = "<Fixed Frame>";

  message->time_from_start = ros::Duration(0.5);

  message->eye.header.stamp = ros::Time::now();
  message->eye.header.frame_id = "<Fixed Frame>";
  message->focus.header.stamp = ros::Time::now();
  message->focus.header.frame_id = "<Fixed Frame>";
  message->up.header.stamp = ros::Time::now();
  message->up.header.frame_id = "<Fixed Frame>";

  message->eye.point.x = position.x;
  message->eye.point.y = position.y;
  message->eye.point.z = position.z;

  message->focus.point.x = focus.x;
  message->focus.point.y = focus.y;
  message->focus.point.z = focus.z;

  message->up.vector.x = 0.0;
  message->up.vector.y = 0.0;
  message->up.vector.z = 1.0;
}
}  // namespace simplecp

int main(int argc, char** argv) {
  ros::init(argc, argv, "viewpoint_node");
  simplecp::ViewpointNode viewpoint_node(
      "/rviz_camera_publisher/camera_pose", "/simplecp/marker_events",
      "/rviz/camera_placement");
  ros::spin();
  return 0;
}
