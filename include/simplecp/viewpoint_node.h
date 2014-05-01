#ifndef SIMPLECP_VIEWPOINT_NODE_H_
#define SIMPLECP_VIEWPOINT_NODE_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <view_controller_msgs/CameraPlacement.h>

#include "simplecp/MarkerEvent.h"

namespace simplecp {
void ComputeOrthogonalPoint(
    const geometry_msgs::Point& current_position,
    const geometry_msgs::Point& marker_position,
    const std::string& control_type,
    geometry_msgs::Point* orthogonal_position);

class ViewpointNode {
 public:
  ViewpointNode(const std::string& pose_topic,
                const std::string& event_topic,
                const std::string& viewpoint_topic);
 private:
  ros::NodeHandle node_handle_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber event_subscriber_;
  ros::Publisher camera_publisher_;
  geometry_msgs::Point current_position_;

  void PoseCallback(const geometry_msgs::Pose& pose);
  void EventCallback(const MarkerEvent& event);
  void MakeCameraPlacementMessage(
      const geometry_msgs::Point& position,
      const geometry_msgs::Point& focus,
      view_controller_msgs::CameraPlacement* message);
};
}

#endif // SIMPLECP_VIEWPOINT_NODE_H_
