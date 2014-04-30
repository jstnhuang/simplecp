#include "simplecp/click_transformer.h"

#include <ros/duration.h>

#include "simplecp/MarkerEvent.h"

namespace simplecp {

ClickTransformer::ClickTransformer(const ros::NodeHandle& node_handle,
                                   const ros::Publisher& event_publisher)
    : node_handle_(node_handle),
      event_publisher_(event_publisher),
      click_state_(ClickState::kStart),
      mouse_down_time_() {
  marker_subscriber_ = node_handle_.subscribe(
      "/pr2_marker_control_transparent/feedback", 5,
      &ClickTransformer::MarkerCallback, this);
}

void ClickTransformer::ProcessFeedback(int event_type,
                                       const std::string& marker_name,
                                       const ros::Time& time) {
  if (marker_name != "head_point_goal" && marker_name != "l_gripper_control"
      && marker_name != "r_gripper_control"
      && marker_name != "l_posture_control"
      && marker_name != "r_posture_control") {
    return;
  }
  if (click_state_ == ClickState::kStart) {
    if (event_type
        == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
      click_state_ = ClickState::kDown;
      mouse_down_time_ = time;
    }
  } else if (click_state_ == ClickState::kDown) {
    if (event_type
        == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      auto click_time = time - mouse_down_time_;
      if (click_time < ros::Duration(0, kClickNanoseconds)) {
        click_state_ = ClickState::kStart;
        simplecp::MarkerEvent event;
        event.type = simplecp::MarkerEvent::CLICK;
        event.marker_name = marker_name;
        event_publisher_.publish(event);
      } else {
        click_state_ = ClickState::kStart;
        simplecp::MarkerEvent event;
        event.type = simplecp::MarkerEvent::DRAG;
        event.marker_name = marker_name;
        event_publisher_.publish(event);
      }
    }
  } else {
    click_state_ = ClickState::kStart;
  }
}

void ClickTransformer::MarkerCallback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback) {
  ProcessFeedback(feedback.event_type, feedback.marker_name, ros::Time::now());
}
}  // namespace simplecp

int main(int argc, char** argv) {
  ros::init(argc, argv, "click_transformer");
  ros::NodeHandle node_handle("simplecp");
  ros::Publisher event_publisher =
      node_handle.advertise<simplecp::MarkerEvent>("marker_events", 5);
  simplecp::ClickTransformer click_transformer(node_handle, event_publisher);
  ros::spin();
  return 0;
}
