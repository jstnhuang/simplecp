#include "simplecp/click_transformer.h"

#include <ros/duration.h>

#include "simplecp/MarkerEvent.h"

namespace simplecp {

ClickTransformer::ClickTransformer(const std::string& feedback_topic,
                                   const std::string& event_topic)
    : node_handle_(),
      marker_subscriber_(
          node_handle_.subscribe(feedback_topic, 5,
                                 &ClickTransformer::MarkerCallback, this)),
      event_publisher_(node_handle_.advertise<MarkerEvent>(event_topic, 5)),
      click_state_(ClickState::kStart),
      mouse_down_time_() {
}

void ClickTransformer::MarkerCallback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback) {
  if (feedback.marker_name != "head_point_goal"
      && feedback.marker_name != "l_gripper_control"
      && feedback.marker_name != "r_gripper_control"
      && feedback.marker_name != "l_posture_control"
      && feedback.marker_name != "r_posture_control") {
    return;
  }
  if (click_state_ == ClickState::kStart) {
    if (feedback.event_type
        == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
      click_state_ = ClickState::kDown;
      mouse_down_time_ = ros::Time::now();
    }
  } else if (click_state_ == ClickState::kDown) {
    if (feedback.event_type
        == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      auto click_time = ros::Time::now() - mouse_down_time_;
      if (click_time < ros::Duration(0, kClickNanoseconds)) {
        click_state_ = ClickState::kStart;
        simplecp::MarkerEvent event;
        event.type = simplecp::MarkerEvent::CLICK;
        event.marker_name = feedback.marker_name;
        event_publisher_.publish(event);
      } else {
        click_state_ = ClickState::kStart;
        simplecp::MarkerEvent event;
        event.type = simplecp::MarkerEvent::DRAG;
        event.marker_name = feedback.marker_name;
        event_publisher_.publish(event);
      }
    }
  } else {
    click_state_ = ClickState::kStart;
  }
}
}  // namespace simplecp

int main(int argc, char** argv) {
  ros::init(argc, argv, "click_transformer");
  simplecp::ClickTransformer click_transformer(
      "/pr2_marker_control_transparent/feedback", "/simplecp/marker_events");
  ros::spin();
  return 0;
}
