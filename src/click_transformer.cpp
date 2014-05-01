#include "simplecp/click_transformer.h"

#include <ros/console.h>
#include <ros/duration.h>

namespace simplecp {

// Maps built by observing /pr2_marker_control_transparent/feedback
// TODO(jstn): These sometimes change between executions :(.
const ControlMap ClickTransformer::kGripperControls = {
  { "_u0", MarkerEvent::X },
  { "_u4", MarkerEvent::Y },
  { "_u2", MarkerEvent::Z },
  { "_u3", MarkerEvent::PITCH },
  { "", MarkerEvent::ROLL },
  { "_u1", MarkerEvent::YAW }
};

const ControlMap ClickTransformer::kPointHeadControls = {
  { "_u1", MarkerEvent::X },
  { "_u5", MarkerEvent::Y },
  { "_u3", MarkerEvent::Z },
  { "_u4", MarkerEvent::PITCH },
  { "_u0", MarkerEvent::ROLL },
  { "_u2", MarkerEvent::YAW }
};

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
  } else {
    if (feedback.event_type
        == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      auto click_time = ros::Time::now() - mouse_down_time_;
      simplecp::MarkerEvent event;
      event.marker_name = feedback.marker_name;
      event.control_type = GetControlType(feedback.marker_name,
                                          feedback.control_name);
      event.position = feedback.pose.position;
      if (click_time < ros::Duration(0, kClickNanoseconds)) {
        event.type = simplecp::MarkerEvent::CLICK;
      } else {
        event.type = simplecp::MarkerEvent::DRAG;
      }
      event_publisher_.publish(event);
      click_state_ = ClickState::kStart;
    }
  }
}

std::string ClickTransformer::GetControlType(const std::string& marker_name,
                                             const std::string& control_name) {
  try {
    if (marker_name == "head_point_goal") {
      return kPointHeadControls.at(control_name);
    } else if (marker_name == "l_gripper_control"
        || marker_name == "r_gripper_control") {
      return kGripperControls.at(control_name);
    } else if (marker_name == "l_posture_control"
        || marker_name == "r_gripper_control") {
      return MarkerEvent::ROLL;
    } else {
      ROS_WARN("Unknown marker %s", marker_name.c_str());
      return "Unknown";
    }
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Unknown control %s for marker %s", control_name.c_str(),
              marker_name.c_str());
    return "Unknown";
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
