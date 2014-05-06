#ifndef SIMPLECP_CLICK_TRANSFORMER_H_
#define SIMPLECP_CLICK_TRANSFORMER_H_

#include <map>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "simplecp/MarkerEvent.h"

namespace simplecp {
enum class ClickState {
  kStart,
  kDown
};

typedef std::map<std::string, std::string> ControlMap;

class ClickTransformer {
 public:
  static const int kClickNanoseconds = 200000000;
  static const ControlMap kGripperControls;
  static const ControlMap kPointHeadControls;

  ClickTransformer(const std::string& feedback_topic,
                   const std::string& event_topic);

 private:
  ros::NodeHandle node_handle_;
  ros::Subscriber marker_subscriber_;
  ros::Publisher event_publisher_;
  ClickState click_state_;
  ros::Time mouse_down_time_;

  void MarkerCallback(
      const visualization_msgs::InteractiveMarkerFeedback& feedback);
  std::string GetControlType(const std::string& marker_name,
                             const std::string& control_name);
};

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

}  // namespace simplecp

#endif  // SIMPLECP_CLICK_TRANSFORMER_H_
