#ifndef SIMPLECP_CLICK_TRANSFORMER_H_
#define SIMPLECP_CLICK_TRANSFORMER_H_

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

namespace simplecp {
enum class ClickState {
  kStart,
  kDown
};

class ClickTransformer {
 public:
  static const int kClickNanoseconds = 100000000;
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
};
}  // namespace simplecp

#endif  // SIMPLECP_CLICK_TRANSFORMER_H_
