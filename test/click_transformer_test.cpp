#include "simplecp/click_transformer.h"

#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/topic.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "simplecp/MarkerEvent.h"

namespace simplecp {

typedef visualization_msgs::InteractiveMarkerFeedback Feedback;

class ClickTransformerTest : public ::testing::Test {
 public:
  ClickTransformerTest()
      : node_handle_("simplecp"),
        marker_publisher_(
            node_handle_.advertise<Feedback>(
                "/pr2_marker_control_transparent/feedback", 5)),
        event_subscriber_(
            node_handle_.subscribe("marker_events", 5,
                                   &ClickTransformerTest::MarkerEventCallback,
                                   this)) {
  }

  void SetUp() {
    while (!IsNodeAlive()) {
      ros::spinOnce();
    }
  }

  void Publish(int event_type, std::string marker_name) {
    Feedback feedback;
    feedback.event_type = event_type;
    feedback.marker_name = marker_name;
    marker_publisher_.publish(feedback);
  }

  boost::shared_ptr<const MarkerEvent> WaitForEvent() {
    return ros::topic::waitForMessage<MarkerEvent>(
        event_subscriber_.getTopic(), ros::Duration(1));
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher marker_publisher_;
  ros::Subscriber event_subscriber_;

  void MarkerEventCallback(const MarkerEvent& event) {
  }

  bool IsNodeAlive() {
    return (marker_publisher_.getNumSubscribers() > 0)
        && (event_subscriber_.getNumPublishers() > 0);
  }
};

TEST_F(ClickTransformerTest, Click) {
  Publish(Feedback::MOUSE_DOWN, "l_gripper_control");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_gripper_control");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::CLICK, event->type);
  EXPECT_EQ("l_gripper_control", event->marker_name);
}

TEST_F(ClickTransformerTest, Drag) {
  Publish(Feedback::MOUSE_DOWN, "r_gripper_control");
  ros::Duration(0, ClickTransformer::kClickNanoseconds + 1000).sleep();
  Publish(Feedback::MOUSE_UP, "r_gripper_control");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::DRAG, event->type);
  EXPECT_EQ("r_gripper_control", event->marker_name);
}

TEST_F(ClickTransformerTest, MultipleClickDrag) {
  Publish(Feedback::MOUSE_DOWN, "l_gripper_control");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_gripper_control");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::CLICK, event->type);
  EXPECT_EQ("l_gripper_control", event->marker_name);

  Publish(Feedback::MOUSE_DOWN, "r_gripper_control");
  ros::Duration(0, ClickTransformer::kClickNanoseconds + 1000).sleep();
  Publish(Feedback::MOUSE_UP, "r_gripper_control");
  event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::DRAG, event->type);
  EXPECT_EQ("r_gripper_control", event->marker_name);
}

TEST_F(ClickTransformerTest, UnsupportedMarkerDoesNothing) {
  Publish(Feedback::MOUSE_DOWN, "unsupported_control");
  ros::Duration(0, ClickTransformer::kClickNanoseconds - 1000).sleep();
  Publish(Feedback::MOUSE_UP, "unsupported_control");
  auto event = WaitForEvent();
  EXPECT_EQ(NULL, event.get());
}

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "click_transformer_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
