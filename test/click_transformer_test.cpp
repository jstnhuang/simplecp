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

  void Publish(int event_type, std::string marker_name,
               std::string control_name) {
    Feedback feedback;
    feedback.event_type = event_type;
    feedback.marker_name = marker_name;
    feedback.control_name = control_name;
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
  Publish(Feedback::MOUSE_DOWN, "l_gripper_control", "_u0");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_gripper_control", "_u0");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::CLICK, event->type);
  EXPECT_EQ("l_gripper_control", event->marker_name);
}

TEST_F(ClickTransformerTest, Drag) {
  Publish(Feedback::MOUSE_DOWN, "r_gripper_control", "_u0");
  ros::Duration(0, ClickTransformer::kClickNanoseconds * 2).sleep();
  Publish(Feedback::MOUSE_UP, "r_gripper_control", "_u0");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::DRAG, event->type);
  EXPECT_EQ("r_gripper_control", event->marker_name);
}

TEST_F(ClickTransformerTest, MultipleClickDrag) {
  Publish(Feedback::MOUSE_DOWN, "l_gripper_control", "_u0");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_gripper_control", "_u0");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::CLICK, event->type);
  EXPECT_EQ("l_gripper_control", event->marker_name);

  Publish(Feedback::MOUSE_DOWN, "r_gripper_control", "_u0");
  ros::Duration(0, ClickTransformer::kClickNanoseconds * 2).sleep();
  Publish(Feedback::MOUSE_UP, "r_gripper_control", "_u0");
  event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::DRAG, event->type);
  EXPECT_EQ("r_gripper_control", event->marker_name);
}

TEST_F(ClickTransformerTest, UnsupportedMarkerDoesNothing) {
  Publish(Feedback::MOUSE_DOWN, "unsupported_control", "_u0");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "unsupported_control", "_u0");
  auto event = WaitForEvent();
  EXPECT_EQ(NULL, event.get());
}

TEST_F(ClickTransformerTest, ConvertKnownControl) {
  Publish(Feedback::MOUSE_DOWN, "l_gripper_control", "_u0");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_gripper_control", "_u0");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(ClickTransformer::kGripperControls.at("_u0"), event->control_type);
}

TEST_F(ClickTransformerTest, ConvertPostureControl) {
  Publish(Feedback::MOUSE_DOWN, "l_posture_control", "");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_posture_control", "");
  auto event = WaitForEvent();
  EXPECT_TRUE(event != NULL);
  EXPECT_EQ(MarkerEvent::ROLL, event->control_type);
}

TEST_F(ClickTransformerTest, UnknownControlDoesNothing) {
  Publish(Feedback::MOUSE_DOWN, "l_gripper_control", "unknown_control");
  ros::Duration(0, ClickTransformer::kClickNanoseconds / 2).sleep();
  Publish(Feedback::MOUSE_UP, "l_gripper_control", "unknown_control");
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
