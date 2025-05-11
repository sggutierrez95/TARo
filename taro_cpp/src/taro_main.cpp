#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
   // Initialize ROS and create the Node
   rclcpp::init(argc, argv);
   auto const node = std::make_shared<rclcpp::Node>(
      "taro_cpp",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
   );

   // Create a ROS logger
   auto const logger = rclcpp::get_logger("taro_cpp");

   // Create the MoveIt MoveGroup Interface
   using moveit::planning_interface::MoveGroupInterface;
   auto move_group_interface = MoveGroupInterface(node, "manipulator");

   // Set a target Pose
   auto const target_pose = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.28;
      msg.position.y = -0.2;
      msg.position.z = 0.5;
      return msg;
   }();
   move_group_interface.setPoseTarget(target_pose);

   // Create a plan to that target pose
   auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
   }();

   // Execute the plan
   if(success) {
      move_group_interface.execute(plan);
   } else {
      RCLCPP_ERROR(logger, "Planning failed!");
   }

   // Shutdown ROS
   rclcpp::shutdown();
   return 0;
}