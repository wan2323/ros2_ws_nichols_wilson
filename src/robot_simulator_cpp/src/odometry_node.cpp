#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "custom_interfaces/srv/reset_position.hpp"


using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node {
public:
  OdometryNode()
  // Name node
  : rclcpp::Node("odometry_node")
  // Create tf broadcast
  , tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  // Create initial values
  , x_(0.0), y_(0.0), theta_(0.0)
  {
    last_update_time_ = now();
    
    // Create Subscription
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, // Subscribed topic name is /cmd_vel
      [this](geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lk(mtx_);
        last_twist_ = *msg;
      });

    // Create Service
    reset_srv_ = create_service<custom_interfaces::srv::ResetPosition>(
    "ResetPosition",  
    [this](const std::shared_ptr<custom_interfaces::srv::ResetPosition::Request> req,
         std::shared_ptr<custom_interfaces::srv::ResetPosition::Response> res) {
    std::lock_guard<std::mutex> lk(mtx_);
    x_ = req->target.pose.position.x; // If service is called, reset stored xy theta with new and proceed normally with cmd_vel
    y_ = req->target.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(req->target.pose.orientation, q);
    double yaw = tf2::getYaw(q);
    theta_ = yaw;
    last_update_time_ = now();
    res->success = true;
  });


    // Set periodic timer for tf broadcast
    timer_ = create_wall_timer(20ms,
      std::bind(&OdometryNode::onTimer, this));
  }

private:
  void onTimer() {

    // If subscriber/client activates callback ensure values don't get overriden by timer
    std::lock_guard<std::mutex> lk(mtx_); 

    // Calc timing from last callback
    const rclcpp::Time t_now = now();
    const double dt = (t_now - last_update_time_).seconds();
    if (dt <= 0.0) return;

    // Import /cmd_vel commands 
    const double vx = last_twist_.linear.x;
    const double vy = last_twist_.linear.y;
    const double wz = last_twist_.angular.z;

    // Calculate new pos based on prior pos and vel cmds
    x_     += (vx * std::cos(theta_) - vy * std::sin(theta_)) * dt;
    y_     += (vx * std::sin(theta_) + vy * std::cos(theta_)) * dt;
    theta_ += wz * dt;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
    last_update_time_ = t_now;


    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t_now;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    tf.transform.rotation = tf2::toMsg(q);
    
    // Update tf broadcast
    tf_broadcaster_->sendTransform(tf);
  }

  // Member variables
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x_, y_, theta_;
  geometry_msgs::msg::Twist last_twist_{};
  rclcpp::Time last_update_time_;
  std::mutex mtx_;
};

// main function to ACTUALLY run the node
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
