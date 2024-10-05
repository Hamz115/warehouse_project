#include "attach_service/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class ShelfDetector {
public:
  void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr &scan) {
    legs_.clear();
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      if (scan->intensities[i] > intensity_threshold_) {
        legs_.push_back({i, scan->ranges[i]});
      }
    }
    angle_increment_ = scan->angle_increment;
  }

  bool has_valid_shelf() const { return legs_.size() == 2; }
  const auto &get_legs() const { return legs_; }
  float get_angle_increment() const { return angle_increment_; }

private:
  std::vector<std::pair<size_t, float>> legs_;
  float angle_increment_ = 0.0;
  const float intensity_threshold_ = 100.0;
};

class ShelfApproachNode : public rclcpp::Node {
public:
  ShelfApproachNode() : Node("shelf_approach_node") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ShelfApproachNode::scan_callback, this,
                  std::placeholders::_1));

    approach_srv_ = create_service<attach_service::srv::GoToLoading>(
        "approach_shelf",
        std::bind(&ShelfApproachNode::approach_shelf, this,
                  std::placeholders::_1, std::placeholders::_2));

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);
    elevator_pub_ = create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "ShelfApproachNode initialized");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    detector_.process_scan(msg);
  }

  void approach_shelf(
      const std::shared_ptr<attach_service::srv::GoToLoading::Request>,
      std::shared_ptr<attach_service::srv::GoToLoading::Response> response) {
    if (!detector_.has_valid_shelf()) {
      RCLCPP_ERROR(get_logger(), "No valid shelf detected");
      response->complete = false;
      return;
    }

    broadcast_shelf_frame();

    if (navigate_to_pose("robot_front_laser_base_link", "shelf_frame") &&
        navigate_to_pose("robot_front_laser_base_link", "shelf_center")) {
      activate_elevator();
      response->complete = true;
      RCLCPP_INFO(get_logger(),
                  "Shelf approach completed and elevator activated");
    } else {
      response->complete = false;
      RCLCPP_ERROR(get_logger(), "Failed to approach shelf");
    }
  }

  void broadcast_shelf_frame() {
    const auto &legs = detector_.get_legs();
    float angle = std::abs(static_cast<float>(legs[0].first - legs[1].first)) *
                  detector_.get_angle_increment();

    float shelf_width = std::hypot(legs[0].second, legs[1].second);
    float dist_to_center =
        0.5f * std::sqrt(2.0f * (legs[0].second * legs[0].second +
                                 legs[1].second * legs[1].second) -
                         shelf_width * shelf_width);

    float alpha = std::asin(legs[1].second * std::sin(angle) / shelf_width);
    float beta =
        std::asin(0.5f * shelf_width * std::sin(alpha) / dist_to_center);
    float gamma = beta - std::abs(static_cast<float>(legs[0].first) - 540.0f) *
                             detector_.get_angle_increment();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "shelf_frame";
    t.transform.translation.x = dist_to_center * std::cos(gamma);
    t.transform.translation.y = dist_to_center * std::sin(gamma);

    tf2::Quaternion q;
    q.setRPY(0, 0,
             gamma + M_PI_2 -
                 std::asin(legs[1].second * std::sin(angle - beta) /
                           (0.5f * shelf_width)));
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);

    t.child_frame_id = "shelf_center";
    t.transform.translation.x += 0.75;
    tf_broadcaster_->sendTransform(t);
  }

  bool navigate_to_pose(const std::string &from_frame,
                        const std::string &to_frame) {
    auto start_time = this->now();
    while (rclcpp::ok() && (this->now() - start_time) < 30s) {
      try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(from_frame, to_frame,
                                        tf2::TimePointZero);

        double distance = std::hypot(transform.transform.translation.x,
                                     transform.transform.translation.y);
        double angle = std::atan2(transform.transform.translation.y,
                                  transform.transform.translation.x);

        if (distance < 0.05)
          return true;

        geometry_msgs::msg::Twist vel;
        vel.angular.z = -1.2 * angle;
        vel.linear.x = std::min(0.3, 0.6 * distance);
        vel_pub_->publish(vel);

        rclcpp::sleep_for(100ms);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
        return false;
      }
    }
    return false;
  }

  void activate_elevator() {
    std_msgs::msg::String msg;
    msg.data = "up";
    elevator_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Elevator activation command sent");
  }

  ShelfDetector detector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<attach_service::srv::GoToLoading>::SharedPtr approach_srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfApproachNode>());
  rclcpp::shutdown();
  return 0;
}