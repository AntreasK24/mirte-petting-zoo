#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

#include "petting_zoo_navigation/walking_around_field.hpp"


using namespace std::chrono_literals;

fieldWalker::fieldWalker()
: Node("walking_around_field")
{
    front_offset_ = this->declare_parameter<double>("front_offset", M_PI / 2.0);
    flip_scan_ = this->declare_parameter<bool>("flip_scan", false);
    min_range_ = this->declare_parameter<double>("min_range", 0.15);

    robot_half_width_ = this->declare_parameter<double>("robot_half_width", 0.19);
    arm_reach_ = this->declare_parameter<double>("arm_reach", 0.35);   // measure this!

    stop_margin_ = this->declare_parameter<double>("stop_margin", 0.15);
    slow_margin_ = this->declare_parameter<double>("slow_margin", 0.30);
    turn_speed_ = this->declare_parameter<double>("turn_speed", 1.0);

    influence_radius_ = this->declare_parameter<double>("influence_radius", 0.6);
    rep_gain_ = this->declare_parameter<double>("rep_gain", 1.5);
    max_ang_ = this->declare_parameter<double>("max_ang", 1.0);

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/mirte_base_controller/cmd_vel",
        1);

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&fieldWalker::scan_callback, this, std::placeholders::_1));

    // Same enable topic as walking_around, so the FSM drives either node
    enable_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "walking_around/enable", rclcpp::QoS(1).transient_local(),
        std::bind(&fieldWalker::enable_callback, this, std::placeholders::_1));

    pick_timer_ = this->create_wall_timer(
        5s,std::bind(&fieldWalker::pick_new_velocity,this)
    );

    publish_timer_ = this->create_wall_timer(
        50ms,std::bind(&fieldWalker::publish_velocity,this)
    );

    pick_new_velocity();
}

void fieldWalker::pick_new_velocity(){
    current_cmd_.linear.x  = 0.1;
    current_cmd_.angular.z = ang_dist_(rng_);

    straighten_timer_ = this->create_wall_timer(2s, [this](){
        current_cmd_.angular.z = 0.0;
        straighten_timer_->cancel();
    });
}

// Re-read tunables so `ros2 param set` takes effect without restarting the node
void fieldWalker::refresh_params(){
    this->get_parameter("front_offset", front_offset_);
    this->get_parameter("flip_scan", flip_scan_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("robot_half_width", robot_half_width_);
    this->get_parameter("arm_reach", arm_reach_);
    this->get_parameter("stop_margin", stop_margin_);
    this->get_parameter("slow_margin", slow_margin_);
    this->get_parameter("turn_speed", turn_speed_);
    this->get_parameter("influence_radius", influence_radius_);
    this->get_parameter("rep_gain", rep_gain_);
    this->get_parameter("max_ang", max_ang_);
}

void fieldWalker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    refresh_params();

    double closest = std::numeric_limits<double>::infinity();
    double left  = std::numeric_limits<double>::infinity();
    double right = std::numeric_limits<double>::infinity();
    double rx = 0.0, ry = 0.0, max_w = 0.0;

    for(size_t i = 0; i < msg->ranges.size(); ++i){
        double r = msg->ranges[i];
        if(!std::isfinite(r) || r < min_range_ || r > msg->range_max) continue;
        double angle = msg->angle_min + i * msg->angle_increment - front_offset_;
        if (flip_scan_) angle = -angle;   // mirrored lidar: make +angle = left again

        // Convert the polar reading to x (forward) / y (left) in the lidar frame
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);

        // Repulsion from every nearby point, in any direction
        if (r < influence_radius_) {
            double w = (influence_radius_ - r) / influence_radius_;   // 0 at the edge, ~1 when touching
            rx -= w * x / r;                                           // unit vector pointing away from the point
            ry -= w * y / r;
            max_w = std::max(max_w, w);
        }

        if (x > 0.0 && std::abs(y) < robot_half_width_) {
            // In the path ahead
            closest = std::min(closest, x);
        } else if (x > -0.2 && x < arm_reach_ + 0.1) {
            // Beside the robot or the lowered arm: gap between its edge and the obstacle
            double gap = std::abs(y) - robot_half_width_;
            if (y > 0) left  = std::min(left, gap);
            else       right = std::min(right, gap);
        }
    }

    front_dist_  = closest;
    left_clear_  = left;
    right_clear_ = right;

    // A wall gives many points, so use the sum only for direction and the closest point for strength
    double len = std::hypot(rx, ry);
    rep_x_ = (len > 0.0) ? rx / len * max_w : 0.0;
    rep_y_ = (len > 0.0) ? ry / len * max_w : 0.0;
}


void fieldWalker::enable_callback(const std_msgs::msg::Bool::SharedPtr msg){
    enabled_ = msg->data;
    if (!enabled_) velocity_publisher_->publish(geometry_msgs::msg::Twist());  // stop now
}

void fieldWalker::publish_velocity(){
    if(!enabled_) return;

    auto cmd = current_cmd_;

    // Steer towards "forward + push away from nearby obstacles"
    if (rep_x_ != 0.0 || rep_y_ != 0.0) {
        double dx = 1.0 + rep_gain_ * rep_x_;
        double dy = rep_gain_ * rep_y_;
        double heading = std::atan2(dy, dx);              // angle we want to drive towards
        cmd.angular.z = std::clamp(2.0 * heading, -max_ang_, max_ang_);
        cmd.linear.x *= std::max(0.0, std::cos(heading)); // slow down when we need to turn a lot
    }

    // Front safety stop. Distances are measured from the lidar, so account for the arm
    const double stop_dist = arm_reach_ + stop_margin_;
    const double slow_dist = stop_dist + slow_margin_;

    if (front_dist_ < stop_dist) {
        // Choose a direction only when the turn starts, then stick with it
        if (!avoiding_) {
            turn_dir_ = (left_clear_ > right_clear_) ? 1.0 : -1.0;
            avoiding_ = true;
        }
        cmd.linear.x  = 0.0;
        cmd.angular.z = turn_dir_ * turn_speed_;
    } else {
        avoiding_ = false;
        if (front_dist_ < slow_dist) {
            // Scale speed down from full at slow_dist towards stop_dist
            double scale = (front_dist_ - stop_dist) / (slow_dist - stop_dist);
            cmd.linear.x *= std::clamp(scale, 0.3, 1.0);
        }
    }
    velocity_publisher_->publish(cmd);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<fieldWalker>());
    rclcpp::shutdown();
    return 0;
}
