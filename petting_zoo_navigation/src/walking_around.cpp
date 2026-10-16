#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

#include "petting_zoo_navigation/walking_around.hpp"


using namespace std::chrono_literals;

randomVelocityPublisher::randomVelocityPublisher()
: Node("walking_around")
{

    front_offset_ = this->declare_parameter<double>("front_offset", M_PI / 2.0);
    flip_scan_ = this->declare_parameter<bool>("flip_scan", false);
    min_range_ = this->declare_parameter<double>("min_range", 0.15);

    robot_half_width_ = this->declare_parameter<double>("robot_half_width", 0.19);
    side_margin_ = this->declare_parameter<double>("side_margin", 0.20);
    arm_reach_ = this->declare_parameter<double>("arm_reach", 0.35);   // measure this!
    stop_margin_ = this->declare_parameter<double>("stop_margin", 0.15);
    slow_margin_ = this->declare_parameter<double>("slow_margin", 0.30);

    random_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/mirte_base_controller/cmd_vel",
        1);

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&randomVelocityPublisher::scan_callback, this, std::placeholders::_1));
    
    enable_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "walking_around/enable", rclcpp::QoS(1).transient_local(),
        std::bind(&randomVelocityPublisher::enable_callback, this, std::placeholders::_1));

    pick_timer_ = this->create_wall_timer(
        5s,std::bind(&randomVelocityPublisher::pick_new_velocity,this)
    );

    publish_timer_ = this->create_wall_timer(
        50ms,std::bind(&randomVelocityPublisher::publish_velocity,this)
    );
    
    pick_new_velocity();
}

void randomVelocityPublisher::pick_new_velocity(){
    current_cmd_.linear.x  = 0.1;
    current_cmd_.angular.z = ang_dist_(rng_);

    straighten_timer_ = this->create_wall_timer(2s, [this](){
        current_cmd_.angular.z = 0.0;
        straighten_timer_->cancel();
    });
}

// Re-read tunables so `ros2 param set` takes effect without restarting the node
void randomVelocityPublisher::refresh_params(){
    this->get_parameter("front_offset", front_offset_);
    this->get_parameter("flip_scan", flip_scan_);
    this->get_parameter("min_range", min_range_);
    this->get_parameter("robot_half_width", robot_half_width_);
    this->get_parameter("side_margin", side_margin_);
    this->get_parameter("arm_reach", arm_reach_);
    this->get_parameter("stop_margin", stop_margin_);
    this->get_parameter("slow_margin", slow_margin_);
}

void randomVelocityPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    refresh_params();

    double closest = std::numeric_limits<double>::infinity();
    double left  = std::numeric_limits<double>::infinity();
    double right = std::numeric_limits<double>::infinity();
    for(size_t i = 0; i < msg->ranges.size(); ++i){
        double r = msg->ranges[i];
        if(!std::isfinite(r) || r < min_range_ ||  r > msg->range_max ) continue;
        double angle = msg->angle_min + i * msg->angle_increment - front_offset_;
        if (flip_scan_) angle = -angle;   // mirrored lidar: make +angle = left again

        // Convert the polar reading to x (forward) / y (left) in the lidar frame
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);

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
}


void randomVelocityPublisher::enable_callback(const std_msgs::msg::Bool::SharedPtr msg){
    enabled_ = msg->data;
    avoiding_ = false;
    if (!enabled_) random_velocity_publisher_->publish(geometry_msgs::msg::Twist());  // stop now
}

void randomVelocityPublisher::publish_velocity(){
    if(!enabled_) return;

    auto cmd = current_cmd_;

    // Don't steer into a side that's too close (+z = left, -z = right)
    if (cmd.angular.z > 0 && left_clear_  < side_margin_) cmd.angular.z = 0.0;
    if (cmd.angular.z < 0 && right_clear_ < side_margin_) cmd.angular.z = 0.0;

    // Distances are measured from the lidar, so account for the arm sticking out
    const double stop_dist = arm_reach_ + stop_margin_;
    const double slow_dist = stop_dist + slow_margin_;

    if (!avoiding_ && front_dist_ < stop_dist) {
        // Blocked ahead: pick the side with more room once, then stick with it
        turn_dir_ = (left_clear_ > right_clear_) ? 1.0 : -1.0;
        avoiding_ = true;
    } else if (avoiding_ && front_dist_ > stop_dist + 0.10) {
        // Only drive again once the front is a bit past the stop point, so it doesn't stutter
        avoiding_ = false;
    }

    if (avoiding_) {
        // Turn in place until the front is clear
        cmd.linear.x  = 0.0;
        cmd.angular.z = turn_dir_ * 1.0;
    } else if (front_dist_ < slow_dist) {
        // Scale speed down from full at slow_dist towards stop_dist
        double scale = (front_dist_ - stop_dist) / (slow_dist - stop_dist);
        cmd.linear.x *= std::clamp(scale, 0.3, 1.0);
    }
    random_velocity_publisher_->publish(cmd);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<randomVelocityPublisher>());
    rclcpp::shutdown();
    return 0;

}
