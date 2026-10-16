#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <vision_msgs/msg/bounding_box2_d_array.hpp>

// One apple colour: its HSV range(s), LAB a* gate, output publisher and the
// BGR colour its boxes are drawn in on the debug image.
struct ColourClass
{
  std::string name;
  std::string label;
  cv::Scalar draw_colour;  // BGR, used when annotating the debug image
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub;  // debug mask
  std::vector<std::pair<cv::Scalar, cv::Scalar>> ranges;  // HSV lower/upper pairs
  // LAB a* channel gate (OpenCV 8-bit: 128 = neutral, >128 red, <128 green).
  // 0/255 = gate off.
  int a_min{0};
  int a_max{255};
};

// One accepted blob: its bounding box and contour area (mask pixels).
struct Detection
{
  cv::Rect rect;
  double area;
};

class AppleDetector : public rclcpp::Node
{
public:
  AppleDetector();

private:
  // Set up one colour from easy parameters: a list of hue ranges plus a
  // saturation/value floor (max S/V are always 255). hue_ranges is a flat
  // list of [lo, hi, lo, hi, ...] pairs, so red can span 0-10 AND 170-179.
  // a_min/a_max gate the LAB a* channel, which is independent of brightness.
  void setup_colour(
    const std::string & name, const std::string & label,
    const cv::Scalar & draw_colour,
    const std::vector<int64_t> & def_hue, int def_sat_min, int def_val_min,
    int def_a_min, int def_a_max);

  static double steady_seconds();

  // Wrap a continuous cv::Mat in a sensor_msgs/Image (no cv_bridge dependency,
  // so the same code builds on every ROS 2 distro). encoding: bgr8 or mono8.
  static sensor_msgs::msg::Image to_image_msg(
    const cv::Mat & img, const std::string & encoding,
    const std_msgs::msg::Header & header);

  void image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  std::vector<Detection> contours_to_detections(const cv::Mat & mask);

  // Where boxes of different colours overlap (e.g. a blushed apple), keep only
  // the one with the larger area.
  void resolve_overlaps(std::vector<std::vector<Detection>> & dets) const;

  // ---- members ---------------------------------------------------------
  std::string image_topic_;
  double min_period_{0.0};
  double last_proc_{0.0};

  int min_area_{500};
  int max_area_{0};
  double min_circularity_{0.6};
  double min_solidity_{0.85};
  double min_aspect_{0.0};    // short side / long side of the box
  double min_fill_{0.0};      // area / area of the enclosing circle
  double overlap_ratio_{0.3};  // red/green overlap (of the smaller box) above this clash
  int blur_ksize_{5};
  cv::Mat morph_kernel_;

  bool publish_debug_image_{true};  // advertise/draw apple_detector/image and masks?

  std::vector<ColourClass> classes_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};
