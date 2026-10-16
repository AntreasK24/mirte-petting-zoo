#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>

#include <vision_msgs/msg/bounding_box2_d.hpp>

#include "mirte_apple_detector/apple_detector_node.hpp"

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using vision_msgs::msg::BoundingBox2D;
using vision_msgs::msg::BoundingBox2DArray;

AppleDetector::AppleDetector()
: Node("apple_detector")
{
  // ---- input -----------------------------------------------------------
  image_topic_ = declare_parameter<std::string>(
    "image_topic", "/camera/image_raw/compressed");
  const int queue_size = declare_parameter<int>("queue_size", 1);
  const double max_rate = declare_parameter<double>("max_rate", 0.0);
  min_period_ = (max_rate > 0.0) ? 1.0 / max_rate : 0.0;
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);

  // ---- shape filtering -------------------------------------------------
  min_area_ = declare_parameter<int>("min_area", 500);
  max_area_ = declare_parameter<int>("max_area", 0);
  min_circularity_ = declare_parameter<double>("min_circularity", 0.6);
  min_solidity_ = declare_parameter<double>("min_solidity", 0.85);
  min_aspect_ = declare_parameter<double>("min_aspect", 0.0);
  min_fill_ = declare_parameter<double>("min_fill", 0.0);
  overlap_ratio_ = declare_parameter<double>("overlap_ratio", 0.3);
  blur_ksize_ = declare_parameter<int>("blur_ksize", 5);
  const int morph = declare_parameter<int>("morph_ksize", 5);
  morph_kernel_ = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(std::max(1, morph), std::max(1, morph)));

  // ---- colour classes --------------------------------------------------
  // Exactly two fixed classes: red and green apples. Only their HSV
  // thresholds are tunable from YAML (<colour>.hue_ranges / sat_min /
  // val_min / a_min / a_max); topic, label and debug-draw colour are fixed.
  setup_colour("red", "red_apple", cv::Scalar(0, 0, 255), {0, 10, 170, 179}, 120, 60, 145, 255);
  setup_colour("green", "green_apple", cv::Scalar(0, 200, 0), {28, 95}, 40, 30, 0, 120);
  if (classes_.empty()) {
    RCLCPP_FATAL(get_logger(), "No apple colours configured; shutting down.");
    throw std::runtime_error("no colours configured");
  }

  // Annotated camera frame; not advertised at all when disabled.
  if (publish_debug_image_) {
    debug_pub_ = create_publisher<Image>("apple_detector/image", 10);
  }

  sub_ = create_subscription<CompressedImage>(
    image_topic_, queue_size,
    std::bind(&AppleDetector::image_cb, this, std::placeholders::_1));

  std::string names;
  for (const auto & c : classes_) {names += (names.empty() ? "" : ", ") + c.name;}
  RCLCPP_INFO(
    get_logger(),
    "apple_detector ready: colours=[%s], min_area=%d, min_circularity=%.2f, "
    "subscribing to '%s', debug image %s",
    names.c_str(), min_area_, min_circularity_, image_topic_.c_str(),
    publish_debug_image_ ? "on 'apple_detector/image'" : "disabled");
}

// ---------------------------------------------------------------------
void AppleDetector::setup_colour(
  const std::string & name, const std::string & label,
  const cv::Scalar & draw_colour,
  const std::vector<int64_t> & def_hue, int def_sat_min, int def_val_min,
  int def_a_min, int def_a_max)
{
  const auto hue = declare_parameter<std::vector<int64_t>>(name + ".hue_ranges", def_hue);
  const int sat_min = declare_parameter<int>(name + ".sat_min", def_sat_min);
  const int val_min = declare_parameter<int>(name + ".val_min", def_val_min);
  const int a_min = declare_parameter<int>(name + ".a_min", def_a_min);
  const int a_max = declare_parameter<int>(name + ".a_max", def_a_max);

  if (hue.size() < 2 || hue.size() % 2 != 0) {
    RCLCPP_ERROR(
      get_logger(), "%s.hue_ranges must be pairs [lo,hi,...]; skipping '%s'",
      name.c_str(), name.c_str());
    return;
  }

  ColourClass c;
  c.name = name;
  c.label = label;
  c.draw_colour = draw_colour;
  c.a_min = a_min;
  c.a_max = a_max;
  for (size_t i = 0; i + 1 < hue.size(); i += 2) {
    c.ranges.emplace_back(
      cv::Scalar(hue[i], sat_min, val_min),
      cv::Scalar(hue[i + 1], 255, 255));
  }
  const std::string topic = "detections/apples/" + name;
  c.pub = create_publisher<BoundingBox2DArray>(topic, 10);
  if (publish_debug_image_) {
    c.mask_pub = create_publisher<Image>("apple_detector/mask/" + name, 10);
  }
  classes_.push_back(c);
  RCLCPP_INFO(
    get_logger(), "%s -> '%s' (label '%s', %zu hue range(s), S>=%d V>=%d, %d<=a*<=%d)",
    name.c_str(), topic.c_str(), label.c_str(), c.ranges.size(), sat_min, val_min,
    a_min, a_max);
}

// ---------------------------------------------------------------------
double AppleDetector::steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void AppleDetector::image_cb(const CompressedImage::SharedPtr msg)
{
  // Rate limit: drop the frame before the costly decode.
  if (min_period_ > 0.0) {
    const double now = steady_seconds();
    if (now - last_proc_ < min_period_) {return;}
    last_proc_ = now;
  }

  cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1,
    const_cast<unsigned char *>(msg->data.data()));
  const cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);
  if (frame.empty()) {
    RCLCPP_WARN(get_logger(), "failed to decode image");
    return;
  }

  cv::Mat proc = frame;
  if (blur_ksize_ >= 3) {
    const int k = blur_ksize_ | 1;  // must be odd
    cv::GaussianBlur(frame, proc, cv::Size(k, k), 0);
  }
  cv::Mat hsv;
  cv::cvtColor(proc, hsv, cv::COLOR_BGR2HSV);
  // LAB a* is a green<->red axis that does not change with brightness.
  cv::Mat lab, a_chan;
  cv::cvtColor(proc, lab, cv::COLOR_BGR2Lab);
  cv::extractChannel(lab, a_chan, 1);

  // Draw/encode only when enabled and someone is subscribed; else it's free.
  const bool debug = debug_pub_ && debug_pub_->get_subscription_count() > 0;
  cv::Mat canvas;
  if (debug) {canvas = frame.clone();}

  std::vector<std::vector<Detection>> dets(classes_.size());
  for (size_t i = 0; i < classes_.size(); ++i) {
    const auto & cls = classes_[i];
    cv::Mat mask;
    for (const auto & r : cls.ranges) {
      cv::Mat m;
      cv::inRange(hsv, r.first, r.second, m);
      if (mask.empty()) {mask = m;} else {cv::bitwise_or(mask, m, mask);}
    }
    if (cls.a_min > 0 || cls.a_max < 255) {
      cv::Mat a_mask;
      cv::inRange(a_chan, cv::Scalar(cls.a_min), cv::Scalar(cls.a_max), a_mask);
      cv::bitwise_and(mask, a_mask, mask);
    }
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_);

    if (cls.mask_pub && cls.mask_pub->get_subscription_count() > 0) {
      cls.mask_pub->publish(to_image_msg(mask, "mono8", msg->header));
    }
    dets[i] = contours_to_detections(mask);
  }

  resolve_overlaps(dets);

  for (size_t i = 0; i < classes_.size(); ++i) {
    const auto & cls = classes_[i];
    BoundingBox2DArray out;
    out.header = msg->header;
    for (const auto & d : dets[i]) {
      const cv::Rect & r = d.rect;
      BoundingBox2D box;
      // vision_msgs boxes are center-based; convert from OpenCV's top-left rect.
      box.center.position.x = r.x + r.width / 2.0;
      box.center.position.y = r.y + r.height / 2.0;
      box.size_x = r.width;
      box.size_y = r.height;
      out.boxes.push_back(box);

      if (debug) {
        cv::rectangle(canvas, r, cls.draw_colour, 2);
        cv::putText(
          canvas, cls.label, cv::Point(r.x, std::max(12, r.y - 4)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cls.draw_colour, 1, cv::LINE_AA);
      }
    }
    cls.pub->publish(out);
  }

  if (debug) {
    debug_pub_->publish(to_image_msg(canvas, "bgr8", msg->header));
  }
}

// ---------------------------------------------------------------------
sensor_msgs::msg::Image AppleDetector::to_image_msg(
  const cv::Mat & img, const std::string & encoding,
  const std_msgs::msg::Header & header)
{
  // img is contiguous 8-bit (a cloned BGR frame or a fresh mask), so copy flat.
  Image out;
  out.header = header;
  out.height = static_cast<uint32_t>(img.rows);
  out.width = static_cast<uint32_t>(img.cols);
  out.encoding = encoding;
  out.is_bigendian = 0;
  out.step = static_cast<uint32_t>(img.cols * img.channels());
  const size_t size = static_cast<size_t>(out.step) * img.rows;
  out.data.resize(size);
  std::memcpy(out.data.data(), img.data, size);
  return out;
}

// ---------------------------------------------------------------------
std::vector<Detection> AppleDetector::contours_to_detections(const cv::Mat & mask)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<Detection> dets;
  for (const auto & c : contours) {
    const double area = cv::contourArea(c);
    if (area < min_area_) {continue;}
    if (max_area_ > 0 && area > max_area_) {continue;}

    const double perimeter = cv::arcLength(c, true);
    if (perimeter <= 0.0) {continue;}
    // Circularity: 1.0 for a perfect circle; apples are roundish.
    const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    if (circularity < min_circularity_) {continue;}
    // Solidity: contour area / convex-hull area; high for a solid blob.
    std::vector<cv::Point> hull;
    cv::convexHull(c, hull);
    const double hull_area = cv::contourArea(hull);
    const double solidity = (hull_area > 0.0) ? area / hull_area : 0.0;
    if (solidity < min_solidity_) {continue;}

    // Aspect: short side / long side; rejects sleeves, bottles and other long shapes.
    const cv::Rect rect = cv::boundingRect(c);
    const double aspect = static_cast<double>(std::min(rect.width, rect.height)) /
      std::max(rect.width, rect.height);
    if (aspect < min_aspect_) {continue;}
    // Fill: area / enclosing circle area; less noisy than circularity on jagged edges.
    cv::Point2f centre;
    float radius = 0.0f;
    cv::minEnclosingCircle(c, centre, radius);
    const double circle_area = CV_PI * radius * radius;
    const double fill = (circle_area > 0.0) ? area / circle_area : 0.0;
    if (fill < min_fill_) {continue;}

    dets.push_back({rect, area});
  }
  return dets;
}

// ---------------------------------------------------------------------
void AppleDetector::resolve_overlaps(std::vector<std::vector<Detection>> & dets) const
{
  if (overlap_ratio_ <= 0.0) {return;}

  std::vector<std::vector<bool>> drop(dets.size());
  for (size_t i = 0; i < dets.size(); ++i) {drop[i].assign(dets[i].size(), false);}

  for (size_t i = 0; i < dets.size(); ++i) {
    for (size_t j = i + 1; j < dets.size(); ++j) {
      for (size_t a = 0; a < dets[i].size(); ++a) {
        for (size_t b = 0; b < dets[j].size(); ++b) {
          const cv::Rect & ra = dets[i][a].rect;
          const cv::Rect & rb = dets[j][b].rect;
          const double inter = (ra & rb).area();
          if (inter <= 0.0) {continue;}
          // Overlap relative to the smaller box, so a red patch inside a green apple counts.
          const double overlap = inter / std::min(ra.area(), rb.area());
          if (overlap < overlap_ratio_) {continue;}
          // Same apple seen as both colours: the colour with more pixels wins.
          if (dets[i][a].area >= dets[j][b].area) {drop[j][b] = true;} else {drop[i][a] = true;}
        }
      }
    }
  }

  for (size_t i = 0; i < dets.size(); ++i) {
    std::vector<Detection> kept;
    for (size_t k = 0; k < dets[i].size(); ++k) {
      if (!drop[i][k]) {kept.push_back(dets[i][k]);}
    }
    dets[i] = std::move(kept);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AppleDetector>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("apple_detector"), "startup failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
