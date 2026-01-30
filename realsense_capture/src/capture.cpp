#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <ctime>
#include <iomanip>
#include <sstream>

class Capture : public rclcpp::Node
{
public:
  Capture()

  : Node("capture"),
    capture_requested(false),
    depth_written(false),
    cloud_written(false)
  {
    callback_group_subscriber1_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_subscriber2_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub1_opt;
    sub1_opt.callback_group = callback_group_subscriber1_;

    rclcpp::SubscriptionOptions sub2_opt;
    sub2_opt.callback_group = callback_group_subscriber2_;

    depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/depth/image_rect_raw", 10,
      std::bind(&Capture::depth_image_callback, this, std::placeholders::_1),
      sub1_opt);

    pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 10,
      std::bind(&Capture::point_cloud_callback, this, std::placeholders::_1),
      sub2_opt);

    writer = std::make_unique<rosbag2_cpp::Writer>();
    writer->open(make_bag_name());
    rclcpp::on_shutdown(std::bind(&Capture::on_shutdown_handler, this));


    RCLCPP_INFO(this->get_logger(),
      "Press ENTER to capture depth + cloud into the same bag.");

    keyboard_thread = std::thread(&Capture::keyboard_loop, this);
  }

  ~Capture()
  {
    keyboard_thread.join();
  }

private:
  std::string make_bag_name()
  {
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "bag_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
  }

  void keyboard_loop()
  {
    while (rclcpp::ok()) {
      std::cin.get();  // wait for ENTER
      depth_written.store(false);
      cloud_written.store(false);
      capture_requested.store(true);
      RCLCPP_INFO(this->get_logger(), "Capture requested");
    }
  }

  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!capture_requested.load() || depth_written.load()) return;

    writer->write(
      *msg,
      "/camera/camera/depth/image_rect_raw",
      rclcpp::Time(msg->header.stamp));

    depth_written.store(true);
    RCLCPP_INFO(this->get_logger(), "Depth captured");

    check_capture_complete();
  }

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!capture_requested.load() || cloud_written.load()) return;

    writer->write(
      *msg,
      "/camera/camera/depth/color/points",
      rclcpp::Time(msg->header.stamp));

    cloud_written.store(true);
    RCLCPP_INFO(this->get_logger(), "Point cloud captured");

    check_capture_complete();
  }

  void check_capture_complete()
  {
    if (depth_written.load() && cloud_written.load()) {
      capture_requested.store(false);
      RCLCPP_INFO(this->get_logger(), "Capture complete (waiting for next key)");
    }
  }

  void on_shutdown_handler()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down, finalizing bag...");
    writer.reset();
  }



  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub;

  std::unique_ptr<rosbag2_cpp::Writer> writer;

  std::atomic<bool> capture_requested;
  std::atomic<bool> depth_written;
  std::atomic<bool> cloud_written;

  std::thread keyboard_thread;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Capture>();
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 2);

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}