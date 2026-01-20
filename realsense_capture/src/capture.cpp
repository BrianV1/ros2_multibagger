#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <thread>
#include <memory>
#include <string>
#include <atomic>

class Capture : public rclcpp::Node
{
public:
    Capture()
        : Node("capture"), depthCaptured(false), cloudCaptured(false)
    {
        callback_group_subscriber1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        RCLCPP_INFO(this->get_logger(), "Setting up subscribers...");
        
        depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10,
            std::bind(&Capture::depth_image_callback, this, std::placeholders::_1), sub1_opt);

        pointCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&Capture::point_cloud_callback, this, std::placeholders::_1), sub2_opt);

        RCLCPP_INFO(this->get_logger(), "...Done setting up subscribers");

        rclcpp::Time t = this->now();
        std::time_t sec = static_cast<std::time_t>(t.seconds());
        std::tm tm = *std::localtime(&sec);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        writer = std::make_unique<rosbag2_cpp::Writer>();
        writer->open("bag_" + oss.str());

        RCLCPP_INFO(this->get_logger(), "Waiting to capture messages...");
    }

private:
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (depthCaptured.load()) return;

        RCLCPP_INFO(this->get_logger(), "Captured DEPTH image");
        
        writer->write(*msg, "/camera/camera/depth/image_rect_raw", rclcpp::Time(msg->header.stamp));
        depthCaptured.store(true);

        check_and_shutdown();
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (cloudCaptured.load()) return;

        RCLCPP_INFO(this->get_logger(), "Captured POINT CLOUD");
        
        writer->write(*msg, "/camera/camera/depth/color/points", rclcpp::Time(msg->header.stamp));
        cloudCaptured.store(true);

        check_and_shutdown();
    }

    void check_and_shutdown()
    {
        if (depthCaptured.load() && cloudCaptured.load()) {
            RCLCPP_INFO(this->get_logger(), "Both depth and cloud captured, shutting down...");
            // Give writer time to flush
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::shutdown();
        }
    }

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSub;
    
    std::unique_ptr<rosbag2_cpp::Writer> writer;
    std::atomic<bool> depthCaptured;
    std::atomic<bool> cloudCaptured;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Capture>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}