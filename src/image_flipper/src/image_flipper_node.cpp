#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageFlipper : public rclcpp::Node
{
public:
    ImageFlipper() : Node("image_flipper")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_flipped", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&ImageFlipper::flip_image_callback, this, std::placeholders::_1));
    }

private:
    void flip_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat flipped_frame;
        cv::flip(frame, flipped_frame, -1); // Flip both horizontally (around y-axis) and vertically (around x-axis)
        auto flipped_msg = cv_bridge::CvImage(msg->header, "bgr8", flipped_frame).toImageMsg();
        publisher_->publish(*flipped_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageFlipper>());
    rclcpp::shutdown();
    return 0;
}
