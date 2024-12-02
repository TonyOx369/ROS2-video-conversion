#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageConversion : public rclcpp::Node {
public:
    ImageConversion() : Node("image_conversion"), is_greyscale_mode_(false) {
        this->declare_parameter<std::string>("image_topic", "/image_raw");
        this->declare_parameter<std::string>("converted_image_topic", "/converted_image");
        this->declare_parameter<std::string>("service_name", "switch_mode");

        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string converted_image_topic = this->get_parameter("converted_image_topic").as_string();
        std::string service_name = this->get_parameter("service_name").as_string();

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&ImageConversion::imageCallback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(converted_image_topic, 10);

        service_ = this->create_service<std_srvs::srv::SetBool>(
            service_name,
            std::bind(&ImageConversion::switchModeCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Image Conversion Node Initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received image message");

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat cv_image = cv_ptr->image;

            if (is_greyscale_mode_) {
                cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);
                RCLCPP_INFO(this->get_logger(), "Converted image to grayscale");
            } else {
                RCLCPP_INFO(this->get_logger(), "Displaying image in color mode");
            }

            if (std::getenv("DISPLAY")) {
                cv::imshow("Video Feed", cv_image);
                cv::waitKey(1);
            }

            auto converted_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();

            publisher_->publish(*converted_msg);
        } 
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
        }
    }

    void switchModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        is_greyscale_mode_ = request->data;
        response->success = true;
        response->message = is_greyscale_mode_ ? "Mode switched to: Grayscale" : "Mode switched to: Color";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    bool is_greyscale_mode_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConversion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
