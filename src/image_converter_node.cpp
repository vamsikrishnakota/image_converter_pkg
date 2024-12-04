#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImageConversionNode : public rclcpp::Node
{
public:
    ImageConversionNode() : Node("image_conversion_node"), mode_(2)
    {
        // Initialize publisher and subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageConversionNode::image_callback, this, std::placeholders::_1));
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("converted_image", 10);

        // Initialize service to toggle mode
        mode_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_mode", std::bind(&ImageConversionNode::set_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Image Conversion Node initialized.");
    }

private:
    // Mode: 1 = Greyscale, 2 = Color
    int mode_;

    // Publisher, Subscriber, and Service
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;

    // Image callback
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image to OpenCV image
            cv::Mat input_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
            cv::Mat processed_image;

            // Handle yuv422_yuy2 encoding
            if (msg->encoding == "yuv422_yuy2")
            {
                cv::cvtColor(input_image, processed_image, cv::COLOR_YUV2BGR_YUY2);
            }
            else if (msg->encoding == "bgr8" || msg->encoding == "rgb8")
            {
                processed_image = input_image;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
                return;
            }

            // Apply grayscale conversion if mode is 1
            if (mode_ == 1)
            {
                cv::cvtColor(processed_image, processed_image, cv::COLOR_BGR2GRAY);
            }

            // Publish the processed image
            auto output_msg = cv_bridge::CvImage(msg->header, 
                              (mode_ == 1 ? "mono8" : "bgr8"), processed_image).toImageMsg();
            image_pub_->publish(*output_msg);
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
    }

    // Mode service callback
    void set_mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            mode_ = 1; // Set to Greyscale
            response->success = true;
            response->message = "Mode set to Greyscale.";
        }
        else
        {
            mode_ = 2; // Set to Color
            response->success = true;
            response->message = "Mode set to Color.";
        }

        RCLCPP_INFO(this->get_logger(), "Mode changed: %s", response->message.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}
