#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
// #include "camera_driver/srv/configure_camera.hpp"
// #include "camera_driver/srv/reconfigure_defaults.hpp"
#include "../../../install/camera_driver/include/camera_driver/camera_driver/srv/configure_camera.hpp"
#include "../../../install/camera_driver/include/camera_driver/camera_driver/srv/reconfigure_defaults.hpp"
#include "../include/camera_driver/cameramanager/CameraManager.hpp"

class CameraDriverNode : public rclcpp::Node {

    public:
        CameraDriverNode() : Node("camera_driver_node"){

            this->declare_parameter<std::string>("camera_name", "camera");
            this->declare_parameter<std::string>("camera_frame_id", "camera_frame");
            this->declare_parameter<int>("camera_index", 1);
            this->declare_parameter<double>("publish_rate", 60); //hz
            this->declare_parameter<bool>("publish_as_gray", false);
            this->declare_parameter<bool>("publish_compressed", true);
            this->declare_parameter<bool>("publish_camera_info", false);

            this->declare_parameter<int>("image_width", -1);
            this->declare_parameter<int>("image_height", -1);
            this->declare_parameter<int>("image_fps", -1);

            camera_name = this->get_parameter("camera_name").as_string();
            camera_frame_id = this->get_parameter("camera_frame_id").as_string();
            camera_index = this->get_parameter("camera_index").as_int();
            publish_rate = this->get_parameter("publish_rate").as_double();
            publish_as_gray = this->get_parameter("publish_as_gray").as_bool();
            publish_compressed = this->get_parameter("publish_compressed").as_bool();
            publish_camera_info = this->get_parameter("publish_camera_info").as_bool();

            image_width = this->get_parameter("image_width").as_int();
            image_height = this->get_parameter("image_height").as_int();
            image_fps = this->get_parameter("image_fps").as_int();

            camera = CameraManager(camera_index, cv::CAP_V4L2);

            if(image_width != -1 && image_height != -1 && image_fps != -1)
                camera.config(image_width, image_height, image_fps);

            image_raw_publisher = this->create_publisher<sensor_msgs::msg::Image>(camera_name + "/image_raw", 10);
            image_compressed_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(camera_name + "/image_raw/compressed", 10);
            camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name + "/camera_info", 10);

            timer = this->create_wall_timer(
                std::chrono::milliseconds(int((1.0 / publish_rate) * 1000)),
                std::bind(&CameraDriverNode::publish_data, this)
            );

            config_service = this->create_service<camera_driver::srv::ConfigureCamera>(
                camera_name + "/configure_camera",
                std::bind(&CameraDriverNode::handle_config, this, std::placeholders::_1, std::placeholders::_2)
            );

            reconfig_service = this->create_service<camera_driver::srv::ReconfigureDefaults>(
                camera_name + "/reconfigure_defaults",
                std::bind(&CameraDriverNode::handle_reconfig, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
    /**
     * Sets property of camera. False means fail.
     */
        bool set_camera_property_ack(cv::VideoCaptureProperties property, double value, int repeat){
            
            bool status = camera.setPropertyAck(property, value, repeat);

            std::string message = status ? "Successfully Configured: " : "Failed to Configure: ";

            switch(property){
                case cv::CAP_PROP_FRAME_WIDTH:
                    message += "WIDTH";
                    break;

                case cv::CAP_PROP_FRAME_HEIGHT:
                    message += "HEIGHT";
                    break;

                case cv::CAP_PROP_FPS:
                    message += "FPS";
                    break;

                case cv::CAP_PROP_FOURCC:
                    message += "FOURCC";
                    break;
            }

            RCLCPP_INFO(this->get_logger(), message.c_str());

            return status;        
        }

        bool config_camera(double width, double height, double fourcc, double fps, int repeat){

            RCLCPP_INFO(this->get_logger(), "Configuring with Values: width=%d, height=%d, fourcc=%d fps=%d",
                (int)width, (int)height, (int)fourcc, (int)fps
            );

            bool all_successful = true;

            if(!set_camera_property_ack(cv::CAP_PROP_FRAME_WIDTH, width, repeat))
                all_successful = false;

            if(!set_camera_property_ack(cv::CAP_PROP_FRAME_HEIGHT, height, repeat))
                all_successful = false;
            
            if(!set_camera_property_ack(cv::CAP_PROP_FOURCC, fourcc, repeat))
                all_successful = false;

            if(!set_camera_property_ack(cv::CAP_PROP_FPS, fps, repeat))
                all_successful = false;

            RCLCPP_INFO(this->get_logger(), "Configuration Status: %s", all_successful ? "success" : "failed");

            return all_successful;
        }

        void handle_reconfig(const std::shared_ptr<camera_driver::srv::ReconfigureDefaults::Request> request, 
            std::shared_ptr<camera_driver::srv::ReconfigureDefaults::Response> response){

            response->status = config_camera(
                image_width,
                image_height,
                cv::VideoWriter::fourcc('M','J','P','G'),
                image_fps,
                5
            );
        }

        void handle_config(const std::shared_ptr<camera_driver::srv::ConfigureCamera::Request> request, 
            std::shared_ptr<camera_driver::srv::ConfigureCamera::Response> response){

            response->status = config_camera(
                request->width,
                request->height,
                cv::VideoWriter::fourcc('M','J','P','G'),
                request->fps,
                5
            );
        }

        void publish_data(){
            cv::Mat frame = publish_as_gray ? camera.getFrameGray() : camera.getFrame();

            if(frame.empty())
                return;

            std_msgs::msg::Header header = std_msgs::msg::Header();
                header.stamp = this->now();
                header.frame_id = camera_frame_id; 

            cv_bridge::CvImage message = cv_bridge::CvImage(
                header, 
                publish_as_gray ? "mono8" : "bgr8", 
                frame
            );

            image_raw_publisher->publish(*message.toImageMsg());

            if(publish_compressed)
                image_compressed_publisher->publish(*message.toCompressedImageMsg(cv_bridge::Format::JPEG));

            if(publish_camera_info){
                sensor_msgs::msg::CameraInfo info = sensor_msgs::msg::CameraInfo();

                info.header = header;
                info.width = frame.cols;
                info.height = frame.rows;

                info.distortion_model = "plumb_bob";

                camera_info_publisher->publish(info);
            }
        }

        CameraManager camera;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;
        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Service<camera_driver::srv::ConfigureCamera>::SharedPtr config_service;
        rclcpp::Service<camera_driver::srv::ReconfigureDefaults>::SharedPtr reconfig_service;


        std::string camera_name;
        std::string camera_frame_id;
        int camera_index;
        double publish_rate;
        bool publish_as_gray;
        bool publish_compressed;
        bool publish_camera_info;

        int image_width;
        int image_height;
        int image_fps;
        
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDriverNode>());
    rclcpp::shutdown();
    return 0;
}