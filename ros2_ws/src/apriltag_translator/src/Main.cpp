#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

class ApriltagTranslatorNode : public rclcpp::Node {

    public:
        ApriltagTranslatorNode() : Node("camera_driver_node"){

        }

    private:
    
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagTranslatorNode>());
    rclcpp::shutdown();
    return 0;
}