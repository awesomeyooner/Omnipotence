#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <apriltag_msgs/msg/apriltag_array_stamped.hpp>
#include <apriltag_msgs/msg/point.hpp>

using std::placeholders::_1;

class ApriltagTranslatorNode : public rclcpp::Node {

    public:
        ApriltagTranslatorNode() : Node("apriltag_translator_node"){

            this->declare_parameter<std::string>("input_topic", "detections");
            this->declare_parameter<std::string>("output_topic", "detections/translate");

            std::string input_topic = this->get_parameter("input_topic").as_string();
            std::string output_topic = this->get_parameter("output_topic").as_string();

            apriltag_publisher = this->create_publisher<apriltag_msgs::msg::ApriltagArrayStamped>(output_topic, 10);

            apriltag_subscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                input_topic, 10, std::bind(&ApriltagTranslatorNode::on_message, this, _1));
        }

        void on_message(const std::shared_ptr<apriltag_msgs::msg::AprilTagDetectionArray> message) const{
            
            apriltag_msgs::msg::ApriltagArrayStamped output = apriltag_msgs::msg::ApriltagArrayStamped();

            output.header = message->header;
        
            output.apriltags = apriltag_detection_to_apriltag_array(message->detections);

            apriltag_publisher->publish(output);
        }

        std::vector<apriltag_msgs::msg::Apriltag> apriltag_detection_to_apriltag_array(const std::vector<apriltag_msgs::msg::AprilTagDetection> &array) const{
            std::vector<apriltag_msgs::msg::Apriltag> output;

            for(const apriltag_msgs::msg::AprilTagDetection &detection : array){
                output.push_back(apriltag_detection_to_apriltag(detection));
            }

            return output;
        }

        apriltag_msgs::msg::Apriltag apriltag_detection_to_apriltag(const apriltag_msgs::msg::AprilTagDetection &message) const{
            apriltag_msgs::msg::Apriltag output = apriltag_msgs::msg::Apriltag();

            output.id = message.id;
            output.family = message.family;
            output.hamming = message.hamming;
            output.border = 1; //im too lazy to automate this with parameters
            output.bits = 36; //TODO: automate this with parameters
            output.center = ap_to_geo_point(message.centre);
            output.corners = ap_to_geo_point_array(message.corners);

            return output;
        }

        std::array<geometry_msgs::msg::Point, 4> ap_to_geo_point_array(const std::array<apriltag_msgs::msg::Point, 4> &array) const{
            return {
                ap_to_geo_point(array[0]),
                ap_to_geo_point(array[1]),
                ap_to_geo_point(array[2]),
                ap_to_geo_point(array[3])
            };
        }

        geometry_msgs::msg::Point ap_to_geo_point(const apriltag_msgs::msg::Point &message) const{
            geometry_msgs::msg::Point output = geometry_msgs::msg::Point();

            output.x = message.x;
            output.y = message.y;
            output.z = 0;
            
            return output;
        }

    private:

        rclcpp::Publisher<apriltag_msgs::msg::ApriltagArrayStamped>::SharedPtr apriltag_publisher;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber;
    
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagTranslatorNode>());
    rclcpp::shutdown();
    return 0;
}