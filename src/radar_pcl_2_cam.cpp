#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/timer.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/callback_group.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "builtin_interfaces/msg/duration.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class RadarToCamConverter : public rclcpp::Node
{
public:
    RadarToCamConverter() : Node("radar_to_cam_node")
    {
        # ---------------------- Initialize Required Attributes ---------------------- #      
        this->declare_parameter("ego_vel", 0.0);
        this->declare_parameter("pre_pos", 0.0);
        this->declare_parameter("dist_2_right", std::numeric_limits<double>::infinity());
        this->declare_parameter("dist_2_left", std::numeric_limits<double>::infinity());
        this->declare_parameter("detected_vel", 0.0);
        # -------------------------- Get Required Attributes ------------------------- #
        ego_vel_ = this->get_parameter("ego_vel").as_double();
        pre_pos_ = this->get_parameter("ego_vel").as_double();
        dist_2_right_ = this->get_parameter("ego_vel").as_double();
        dist_2_left_ = this->get_parameter("ego_vel").as_double();
        detected_vel_ = this->get_parameter("ego_vel").as_double();

        # -------------------------------- QOS Profile ------------------------------- #
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile.depth(1);

        rclcpp::QoS qos_profile2(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile2.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile2.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile2.depth(1);

        # --------------------------- Sensor Subscriptions --------------------------- #
        # ----------------------------------- RADAR ---------------------------------- #

        radar_sub = this->create_subscription<delphi_esr_msgs::msg::EsrTrack>(
                    "/radar_front_est_input", qos_profile,
                    std::bind(&RadarToCamConverter::radar_callback, this, _1));
        static_cast<void>(radar_sub);
        # ---------------------------------- GPS VEL --------------------------------- #
        gps_vel_sub = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
                    "/novatel_top_input", qos_profile,
                    std::bind(&RadarToCamConverter::gps_vel_callback, this, _1));
        static_cast<void>(gps_vel_sub);
        # ---------------------------------- GPS VEL --------------------------------- #
        gps_pos_sub = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
                    "/novatel_pos_input", qos_profile,
                    std::bind(&RadarToCamConverter::gps_pos_callback, this, _1));
        static_cast<void>(gps_pos_sub);
        # ------------------------- moving objects from radar ------------------------ #
        front_radar_moving = this->create_subscription<visualization_msgs::msg::Marker>(
                    "/radar_front/radar_visz_static", qos_profile,   
                    std::bind(&RadarToCamConverter::get_detection, this, _1));
        # ------------------------- Sensor Subscriptions END ------------------------- #


        # ----------------------------- Marker Publisher ----------------------------- #
        deted_objs_pub = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("/output", qos_profile2);
        static_cast<void>(deted_objs_pub);
        # ----------------------------- Timer to Publish ----------------------------- #
        timer = this->create_wall_timer(std::chrono::milliseconds(200),
        std::bind(&RadarToCamConverter::timer_callback, this));

    }
private:
    void get_detection(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        double y = msg->pose.position.y;
        double x = msg->pose.position.x;
        bool publish = false;
        if ((x <= 200) && (ego_vel_ != nullptr) && (ego_vel_[0] > 5))  // MAX_DET_RANGE and STATIC_VEL_THRESHOLD 
        {
            if (y <= 0)
            {
                if ((-y) < (dist_2_right_ - 2.0))
                {
                    publish = true;
                }
            }
            else
            {
                if (y < (dist_2_left_ - (-2.0)))
                {
                    publish = true;
                }
            }
        }
        if (publish)
        {
            autoware_auto_perception_msgs::msg::DetectedObjects detectedobjs_msg;
            autoware_auto_percpetion_msgs::msg::DetectedObject object_msg;

            detectedobjs_msg->header.frame_id = "radar_front";
            detectedobjs_msg->header.stamp = msg->header.stamp;

            object_msg->existence_probability = 0.0;
            object_msg->classification.push_back(autoware_auto_percpetion_msgs::msg::ObjectClassification{1, 1.0});
            object_msg->kinematics.pose_with_covariance.pose.position.x = x;
            object_msg->kinematics.pose_with_covariance.pose.position.y = y;
            object_msg->kinematics.pose_with_covariance.pose.position.z = 0.0;
            object_msg->kinematics.pose_with_covariance.pose.orientation.w = 1.0;
            if (ego_vel_ != nullptr && detected_vel_ != nullptr)
            {
                object_msg->kinematics.twist_with_convariance.twist.linear.x = ego_vel_[0] + detected_vel_[0];
                object_msg->kinematics.twist_with_convariance.twist.linear.y = ego_vel_[1] + detected_vel_[0];
                object_msg->kinematics.twist_with_convariance.twist.linear.z = 0.0;
            }
            object_msg->shape.dimensions.x = 2.9;
            object_msg->shape.dimensions.y = 1.6;
            object_msg->shape.dimensions.z = 1.0;
            detectedobjs_msg.objects.push_back(object_msg);
            latest_detection = detectedobjs_msg;
        }
    }

    void get_dist(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        dist_2_right_ = msg->pose.position.x;
        dist_2_left_ = 17.6784 - dist_2_right_; // MAX_DIST_TO_WALL
    }

    void gps_vel_callback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg){
        ego_vel_[0] = msg->hor_speed;
        ego_vel_[1] = msg->ver_speed;
        
    }

    void gps_pos_callback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
    {
        this->ego_pos = Eigen::Vector2d(msg->lat, msg->lon);
        RCLCPP_INFO(this->get_logger(), "[%f, %f]", ego_pos[0], ego_pos[1])
    }

    void radar_callback(const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg){
        return;
    }
    void timer_callback()
    {
        if (latest_detection != nullptr)
        {
            deted_objs_pub->publish(latest_detection);
            RCLCPP_INFO(this->get_logger(), "%f", latest_detection->objects[0]->kinematics.pose_with_covariance.pose.position);
            latest_detection = nullptr;
        }
        else
        {
            
        }
    }

    rclcpp::Subscription<delphi_esr_msgs::msg::EsrTrack>::SharedPtr radar_sub;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr gps_vel_sub;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr gps_pos_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr front_radar_moving;
    rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr deted_objs_pub;
    rclcpp::TimerBase::SharedPtr timer;
    autoware_auto_perception_msgs::msg::DetectedObjects latest_detection;
    Eigen::Vector2d ego_vel_;
    double pre_pos_;
    double dist_2_right_;
    double dist_2_left_;
    double detected_vel_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarToCamConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}