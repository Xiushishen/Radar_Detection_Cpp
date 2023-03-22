// #ifndef RADAR_PCL_TO_CAM_HPP_
// #define RADAR_PCL_TO_CAM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/callback_group.hpp"
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"


class RadarToCamConverter : public rclcpp::Node
{
    public:
        RadarToCamConverter();

    private:
        bool isinside_polygon(const geometry_msgs::msg::Point32::SharedPtr point, const geometry_msgs::msg::Polygon::SharedPtr vertices);    

        bool is_on_track(const geometry_msgs::msg::Point32::SharedPtr point,
        const geometry_msgs::msg::Polygon::SharedPtr inside_polygon,
        const geometry_msgs::msg::Polygon::SharedPtr outside_polygon) const;
        void gps_vel_callback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg);
        void radar_callback(const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg);
        void timer_callback();
        void inside_polygon_calback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
        void outside_polygon_calback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
        void get_detection(const visualization_msgs::msg::Marker::SharedPtr msg);

        rclcpp::Subscription<delphi_esr_msgs::msg::EsrTrack>::SharedPtr radar_sub;
        rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr gps_vel_sub;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr front_radar_moving;
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr front_radar_moving_objs;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr inside_polygon_sub;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr outside_polygon_sub;
        rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr deted_objs_pub;
    
        rclcpp::TimerBase::SharedPtr timer;
        autoware_auto_perception_msgs::msg::DetectedObjects latest_detection;
        geometry_msgs::msg::Polygon inside_polygon;
        geometry_msgs::msg::Polygon outside_polygon;
        geometry_msgs::msg::Vector3 get_detection_velocity;
        std::vector<double> ego_vel_;
        double pre_pos_;
        double dist_2_right_;
        double dist_2_left_;
        std::vector<double> detected_vel_;

};

// #endif  // RADAR_PCL_TO_CAM_HPP_
