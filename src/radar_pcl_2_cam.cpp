#include "radar_pcl_to_cam.hpp"
#include "radar_detection.hpp"
 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "builtin_interfaces/msg/duration.hpp"
#include <tuple>
using std::placeholders::_1;
using std::placeholders::_2;

class RadarToCamConverter : public rclcpp::Node
{
public:
    RadarToCamConverter() : Node("radar_to_cam_node")
    {
        // ---------------------- Initialize Required Attributes ---------------------- //      
        this->declare_parameter("ego_vel", 0.0);
        this->declare_parameter("pre_pos", 0.0);
        this->declare_parameter("dist_2_right", std::numeric_limits<double>::infinity());
        this->declare_parameter("dist_2_left", std::numeric_limits<double>::infinity());
        this->declare_parameter("detected_vel", 0.0);
        // -------------------------- Get Required Attributes ------------------------- //
        ego_vel_ = this->get_parameter("ego_vel").as_double_array();
        pre_pos_ = this->get_parameter("ego_vel").as_double();
        dist_2_right_ = this->get_parameter("ego_vel").as_double();
        dist_2_left_ = this->get_parameter("ego_vel").as_double();
        detected_vel_ = this->get_parameter("ego_vel").as_double_array();

        // -------------------------------- QOS Profile ------------------------------- //
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile.depth();

        rclcpp::QoS qos_profile2(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile2.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile2.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_profile2.depth();

        // --------------------------- Sensor Subscriptions --------------------------- //
        // ----------------------------------- RADAR ---------------------------------- //

        this->radar_sub = this->create_subscription<delphi_esr_msgs::msg::EsrTrack>(
                    "/radar_front_esr_input", qos_profile,
                    std::bind(&RadarToCamConverter::radar_callback, this, _1));
        static_cast<void>(radar_sub);

        this->front_radar_moving_objs = this->create_subscription<visualization_msgs::msg::Marker>(
                    "/radar_front/radar_visz_static", qos_profile,
                    std::bind(&RadarToCamConverter::get_detection, this, _1));
        static_cast<void>(front_radar_moving_objs);
        // ----------------------------- GPS VEL and POS------------------------------ //
        gps_vel_sub = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
                    "/novatel_top_input", qos_profile,
                    std::bind(&RadarToCamConverter::gps_vel_callback, this, _1));
        static_cast<void>(gps_vel_sub);
        // ------------------------- Sensor Subscriptions END ------------------------- //

        // ---------------------- Subscribing Polygon for Track ----------------------- //
        inside_polygon_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
                    "/inside_polygon", qos_profile,
                    std::bind(&RadarToCamConverter::inside_polygon_calback, this, _1));

        outside_polygon_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
                    "/outside_polygon", qos_profile,
                    std::bind(&RadarToCamConverter::outside_polygon_calback, this, _1));  

        // ----------------------------- Marker Publisher ----------------------------- //
        deted_objs_pub = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>("/output", qos_profile2);
        static_cast<void>(deted_objs_pub);
        // ----------------------------- Timer to Publish ----------------------------- //
        RCLCPP_INFO(this->get_logger(), "Timer Started");
        timer = this->create_wall_timer(std::chrono::milliseconds(200),
        std::bind(&RadarToCamConverter::timer_callback, this));
    }
private:
    bool isinside_polygon(std::vector<double> point, geometry_msgs::msg::Polygon vertices)
    const {
      std::complex<double> sum_(0, 0);
      for (int i = 1; i < vertices.points.size(); i++)
      {
        const auto& v0 = std::complex<double>(vertices.points[i - 1].x, vertices.points[i - 1].y);
        const auto& v1 = std::complex<double>(vertices.points[i % vertices.points.size()].x, vertices.points[i % vertices.points.size()].y);
        sum_ += std::log((v1 - std::complex<double>(point[0], point[1])) / (v0 - std::complex<double>(point[0], point[1])));
      }
      return std::abs(sum_) > 1;
    }

    bool is_on_track(
      std::vector<double> point,
      geometry_msgs::msg::Polygon inside_polygon,
      geometry_msgs::msg::Polygon outside_polygon
      ) const {
        if (point.empty()) 
        {
          return false;
        }
        if (isinside_polygon(point, inside_polygon))
        {
          return false;
        }
        if (!isinside_polygon(point, outside_polygon))
        {
          return true;
        }
        return false;
      } 
    void gps_vel_callback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg){
        ego_vel_[0] = msg->hor_speed;
        ego_vel_[1] = msg->ver_speed;
    }

    void radar_callback(const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg)
    {
      RadarDetection detection(msg);
      if (detection.get_vaild_preproc() && !ego_vel_.empty())
      {
        // -------------------------- Get Cartesian Position -------------------------- //
        std::vector<double> cartesian_pos = detection.get_cartesian_pos();
        // ------------------------------ Get Kinematics ------------------------------ //
        double det_speed_wrt_ego = detection.get_speed();

        // -------------------------- Check if static object -------------------------- //
        bool is_static = detection.get_static(std::abs(ego_vel_[0]));
        if (detection.get_valid_postproc())
        {
        // ---------------------- Prepare and publish marker msg ---------------------- //
        // RCLCPP_INF(this->get_logger(), "checking polygon");
          std::vector<double> cartesian_pos_new {cartesian_pos[0], cartesian_pos[1]};
          if (!is_on_track(cartesian_pos_new, inside_polygon, outside_polygon))
          {
            return;
          }
          autoware_auto_perception_msgs::msg::DetectedObjects detectedobjs_msg;
          autoware_auto_perception_msgs::msg::DetectedObject object_msg;
          detectedobjs_msg.header.frame_id = "radar_front";
          detectedobjs_msg.header.stamp = msg->header.stamp;

          object_msg.existence_probability = 0.0;
          autoware_auto_perception_msgs::msg::ObjectClassification object_classification;
          object_classification.label = 1;
          object_classification.probability = 1.0;
          object_msg.classification.push_back(object_classification);
          object_msg.kinematics.pose_with_covariance.pose.position.x = cartesian_pos[0];
          object_msg.kinematics.pose_with_covariance.pose.position.y = cartesian_pos[1];
          object_msg.kinematics.pose_with_covariance.pose.position.z = 0.0;
          object_msg.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

          detected_vel_ = detection.get_cartesian_vel();

          object_msg.kinematics.twist_with_covariance.twist.linear.x = ego_vel_[0] + detected_vel_[0];
          object_msg.kinematics.twist_with_covariance.twist.linear.y = ego_vel_[1] + detected_vel_[0];
          object_msg.kinematics.twist_with_covariance.twist.linear.z = 0.0;

          object_msg.shape.dimensions.x = 2.9;
          object_msg.shape.dimensions.y = 1.6;
          object_msg.shape.dimensions.z = 1.0;
          RCLCPP_INFO(this->get_logger(), "Detection: %s", detection.toString().c_str());
          detectedobjs_msg.objects.push_back(object_msg);
          latest_detection = detectedobjs_msg;
        }
      } 
    }
    
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "timer callback"); 
        if (latest_detection.objects.size() != 0)
        {
            deted_objs_pub->publish(latest_detection);
            RCLCPP_INFO(this->get_logger(), "%f", latest_detection.objects[0].kinematics.pose_with_covariance.pose.position);
            latest_detection.objects.clear();
        }
        else
        {
            autoware_auto_perception_msgs::msg::DetectedObjects detectedobjs_msg;
            detectedobjs_msg.header.frame_id = "radar_front";
            detectedobjs_msg.header.stamp = this->now();
            RCLCPP_INFO(this->get_logger(), "no detection");
            deted_objs_pub->publish(detectedobjs_msg);
        }
    }

    void inside_polygon_calback(geometry_msgs::msg::PolygonStamped msg)
    {
        inside_polygon = msg.polygon;
    }

    void outside_polygon_calback(geometry_msgs::msg::PolygonStamped msg)
    {
        outside_polygon = msg.polygon;
    }
    void get_detection(const visualization_msgs::msg::Marker::SharedPtr msg){
        double y = msg->pose.position.y;
        double x = msg->pose.position.x;
        bool publish = false;
        if ((x <= 200) && (!ego_vel_.size()) && (ego_vel_[0] > 5) && inside_polygon.points.size() != 0 && outside_polygon.points.size() != 0)  // MAX_DET_RANGE and STATIC_VEL_THRESHOLD 
        {
          if (is_on_track({x, y}, inside_polygon, outside_polygon))
          {
            publish = true;
          }
          if (publish)
          {
              autoware_auto_perception_msgs::msg::DetectedObjects detectedobjs_msg;
              autoware_auto_perception_msgs::msg::DetectedObject object_msg;

              detectedobjs_msg.header.frame_id = "radar_front";
              detectedobjs_msg.header.stamp = msg->header.stamp;

              object_msg.existence_probability = 0.0;
              autoware_auto_perception_msgs::msg::ObjectClassification object_classification;
              object_classification.label = 1;
              object_classification.probability = 1.0;
              object_msg.classification.push_back(object_classification);
              object_msg.kinematics.pose_with_covariance.pose.position.x = x;
              object_msg.kinematics.pose_with_covariance.pose.position.y = y;
              object_msg.kinematics.pose_with_covariance.pose.position.z = 0.0;
              object_msg.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
              if (!ego_vel_.empty() && !detected_vel_.empty())
              {
                get_detection_velocity.x = ego_vel_[0] + detected_vel_[1];
                get_detection_velocity.y = ego_vel_[1] + detected_vel_[0];
                get_detection_velocity.z = 0.0;
                object_msg.kinematics.twist_with_covariance.twist.linear = get_detection_velocity;

              }
              object_msg.shape.dimensions.x = 2.9;
              object_msg.shape.dimensions.y = 1.6;
              object_msg.shape.dimensions.z = 1.0;
              detectedobjs_msg.objects.push_back(object_msg);
              // latest_detection = detectedobjs_msg;
          }
        }
    }
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
    geometry_msgs::msg::Point32 point_;
    std::vector<double> point_ {point_.x, point_.y};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarToCamConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}