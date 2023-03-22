#ifndef RADAR_DETECTION_HPP_
#define RADAR_DETECTION_HPP_

#include <iostream>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"

class RadarDetection
{
    public:
        RadarDetection::RadarDetection(const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg);

        RadarDetection::~RadarDetection()=default;

        bool get_vaild_preproc();

        bool get_valid_postproc();

        bool get_static(double ego_speed_);

        double get_tangential_vel();

        std::vector<double> get_cartesian_pos();

        std::vector<double> get_cartesian_vel();

        double get_speed();

        std::string toString();

    private:
        rclcpp::Time stamp_;
        int id;
        bool group_changed;
        int status;
        bool is_bridge;
        int rolling_count;
        int width;
        int det_mode;
        double epsilon = 1e-10;
        double angle;
        double range;
        double cos_angle;
        double sin_angle;
        double range_vel;
        double range_acc;
        double lateral_vel;
        bool valid = true;
        double rel_speed;
        bool is_static;
};
#endif // RADAR_DETECTION_HPP_