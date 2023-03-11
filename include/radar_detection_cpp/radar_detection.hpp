#ifndef RADAR_DETECTION_CPP__RADAR_DETECTION_HPP
#define RADAR_DETECTION_CPP__RADAR_DETECTION_HPP

#include <iostream>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"

class RadarDetection
{
    public:
        RadarDetection::RadarDetection(const delphi_msgs::msg::ESRTrack::SharedPtr msg);

        RadarDetection::~RadarDetection();

    private:

        rclcpp::Time stamp;
        int id;
        bool group_changed;
        int status;
        bool is_bridge;
        int rolling_count;
        int width;
        int det_mode;
        double spsilon = 1e-10;
        double angle;
        double range;
        double cos_angle;
        double sin_angle;
        double range_vel;
        double range_acc;
        double lateral_vel;


        bool valid = true;


};



#endif // RADAR_DETECTION_CPP__RADAR_DETECTION_HPP