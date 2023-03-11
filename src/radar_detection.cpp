#include "radar_detection_cpp/radar_detection.hpp"
#include <cmath>
#include <iostream>

class RadarDetection
{

    public:
        RadarDetection::RadarDetection(const delphi_msgs::msg::ESRTrack::SharedPtr msg)
        {
            stamp = msg->header.stamp;
            id = msg->track_id;
            group_changed msg->track_group_changed;
            status = msg.track_status;
            is_bridge = msg.track_bridge_object;
            rolling_count = msg.track_rolling_count;
            width = msg.track_width;
            det_mode = msg.track_med_range_mode;
        }

        RadarDetection::~RadarDetection()
        {

        }

    private:

        bool get_vaild_preproc()
        {
            valid = valid && (status == 3 || status == 4);
            valid = valid && (range < 200) && (range > 0);
            valid = valid && (lateral_vel < 7.75) && (lateral_vel > -8);
            return valid;
        }


        bool get_valid_postproc()
        {
            return;
        }


        std::string toString() {
            std::string print_string = "";
            print_string += "\n=======================================================\n";
            print_string += "ID: " + std::to_string(id) + "\n";
            print_string += "=======================================================\n";
            print_string += "Msg Time: " + std::to_string(stamp) + "\n";
            print_string += "Radar R: " + std::to_string(range) + ", Radar Theta: " + std::to_string(angle) + "\n";
            print_string += "Radar X: " + std::to_string(cartesian_pos[0]) + ", Radar Y: " + std::to_string(cartesian_pos[1]) + "\n";
            print_string += "Detection Velocity in Radial Frame: Vr: " + std::to_string(range_vel) + " Vl: " + std::to_string(lateral_vel) + "\n";
            print_string += "Detection Velocity in Ego Frame: Vx: " + std::to_string(cartesian_vel[0]) + " Vy: " + std::to_string(cartesian_vel[1]) + "\n";
            print_string += "Detection Speed in Ego Frame: " + std::to_string(rel_speed) + "\n";
            print_string += "Ego Ground Speed: " + std::to_string(ego_speed) + "\n";
            return print_string;
        }

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