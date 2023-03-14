#include "radar_detection.hpp"
#include <cmath>
#include <iostream>
#include "delphi_esr_msgs/msg/esr_track.hpp"
class RadarDetection
{
    public:
        RadarDetection::RadarDetection(const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg)
        {
        // --------------------------- Administrative Stuff --------------------------- //
            stamp_ = msg->header.stamp;
            id = msg->track_id;
            group_changed = msg->track_group_changed;
            status = msg->track_status;
            is_bridge = msg->track_bridge_object;
            rolling_count = msg->track_rolling_count;
            width = msg->track_width;
            det_mode = msg->track_med_range_mode;
        // --------------------------- Position Information --------------------------- //
            angle = msg->track_angle;
            range = msg->track_range;
            cos_angle = std::cos(M_PI * angle / 180);
            sin_angle = std::sin(M_PI * angle / 180);
        // --------------------------- Kinematic Information -------------------------- //
            range_vel = msg->track_range_rate;
            range_acc = msg->track_range_accel;
            lateral_vel = msg->track_lat_rate - 8; // Adjust as per radar documentation   
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
            valid = valid && (!is_static);
            return valid;
        }

        bool get_static(double ego_speed_)
        {
          if (rel_speed != 0)
          {
            get_speed();
          }
          ego_speed = ego_speed_;
          double speed_diff = std::abs(rel_speed - ego_speed); 
          is_static = (speed_diff < 5) || (ego_speed < 5);
          return is_static;
        }

        double get_tangential_vel()
        {
          trangential_vel = (lateral_vel - range_vel * sin_angle) / (cos_angle + epsilon);
        }

        std::vector<double> get_cartesian_pos()
        {
          cartesian_pos[0] = range * cos_angle;
          cartesian_pos[1] = range * sin_angle;
          cartesian_pos[2] = 0.0;
          return cartesian_pos;
        }

        std::vector<double> get_cartesian_vel()
        {
          if (trangential_vel == 0)
          {
            get_tangential_vel();
          }
          cartesian_vel[0] = range_vel * cos_angle - trangential_vel * sin_angle;
          cartesian_vel[1] = lateral_vel;

          return cartesian_vel;
        }

        double get_speed()
        {
          if (!cartesian_vel.empty())
          {
            get_cartesian_vel();
          }
          rel_speed = std::sqrt(std::pow(cartesian_vel[0], 2) + std::pow(cartesian_vel[1], 2));
          return rel_speed;
        }

        std::string toString() {
            std::string print_string = "";
            print_string += "\n=======================================================\n";
            print_string += "ID: " + std::to_string(id) + "\n";
            print_string += "=======================================================\n";
            print_string += "Msg Time: " + std::to_string(stamp_.seconds()) + "\n";
            print_string += "Radar R: " + std::to_string(range) + ", Radar Theta: " + std::to_string(angle) + "\n";
            print_string += "Radar X: " + std::to_string(cartesian_pos[0]) + ", Radar Y: " + std::to_string(cartesian_pos[1]) + "\n";
            print_string += "Detection Velocity in Radial Frame: Vr: " + std::to_string(range_vel) + " Vl: " + std::to_string(lateral_vel) + "\n";
            print_string += "Detection Velocity in Ego Frame: Vx: " + std::to_string(cartesian_vel[0]) + " Vy: " + std::to_string(cartesian_vel[1]) + "\n";
            print_string += "Detection Speed in Ego Frame: " + std::to_string(rel_speed) + "\n";
            print_string += "Ego Ground Speed: " + std::to_string(ego_speed) + "\n";
            return print_string;
        }

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
        // -------------------------- Stuff to be calculated -------------------------- //        
        double trangential_vel;
        std::vector<double> cartesian_vel;
        std::vector<double> cartesian_pos;
        double ego_speed;
        bool valid;

};