#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iii_drone_core/utils/types.hpp"
#include "iii_drone_core/utils/math.hpp"

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    class SingleLine {
    public:
        SingleLine(
            int id, 
            iii_drone::types::point_t initial_point, 
            float r, 
            float q, 
            rclcpp::Logger logger, 
            int alive_cnt_low_thresh, 
            int alive_cnt_high_thresh, 
            int alive_cnt_ceiling,
            std::string drone_frame_id, 
            std::string mmwave_frame_id, 
            bool simulation
        );

        SingleLine GetCopy();
        iii_drone::types::point_t GetPoint();
        void SetPoint(iii_drone::types::point_t point);

        bool IsAlive(
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope
        );
        bool IsVisible();
        bool IsInFOV(
            iii_drone::types::point_t point,
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope
        );
        bool IsInFOV(
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope
        );

        void Update(iii_drone::types::point_t point);
        void Predict(
            iii_drone::types::vector_t delta_position, 
            iii_drone::types::quat_t delta_quat,
            iii_drone::types::plane_t projection_plane, 
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer
        );

        int GetId();

    private:
        typedef struct {

            float state_est;
            float var_est;

        } kf_est_t;

        bool simulation_;

        iii_drone::types::point_t pl_point_;
        kf_est_t estimates[3];

        iii_drone::types::point_t projected_point_;

        std::string drone_frame_id_, mmwave_frame_id_;

        int id_;

        int alive_cnt_;
        int alive_cnt_low_thresh_;
        int alive_cnt_high_thresh_;
        int alive_cnt_ceiling_;

        float r_;
        float q_;

        rclcpp::Logger logger_;
        
    };

} // namespace perception
} // namespace iii_drone