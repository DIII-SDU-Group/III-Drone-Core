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
#include <math.h>
#include <limits>
#include <eigen3/Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_core/utils/types.hpp"
#include "iii_drone_core/perception/single_line.hpp"

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    class Powerline {
    public:
        Powerline(
            rclcpp::Logger logger, 
            bool simulation
        );

        void SetParams(
            float r, 
            float q, 
            int alive_cnt_low_thresh, 
            int alive_cnt_high_thresh, 
            int alive_cnt_ceiling, 
            float matching_line_max_dist,
            std::string drone_frame_id, 
            std::string mmwave_frame_id, 
            int max_lines
        );

        std::vector<SingleLine> GetVisibleLines();
        iii_drone::types::quat_t GetDirection();
        iii_drone::types::plane_t GetProjectionPlane();
        iii_drone::types::quat_t GetQuat();

        iii_drone::types::point_t UpdateLine(iii_drone::types::point_t point);
        void UpdateDirection(iii_drone::types::quat_t pl_direction);
        void UpdateOdometry(
            iii_drone::types::point_t position, 
            iii_drone::types::quat_t quat, 
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope
        );

        void CleanupLines(
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope
        );
        void ComputeInterLinePositions(
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope, 
            int inter_pos_window_size
        );

        int GetLinesCount();

        void SetSimulation(bool simulation);

    private:
        bool simulation_;

        struct inter_line_positions_t {

            int line_id_1;
            int line_id_2;
            std::vector<iii_drone::types::vector_t> inter_line_position_window;

        };

        bool received_pl_dir = false;
        bool received_first_odom = false;
        bool received_second_odom = false;

        std::mutex lines_mutex_;
        std::mutex direction_mutex_;
        std::mutex odometry_mutex_;
        std::mutex projection_plane_mutex_;

        std::vector<SingleLine> lines_;
        iii_drone::types::quat_t direction_;
        iii_drone::types::point_t position_;
        iii_drone::types::quat_t quat_;
        iii_drone::types::point_t last_position_;
        iii_drone::types::quat_t last_quat_;
        iii_drone::types::plane_t projection_plane_;

        int max_lines_;

        int alive_cnt_low_thresh_;
        int alive_cnt_high_thresh_;
        int alive_cnt_ceiling_;

        int id_cnt_;

        float r_, q_;

        float matching_line_max_dist_;

        std::string drone_frame_id_, mmwave_frame_id_;

        rclcpp::Logger logger_;

        void updateProjectionPlane();

        int findMatchingLine(iii_drone::types::point_t point);
        iii_drone::types::point_t projectPoint(iii_drone::types::point_t point);

        void predictLines(
            std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, 
            float max_point_dist, 
            float view_cone_slope
        );

        std::vector<inter_line_positions_t> inter_line_positions_;
        
    };

} // namespace perception
} // namespace iii_drone