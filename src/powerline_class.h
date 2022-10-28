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

#include "geometry.h"
#include "single_line_class.h"

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define POINT_MATCH_DISTANCE_THRESHOLD 2.

/*****************************************************************************/
// Class
/*****************************************************************************/

class Powerline
{
public:
    Powerline(rclcpp::Logger logger);

    void SetParams(float r, float q, int alive_cnt_low_thresh, int alive_cnt_high_thresh, int alive_cnt_ceiling, float matching_line_max_dist,
            std::string drone_frame_id, std::string mmwave_frame_id);

    std::vector<SingleLine> GetVisibleLines();
    quat_t GetDirection();
    plane_t GetProjectionPlane();
    //orientation_t GetPlaneOrientation();
    quat_t GetQuat();

    point_t UpdateLine(point_t point);
    void UpdateDirection(quat_t pl_direction);
    void UpdateOdometry(point_t position, quat_t quat, std::unique_ptr<tf2_ros::Buffer> &tf_buffer, float min_point_dist, float max_point_dist, float view_cone_slope);

    void CleanupLines(std::unique_ptr<tf2_ros::Buffer> &tf_buffer, float min_point_dist, float max_point_dist, float view_cone_slope);
    void ComputeInterLinePositions(std::unique_ptr<tf2_ros::Buffer> &tf_buffer, 
            float min_point_dist, float max_point_dist, float view_cone_slope, int inter_pos_window_size);

    int GetLinesCount() {

        int cnt;

        lines_mutex_.lock(); {

            cnt = lines_.size();

        } lines_mutex_.unlock();

        return cnt;

    }

private:
    struct inter_line_positions_t {

        int line_id_1;
        int line_id_2;
        std::vector<vector_t> inter_line_position_window;

    };

    bool received_pl_dir = false;
    bool received_first_odom = false;
    bool received_second_odom = false;

    std::mutex lines_mutex_;
    std::mutex direction_mutex_;
    std::mutex odometry_mutex_;
    std::mutex projection_plane_mutex_;

    std::vector<SingleLine> lines_;
    quat_t direction_;
    //float last_global_input_direction_;
    //float last_global_output_direction_;
    //float last_last_global_output_direction_;
    point_t position_;
    quat_t quat_;
    point_t last_position_;
    quat_t last_quat_;
    plane_t projection_plane_;
    //orientation_t plane_orientation_;

    int alive_cnt_low_thresh_;
    int alive_cnt_high_thresh_;
    int alive_cnt_ceiling_;

    int id_cnt_;

    float r_, q_;

    float matching_line_max_dist_;

    std::string drone_frame_id_, mmwave_frame_id_;

    rclcpp::Logger logger_;

    void updateProjectionPlane();

    int findMatchingLine(point_t point);
    point_t projectPoint(point_t point);

    void predictLines(std::unique_ptr<tf2_ros::Buffer> &tf_buffer, float min_point_dist, float max_point_dist, float view_cone_slope);

    std::vector<inter_line_positions_t> inter_line_positions_;
    
};
