#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <mutex>
#include <shared_mutex>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class for storing the parameters of the powerline object.
     */
    class PowerlineParameters {
    public:
        /**
         * @brief Constructor.
         * 
         * @param kf_r The Kalman Filter r parameter.
         * @param kf_q The Kalman Filter q parameter.
         * @param alive_cnt_low_thresh The low threshold for the alive counter.
         * @param alive_cnt_high_thresh The high threshold for the alive counter.
         * @param alive_cnt_ceiling The ceiling for the alive counter.
         * @param matching_line_max_dist The maximum distance between a line and a point for a match.
         * @param max_lines The maximum number of lines to be stored.
         * @param min_point_dist The minimum distance to a point for a line to be considered.
         * @param max_point_dist The maximum distance to a point for a line to be considered.
         * @param view_cone_slope The slope of the view cone for FOV calculation.
         * @param strict_min_point_dist The strict minimum distance to a point for a line to be considered.
         * @param strict_max_point_dist The strict maximum distance to a point for a line to be considered.
         * @param strict_view_cone_slope The strict slope of the view cone for FOV calculation.
         * @param inter_pos_window_size The size of the window for computing the inter line positions.
         * @param drone_frame_id The frame id of the drone.
         * @param mmwave_frame_id The frame id of the mmWave sensor.
         * @param simulation Whether in simulation or not.
        */
        PowerlineParameters(
            const float kf_r,
            const float kf_q,
            const int alive_cnt_low_thresh,
            const int alive_cnt_high_thresh,
            const int alive_cnt_ceiling,
            const float matching_line_max_dist,
            const int max_lines,
            const float min_point_dist,
            const float max_point_dist,
            const float view_cone_slope,
            const float strict_min_point_dist,
            const float strict_max_point_dist,
            const float strict_view_cone_slope,
            const int inter_pos_window_size,
            const std::string drone_frame_id,
            const std::string mmwave_frame_id,
            const bool simulation
        );

        /**
         * @brief Kalman Filter r parameter getter
        */
        const float & kf_r() const;

        /**
         * @brief Kalman Filter r parameter setter
        */
        float & kf_r();

        /**
         * @brief Kalman Filter q parameter getter
        */
        const float & kf_q() const;

        /**
         * @brief Kalman Filter q parameter setter
        */
        float & kf_q();

        /**
         * @brief Low threshold for the alive counter getter.
        */
        const int & alive_cnt_low_thresh() const;

        /**
         * @brief Low threshold for the alive counter setter.
        */
        int & alive_cnt_low_thresh();

        /**
         * @brief High threshold for the alive counter getter.
        */
        const int & alive_cnt_high_thresh() const;

        /**
         * @brief High threshold for the alive counter setter.
        */
        int & alive_cnt_high_thresh();

        /**
         * @brief Ceiling for the alive counter getter.
        */
        const int & alive_cnt_ceiling() const;

        /**
         * @brief Ceiling for the alive counter setter.
        */
        int & alive_cnt_ceiling();

        /**
         * @brief Maximum distance between a line and a point for a match getter.
        */
        const float & matching_line_max_dist() const;

        /**
         * @brief Maximum distance between a line and a point for a match setter.
        */
        float & matching_line_max_dist();

        /**
         * @brief Maximum number of lines to be stored getter.
        */
        const int & max_lines() const;

        /**
         * @brief Maximum number of lines to be stored setter.
        */
        int & max_lines();

        /**
         * @brief Minimum distance to a point for a line to be considered getter.
        */
        const float & min_point_dist() const;

        /**
         * @brief Minimum distance to a point for a line to be considered setter.
        */
        float & min_point_dist();

        /**
         * @brief Maximum distance to a point for a line to be considered getter.
        */
        const float & max_point_dist() const;

        /**
         * @brief Maximum distance to a point for a line to be considered setter.
        */
        float & max_point_dist();

        /**
         * @brief Slope of the view cone for FOV calculation getter.
        */
        const float & view_cone_slope() const;

        /**
         * @brief Slope of the view cone for FOV calculation setter.
        */
        float & view_cone_slope();

        /**
         * @brief Strict minimum distance to a point for a line to be considered getter.
        */
        const float & strict_min_point_dist() const;

        /**
         * @brief Strict minimum distance to a point for a line to be considered setter.
        */
        float & strict_min_point_dist();

        /**
         * @brief Strict maximum distance to a point for a line to be considered getter.
        */
        const float & strict_max_point_dist() const;

        /**
         * @brief Strict maximum distance to a point for a line to be considered setter.
        */
        float & strict_max_point_dist();

        /**
         * @brief Strict slope of the view cone for FOV calculation getter.
        */
        const float & strict_view_cone_slope() const;

        /**
         * @brief Strict slope of the view cone for FOV calculation setter.
        */
        float & strict_view_cone_slope();

        /**
         * @brief Size of the window for computing the inter line positions getter.
        */
        const int & inter_pos_window_size() const;

        /**
         * @brief Size of the window for computing the inter line positions setter.
        */
        int & inter_pos_window_size();

        /**
         * @brief Frame id of the drone getter.
        */
        const std::string & drone_frame_id() const;

        /**
         * @brief Frame id of the mmWave sensor getter.
        */
        const std::string & mmwave_frame_id() const;

        /**
         * @brief Whether in simulation or not getter.
        */
        const bool & simulation() const;

    private:
        /**
         * @brief Parameter mutex.
        */
        mutable std::shared_mutex parameter_mutex_;

        /**
         * @brief Kalman Filter r parameter.
        */
        float kf_r_;

        /**
         * @brief Kalman Filter q parameter.
        */
        float kf_q_;

        /**
         * @brief Low threshold for the alive counter.
        */
        int alive_cnt_low_thresh_;

        /**
         * @brief High threshold for the alive counter.
        */
        int alive_cnt_high_thresh_;

        /**
         * @brief Ceiling for the alive counter.
        */
        int alive_cnt_ceiling_;

        /**
         * @brief Maximum distance between a line and a point for a match.
        */
        float matching_line_max_dist_;

        /**
         * @brief Maximum number of lines to be stored.
        */
        int max_lines_;

        /**
         * @brief Minimum distance to a point for a line to be considered.
        */
        float min_point_dist_;

        /**
         * @brief Maximum distance to a point for a line to be considered.
        */
        float max_point_dist_;

        /**
         * @brief Slope of the view cone for FOV calculation.
        */
        float view_cone_slope_;

        /**
         * @brief Strict minimum distance to a point for a line to be considered.
        */
        float strict_min_point_dist_;

        /**
         * @brief Strict maximum distance to a point for a line to be considered.
        */
        float strict_max_point_dist_;

        /**
         * @brief Strict slope of the view cone for FOV calculation.
        */
        float strict_view_cone_slope_;

        /**
         * @brief Size of the window for computing the inter line positions.
        */
        int inter_pos_window_size_;

        /**
         * @brief Frame id of the drone.
        */
        std::string drone_frame_id_;

        /**
         * @brief Frame id of the mmWave sensor.
        */
        std::string mmwave_frame_id_;

        /**
         * @brief Whether in simulation or not.
        */
        bool simulation_;

    };

} // namespace perception
} // namespace iii_drone