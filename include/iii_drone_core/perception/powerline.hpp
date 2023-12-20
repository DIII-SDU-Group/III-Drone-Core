#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include <math.h>
#include <limits>

/*****************************************************************************/
// Eigen:

#include <eigen3/Eigen/Core>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/timestamp.hpp>
#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/perception/powerline_parameters.hpp>
#include <iii_drone_core/perception/single_line.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/adapters/point_cloud_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class for estimation of the powerline cable poses.
     * Attention: Requires the tf_buffer to be set after construction.
     */
    class Powerline {
    public:
        /**
         * @brief Constructor.
         * 
         * @param powerline_parameters Pointer to the powerline parameters.
        */
        Powerline(std::shared_ptr<PowerlineParameters> powerline_parameters);

        /**
         * @brief Converts to a PowerlineAdapter.
         * 
         * @param only_visible Whether to only return the visible lines, default false.
         * 
         * @return The PowerlineAdapter.
         */
        const iii_drone::adapters::PowerlineAdapter ToAdapter(bool only_visible = false) const;

        /**
         * @brief Converts to a PointCloudAdapter.
         * 
         * @param only_visible Whether to only return the visible lines, default false.
         * 
         * @return The PointCloudAdapter.
         */
        const iii_drone::adapters::PointCloudAdapter ToPointCloudAdapter(bool only_visible = false) const;

        /**
         * @brief Returns the lines that are currently in the sensor FOV.
         * 
         * @return The lines that are currently in the sensor FOV.
        */
        const std::vector<SingleLine> GetVisibleLines() const;

        /**
         * @brief Returns the number of tracked lines.
         * 
         * @return The number of tracked lines.
        */
        int GetLinesCount() const;

        /**
         * @brief Updates from a point. If match is found, runs the Kalman Filter update step for the matching line,
         *       otherwise creates a new line. Returns the projected point.
         * 
         * @param point The point to update from.
         * 
         * @return The projected point.
        */
        iii_drone::types::point_t UpdateLine(const iii_drone::types::point_t & point);

        /**
         * @brief Updates the powerline direction and updates the projection plane.
         * 
         * @param pl_direction The powerline direction.
        */
        void UpdateDirection(const iii_drone::types::quaternion_t & pl_direction);

        /**
         * @brief Updates the powerline from odometry.
         * 
         * @param drone_pose The drone pose.
        */
        void UpdateOdometry(const iii_drone::types::pose_t & drone_pose);

        /**
         * @brief Cleans up the lines that are not in the sensor FOV.
        */
        void CleanupLines();

        /**
         * @brief Computes the inter line positions.
        */
        void ComputeInterLinePositions();

        /**
         * @brief Resets the powerline object.
        */
        void Reset();

        /**
         * @brief tf_buffer setter.
        */
        std::shared_ptr<tf2_ros::Buffer> & tf_buffer();

        /**
         * @brief Stamp getter
         */
        const rclcpp::Time stamp() const;

        /**
         * @brief Returns the powerline quaternion direction.
         * 
         * @return The powerline quaternion direction.
        */
        const iii_drone::types::quaternion_t powerline_direction() const;

        /**
         * @brief Returns the powerline projection plane.
         * 
         * @return The powerline projection plane.
        */
        const iii_drone::types::plane_t projection_plane() const;

        /**
         * @brief Returns the drone quaternion orientation.
         * 
         * @return The drone quaternion orientation.
        */
        const iii_drone::types::pose_t drone_pose() const;

    private:
        /**
         * @brief Time stamp for latest change.
        */
        iii_drone::utils::Timestamp stamp_;

        /**
         * @brief The powerline parameters.
        */
        std::shared_ptr<PowerlineParameters> parameters_;

        /**
         * @brief The tf buffer.
        */
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        /**
         * @brief The inter line positions struct type.
         * 
         * @param line_id_1 The id of the first line.
         * @param line_id_2 The id of the second line.
         * @param inter_line_position_window The window of inter line positions.
        */
        struct inter_line_positions_t {

            int line_id_1;
            int line_id_2;
            std::vector<iii_drone::types::vector_t> inter_line_position_window;

        };

        /**
         * @brief The currently tracked lines.
        */
        std::vector<SingleLine> lines_;

        /**
         * @brief The inter line positions.
        */
        std::vector<inter_line_positions_t> inter_line_positions_;

        /**
         * @brief Mutex for protection of lines and inter line positions.
        */
        mutable std::shared_mutex lines_mutex_;

        /**
         * @brief The powerline direction quaternion history.
        */
        iii_drone::utils::History<iii_drone::types::quaternion_t> pl_dir_history_;

        /**
         * @brief The drone pose history.
        */
        iii_drone::utils::History<iii_drone::types::pose_t> drone_pose_history_;

        /**
         * @brief The powerline projection plane.
        */
        iii_drone::utils::Atomic<iii_drone::types::plane_t> projection_plane_;

        /**
         * @brief The counter for unique line ids.
        */
        iii_drone::utils::Atomic<int> id_cnt_ = 0;

        /**
         * @brief Updates the projection plane.
        */
        void updateProjectionPlane();

        /**
         * @brief Finds the matching line for a point.
         * 
         * @param point The point to find the matching line for.
         * 
         * @return The id of the matching line.
        */
        int findMatchingLine(const iii_drone::types::point_t & point) const;

        /**
         * @brief Registers a new line.
         * 
         * @param point The point to register the new line from.
         */
        void registerNewLine(const iii_drone::types::point_t & point);

        /**
         * @brief Projects a point onto the powerline projection plane.
         * 
         * @param point The point to project.
         * 
         * @return The projected point.
        */
        const iii_drone::types::point_t projectPoint(const iii_drone::types::point_t & point) const;

        /**
         * @brief Runs the Kalman Filter predict step for all lines.
        */
        void predictLines();

    };

} // namespace perception
} // namespace iii_drone