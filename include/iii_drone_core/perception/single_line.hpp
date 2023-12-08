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

/*****************************************************************************/
// ROS2:

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/timestamp.hpp>

#include <iii_drone_core/adapters/single_line_adapter.hpp>

#include <iii_drone_core/perception/powerline_parameters.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief SingleLine class for pose estimation of a single powerline.
     */
    class SingleLine {
    public:
        /**
         * @brief Copy constructor.
         * 
         * @param other The other SingleLine object.
         */
        SingleLine(const SingleLine & other);

        /**
         * @brief Constructor from Pose msg.
         * 
         * @param id The id of the line.
         * @param pose The pose msg.
         * @param tf_buffer The tf buffer.
         * @param parameters The powerline parameters.
         */
        SingleLine(
            const int & id,
            const geometry_msgs::msg::Pose & pose,
            const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
            const std::shared_ptr<PowerlineParameters> & parameters
        );

        /**
         * @brief Constructor from position and quaternion.
         * 
         * @param id The id of the line.
         * @param position The position.
         * @param quaternion The quaternion.
         * @param tf_buffer The tf buffer.
         * @param parameters The powerline parameters.
         */
        SingleLine(
            const int & id,
            const iii_drone::types::point_t & position,
            const iii_drone::types::quaternion_t & quaternion,
            const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
            const std::shared_ptr<PowerlineParameters> & parameters
        );

        /**
         * @brief Constructor from position, quaternion and frame_id.
         * 
         * @param id The id of the line.
         * @param position The position.
         * @param quaternion The quaternion.
         * @param frame_id The frame id.
         * @param tf_buffer The tf buffer.
         * @param parameters The powerline parameters.
         */
        SingleLine(
            const int & id,
            const iii_drone::types::point_t & position,
            const iii_drone::types::quaternion_t & quaternion,
            const std::string & frame_id,
            const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
            const std::shared_ptr<PowerlineParameters> & parameters
        );

        /**
         * @brief Constructor from SingleLineAdapter.
         * 
         * @param adapter The SingleLineAdapter.
         * @param tf_buffer The tf buffer.
         * @param parameters The powerline parameters.
         */
        SingleLine(
            const iii_drone::adapters::SingleLineAdapter & adapter,
            const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
            const std::shared_ptr<PowerlineParameters> & parameters
        );

        /**
         * @brief Converts to SingleLineAdapter.
         * 
         * @return iii_drone::adapters::SingleLineAdapter The SingleLineAdapter.
         */
        const iii_drone::adapters::SingleLineAdapter ToAdapter() const;

        /**
         * @brief Overwrites the timestamp.
         * 
         * @param stamp The new timestamp.
         */
        void OverwriteStamp(const rclcpp::Time & stamp);

        /**
         * @brief Returns whether the line is alive, i.e. whether it is not in the FOV or alive_cnt_ is above alive_cnt_low_thresh_.
         * If in FOV, alive_cnt_ is decremented.
         * 
         * @return bool Whether the line is alive.
         */
        bool IsAlive();

        /**
         * @brief Returns whether the line is visible, i.e. whether it is in the FOV.
         * 
         * @return bool Whether the line is visible.
         */
        bool IsVisible() const;

        /**
         * @brief Returns whether the position is in the FOV.
         * 
         * @param position The position.
         * @param min_point_dist The minimum distance of the position.
         * @param max_point_dist The maximum distance of the position.
         * @param view_cone_slope The slope of the view cone.
         * 
         * @return bool Whether the position is in the FOV.
         */
        bool IsInFOV(
            const iii_drone::types::point_t & position,
            const float min_point_dist,
            const float max_point_dist,
            const float view_cone_slope
        ) const;

        /**
         * @brief Returns whether the line is in FOV, non strict.
         * 
         * @return bool Whether the line is in FOV, non strict.
         */
        bool IsInFOV() const;

        /**
         * @brief Returns whether the line is in FOV, using the strict parameters.
         * 
         * @return bool Whether the line is in FOV, strict.
         */
        bool IsInFOVStrict() const;

        /**
         * @brief Runs the Kalman Filter update step and increments the alive counter.
         * 
         * @param position The new projected position.
         */
        void Update(const iii_drone::types::point_t & projected_position);

        /**
         * @brief Runs the Kalman Filter predict step.
         * 
         * @param delta_position The delta drone position.
         * @param delta_quat The delta drone quaternion orientation.
         * @param projection_plane The projection plane.
         */
        void Predict(
            const iii_drone::types::vector_t & delta_drone_position, 
            const iii_drone::types::quaternion_t & delta_drone_quat,
            const iii_drone::types::plane_t & projection_plane
        );

        /**
         * @brief Sets the position and updates the Kalman Filter estimates.
         * 
         * @param position The new position.
         */
        void SetPosition(const iii_drone::types::point_t & position);

        /**
         * @brief position getter.
         */
        const iii_drone::types::point_t position() const;

        /**
         * @brief Id getter.
         */
        int id() const;

        /**
         * @brief Stamp getter.
         */
        rclcpp::Time stamp() const;

        /**
         * @brief frame_id getter.
         */
        std::string frame_id() const;

        /**
         * @brief quaternion getter.
         */
        iii_drone::types::quaternion_t quaternion() const;

        /**
         * @brief projected_position getter.
         */
        iii_drone::types::point_t projected_position() const;

        /**
         * @brief tf_buffer getter.
         */
        std::shared_ptr<tf2_ros::Buffer> tf_buffer() const;

        /**
         * @brief parameters getter.
         */
        std::shared_ptr<PowerlineParameters> parameters() const;

        /**
         * @brief Alive counter getter.
         */
        int alive_cnt() const;

        /**
         * @brief State estimate getter.
         */
        float state_est(const int & i) const;

        /**
         * @brief Variance estimate getter.
         */
        float var_est(const int & i) const;

        /**
         * @brief Assignment operator overload.
         */
        SingleLine & operator=(const SingleLine & other);

    private:
        /**
         * @brief The timestamp.
         */
        iii_drone::utils::Timestamp stamp_;

        /**
         * @brief The frame id.
         */
        std::string frame_id_;

        /**
         * @brief The tf buffer.
         */
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        /**
         * @brief The powerline parameters.
         */
        std::shared_ptr<PowerlineParameters> parameters_;

        /**
         * @brief The mutex.
         */
        mutable std::shared_mutex mutex_;

        /**
         * @brief Kalman Filter estimate struct.
         */
        typedef struct {

            float state_est;
            float var_est;

        } kf_est_t;

        /**
         * @brief The Kalman Filter estimates.
         */
        kf_est_t estimates[3];

        /**
         * @brief Resets the Kalman Filter estimates.
         */
        void resetKalmanFilter();

        /**
         * @brief The powerline position.
         */
        iii_drone::types::point_t position_;

        /**
         * @brief The projected powerline position.
         */
        iii_drone::types::point_t projected_position_;

        /**
         * @brief The powerline quaternion.
         */
        iii_drone::types::quaternion_t quaternion_;

        /**
         * @brief The line id.
         */
        int id_;

        /**
         * @brief The line alive counter.
         */
        int alive_cnt_;
        
    };

} // namespace perception
} // namespace iii_drone