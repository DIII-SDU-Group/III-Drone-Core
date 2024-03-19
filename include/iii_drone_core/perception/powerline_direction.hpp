#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline_direction.hpp>
#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/timestamp.hpp>
#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/configuration/parameter_bundle.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>

/*****************************************************************************/
// Std:

#include <vector>
#include <mutex>
#include <shared_mutex>
#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class for computing the powerline direction and converting to ROS2 message, as well as Kalman Filtering.
     */
    class PowerlineDirection {
    public:
        /**
         * @brief Default constructor.
         * 
         * @param parameters The powerline direction parameters.
         */
        PowerlineDirection(iii_drone::configuration::ParameterBundle::SharedPtr parameters);

        /**
         * @brief Destructor.
         */
        ~PowerlineDirection();

        /**
         * @brief Converts to pose stamped message.
         * 
         * @param drone_frame_id The drone frame id.
         * 
         * @return geometry_msgs::msg::PoseStamped The pose stamped message.
         */
        const geometry_msgs::msg::PoseStamped ToPoseStampedMsg(const std::string & drone_frame_id) const;

        /**
         * @brief Converts to quaternion stamped message.
         * 
         * @param drone_frame_id The drone frame id.
         * 
         * @return geometry_msgs::msg::QuaternionStamped The quaternion stamped message.
         */
        const geometry_msgs::msg::QuaternionStamped ToQuaternionStampedMsg(const std::string & drone_frame_id) const;

        /**
         * @brief Runs the Kalman Filter update step from an observed yaw angle.
         * 
         * @param pl_yaw The observed powerline yaw.
         */
        void Update(float pl_yaw);

        /**
         * @brief Runs the Kalman Filter predict step from a drone quaternion.
         * 
         * @param drone_quat The drone quaternion.
         */
        void Predict(const iii_drone::types::quaternion_t & drone_quat);

        /**
         * @brief Updates the PowerlineAdapter object.
         * 
         * @param msg The powerline ROS2 message.
         */
        void UpdatePowerlineAdapter(const iii_drone_interfaces::msg::Powerline::SharedPtr msg);

        /**
         * @brief Powerline quaternion getter.
         */
        const iii_drone::types::quaternion_t quaternion() const;

        /**
         * @brief Stamp getter.
         */
        const rclcpp::Time stamp() const;

    private:
        /**
         * @brief Time stamp for latest change.
        */
        iii_drone::utils::Timestamp stamp_;

        /**
         * @brief The powerline direction parameters.
        */
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        /**
         * @brief Checks if any cable is in the field of view
         * 
         * @return bool True if any cable is in the field of view
         */
        bool anyCableInFOV() const;

        /**
         * @brief Computes the powerline euler angles from observed powerline yaw.
         * 
         * @param pl_yaw The observed powerline yaw.
         * @param drone_quat The drone quaternion.
         * 
         * @return iii_drone::types::euler_angles_t The powerline euler angles.
        */
        const iii_drone::types::euler_angles_t computePowerlineEulerAngles(
            const float & pl_yaw,
            const iii_drone::types::quaternion_t & drone_quat
        );

        /**
         * @brief Maps the angle into continuous range based on current angle
         * 
         * @param curr_angle Current angle
         * @param new_angle New angle
         * 
         * @return float Mapped angle
         */
        float mapAngle(
            float curr_angle, 
            float new_angle
        ) const;

        /**
         * @brief Maps the angle into continuous range based on current angle
         * 
         * @param curr_angle Current angle
         * @param new_angle New angle
         * 
         * @return float Mapped angle
         */
        float mapAngle2(
            float curr_angle, 
            float new_angle
        ) const;

        /**
         * @brief Maps the angle back into the correct range after having performed the Kalman filter
         * 
         * @param angle Angle to map
         * 
         * @return float Back mapped angle
         */
        float backmapAngle( float angle) const;

        /**
         * @brief Struct for holding Kalman filtering data
         */
        typedef struct {

            float state_est;
            float var_est;

        } kf_est_t;

        /**
         * @brief Kalman filter estimates
        */
        kf_est_t pl_angle_est_[3];

        /**
         * @brief Resets the Kalman filter estimates
         */
        void resetKalmanFilter();

        /**
         * @brief The drone quaternions
        */
        iii_drone::utils::History<iii_drone::types::quaternion_t> drone_quat_history_;

        /**
         * @brief The powerline quaternion
        */
        iii_drone::utils::History<iii_drone::types::quaternion_t> pl_quat_history_;

        /**
         * @brief The Powerline object
        */
        iii_drone::adapters::PowerlineAdapter pl_;

        /**
         * @brief Mutex for powerline access.
        */
        mutable std::shared_mutex pl_mutex_;

    };


} // namespace perception
} // namespace iii_drone