#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Standard includes:

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/offboard_control_mode.hpp>

/*****************************************************************************/
// III:

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /*
    * @brief PX4 Offboard Control Mode adapter class
    */
    class OffboardControlModeAdapter {
    public:
        /**
         * @brief Constructor
         * 
         * @param stamp ROS2 timestamp
         * @param position Position control enabled
         * @param velocity Velocity control enabled
         * @param acceleration Acceleration control enabled
         * @param attitude Attitude control enabled
         * @param body_rate Body rate control enabled
         * @param actuator Actuator control enabled
         */
        OffboardControlModeAdapter(
            const rclcpp::Time & stamp,
            const bool & position,
            const bool & velocity,
            const bool & acceleration,
            const bool & attitude,
            const bool & body_rate,
            const bool & actuator
        );

        /**
         * @brief Convert to ROS2 message
         *
         * @return PX4 OffboardControlMode message
         */
        px4_msgs::msg::OffboardControlMode ToMsg() const;

        /** 
         * @brief Timestamp getter
         *
         * @return ROS2 timestamp
         */
        const rclcpp::Time & stamp() const;

        /**
         * @brief Position control enabled getter
         *
         * @return Position control enabled
         */
        const bool & position() const;

        /**
         * @brief Velocity control enabled getter
         *
         * @return Velocity control enabled
         */
        const bool & velocity() const;

        /**
         * @brief Acceleration control enabled getter
         *
         * @return Acceleration control enabled
         */
        const bool & acceleration() const;

        /**
         * @brief Attitude control enabled getter
         *
         * @return Attitude control enabled
         */
        const bool & attitude() const;

        /**
         * @brief Body rate control enabled getter
         *
         * @return Body rate control enabled
         */
        const bool & body_rate() const;

        /**
         * @brief Actuator control enabled getter
         *
         * @return Actuator control enabled
         */
        const bool & actuator() const;

    private:
        /**
         * @brief ROS2 timestamp
         */
        rclcpp::Time stamp_;

        /**
         * @brief PX4 timestamp
         */
        bool position_;

        /**
         * @brief Position control enabled
         */
        bool velocity_;

        /**
         * @brief Velocity control enabled
         */
        bool acceleration_;

        /**
         * @brief Acceleration control enabled
         */
        bool attitude_;

        /**
         * @brief Attitude control enabled
         */
        bool body_rate_;

        /**
         * @brief Body rate control enabled
         */
        bool actuator_;

    };

} // namespace px4
} // namespace adapters
} // namespace iii_drone