#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <mutex>
#include <shared_mutex>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace utils {

    /**
     * @brief Timestamp class for storing and loading thread safe ROS2 timestamps.
     */
    class Timestamp {
    public:
        /**
         * @brief Default constructor.
         */
        Timestamp();

        /**
         * @brief Constructor from ROS2 timestamp.
         * 
         * @param stamp The ROS2 timestamp.
         */
        Timestamp(const rclcpp::Time & stamp);

        /**
         * @brief Updates the timestamp to now.
         */
        void Update();

        /**
         * @brief Assignment operator.
         */
        Timestamp & operator=(const rclcpp::Time & stamp);

        /**
         * @brief Assignment operator.
         */
        Timestamp & operator=(const Timestamp & timestamp);

        /**
         * @brief Cast operator.
         */
        operator rclcpp::Time() const;

    private:
        /**
         * @brief The ROS2 timestamp.
         */
        rclcpp::Time stamp_;

        /**
         * @brief Mutex for thread safety.
         */
        mutable std::shared_mutex mutex_;

    };

} // namespace utils
} // namespace iii_drone