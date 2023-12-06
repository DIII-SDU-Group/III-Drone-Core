#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline_direction.hpp>

/*****************************************************************************/
// CV:

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*****************************************************************************/
// Std:

#include <vector>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class for computing the powerline direction and converting to ROS2 message.
     */
    class PowerlineDirection {
    public:
        /**
         * @brief Constructor.
         * 
         * @param angle The powerline direction angle.
         */
        PowerlineDirection(const float angle);

        /**
         * @brief Constructs a PowerlineDirection object from hough lines and image dimensions.
         * 
         * @param lines The hough lines.
         * @param rows The number of rows in the image.
         * @param cols The number of columns in the image.
         */
        PowerlineDirection(
            const std::vector<cv::Vec2f> lines,
            const int rows,
            const int cols
        );

        /**
         * @brief Constructs a PowerlineDirection object from a ROS2 message.
         * 
         * @param msg The powerline direction ROS2 message.
         */
        PowerlineDirection(const iii_drone_interfaces::msg::PowerlineDirection::SharedPtr msg);

        /**
         * @brief Destructor.
         */
        ~PowerlineDirection();

        /**
         * @brief Converts the powerline direction to a ROS2 message.
         * 
         * @return The powerline direction ROS2 message.
         */
        const iii_drone_interfaces::msg::PowerlineDirection ToMsg() const;

        /**
         * @brief Powerline direction angle getter
         * 
         * @return The powerline direction angle.
        */
        const float & angle() const;

    private:
        /**
         * @brief Powerline direction angle.
         */
        float angle_;

        /**
         * @brief Computes the powerline direction angle.
         * 
         * @param lines The hough lines.
         * @param rows The number of rows in the image.
         * @param cols The number of columns in the image.
        */
        const float computeAngle(
            const std::vector<cv::Vec2f> lines,
            const int rows,
            const int cols
        ) const;

        /**
         * @brief Finds the index of the best line match.
         * 
         * @param lines The hough lines.
         * @param rows The number of rows in the image.
         * @param cols The number of columns in the image.
         * 
         * @return The index of the best line match.
         */
        const int getBestLineIndex(
            const std::vector<cv::Vec2f> lines,
            const int rows,
            const int cols
        ) const;

    };


} // namespace perception
} // namespace iii_drone