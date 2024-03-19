#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

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
#include <memory>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/parameter_bundle.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class for performing the hough transform on images to detect powerline direction.
     */
    class HoughTransformer {
    public:
        /**
         * @brief Constructor.
         * 
         * @param parameters The hough transformer parameter bundle.
         */
        HoughTransformer(iii_drone::configuration::ParameterBundle::SharedPtr parameters);

        /**
         * @brief Destructor.
         * 
         * 
         */
        ~HoughTransformer();

        /**
         * @brief Performs the hough transform on the given image.
         * 
         * @param img The image to perform the hough transform on.
         * 
         * @return The hough transform lines.
         */
        const std::vector<cv::Vec2f> GetHoughLines(const cv::Mat img) const;

        /**
         * @brief Computes the hough lines direction angle from hough lines.
         * 
         * @param lines The hough lines.
         * @param rows The number of rows in the image.
         * @param cols The number of columns in the image.
         * 
         * @return The hough lines direction angle.
        */
        float ComputeAngle(
            const std::vector<cv::Vec2f> lines,
            int rows,
            int cols
        ) const;

        /**
         * @brief Computes the hough lines direction angle from an image.
         * 
         * @param img The image to perform the hough transform on.
         * @param angle The computed angle output.
         * 
         * @return True if the angle was computed successfully, false otherwise.
        */
        bool ComputeAngle(
            const cv::Mat img,
            float & angle
        ) const;

    private:
        /**
         * @brief The hough transformer parameter bundle.
         */
        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        /**
         * @brief Finds the index of the best hough line match.
         * 
         * @param lines The hough lines.
         * @param rows The number of rows in the image.
         * @param cols The number of columns in the image.
         * 
         * @return The index of the best line match.
         */
        int getBestLineIndex(
            const std::vector<cv::Vec2f> lines,
            int rows,
            int cols
        ) const;

        

    };

} // namespace perception
} // namespace iii_drone