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

#include <iii_drone_core/perception/hough_transformer_parameters.hpp>

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
         * @param parameters The hough transformer parameters.
         */
        HoughTransformer(std::shared_ptr<HoughTransformerParameters> parameters);

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

    private:
        /**
         * @brief The hough transformer parameters.
         */
        std::shared_ptr<HoughTransformerParameters> parameters_;
        

    };

} // namespace perception
} // namespace iii_drone