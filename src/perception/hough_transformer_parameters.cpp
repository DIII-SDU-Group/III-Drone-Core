/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/hough_transformer_parameters.hpp"

using namespace iii_drone::perception;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

HoughTransformerParameters::HoughTransformerParameters(
    int canny_low_threshold,
    int canny_ratio,
    int canny_kernel_size
) : canny_low_threshold_(canny_low_threshold),
    canny_ratio_(canny_ratio),
    canny_kernel_size_(canny_kernel_size) { }

HoughTransformerParameters::~HoughTransformerParameters() { }

const int & HoughTransformerParameters::canny_low_threshold() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return canny_low_threshold_;

}

int & HoughTransformerParameters::canny_low_threshold() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return canny_low_threshold_;

}

const int & HoughTransformerParameters::canny_ratio() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return canny_ratio_;

}

int & HoughTransformerParameters::canny_ratio() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return canny_ratio_;

}

const int & HoughTransformerParameters::canny_kernel_size() const {

    std::shared_lock<std::shared_mutex> lock(parameter_mutex_);

    return canny_kernel_size_;

}

int & HoughTransformerParameters::canny_kernel_size() {

    std::unique_lock<std::shared_mutex> lock(parameter_mutex_);

    return canny_kernel_size_;

}