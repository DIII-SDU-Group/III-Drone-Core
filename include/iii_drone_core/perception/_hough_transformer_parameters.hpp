#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <shared_mutex>
#include <mutex>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class storing the parameters for the hough transformer.
    */
    class HoughTransformerParameters {
        public:
            /**
             * @brief Constructor.
             * 
             * @param canny_low_threshold The canny edge detector parameter low_threshold.
             * @param canny_ratio The canny edge detector parameter ratio.
             * @param canny_kernel_size The canny edge detector parameter kernel_size.
             */
            HoughTransformerParameters(
                int canny_low_threshold,
                int canny_ratio,
                int canny_kernel_size
            );

            /**
             * @brief Destructor.
             */
            ~HoughTransformerParameters();

            /**
             * @brief Canny edge detector parameter low_threshold getter.
             * 
             * @return The canny edge detector parameter low_threshold.
             */
            const int & canny_low_threshold() const;

            /**
             * @brief Canny edge detector parameter low_threshold setter.
             * 
             * @param canny_low_threshold The canny edge detector parameter low_threshold.
             */
            int & canny_low_threshold();

            /**
             * @brief Canny edge detector parameter ratio getter.
             * 
             * @return The canny edge detector parameter ratio.
             */
            const int & canny_ratio() const;

            /**
             * @brief Canny edge detector parameter ratio setter.
             * 
             * @param canny_ratio The canny edge detector parameter ratio.
             */
            int & canny_ratio();

            /**
             * @brief Canny edge detector parameter kernel_size getter.
             * 
             * @return The canny edge detector parameter kernel_size.
             */
            const int & canny_kernel_size() const;

            /**
             * @brief Canny edge detector parameter kernel_size setter.
             * 
             * @param canny_kernel_size The canny edge detector parameter kernel_size.
             */
            int & canny_kernel_size();

        private:
            /**
             * @brief The parameter mutex.
             */
            mutable std::shared_mutex parameter_mutex_;

            /**
             * @brief The canny edge detector parameter low_threshold.
             */
            int canny_low_threshold_;

            /**
             * @brief The canny edge detector parameter ratio.
             */
            int canny_ratio_;

            /**
             * @brief The canny edge detector parameter kernel_size.
             */
            int canny_kernel_size_;

    };
} // namespace perception
} // namespace iii_drone