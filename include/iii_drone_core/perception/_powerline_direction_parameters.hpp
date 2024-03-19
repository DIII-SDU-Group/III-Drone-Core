#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <shared_mutex>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace perception {

    /**
     * @brief Class for storing the parameters of the powerline direction.
     */
    class PowerlineDirectionParameters {
    public:
        /**
         * @brief Constructor.
         * 
         * @param drone_frame_id The frame id of the drone.
         * @param kf_r The kalman filter r parameter.
         * @param kf_q The kalman filter q parameter.
        */
        PowerlineDirectionParameters(
            std::string drone_frame_id,
            float kf_r,
            float kf_q
        );

        /**
         * @brief Getter for the drone frame id.
         */
        const std::string & drone_frame_id() const;

        /**
         * @brief Setter for the drone frame id.
         */
        std::string & drone_frame_id();

        /**
         * @brief Getter for the kalman filter r parameter.
         */
        const float & kf_r() const;

        /**
         * @brief Setter for the kalman filter r parameter.
         */
        float & kf_r();

        /**
         * @brief Getter for the kalman filter q parameter.
         */
        const float & kf_q() const;

        /**
         * @brief Setter for the kalman filter q parameter.
         */
        float & kf_q();

    private:
        /**
         * @brief The frame id of the drone.
         */
        std::string drone_frame_id_;

        /**
         * @brief The kalman filter r parameter.
         */
        float kf_r_;

        /**
         * @brief The kalman filter q parameter.
         */
        float kf_q_;

    };

} // namespace perception
} // namespace iii_drone