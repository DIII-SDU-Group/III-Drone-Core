#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Std:

#include <mutex>
#include <shared_mutex>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace control {

    /**
     * @brief Class for handling parameters for positional MPC.
    */
    class MPCParameters {
    public:
        /**
         * @brief Constructor
         * 
         * @param use_state_feedback Whether to use state feedback
         * @param N Number of steps in the MPC horizon
         * @param dt Time step
         * @param v_max Maximum velocity vector plus/minus
         * @param a_max Maximum acceleration vector plus/minus
         * @param wp Position tracking weights vector
         * @param wv Velocity tracking weights vector
         * @param wa Acceleration tracking weights vector
         * @param wj Jerk tracking weights vector
        */
        MPCParameters(
            bool use_state_feedback,
            int N,
            double dt,
            const types::vector_t & v_max,
            const types::vector_t & a_max,
            const types::vector_t & wp,
            const types::vector_t & wv,
            const types::vector_t & wa,
            const types::vector_t & wj
        );

        /**
         * @brief Get whether to use state feedback
         * 
         * @return Whether to use state feedback
        */
        bool use_state_feedback() const;

        /**
         * @brief Get/set whether to use state feedback
        */
        bool & use_state_feedback();

        /**
         * @brief Get the number of steps in the MPC horizon
         * 
         * @return Number of steps in the MPC horizon
        */
        int N() const;

        /**
         * @brief Get the time step
         * 
         * @return Time step
        */
        double dt() const;

        /**
         * @brief Get the maximum velocity vector plus/minus
         * 
         * @return Maximum velocity vector plus/minus reference
        */
        const types::vector_t & v_max() const;

        /**
         * @brief Get/set the maximum velocity vector plus/minus
         * 
         * @return Maximum velocity vector plus/minus reference
        */
        types::vector_t & v_max();

        /**
         * @brief Get the maximum acceleration vector plus/minus
         * 
         * @return Maximum acceleration vector plus/minus reference
        */
        const types::vector_t & a_max() const;

        /**
         * @brief Get/set the maximum acceleration vector plus/minus
         * 
         * @return Maximum acceleration vector plus/minus reference
        */
        types::vector_t & a_max();

        /**
         * @brief Get the position tracking weights vector
         * 
         * @return Position tracking weights vector reference
        */
        const types::vector_t & wp() const;

        /**
         * @brief Get/set the position tracking weights vector
         * 
         * @return Position tracking weights vector reference
        */
        types::vector_t & wp();

        /**
         * @brief Get the velocity tracking weights vector
         * 
         * @return Velocity tracking weights vector reference
        */
        const types::vector_t & wv() const;

        /**
         * @brief Get/set the velocity tracking weights vector
         * 
         * @return Velocity tracking weights vector reference
        */
        types::vector_t & wv();

        /**
         * @brief Get the acceleration tracking weights vector
         * 
         * @return Acceleration tracking weights vector reference
        */
        const types::vector_t & wa() const;

        /**
         * @brief Get/set the acceleration tracking weights vector
         * 
         * @return Acceleration tracking weights vector reference
        */
        types::vector_t & wa();

        /**
         * @brief Get the jerk tracking weights vector
         * 
         * @return Jerk tracking weights vector reference
        */      
        const types::vector_t & wj() const;

        /**
         * @brief Get/set the jerk tracking weights vector
         * 
         * @return Jerk tracking weights vector reference
        */
        types::vector_t & wj();

    private:
        /**
         * @brief Shared mutex protecting the parameter access
         */
        mutable std::shared_mutex parameters_mutex_;

        /**
         * @brief Whether to use state feedback
         */
        bool use_state_feedback_;

        /**
         * @brief Number of steps in the MPC horizon
         */
        int N_;

        /**
         * @brief Time step
         */
        double dt_;

        /**
         * @brief Maximum velocity vector plus/minus
         */
        types::vector_t v_max_;

        /**
         * @brief Maximum acceleration vector plus/minus
         */
        types::vector_t a_max_;

        /**
         * @brief Position tracking weights vector
         */
        types::vector_t wp_;

        /**
         * @brief Velocity tracking weights vector
         */
        types::vector_t wv_;

        /**
         * @brief Acceleration tracking weights vector
         */
        types::vector_t wa_;

        /**
         * @brief Jerk tracking weights vector
         */
        types::vector_t wj_;

    };

} // namespace control
} // namespace iii_drone
