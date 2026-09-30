#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <chrono>
#include <cmath>
#include <optional>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {

    /**
     * @brief Measures the thrust the vehicle actually needs to hover from the
     * thrust PX4's position controller commands in steady free flight.
     *
     * In steady flight the vertical component of the commanded thrust carries
     * exactly the vehicle's weight, whatever hover thrust PX4 assumes
     * (MPC_THR_HOVER, which PX4 restores on every disarm): PX4's velocity
     * integrator makes up any difference. The measurement survives disarming,
     * e.g. while the vehicle hangs on the cable.
     */
    class HoverThrustMeter {
    public:
        using Clock = std::chrono::steady_clock;

        /** Steady: slower than these, commanding thrust, for kMinSteady. */
        static constexpr double kMaxSpeed = 0.10;
        static constexpr double kMaxVerticalSpeed = 0.05;
        static constexpr double kMinThrust = 0.1;
        static constexpr std::chrono::milliseconds kMinSteady{1500};
        /** A longer gap between samples ends a steady run. */
        static constexpr std::chrono::milliseconds kMaxGap{500};

        struct Estimate {
            double hover_thrust;
            Clock::time_point measured_at;
        };

        /**
         * @param thrust_up Vertical component of PX4's commanded thrust
         * (normalized).
         * @param free_flight Airborne and not held by the cable.
         */
        void Add(
            Clock::time_point now,
            double thrust_up,
            double speed,
            double vertical_speed,
            bool free_flight
        ) {
            const bool steady = free_flight && std::isfinite(thrust_up) && thrust_up >= kMinThrust &&
                std::isfinite(speed) && speed <= kMaxSpeed &&
                std::isfinite(vertical_speed) && std::abs(vertical_speed) <= kMaxVerticalSpeed;
            if (!steady || !last_ || now - *last_ > kMaxGap) {
                run_start_.reset();
                thrust_sum_ = 0.0;
                samples_ = 0;
            }
            last_ = now;
            if (!steady) return;
            if (!run_start_) run_start_ = now;
            thrust_sum_ += thrust_up;
            ++samples_;
            if (now - *run_start_ >= kMinSteady) {
                estimate_ = Estimate{thrust_sum_ / samples_, now};
            }
        }

        /**
         * @brief The mean thrust of the latest steady run, if one lasted at
         * least kMinSteady.
         */
        std::optional<Estimate> estimate() const { return estimate_; }

    private:
        std::optional<Clock::time_point> last_;
        std::optional<Clock::time_point> run_start_;
        double thrust_sum_ = 0.0;
        int samples_ = 0;
        std::optional<Estimate> estimate_;
    };

} // namespace control
} // namespace iii_drone
