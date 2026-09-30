#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief Upward push of a vehicle holding a cable from below, commanded as
     * a vertical acceleration setpoint.
     *
     * PX4 turns an acceleration setpoint into thrust = hover thrust * (1 + a/g),
     * so the push force is set by the target. A velocity setpoint cannot bound
     * it: the cable stops the vehicle, the velocity error never closes and
     * PX4's integrator winds thrust up to its maximum.
     *
     * PX4 keeps thrust at zero until an upward setpoint takes it out of its
     * takeoff state machine (and for its spool-up time after arming). The
     * profile therefore first requests takeoff with a small acceleration and
     * ramps to the target at a bounded jerk only while PX4 applies the push:
     * it reports the vehicle airborne and its position controller commands
     * thrust. Far above the ground PX4 reports airborne as soon as it arms,
     * so the land detector alone does not show that thrust is on.
     */
    class CablePushProfile {
    public:
        using Clock = std::chrono::steady_clock;

        /**
         * @brief What PX4 reports about the push.
         */
        enum class Px4 {
            /** Airborne and commanding at least kMinPushingThrust. */
            kPushing,
            /** Fresh evidence of landed, maybe landed, ground contact or no thrust. */
            kNotPushing,
            /** No fresh land-detector or thrust sample. */
            kUnknown,
        };

        /**
         * @brief Normalized thrust above which PX4 applies the push; before
         * takeoff PX4 commands zero, any push is near hover thrust.
         */
        static constexpr double kMinPushingThrust = 0.1;

        static Px4 Classify(bool evidence_fresh, bool airborne, double thrust_up) {
            if (!evidence_fresh) return Px4::kUnknown;
            return airborne && thrust_up >= kMinPushingThrust ? Px4::kPushing : Px4::kNotPushing;
        }

        struct Limits {
            /**
             * @brief Acceleration commanded until PX4 applies the push; it
             * only has to be upward to request takeoff.
             */
            double takeoff_request_acceleration_m_s2 = 0.2;

            /**
             * @brief Ramp rate between accelerations while PX4 pushes.
             */
            double jerk_m_s3 = 1.0;

            /**
             * @brief Time from start within which PX4 must apply the push.
             */
            std::chrono::duration<double> start_timeout{8.0};
        };

        CablePushProfile(
            double target_acceleration_m_s2,
            Limits limits,
            Clock::time_point started_at
        ) : target_(target_acceleration_m_s2),
            limits_(limits),
            started_at_(started_at),
            acceleration_(limits.takeoff_request_acceleration_m_s2) {
            if (!(target_ > 0.0) || !std::isfinite(target_)) {
                throw std::invalid_argument("cable push target acceleration must be positive and finite");
            }
            if (!(limits_.takeoff_request_acceleration_m_s2 > 0.0) ||
                limits_.takeoff_request_acceleration_m_s2 > target_ ||
                !(limits_.jerk_m_s3 > 0.0) ||
                !(limits_.start_timeout.count() > 0.0)) {
                throw std::invalid_argument(
                    "cable push limits need 0 < takeoff request <= target, a positive jerk and timeout");
            }
        }

        /**
         * @brief Advances the push to now and returns the upward acceleration
         * to command. The ramp advances only while PX4 applies the push;
         * unknown holds it. Once PX4 has applied the push, explicit evidence
         * that it no longer does fails it.
         */
        double Update(Clock::time_point now, Px4 px4) {
            if (px4 == Px4::kPushing) {
                pushing_seen_ = true;
            } else if (px4 == Px4::kNotPushing && pushing_seen_) {
                pushing_lost_ = true;
            }
            if (px4 == Px4::kPushing && px4_ == Px4::kPushing && last_update_) {
                const double dt = std::max(
                    0.0, std::chrono::duration<double>(now - *last_update_).count());
                const double step = limits_.jerk_m_s3 * dt;
                const double remaining = target_ - acceleration_;
                acceleration_ = std::abs(remaining) <= step
                    ? target_ : acceleration_ + std::copysign(step, remaining);
            }
            last_update_ = now;
            px4_ = px4;
            return acceleration_;
        }

        /**
         * @brief Changes the acceleration the push ramps to (at least the
         * takeoff request acceleration).
         */
        void SetTarget(double target_acceleration_m_s2) {
            if (!std::isfinite(target_acceleration_m_s2)) {
                throw std::invalid_argument("cable push target acceleration must be finite");
            }
            target_ = std::max(target_acceleration_m_s2, limits_.takeoff_request_acceleration_m_s2);
        }

        /**
         * @brief The push acceleration that makes PX4 command push_ratio times
         * the thrust the vehicle actually needs to hover.
         *
         * PX4 realizes an acceleration setpoint as px4_hover_thrust *
         * (1 + a/g), with the hover thrust it assumes; measured_hover_thrust is
         * the thrust the vehicle actually hovers at. The result is capped so
         * the commanded thrust stays at or below max_thrust and is at least
         * min_acceleration.
         */
        static double CalibratedAcceleration(
            double push_ratio,
            double measured_hover_thrust,
            double px4_hover_thrust,
            double max_thrust,
            double min_acceleration
        ) {
            constexpr double g = 9.80665;
            const double wanted = g * (push_ratio * measured_hover_thrust / px4_hover_thrust - 1.0);
            const double cap = g * (max_thrust / px4_hover_thrust - 1.0);
            return std::max(min_acceleration, std::min(wanted, cap));
        }

        double acceleration() const { return acceleration_; }

        double target() const { return target_; }

        /**
         * @brief PX4 applies the push and it has reached its target.
         */
        bool established() const {
            return px4_ == Px4::kPushing && !pushing_lost_ && acceleration_ == target_;
        }

        /**
         * @brief PX4 never applied the push within the timeout, or stopped
         * applying it (landed, maybe landed, ground contact or no thrust).
         */
        bool failed(Clock::time_point now) const {
            return pushing_lost_ ||
                (!pushing_seen_ && now - started_at_ > limits_.start_timeout);
        }

        bool pushingLost() const { return pushing_lost_; }

    private:
        double target_;
        Limits limits_;
        Clock::time_point started_at_;
        double acceleration_;
        std::optional<Clock::time_point> last_update_;
        Px4 px4_ = Px4::kUnknown;
        bool pushing_seen_ = false;
        bool pushing_lost_ = false;
    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
