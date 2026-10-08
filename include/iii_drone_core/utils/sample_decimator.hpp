#pragma once

#include <cstdint>
#include <optional>

namespace iii_drone::utils {

/**
 * Passes one sample per period of source sample time (not receive time, so
 * delivery jitter does not skew the rate). A sample time that goes back (a
 * PX4 restart) passes and restarts the period.
 */
class SampleDecimator {
public:
    explicit SampleDecimator(uint64_t period_us) : period_us_(period_us) {}

    bool pass(uint64_t sample_us) {
        if (last_us_ && sample_us >= *last_us_ && sample_us - *last_us_ < period_us_) {
            return false;
        }
        last_us_ = sample_us;
        return true;
    }

private:
    uint64_t period_us_;
    std::optional<uint64_t> last_us_;
};

}  // namespace iii_drone::utils
