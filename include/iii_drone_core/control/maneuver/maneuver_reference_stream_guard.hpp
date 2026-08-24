#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <iii_drone_interfaces/msg/maneuver_reference_stream.hpp>

namespace iii_drone::control::maneuver {

enum class ManeuverReferenceStreamDecision {
    NewActive,
    FreshHeld,
    Prepared,
    Paused,
    Invalid,
    Expired,
    WrongGeneration,
    OutOfOrder,
};

class ManeuverReferenceStreamGuard {
public:
    void reset() {
        stream_id_.clear();
        last_applied_sequence_ = 0;
        candidate_sequence_ = 0;
        successor_generation_expected_ = false;
    }

    void expectGeneration(const std::string & stream_id) {
        stream_id_ = stream_id;
        last_applied_sequence_ = 0;
        candidate_sequence_ = 0;
        successor_generation_expected_ = false;
    }

    void expectSuccessorGeneration() {
        successor_generation_expected_ = true;
    }

    ManeuverReferenceStreamDecision observe(
        const iii_drone_interfaces::msg::ManeuverReferenceStream & message,
        const rclcpp::Time & now
    ) {
        using Stream = iii_drone_interfaces::msg::ManeuverReferenceStream;
        if (!message.is_valid) {
            return ManeuverReferenceStreamDecision::Invalid;
        }
        const int64_t valid_until_ns =
            static_cast<int64_t>(message.valid_until.sec) * 1000000000LL +
            static_cast<int64_t>(message.valid_until.nanosec);
        if (now.nanoseconds() > valid_until_ns) {
            return ManeuverReferenceStreamDecision::Expired;
        }
        if (message.state == Stream::STATE_PREPARED) {
            return ManeuverReferenceStreamDecision::Prepared;
        }
        if (message.state == Stream::STATE_PAUSED) {
            return ManeuverReferenceStreamDecision::Paused;
        }
        if (message.state != Stream::STATE_ACTIVE) {
            return ManeuverReferenceStreamDecision::Invalid;
        }
        if (stream_id_.empty()) {
            stream_id_ = message.stream_id;
        }
        if (message.stream_id != stream_id_) {
            if (!successor_generation_expected_) {
                return ManeuverReferenceStreamDecision::WrongGeneration;
            }
            stream_id_ = message.stream_id;
            last_applied_sequence_ = 0;
            candidate_sequence_ = 0;
            successor_generation_expected_ = false;
        }
        if (message.sequence < last_applied_sequence_) {
            return ManeuverReferenceStreamDecision::OutOfOrder;
        }
        if (message.sequence == last_applied_sequence_) {
            return ManeuverReferenceStreamDecision::FreshHeld;
        }
        candidate_sequence_ = message.sequence;
        return ManeuverReferenceStreamDecision::NewActive;
    }

    void commitCandidate() {
        last_applied_sequence_ = candidate_sequence_;
    }

    const std::string & streamId() const { return stream_id_; }
    uint64_t lastAppliedSequence() const { return last_applied_sequence_; }
    uint64_t candidateSequence() const { return candidate_sequence_; }
    bool successorGenerationExpected() const { return successor_generation_expected_; }

private:
    std::string stream_id_;
    uint64_t last_applied_sequence_ = 0;
    uint64_t candidate_sequence_ = 0;
    bool successor_generation_expected_ = false;
};

}  // namespace iii_drone::control::maneuver
