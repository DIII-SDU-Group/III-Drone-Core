#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <iii_drone_interfaces/msg/maneuver_reference_stream.hpp>

namespace iii_drone::control::maneuver {

enum class ManeuverReferenceStreamDecision {
    NewActive,
    FreshHeld,
    AwaitingSuccessor,
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
        predecessor_updates_allowed_ = false;
    }

    void expectGeneration(const std::string & stream_id, uint64_t last_applied_sequence = 0) {
        stream_id_ = stream_id;
        last_applied_sequence_ = last_applied_sequence;
        candidate_sequence_ = last_applied_sequence;
        successor_generation_expected_ = false;
        predecessor_updates_allowed_ = false;
    }

    void expectSuccessorGeneration(bool allow_predecessor_updates = false) {
        successor_generation_expected_ = true;
        predecessor_updates_allowed_ = allow_predecessor_updates;
    }

    void cancelSuccessorGenerationExpectation() {
        successor_generation_expected_ = false;
        predecessor_updates_allowed_ = false;
        candidate_sequence_ = last_applied_sequence_;
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
        if (message.state != Stream::STATE_ACTIVE &&
            message.state != Stream::STATE_TERMINAL_DEGRADED &&
            message.state != Stream::STATE_TERMINAL_UNRECOVERABLE &&
            message.state != Stream::STATE_OBJECT_STOPPING &&
            message.state != Stream::STATE_OBJECT_STOPPED) {
            return ManeuverReferenceStreamDecision::Invalid;
        }
        if (stream_id_.empty()) {
            stream_id_ = message.stream_id;
        }
        if (
            successor_generation_expected_ &&
            message.stream_id == stream_id_ &&
            message.sequence > last_applied_sequence_ &&
            !predecessor_updates_allowed_
        ) {
            // Keep the predecessor command available while a goal response is
            // pending, but never let a late predecessor sample establish a
            // new baseline for the explicitly authorized successor.
            return ManeuverReferenceStreamDecision::AwaitingSuccessor;
        }
        if (message.stream_id != stream_id_) {
            if (!successor_generation_expected_) {
                return ManeuverReferenceStreamDecision::WrongGeneration;
            }
            stream_id_ = message.stream_id;
            last_applied_sequence_ = 0;
            candidate_sequence_ = 0;
            successor_generation_expected_ = false;
            predecessor_updates_allowed_ = false;
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
    bool predecessorUpdatesAllowed() const { return predecessor_updates_allowed_; }

private:
    std::string stream_id_;
    uint64_t last_applied_sequence_ = 0;
    uint64_t candidate_sequence_ = 0;
    bool successor_generation_expected_ = false;
    bool predecessor_updates_allowed_ = false;
};

}  // namespace iii_drone::control::maneuver
