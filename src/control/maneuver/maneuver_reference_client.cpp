/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::utils;
using namespace iii_drone::types;
using namespace iii_drone::adapters;
using namespace iii_drone::adapters::px4;
using namespace iii_drone::control;
using namespace iii_drone::configuration;

namespace {

bool finiteReference(const Reference & reference) {
    return reference.position().allFinite() &&
        reference.velocity().allFinite() &&
        reference.acceleration().allFinite() &&
        std::isfinite(reference.yaw()) &&
        std::isfinite(reference.yaw_rate()) &&
        std::isfinite(reference.yaw_acceleration());
}

const char * streamDecisionName(ManeuverReferenceStreamDecision decision) {
    switch (decision) {
    case ManeuverReferenceStreamDecision::Prepared: return "prepared";
    case ManeuverReferenceStreamDecision::Paused: return "paused";
    case ManeuverReferenceStreamDecision::FreshHeld: return "fresh_held";
    case ManeuverReferenceStreamDecision::AwaitingSuccessor: return "awaiting_successor";
    case ManeuverReferenceStreamDecision::NewActive: return "new_active";
    case ManeuverReferenceStreamDecision::Invalid: return "invalid";
    case ManeuverReferenceStreamDecision::Expired: return "expired";
    case ManeuverReferenceStreamDecision::WrongGeneration: return "wrong_generation";
    case ManeuverReferenceStreamDecision::OutOfOrder: return "out_of_order";
    }
    return "unknown";
}

int64_t rosTimeNs(const builtin_interfaces::msg::Time & time) {
    return static_cast<int64_t>(time.sec) * 1000000000LL + time.nanosec;
}

}  // namespace

void ManeuverReferenceClient::receiveReferenceStream(
    const iii_drone_interfaces::msg::ManeuverReferenceStream::SharedPtr message
) {
    const auto callback_start = std::chrono::steady_clock::now();
    auto callback_entry = iii_drone::diagnostics::HilTrace::event("callback_group_callback_entry");
    callback_entry.text("callback", "maneuver_reference_stream_subscription");
    callback_entry.text("callback_group", "mission_executor_reference_mutually_exclusive");
    callback_entry.text("callback_group_type", "MutuallyExclusive");
    callback_entry.text("node", "/mission_executor");
    callback_entry.commit();

    auto callback_exit = [&callback_start]() {
        const auto callback_end = std::chrono::steady_clock::now();
        auto event = iii_drone::diagnostics::HilTrace::event("callback_group_callback_exit");
        event.text("callback", "maneuver_reference_stream_subscription");
        event.text("callback_group", "mission_executor_reference_mutually_exclusive");
        event.text("callback_group_type", "MutuallyExclusive");
        event.text("node", "/mission_executor");
        event.number(
            "duration_ns",
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                callback_end - callback_start).count())
        );
        event.commit();
    };

    // Identity admission is intentionally before duplicate/cache bookkeeping.
    // A canceled request must not refresh DDS age, consume a guard candidate,
    // emit an ACK, or become the identity used for a later pause/rebase.
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    const bool recovery_owns_stream =
        reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP;
    if (!ownsReferenceStreamLocked(*message)) {
        auto rejected = iii_drone::diagnostics::HilTrace::event("reference_stream_identity_rejected");
        rejected.text("stream_id", message->stream_id);
        rejected.number("sequence", message->sequence);
        rejected.text("request_identity", message->request_identity);
        rejected.text("active_request_identity", active_request_identity_);
        rejected.text(
            "pending_request_identity",
            pending_goal_handoff_ ? pending_goal_handoff_->request_identity : ""
        );
        rejected.boolean("recovery_owns_stream", recovery_owns_stream);
        rejected.commit();
        callback_exit();
        return;
    }

    auto ingress = iii_drone::diagnostics::HilTrace::event("reference_stream_received");
    ingress.text("stream_id", message->stream_id);
    ingress.text("request_identity", message->request_identity);
    ingress.number("sequence", message->sequence);
    ingress.number("state", message->state);
    ingress.boolean("is_valid", message->is_valid);
    ingress.signed_number("produced_at_ns", rosTimeNs(message->produced_at));
    ingress.signed_number("valid_until_ns", rosTimeNs(message->valid_until));
    ingress.decimal("trajectory_time_s", message->trajectory_time_s);
    ingress.text("callback_group", "mission_executor_reference_mutually_exclusive");
    ingress.text("callback_group_type", "MutuallyExclusive");
    ingress.commit();

    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    if (
        latest_stream_message_ &&
        latest_stream_message_->stream_id == message->stream_id &&
        message->sequence <= latest_stream_message_->sequence
    ) {
        auto duplicate = iii_drone::diagnostics::HilTrace::event("reference_stream_duplicate_ignored");
        duplicate.text("stream_id", message->stream_id);
        duplicate.text("request_identity", message->request_identity);
        duplicate.number("sequence", message->sequence);
        duplicate.commit();
        callback_exit();
        return;
    }
    latest_stream_message_ = *message;
    latest_stream_received_at_ = std::chrono::steady_clock::now();
    callback_exit();
}

bool ManeuverReferenceClient::ownsReferenceStreamLocked(
    const iii_drone_interfaces::msg::ManeuverReferenceStream & message
) {
    if (!isValidManeuverRequestIdentity(message.request_identity)) {
        return false;
    }
    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        // Explicit recovery may rebase a stream ID, but only the identity
        // latched at the fault may drive that recovery.
        return fault_stream_identity_ &&
            fault_stream_identity_->request_identity == message.request_identity;
    }

    if (pending_goal_handoff_) {
        if (pending_goal_handoff_->request_identity == message.request_identity) {
            return true;
        }
        // A current request may still make progress while its successor is
        // pending, but only on the already-owned predecessor stream. Its
        // identity alone never authorizes another generation.
        return active_request_identity_ == message.request_identity &&
            message.stream_id == pending_goal_handoff_->predecessor_stream_id;
    }
    return active_request_identity_ == message.request_identity;
}

void ManeuverReferenceClient::retireInadmissibleCachedStreamLocked() {
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    if (latest_stream_message_ && !ownsReferenceStreamLocked(*latest_stream_message_)) {
        latest_stream_message_.reset();
    }
}

ManeuverReferenceClient::StreamReadResult
ManeuverReferenceClient::readReferenceStream(Reference & reference) {
    // Start/stop/handoff transitions mutate the generation guard and clear the
    // latest DDS sample.  Serialize the reader with those transitions so a
    // callback cannot observe a half-applied successor handoff and consume the
    // one-shot expectation with the previous maneuver generation.
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    iii_drone_interfaces::msg::ManeuverReferenceStream message;
    std::chrono::steady_clock::time_point received_at;
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (!latest_stream_message_) {
            auto event = iii_drone::diagnostics::HilTrace::event("reference_stream_read");
            event.text("decision", "unavailable_no_sample");
            event.commit();
            // Expected until the first sample of a new generation arrives;
            // sustained loss is reported by the reference-loss paths.
            RCLCPP_DEBUG_THROTTLE(
                logger_, *clock_, 1000,
                "ManeuverReferenceClient::readReferenceStream(): no stream sample is available"
            );
            return StreamReadResult::Unavailable;
        }
        message = *latest_stream_message_;
        received_at = latest_stream_received_at_;
    }

    const auto timeout = std::chrono::milliseconds(
        configuration_->GetParameter(
            "/control/maneuver_controller/reference_stream_timeout_ms"
        ).as_int()
    );
    const bool sample_fresh = std::chrono::steady_clock::now() - received_at <= timeout;

    if (!ownsReferenceStreamLocked(message)) {
        if (sample_fresh) noteStreamHeld("not_owned", message, timeout);
        retireInadmissibleCachedStreamLocked();
        auto event = iii_drone::diagnostics::HilTrace::event(
            "reference_stream_cached_identity_retired"
        );
        event.text("stream_id", message.stream_id);
        event.number("sequence", message.sequence);
        event.text("request_identity", message.request_identity);
        event.commit();
        return StreamReadResult::Unavailable;
    }

    if (!sample_fresh) {
        auto event = iii_drone::diagnostics::HilTrace::event("reference_stream_read");
        event.text("decision", "unavailable_stale");
        event.text("stream_id", message.stream_id);
        event.number("sequence", message.sequence);
        event.number("timeout_ms", static_cast<uint64_t>(timeout.count()));
        event.commit();
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "ManeuverReferenceClient::readReferenceStream(): latest stream sample is stale; stream=%s sequence=%lu timeout_ms=%ld",
            message.stream_id.c_str(),
            static_cast<unsigned long>(message.sequence),
            static_cast<long>(timeout.count())
        );
        return StreamReadResult::Unavailable;
    }
    const auto decision = reference_stream_guard_.observe(message, clock_->now());
    auto decision_event = iii_drone::diagnostics::HilTrace::event("reference_stream_guard_decision");
    decision_event.text("decision", streamDecisionName(decision));
    decision_event.text("stream_id", message.stream_id);
    decision_event.number("sequence", message.sequence);
    decision_event.number("candidate_sequence", reference_stream_guard_.candidateSequence());
    decision_event.number("last_applied_sequence", reference_stream_guard_.lastAppliedSequence());
    decision_event.number("state", message.state);
    decision_event.commit();
    if (decision == ManeuverReferenceStreamDecision::Prepared) {
        noteStreamHeld("prepared", message, timeout);
        reference = ReferenceAdapter(message.reference).reference();
        return StreamReadResult::Prepared;
    }
    if (decision == ManeuverReferenceStreamDecision::Paused) {
        return StreamReadResult::Paused;
    }
    if (
        decision == ManeuverReferenceStreamDecision::FreshHeld ||
        decision == ManeuverReferenceStreamDecision::AwaitingSuccessor
    ) {
        noteStreamHeld(streamDecisionName(decision), message, timeout);
        std::lock_guard<std::mutex> lock(reference_mutex_);
        reference = reference_;
        return StreamReadResult::FreshHeld;
    }
    if (decision != ManeuverReferenceStreamDecision::NewActive) {
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "ManeuverReferenceClient::readReferenceStream(): rejected stream sample; decision=%d stream=%s expected_stream=%s sequence=%lu last_applied=%lu state=%u valid=%s",
            static_cast<int>(decision),
            message.stream_id.c_str(),
            reference_stream_guard_.streamId().c_str(),
            static_cast<unsigned long>(message.sequence),
            static_cast<unsigned long>(reference_stream_guard_.lastAppliedSequence()),
            static_cast<unsigned int>(message.state),
            message.is_valid ? "true" : "false"
        );
        return StreamReadResult::Unavailable;
    }
    const bool predecessor_active =
        pending_goal_handoff_ &&
        message.request_identity == active_request_identity_ &&
        message.stream_id == pending_goal_handoff_->predecessor_stream_id;
    const bool new_active_stream =
        active_stream_id_ != reference_stream_guard_.streamId();
    active_stream_id_ = reference_stream_guard_.streamId();
    if (new_active_stream) {
        // A guard switches generations before the candidate is committed.
        // Keep the client-side acknowledgement identity in that same
        // generation: it must never carry a predecessor sequence into a
        // successor pause/rebase request.
        last_applied_sequence_ = reference_stream_guard_.lastAppliedSequence();
        if (terminal_hold_continuity_required_) {
            startup_reference_policy_.reset();
        } else {
            startup_reference_policy_.arm();
        }
    }
    reference = ReferenceAdapter(message.reference).reference();
    candidate_sequence_ = reference_stream_guard_.candidateSequence();
    candidate_request_identity_ = message.request_identity;
    candidate_stream_state_ = message.state;
    candidate_object_tracking_active_ = message.object_tracking_active &&
        (message.state == iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE ||
         message.state ==
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPING ||
         message.state ==
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED);
    if ((message.state ==
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPING ||
         message.state ==
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED) &&
        !message.object_tracking_active) {
        noteStreamHeld("object_stop_without_tracking", message, timeout);
        return StreamReadResult::Unavailable;
    }
    noteStreamConsumed();
    return predecessor_active ? StreamReadResult::PredecessorActive : StreamReadResult::NewActive;
}

void ManeuverReferenceClient::noteStreamConsumed() {
    stream_hold_since_.reset();
    stream_hold_reported_ = false;
}

void ManeuverReferenceClient::noteStreamHeld(
    const char * branch,
    const iii_drone_interfaces::msg::ManeuverReferenceStream & message,
    std::chrono::milliseconds timeout
) {
    // HIL soak run 25: the consumer acknowledged nothing of a fresh successor
    // stream for 1.5 s and the producer paused, without a log saying why.
    // Report once which branch held fresh samples for half a stream deadline.
    const auto now = std::chrono::steady_clock::now();
    if (!stream_hold_since_) {
        stream_hold_since_ = now;
        return;
    }
    if (stream_hold_reported_ || now - *stream_hold_since_ < timeout / 2) {
        return;
    }
    stream_hold_reported_ = true;
    // INFO: a long legitimate hold (a slow goal acceptance) must not fail the
    // strict qualification; a real stall is reported by the producer's ERROR.
    RCLCPP_INFO(
        logger_,
        "ManeuverReferenceClient::readReferenceStream(): applied no fresh stream sample for %.2f s; branch=%s "
        "sample_stream=%s sample_sequence=%lu sample_state=%u sample_request=%s guard_stream=%s "
        "guard_last_applied=%lu successor_expected=%s predecessor_updates=%s active_request=%s",
        std::chrono::duration<double>(now - *stream_hold_since_).count(),
        branch,
        message.stream_id.c_str(),
        static_cast<unsigned long>(message.sequence),
        static_cast<unsigned int>(message.state),
        message.request_identity.c_str(),
        reference_stream_guard_.streamId().c_str(),
        static_cast<unsigned long>(reference_stream_guard_.lastAppliedSequence()),
        reference_stream_guard_.successorGenerationExpected() ? "true" : "false",
        reference_stream_guard_.predecessorUpdatesAllowed() ? "true" : "false",
        active_request_identity_.c_str()
    );
}

ManeuverReferenceClient::ReferenceConsumption
ManeuverReferenceClient::consumeReferenceCandidate(
    Reference & reference,
    reference_mode_t expected_mode,
    bool mark_maneuver_reference_valid,
    bool transition_to_maneuver
) {
    // A candidate has no externally visible effect until its stream
    // generation, safety decision, cached reference, committed sequence, and
    // acknowledgement identity agree. Start/stop can otherwise splice a new
    // generation into the middle of that sequence.
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    ReferenceConsumption consumption;
    if (reference_mode_.Load() != expected_mode) {
        return consumption;
    }

    consumption.stream_result = readReferenceStream(reference);
    if (
        consumption.stream_result != StreamReadResult::NewActive &&
        consumption.stream_result != StreamReadResult::PredecessorActive
    ) {
        return consumption;
    }
    if (object_stopped_hold_ &&
        (expected_mode == reference_mode_t::HOVER ||
         consumption.stream_result == StreamReadResult::PredecessorActive)) {
        // A local stopped-object Hold may only consume the exact certified
        // rest stream. A late ACTIVE/foreign sample must not restart motion.
        if (!candidate_object_tracking_active_ ||
            candidate_stream_state_ !=
                iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED ||
            reference_stream_guard_.streamId() != object_stopped_hold_->stream_id ||
            candidate_request_identity_ != object_stopped_hold_->request_identity ||
            !reference.position().allFinite() || !reference.velocity().allFinite() ||
            !reference.acceleration().allFinite() || !std::isfinite(reference.yaw()) ||
            !std::isfinite(reference.yaw_rate()) ||
            !std::isfinite(reference.yaw_acceleration()) ||
            reference.velocity().norm() > 1.0e-5 ||
            reference.acceleration().norm() > 1.0e-5 ||
            std::abs(reference.yaw_rate()) > 1.0e-5 ||
            std::abs(reference.yaw_acceleration()) > 1.0e-5 ||
            (reference.position() - object_stopped_hold_->anchor.position()).norm() > 1.0e-5 ||
            std::abs(std::atan2(
                std::sin(reference.yaw() - object_stopped_hold_->anchor.yaw()),
                std::cos(reference.yaw() - object_stopped_hold_->anchor.yaw()))) > 1.0e-5) {
            return consumption;
        }
    }
    const bool successor_candidate =
        consumption.stream_result == StreamReadResult::NewActive;

    ManeuverReferenceSafetyEvaluation safety_evaluation;
    {
        std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
        if (
            successor_candidate &&
            startup_reference_policy_.consumeFirstPlannedBaseline(reference)
        ) {
            // The first planned baseline for this generation may be fully
            // finite or use HoverOnCable's velocity-only shape. The policy
            // consumes it exactly once; later samples use the existing
            // continuity envelope unchanged.
            reference_safety_guard_->reset();
            safety_evaluation = reference_safety_guard_->observeReference(reference);
        } else {
            // HIL soak run 19: right after the gripper closed, the vehicle
            // swung about the cable at up to 0.6 m/s; seeding from that
            // velocity failed HoverOnCable's (0, 0, 0.1) m/s reference. Its
            // shapes carry no position and are only used on the cable, where
            // the measured velocity is the swing, so they baseline themselves.
            if (
                !reference_safety_guard_->hasAcceptedReference() &&
                !vehicle_odometry_adapter_history_->empty() &&
                !ManeuverReferenceStartupPolicy::onCableShape(reference)
            ) {
                safety_evaluation = reference_safety_guard_->observeReference(
                    Reference((*vehicle_odometry_adapter_history_)[0].ToState())
                );
            }
            if (safety_evaluation.decision != ManeuverReferenceSafetyDecision::BEGIN_STOP) {
                safety_evaluation = reference_safety_guard_->observeReference(reference);
            }
        }
    }

    if (safety_evaluation.decision != ManeuverReferenceSafetyDecision::ACCEPT) {
        auto loss_stop = beginReferenceLossStopLocked(safety_evaluation);
        reference = loss_stop.reference;
        consumption.began_reference_loss_stop = true;
        consumption.pause_identity = std::move(loss_stop.pause_identity);
        return consumption;
    }

    {
        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
        reference_ = reference;
    }
    if (mark_maneuver_reference_valid) {
        maneuver_reference_valid_.Store(true);
    }
    last_applied_sequence_ = reference_stream_guard_.candidateSequence();
    reference_stream_guard_.commitCandidate();
    if (candidate_object_tracking_active_ &&
        candidate_sequence_ == reference_stream_guard_.lastAppliedSequence()) {
        applied_object_tracking_stream_id_ = reference_stream_guard_.streamId();
        applied_object_tracking_request_identity_ = candidate_request_identity_;
        applied_object_tracking_sequence_ = candidate_sequence_;
        applied_object_tracking_at_ = std::chrono::steady_clock::now();
        applied_object_tracking_state_ = candidate_stream_state_;
    } else {
        applied_object_tracking_stream_id_.clear();
        applied_object_tracking_request_identity_.clear();
        applied_object_tracking_sequence_ = 0;
        applied_object_tracking_state_ = 0;
    }
    consumption.applied_ack_identity = ReferenceStreamIdentity{
        reference_stream_guard_.streamId(),
        candidate_request_identity_,
        reference_stream_guard_.lastAppliedSequence(),
    };
    if (successor_candidate) {
        active_request_identity_ = candidate_request_identity_;
        if (object_stopped_hold_ &&
            object_stopped_hold_->stream_id != reference_stream_guard_.streamId()) {
            object_stopped_hold_.reset();
        }
    }
    if (
        pending_goal_handoff_ &&
        successor_candidate &&
        reference_stream_guard_.streamId() != pending_goal_handoff_->predecessor_stream_id
    ) {
        pending_goal_handoff_->successor_consumed = true;
        pending_goal_handoff_->successor_stream_id = reference_stream_guard_.streamId();
    }
    if (transition_to_maneuver && successor_candidate) {
        reference_mode_.Store(reference_mode_t::MANEUVER);
    }
    if (
        pending_goal_handoff_ &&
        pending_goal_handoff_->goal_accepted &&
        pending_goal_handoff_->successor_consumed
    ) {
        pending_goal_handoff_.reset();
    }
    consumption.accepted = true;
    {
        std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
        const bool applied_terminal_message = latest_stream_message_ &&
            latest_stream_message_->terminal_hold_active &&
            latest_stream_message_->stream_id == reference_stream_guard_.streamId() &&
            latest_stream_message_->request_identity == active_request_identity_ &&
            latest_stream_message_->sequence == reference_stream_guard_.lastAppliedSequence();
        if (applied_terminal_message) {
            applied_terminal_stream_id_ = latest_stream_message_->stream_id;
            applied_terminal_request_identity_ = latest_stream_message_->request_identity;
            applied_terminal_stream_state_ = latest_stream_message_->state;
        } else if (successor_candidate) {
            applied_terminal_stream_id_.clear();
            applied_terminal_request_identity_.clear();
            applied_terminal_stream_state_.reset();
        }
        consumption.terminal_degraded = applied_terminal_message &&
            (*applied_terminal_stream_state_ ==
                iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_DEGRADED ||
             *applied_terminal_stream_state_ ==
                iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_UNRECOVERABLE);
        consumption.object_unrecoverable = latest_stream_message_ &&
            latest_stream_message_->stream_id == reference_stream_guard_.streamId() &&
            latest_stream_message_->request_identity == active_request_identity_ &&
            latest_stream_message_->sequence == reference_stream_guard_.lastAppliedSequence() &&
            !latest_stream_message_->terminal_hold_active &&
            latest_stream_message_->state ==
                iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_UNRECOVERABLE;
    }
    return consumption;
}

void ManeuverReferenceClient::publishReferenceAck(
    uint8_t status,
    const std::string & detail
) {
    if (vehicle_odometry_adapter_history_->empty()) {
        return;
    }
    iii_drone_interfaces::msg::ManeuverReferenceAck ack;
    if (
        status != iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_STOPPED &&
        !active_stream_id_.empty()
    ) {
        ack.stream_id = active_stream_id_;
        ack.last_applied_sequence = last_applied_sequence_;
    } else {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (!latest_stream_message_) {
            return;
        }
        ack.stream_id = latest_stream_message_->stream_id;
        ack.last_applied_sequence = latest_stream_message_->sequence;
    }
    ack.applied_at = clock_->now();
    ack.consumer_identity = terminal_consumer_identity_;
    ack.vehicle_state = StateAdapter(
        (*vehicle_odometry_adapter_history_)[0].ToState()
    ).ToMsg();
    ack.consumer_status = status;
    ack.detail = detail;
    auto event = iii_drone::diagnostics::HilTrace::event("reference_ack_publication");
    event.text("stream_id", ack.stream_id);
    event.number("last_applied_sequence", ack.last_applied_sequence);
    event.number("consumer_status", ack.consumer_status);
    event.signed_number("applied_at_ns", rosTimeNs(ack.applied_at));
    event.text("detail", detail);
    event.commit();
    reference_ack_publisher_->publish(ack);
}

void ManeuverReferenceClient::publishReferenceAckForStream(
    const std::string & stream_id,
    uint64_t last_applied_sequence,
    uint8_t status,
    const std::string & detail
) {
    if (stream_id.empty() || vehicle_odometry_adapter_history_->empty()) {
        return;
    }
    iii_drone_interfaces::msg::ManeuverReferenceAck ack;
    ack.stream_id = stream_id;
    ack.consumer_identity = terminal_consumer_identity_;
    ack.last_applied_sequence = last_applied_sequence;
    ack.applied_at = clock_->now();
    ack.vehicle_state = StateAdapter(
        (*vehicle_odometry_adapter_history_)[0].ToState()
    ).ToMsg();
    ack.consumer_status = status;
    ack.detail = detail;
    auto event = iii_drone::diagnostics::HilTrace::event("reference_ack_publication");
    event.text("stream_id", ack.stream_id);
    event.number("last_applied_sequence", ack.last_applied_sequence);
    event.number("consumer_status", ack.consumer_status);
    event.signed_number("applied_at_ns", rosTimeNs(ack.applied_at));
    event.text("detail", detail);
    event.commit();
    reference_ack_publisher_->publish(ack);
}

void ManeuverReferenceClient::requestProducerPause(
    const ReferenceStreamIdentity & identity,
    const std::string & reason
) {
    if (!identity.valid()) {
        return;
    }
    publishReferenceAckForStream(
        identity.stream_id,
        identity.last_applied_sequence,
        iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_PAUSING,
        reason
    );
    if (!pause_reference_stream_client_->service_is_ready()) {
        return;
    }
    auto request = std::make_shared<iii_drone_interfaces::srv::PauseReferenceStream::Request>();
    request->stream_id = identity.stream_id;
    request->last_applied_sequence = identity.last_applied_sequence;
    request->reason = reason;
    pause_reference_stream_client_->async_send_request(request);
}

void ManeuverReferenceClient::resetReferenceStreamState() {
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        latest_stream_message_.reset();
    }
    active_stream_id_.clear();
    active_request_identity_.clear();
    object_stopped_hold_.reset();
    candidate_request_identity_.clear();
    prepared_stream_id_.clear();
    prepared_reference_anchor_.reset();
    last_applied_sequence_ = 0;
    candidate_sequence_ = 0;
    startup_reference_policy_.reset();
    reference_stream_guard_.reset();
    pending_goal_handoff_.reset();
    fault_stream_identity_.reset();
    recovery_phase_ = RecoveryPhase::None;
    pending_rebase_request_.reset();
    pending_commit_request_.reset();
}

ManeuverReferenceSafetyConfig ManeuverReferenceClient::referenceSafetyConfig() const {
    ManeuverReferenceSafetyConfig config;
    config.loss_timeout = std::chrono::milliseconds(
        configuration_->GetParameter("/mission/reference_loss_timeout_ms").as_int()
    );
    config.max_jerk_m_s3 = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_jerk_m_s3"
    ).as_double();
    config.max_yaw_jerk_rad_s3 = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3"
    ).as_double();
    config.position_tolerance_m = configuration_->GetParameter(
        "/mission/reference_continuity_position_tolerance_m"
    ).as_double();
    config.velocity_tolerance_m_s = configuration_->GetParameter(
        "/mission/reference_continuity_velocity_tolerance_m_s"
    ).as_double();
    config.acceleration_tolerance_m_s2 = configuration_->GetParameter(
        "/mission/reference_continuity_acceleration_tolerance_m_s2"
    ).as_double();
    config.yaw_tolerance_rad = configuration_->GetParameter(
        "/mission/reference_continuity_yaw_tolerance_rad"
    ).as_double();
    config.yaw_rate_tolerance_rad_s = configuration_->GetParameter(
        "/mission/reference_continuity_yaw_rate_tolerance_rad_s"
    ).as_double();
    config.yaw_acceleration_tolerance_rad_s2 = configuration_->GetParameter(
        "/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2"
    ).as_double();
    return config;
}

ControlledCancellationConfig ManeuverReferenceClient::referenceLossStopConfig() const {
    ControlledCancellationConfig config;
    config.limits.max_acceleration_m_s2 = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2"
    ).as_double();
    config.limits.max_jerk_m_s3 = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_jerk_m_s3"
    ).as_double();
    config.limits.max_yaw_acceleration_rad_s2 = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2"
    ).as_double();
    config.limits.max_yaw_jerk_rad_s3 = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3"
    ).as_double();
    config.velocity_threshold_m_s = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s"
    ).as_double();
    config.yaw_rate_threshold_rad_s = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s"
    ).as_double();
    config.settle_time_s = configuration_->GetParameter(
        "/control/maneuver_controller/controlled_cancel_settle_time_s"
    ).as_double();
    return config;
}

void ManeuverReferenceClient::resetReferenceSafety() {
    std::lock_guard<std::mutex> lock(reference_safety_mutex_);
    reference_safety_guard_.emplace(referenceSafetyConfig());
    reference_loss_stop_trajectory_.reset();
    reference_loss_below_threshold_since_.reset();
    reference_loss_failure_reported_ = false;
    reference_loss_reason_.clear();
}

ManeuverReferenceClient::ReferenceLossStopStart
ManeuverReferenceClient::beginReferenceLossStop(
    const ManeuverReferenceSafetyEvaluation & evaluation
) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    return beginReferenceLossStopLocked(evaluation);
}

ManeuverReferenceClient::ReferenceLossStopStart
ManeuverReferenceClient::beginReferenceLossStopLocked(
    const ManeuverReferenceSafetyEvaluation & evaluation
) {
    const ControlledCancellationConfig stop_config = referenceLossStopConfig();
    Reference initial;
    // A rejected reference is specifically evidence that the producer and the
    // vehicle may no longer agree.  Anchor the bounded stop in the measured
    // vehicle state whenever it is available, rather than in the last command.
    // This matters at the cable-release handoff: the retained pre-release
    // velocity can be non-zero even though PX4 has already settled on cable.
    // Starting a stop from that stale command stretches the recovery window and
    // can cause the otherwise-recoverable successor maneuver to time out.
    if (!vehicle_odometry_adapter_history_->empty()) {
        initial = Reference((*vehicle_odometry_adapter_history_)[0].ToState());
    }
    if (!finiteReference(initial)) {
        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
        initial = reference_;
    }
    if (!finiteReference(initial)) {
        throw std::runtime_error("cannot start reference-loss stop without a finite reference or vehicle state");
    }

    initial = Reference(
        initial.position(),
        initial.yaw(),
        initial.velocity(),
        initial.yaw_rate(),
        initial.acceleration(),
        initial.yaw_acceleration(),
        clock_->now()
    );

    double stop_duration_s = 0.0;
    {
        std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
        reference_loss_stop_trajectory_.emplace(initial, stop_config.limits);
        stop_duration_s = reference_loss_stop_trajectory_->durationS();
        const double ground = ground_altitude_estimate_.load();
        const double floor = ground + configuration_->GetParameter(
            "/control/maneuver_controller/minimum_target_altitude").as_double();
        const double end_z = reference_loss_stop_trajectory_->sample(stop_duration_s).position()(2);
        reference_loss_floor_breach_ =
            std::isfinite(floor) && std::isfinite(end_z) &&
            end_z < floor && end_z < initial.position()(2);
        reference_loss_floor_breach_detail_ = reference_loss_floor_breach_
            ? "bounded stop from z " + std::to_string(initial.position()(2)) +
              " at vz " + std::to_string(initial.velocity()(2)) +
              " would end at z " + std::to_string(end_z) +
              ", below the floor " + std::to_string(floor)
            : std::string();
        reference_loss_stop_start_time_ = std::chrono::steady_clock::now();
        reference_loss_below_threshold_since_.reset();
        reference_loss_failure_reported_ = false;
        reference_loss_reason_ = evaluation.reason;
    }
    clearPendingReferenceRequest();
    // A continuity fault transfers reference ownership to the bounded stop.
    // A late goal-acceptance callback must not revive the failed successor.
    pending_goal_handoff_.reset();
    recovery_phase_ = RecoveryPhase::Stopping;
    recovery_phase_started_ = std::chrono::steady_clock::now();
    reference_mode_.Store(reference_mode_t::REFERENCE_LOSS_STOP);

    // Capture the rejecting generation before any asynchronous pause/rebase
    // exchange. In particular, a rejected first successor sample has a
    // committed sequence of zero, never the predecessor's last sequence.
    fault_stream_identity_ = ReferenceStreamIdentity{
        reference_stream_guard_.streamId(),
        candidate_request_identity_,
        reference_stream_guard_.lastAppliedSequence(),
    };

    RCLCPP_ERROR(
        logger_,
        "ManeuverReferenceClient::beginReferenceLossStop(): %s after %.3f s; "
        "position %.3f/%.3f m, velocity %.3f/%.3f m/s, acceleration %.3f/%.3f m/s^2, "
        "yaw %.3f/%.3f rad, yaw_rate %.3f/%.3f rad/s, yaw_acceleration %.3f/%.3f rad/s^2. "
        "Rejecting further server references and starting %.3f s local bounded stop.",
        evaluation.reason.c_str(),
        evaluation.reference_age_s,
        evaluation.position_error_m,
        evaluation.position_limit_m,
        evaluation.velocity_error_m_s,
        evaluation.velocity_limit_m_s,
        evaluation.acceleration_error_m_s2,
        evaluation.acceleration_limit_m_s2,
        evaluation.yaw_error_rad,
        evaluation.yaw_limit_rad,
        evaluation.yaw_rate_error_rad_s,
        evaluation.yaw_rate_limit_rad_s,
        evaluation.yaw_acceleration_error_rad_s2,
        evaluation.yaw_acceleration_limit_rad_s2,
        stop_duration_s
    );
    ReferenceLossStopStart start;
    start.reference = initial;
    if (fault_stream_identity_->valid()) {
        start.pause_identity = *fault_stream_identity_;
    }
    return start;
}

Reference ManeuverReferenceClient::sampleReferenceLossStop(
    std::function<void()> on_fail_during_maneuver,
    std::string & reference_mode
) {
    const ControlledCancellationConfig stop_config = referenceLossStopConfig();
    const auto now = std::chrono::steady_clock::now();
    Reference reference;
    bool report_failure = false;
    std::string failure_reason;

    {
        std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
        if (!reference_loss_stop_trajectory_) {
            throw std::logic_error("reference-loss stop mode entered without a stop trajectory");
        }
        const double elapsed_s = std::chrono::duration<double>(
            now - reference_loss_stop_start_time_
        ).count();
        reference = reference_loss_stop_trajectory_->sample(elapsed_s, clock_->now());

        if (elapsed_s < reference_loss_stop_trajectory_->durationS()) {
            reference_loss_below_threshold_since_.reset();
        } else if (!vehicle_odometry_adapter_history_->empty()) {
            const State state = (*vehicle_odometry_adapter_history_)[0].ToState();
            const bool below_threshold =
                state.velocity().allFinite() &&
                state.angular_velocity().allFinite() &&
                state.velocity().norm() <= stop_config.velocity_threshold_m_s &&
                std::abs(state.angular_velocity()(2)) <= stop_config.yaw_rate_threshold_rad_s;
            if (!below_threshold) {
                reference_loss_below_threshold_since_.reset();
            } else if (!reference_loss_below_threshold_since_) {
                reference_loss_below_threshold_since_ = now;
            } else if (
                std::chrono::duration<double>(now - *reference_loss_below_threshold_since_).count() >=
                stop_config.settle_time_s
            ) {
                report_failure = !reference_loss_failure_reported_;
                reference_loss_failure_reported_ = true;
                failure_reason = reference_loss_reason_;
            }
        }
    }

    {
        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
        reference_ = reference;
    }

    bool floor_breach = false;
    std::string floor_detail;
    {
        std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
        floor_breach = reference_loss_floor_breach_;
        reference_loss_floor_breach_ = false;
        floor_detail = reference_loss_floor_breach_detail_;
    }
    if (floor_breach) {
        uint64_t failure_epoch;
        {
            std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
            failure_epoch = maneuver_failure_epoch_;
        }
        RCLCPP_ERROR(
            logger_,
            "ManeuverReferenceClient: %s; failing the maneuver into a measured Hover.",
            floor_detail.c_str()
        );
        on_fail_during_maneuver();
        const bool hovered = hoverIfFailureEpochUnchanged(failure_epoch);
        reference_mode = hovered ? "hover_after_reference_loss_floor" : currentReferenceModeLabel();
        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
        return reference_;
    }

    if (recovery_phase_ != RecoveryPhase::Stopping) {
        advanceReferenceRecovery(reference, std::move(on_fail_during_maneuver), reference_mode);
        return reference;
    }

    if (!report_failure) {
        reference_mode = "reference_loss_stopping";
        return reference;
    }

    (void)failure_reason;
    advanceReferenceRecovery(reference, std::move(on_fail_during_maneuver), reference_mode);
    return reference;
}

bool ManeuverReferenceClient::rebaseBudgetExhausted(
    const std::string & request_identity,
    std::chrono::steady_clock::time_point fault_at
) {
    const bool recurring =
        last_rebase_committed_at_ &&
        request_identity == last_rebase_request_identity_ &&
        std::chrono::duration<double>(fault_at - *last_rebase_committed_at_).count() <=
            kRebaseRecurrenceWindowS;
    recurring_rebases_ = recurring ? recurring_rebases_ + 1 : 0;
    return recurring_rebases_ >= kMaxRecurringRebases;
}

bool ManeuverReferenceClient::advanceReferenceRecovery(
    Reference & reference,
    std::function<void()> on_fail_during_maneuver,
    std::string & reference_mode
) {
    uint64_t failure_epoch;
    {
        std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
        failure_epoch = maneuver_failure_epoch_;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(
        configuration_->GetParameter(
            "/mission/reference_rebase_timeout_ms"
        ).as_int()
    );
    auto fail_recovery = [&](const std::string & reason) {
        RCLCPP_ERROR(
            logger_,
            "ManeuverReferenceClient: reference stream recovery failed: %s",
            reason.c_str()
        );
        on_fail_during_maneuver();
        const bool hovered = hoverIfFailureEpochUnchanged(failure_epoch);
        reference_mode = hovered ? "hover_after_reference_loss" : currentReferenceModeLabel();
        return false;
    };

    if (
        recovery_phase_ != RecoveryPhase::Stopping &&
        now - recovery_phase_started_ > timeout
    ) {
        return fail_recovery("rebase handshake timed out");
    }

    if (recovery_phase_ == RecoveryPhase::Stopping) {
        if (vehicle_odometry_adapter_history_->empty()) {
            return fail_recovery("vehicle state unavailable after bounded stop");
        }
        if (!fault_stream_identity_ || !fault_stream_identity_->valid()) {
            return fail_recovery("rejecting stream identity unavailable after bounded stop");
        }
        std::chrono::steady_clock::time_point fault_at;
        {
            std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
            fault_at = reference_loss_stop_start_time_;
        }
        if (rebaseBudgetExhausted(fault_stream_identity_->request_identity, fault_at)) {
            return fail_recovery(
                "continuity fault recurred after " + std::to_string(recurring_rebases_) +
                " consecutive rebases of the same maneuver"
            );
        }
        if (!rebase_reference_stream_client_->service_is_ready()) {
            return fail_recovery("rebase service unavailable after bounded stop");
        }
        const ReferenceStreamIdentity fault_identity = *fault_stream_identity_;
        publishReferenceAckForStream(
            fault_identity.stream_id,
            fault_identity.last_applied_sequence,
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_STOPPED,
            "bounded stop settled"
        );
        auto request = std::make_shared<iii_drone_interfaces::srv::RebaseReferenceStream::Request>();
        request->stream_id = fault_identity.stream_id;
        request->last_applied_sequence = fault_identity.last_applied_sequence;
        request->stopped_state = StateAdapter(
            (*vehicle_odometry_adapter_history_)[0].ToState()
        ).ToMsg();
        pending_rebase_request_.emplace(
            rebase_reference_stream_client_->async_send_request(request)
        );
        recovery_phase_ = RecoveryPhase::RebaseRequested;
        recovery_phase_started_ = now;
        reference_mode = "reference_rebase_requested";
        return true;
    }

    if (recovery_phase_ == RecoveryPhase::RebaseRequested) {
        if (
            !pending_rebase_request_ ||
            pending_rebase_request_->wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready
        ) {
            reference_mode = "reference_rebase_requested";
            return true;
        }
        const auto response = pending_rebase_request_->get();
        pending_rebase_request_.reset();
        if (!response->accepted) {
            return fail_recovery(response->reason);
        }
        if (response->abort_action) {
            const ReferenceStreamIdentity stopped_identity =
                fault_stream_identity_.value_or(ReferenceStreamIdentity{});
            RCLCPP_WARN(
                logger_,
                "ManeuverReferenceClient: producer requested action abort after bounded stop: %s",
                response->reason.c_str()
            );
            SetReferenceModeHover(true);
            publishReferenceAckForStream(
                stopped_identity.stream_id,
                stopped_identity.last_applied_sequence,
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_ACTION_ABORT_READY,
                "consumer entered hover; producer may abort action"
            );
            failed_attempts_ = 0;
            reference_mode = "hover_after_action_abort";
            return true;
        }
        prepared_stream_id_ = response->prepared_stream_id;
        recovery_phase_ = RecoveryPhase::WaitPrepared;
        recovery_phase_started_ = now;
    }

    if (recovery_phase_ == RecoveryPhase::WaitPrepared) {
        Reference prepared;
        if (readReferenceStream(prepared) != StreamReadResult::Prepared) {
            reference_mode = "wait_for_prepared_reference";
            return true;
        }
        iii_drone_interfaces::msg::ManeuverReferenceStream latest;
        {
            std::lock_guard<std::mutex> lock(reference_stream_mutex_);
            latest = *latest_stream_message_;
        }
        if (latest.stream_id != prepared_stream_id_) {
            reference_mode = "wait_for_prepared_reference";
            return true;
        }
        const State measured = (*vehicle_odometry_adapter_history_)[0].ToState();
        const double position_error = (prepared.position() - measured.position()).norm();
        const double yaw_error = std::abs(std::atan2(
            std::sin(prepared.yaw() - measured.yaw()),
            std::cos(prepared.yaw() - measured.yaw())
        ));
        const auto config = referenceLossStopConfig();
        if (
            position_error > referenceSafetyConfig().position_tolerance_m ||
            prepared.velocity().norm() > config.velocity_threshold_m_s ||
            yaw_error > referenceSafetyConfig().yaw_tolerance_rad ||
            std::abs(prepared.yaw_rate()) > config.yaw_rate_threshold_rad_s
        ) {
            return fail_recovery("prepared anchor does not match stopped vehicle state");
        }
        prepared_reference_anchor_ = prepared;
        publishReferenceAck(
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_STOPPED,
            "prepared anchor verified"
        );
        if (!commit_reference_stream_client_->service_is_ready()) {
            return fail_recovery("commit service unavailable");
        }
        auto request = std::make_shared<iii_drone_interfaces::srv::CommitReferenceStream::Request>();
        request->stream_id = prepared_stream_id_;
        request->prepared_sequence = latest.sequence;
        pending_commit_request_.emplace(
            commit_reference_stream_client_->async_send_request(request)
        );
        recovery_phase_ = RecoveryPhase::CommitRequested;
        recovery_phase_started_ = now;
        reference_mode = "reference_commit_requested";
        return true;
    }

    if (recovery_phase_ == RecoveryPhase::CommitRequested) {
        if (
            !pending_commit_request_ ||
            pending_commit_request_->wait_for(std::chrono::milliseconds(0)) !=
                std::future_status::ready
        ) {
            reference_mode = "reference_commit_requested";
            return true;
        }
        const auto response = pending_commit_request_->get();
        pending_commit_request_.reset();
        if (!response->accepted) {
            return fail_recovery(response->reason);
        }
        active_stream_id_ = prepared_stream_id_;
        last_applied_sequence_ = 0;
        reference_stream_guard_.expectGeneration(prepared_stream_id_);
        recovery_phase_ = RecoveryPhase::WaitActive;
        recovery_phase_started_ = now;
    }

    if (recovery_phase_ == RecoveryPhase::WaitActive) {
        Reference resumed;
        bool waiting_for_active = false;
        bool continuity_failed = false;
        std::optional<ReferenceStreamIdentity> applied_ack_identity;
        std::string resumed_stream_id;
        {
            // Recovery is also a stream consume operation. Keep its guard,
            // safety seed, candidate commit, and ACK identity together so a
            // Start/Stop transition cannot rebase a different generation.
            std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
            if (recovery_phase_ != RecoveryPhase::WaitActive) {
                waiting_for_active = true;
            } else if (readReferenceStream(resumed) != StreamReadResult::NewActive) {
                waiting_for_active = true;
            } else {
                ManeuverReferenceSafetyEvaluation evaluation;
                {
                    std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                    reference_safety_guard_->reset();
                    evaluation = reference_safety_guard_->observeReference(
                        *prepared_reference_anchor_
                    );
                    if (evaluation.decision == ManeuverReferenceSafetyDecision::ACCEPT) {
                        evaluation = reference_safety_guard_->observeReference(resumed);
                    }
                }
                if (evaluation.decision != ManeuverReferenceSafetyDecision::ACCEPT) {
                    continuity_failed = true;
                } else {
                    reference = resumed;
                    last_applied_sequence_ = reference_stream_guard_.candidateSequence();
                    reference_stream_guard_.commitCandidate();
                    {
                        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
                        reference_ = resumed;
                    }
                    applied_ack_identity = ReferenceStreamIdentity{
                        reference_stream_guard_.streamId(),
                        candidate_request_identity_,
                        reference_stream_guard_.lastAppliedSequence(),
                    };
                    resumed_stream_id = applied_ack_identity->stream_id;
                    active_request_identity_ = applied_ack_identity->request_identity;
                    last_rebase_request_identity_ = applied_ack_identity->request_identity;
                    last_rebase_committed_at_ = now;
                    recovery_phase_ = RecoveryPhase::None;
                    reference_loss_stop_trajectory_.reset();
                    reference_loss_failure_reported_ = false;
                    fault_stream_identity_.reset();
                    reference_mode_.Store(reference_mode_t::MANEUVER);
                }
            }
        }
        if (waiting_for_active) {
            reference_mode = "wait_for_rebased_reference";
            return true;
        }
        if (continuity_failed) {
            return fail_recovery("first committed reference failed continuity validation");
        }
        publishReferenceAckForStream(
            applied_ack_identity->stream_id,
            applied_ack_identity->last_applied_sequence,
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
            "rebased reference applied"
        );
        reference_mode = "maneuver_rebased";
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient: committed rebased stream %s; maneuver resumed from stopped state.",
            resumed_stream_id.c_str()
        );
        return true;
    }

    reference_mode = "reference_loss_stopping";
    return true;
}

/*****************************************************************************/
// Implementation
/*****************************************************************************/

void ManeuverReferenceClient::UpdateReference(bool force) {

    if (!force && isManeuverMode()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::UpdateReference(): Cannot update reference while in MANEUVER mode, returning.");
        return;
    }

    if (vehicle_odometry_adapter_history_->empty()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::UpdateReference(): Vehicle odometry adapter history is empty, returning.");
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::UpdateReference(): Updating hover reference with current state."
    );

    State state = (*vehicle_odometry_adapter_history_)[0].ToState();

    std::lock_guard<std::mutex> lock(reference_mutex_);
    
    if (configuration_->GetParameter("/mission/use_nans_when_hovering").as_bool()) {

        reference_ = Reference(
            state.position(),
            state.yaw(),
            vector_t::Constant(NAN),
            NAN,
            vector_t::Constant(NAN),
            NAN,
            state.stamp()
        );

    } else {

        reference_ = Reference(
            state.position(),
            state.yaw(),
            vector_t::Zero(),
            0,
            vector_t::Zero(),
            0,
            state.stamp()
        );

    }
}

void ManeuverReferenceClient::SetReference(Reference reference) {

    if (isManeuverMode()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::SetReference(): Cannot set reference while in a maneuver is active.");
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::SetReference(): Setting reference."
    );

    std::lock_guard<std::mutex> lock(reference_mutex_);

    reference_ = reference;

}

void ManeuverReferenceClient::SetReferenceModePassthrough() {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    auto reference_mode = reference_mode_.Load();

    if(isManeuverMode(reference_mode)) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::SetReferenceModePassthrough(): Cannot set reference mode to PASSTHROUGH while in a maneuver mode.");
        return;
    }

    if (reference_mode_.Load() == reference_mode_t::PASSTHROUGH) {
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::SetReferenceModePassthrough(): Setting reference mode to PASSTHROUGH."
    );

    reference_mode_.Store(reference_mode_t::PASSTHROUGH);
    resetReferenceSafety();
    clearPendingReferenceRequest();
    resetReferenceStreamState();

}

void ManeuverReferenceClient::SetReferenceModeHover(bool force) {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    auto reference_mode = reference_mode_.Load();

    if (!force && isManeuverMode(reference_mode)) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::SetReferenceModeHover(): Cannot set reference mode to HOVER while in a maneuver mode.");
        return;
    }

    if (!terminal_degraded_hold_ && !object_stop_failure_hold_) {
        UpdateReference(true);
    }

    if (reference_mode == reference_mode_t::HOVER) {
        resetReferenceSafety();
        clearPendingReferenceRequest();
        resetReferenceStreamState();
        if (!terminal_degraded_hold_ && !object_stop_failure_hold_) {
            terminal_consumer_identity_.clear();
            terminal_hold_continuity_required_ = false;
        }
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::SetReferenceModeHover(): Setting reference mode to HOVER."
    );

    reference_mode_.Store(reference_mode_t::HOVER);
    maneuver_reference_valid_.Store(false);
    resetReferenceSafety();
    clearPendingReferenceRequest();
    resetReferenceStreamState();
    if (!terminal_degraded_hold_ && !object_stop_failure_hold_) {
        terminal_consumer_identity_.clear();
        terminal_hold_continuity_required_ = false;
    }

}

uint64_t ManeuverReferenceClient::AcquireReferenceControl() {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    SetReferenceModeHover(true);
    reference_control_owner_ = ++next_reference_control_generation_;
    return reference_control_owner_;
}

std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Response>
ManeuverReferenceClient::requestTerminalHoldTransfer(
    const iii_drone_interfaces::srv::TerminalHoldTransfer::Request & request,
    int timeout_ms
) {
    if (!terminal_hold_transfer_client_ ||
        !terminal_hold_transfer_client_->service_is_ready()) return nullptr;
    auto pending = terminal_hold_transfer_client_->async_send_request(
        std::make_shared<iii_drone_interfaces::srv::TerminalHoldTransfer::Request>(request)
    );
    if (pending.wait_for(std::chrono::milliseconds(std::max(1, timeout_ms))) !=
        std::future_status::ready) {
        terminal_hold_transfer_client_->remove_pending_request(pending);
        return nullptr;
    }
    return pending.get();
}

ManeuverReferenceClient::TerminalHoldAdoption
ManeuverReferenceClient::TryAdoptTerminalHold(int timeout_ms) {
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    Transfer::Request query;
    query.operation = Transfer::Request::OP_QUERY;
    query.consumer_identity = nextProcessManeuverRequestIdentity();
    // HIL: Core answered a QUERY after 253 ms against a 250 ms budget and the
    // mode failed. The retained hold tolerates unacknowledged streaming for
    // reference_stream_timeout_ms; adoption may use half of that.
    const int stream_timeout_ms = static_cast<int>(configuration_->GetParameter(
        "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
    timeout_ms = std::max(timeout_ms, stream_timeout_ms / 2);
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::shared_ptr<Transfer::Response> offer;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        offer = requestTerminalHoldTransfer(query, std::max(1, static_cast<int>(remaining)));
        if (!offer) {
            RCLCPP_ERROR(logger_, "Terminal hold adoption QUERY unavailable");
            return TerminalHoldAdoption::Failed;
        }
        if (offer->accepted ||
            (offer->reason != "terminal callback finalizing" &&
             offer->reason != "terminal generation awaiting first applied acknowledgement")) break;
        const auto retry_budget = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (retry_budget.count() > 0) {
            std::this_thread::sleep_for(std::min(std::chrono::milliseconds(20), retry_budget));
        }
    }
    if (!offer || (!offer->accepted &&
        (offer->reason == "terminal callback finalizing" ||
         offer->reason == "terminal generation awaiting first applied acknowledgement"))) {
        RCLCPP_ERROR(logger_, "Terminal hold adoption QUERY timed out while Core finalized the exact owner");
        return TerminalHoldAdoption::Failed;
    }
    if (!offer->accepted) {
        if (offer->reason != "no retained terminal hold") {
            RCLCPP_ERROR(logger_, "Terminal hold adoption QUERY rejected: %s", offer->reason.c_str());
            return TerminalHoldAdoption::Failed;
        }
        std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
        terminal_degraded_hold_ = false;
        terminal_hold_continuity_required_ = false;
        terminal_consumer_identity_.clear();
        UpdateReference(true);
        return TerminalHoldAdoption::NoOffer;
    }
    const Reference anchor = ReferenceAdapter(offer->reference).reference();
    if (!finiteReference(anchor) ||
        !isValidManeuverRequestIdentity(offer->source_request_identity) ||
        offer->source_stream_id.empty() || offer->source_ack_sequence == 0) {
        RCLCPP_ERROR(logger_,
            "Terminal hold adoption QUERY returned an unusable offer (finite anchor %d, valid source %d, stream '%s', ack sequence %lu)",
            static_cast<int>(finiteReference(anchor)),
            static_cast<int>(isValidManeuverRequestIdentity(offer->source_request_identity)),
            offer->source_stream_id.c_str(),
            static_cast<unsigned long>(offer->source_ack_sequence));
        return TerminalHoldAdoption::Failed;
    }
    Transfer::Request claim;
    claim.operation = Transfer::Request::OP_CLAIM;
    claim.source_request_identity = offer->source_request_identity;
    claim.source_stream_id = offer->source_stream_id;
    claim.source_ack_sequence = offer->source_ack_sequence;
    claim.consumer_identity = query.consumer_identity;
    auto result = requestTerminalHoldTransfer(claim, timeout_ms);
    if (!result || !result->accepted ||
        result->source_stream_id != claim.source_stream_id ||
        result->source_request_identity != claim.source_request_identity ||
        result->source_ack_sequence < claim.source_ack_sequence) {
        RCLCPP_ERROR(logger_, "Terminal hold adoption CLAIM rejected: %s",
            result ? result->reason.c_str() : "service unavailable");
        return TerminalHoldAdoption::Failed;
    }
    const Reference claimed_anchor = ReferenceAdapter(result->reference).reference();
    if (!finiteReference(claimed_anchor)) {
        RCLCPP_ERROR(logger_, "Terminal hold adoption CLAIM returned a non-finite anchor");
        return TerminalHoldAdoption::Failed;
    }

    std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
    if (pending_goal_handoff_ || isManeuverMode()) {
        RCLCPP_ERROR(logger_,
            "Terminal hold adoption: claimed hold superseded locally (pending goal hand-off %d, maneuver mode %d)",
            static_cast<int>(pending_goal_handoff_.has_value()), static_cast<int>(isManeuverMode()));
        return TerminalHoldAdoption::Failed;
    }
    resetReferenceSafety();
    {
        std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
        if (reference_safety_guard_->observeReference(claimed_anchor).decision !=
            ManeuverReferenceSafetyDecision::ACCEPT) {
            RCLCPP_ERROR(logger_, "Terminal hold adoption: claimed anchor rejected by the reference safety guard");
            return TerminalHoldAdoption::Failed;
        }
    }
    {
        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
        reference_ = claimed_anchor;
    }
    {
        std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
        latest_stream_message_.reset();
    }
    terminal_consumer_identity_ = claim.consumer_identity;
    terminal_hold_continuity_required_ = true;
    terminal_degraded_hold_ = false;
    active_request_identity_ = claim.source_request_identity;
    active_stream_id_ = claim.source_stream_id;
    reference_stream_guard_.expectGeneration(claim.source_stream_id, result->source_ack_sequence);
    startup_reference_policy_.reset();
    reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_START);
    object_stop_.reset();
    object_stop_failure_hold_ = false;
    maneuver_reference_valid_.Store(true);
    maneuver_start_time_.Store(rclcpp::Clock().now());
    return TerminalHoldAdoption::Adopted;
}

ManeuverReferenceClient::TerminalHoldRetention
ManeuverReferenceClient::RetainCompletedTerminalHold(
    const std::string & request_identity, int timeout_ms
) {
    using Transfer = iii_drone_interfaces::srv::TerminalHoldTransfer;
    if (!isValidManeuverRequestIdentity(request_identity)) return TerminalHoldRetention::Failed;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::string last_pending_reason;
    do {
        Transfer::Request query;
        query.operation = Transfer::Request::OP_QUERY;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        auto offer = requestTerminalHoldTransfer(
            query, std::max(1, std::min(100, static_cast<int>(remaining))));
        if (offer && !offer->accepted) {
            if (offer->reason == "terminal action has not completed" ||
                offer->reason == "terminal callback finalizing" ||
                offer->reason == "terminal generation awaiting first applied acknowledgement") {
                // Completion/token return and the first published generation
                // are bounded transitions. None substitutes for an APPLIED ACK.
                last_pending_reason = offer->reason;
                const auto retry_budget = std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - std::chrono::steady_clock::now());
                if (retry_budget.count() > 0) {
                    std::this_thread::sleep_for(std::min(std::chrono::milliseconds(20), retry_budget));
                }
                continue;
            }
            if (offer->reason != "no retained terminal hold") {
                RCLCPP_ERROR(logger_,
                    "Terminal hold retention QUERY rejected for request %s: %s",
                    request_identity.c_str(), offer->reason.c_str());
            }
            return offer->reason == "no retained terminal hold"
                ? TerminalHoldRetention::NoOffer : TerminalHoldRetention::Failed;
        }
        if (offer && offer->accepted &&
            offer->source_request_identity == request_identity &&
            finiteReference(ReferenceAdapter(offer->reference).reference())) {
            std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
            if (pending_goal_handoff_ &&
                pending_goal_handoff_->request_identity != request_identity) {
                RCLCPP_ERROR(logger_, "Terminal hold retention lost pending request ownership: %s",
                    request_identity.c_str());
                return TerminalHoldRetention::Failed;
            }
            if (!pending_goal_handoff_ && active_request_identity_ != request_identity) {
                RCLCPP_ERROR(logger_, "Terminal hold retention lost active request ownership: %s",
                    request_identity.c_str());
                return TerminalHoldRetention::Failed;
            }
            pending_goal_handoff_.reset();
            active_request_identity_ = request_identity;
            reference_mode_.Store(reference_mode_t::MANEUVER);
            terminal_hold_continuity_required_ = true;
            return TerminalHoldRetention::Retained;
        }
        if (offer && offer->accepted) {
            RCLCPP_ERROR(logger_,
                "Terminal hold retention offer identity or reference invalid: request=%s offered=%s",
                request_identity.c_str(), offer->source_request_identity.c_str());
            return TerminalHoldRetention::Failed;
        }
        const auto retry_budget = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (retry_budget.count() > 0) {
            std::this_thread::sleep_for(std::min(std::chrono::milliseconds(20), retry_budget));
        }
    } while (std::chrono::steady_clock::now() < deadline);
    RCLCPP_ERROR(logger_,
        "Terminal hold retention QUERY timed out for request %s (last pending reason: %s)",
        request_identity.c_str(),
        last_pending_reason.empty() ? "service unavailable" : last_pending_reason.c_str());
    return TerminalHoldRetention::Failed;
}

bool ManeuverReferenceClient::terminalHoldContinuityRequired() const {
    return terminal_hold_continuity_required_.load();
}

void ManeuverReferenceClient::ResetTerminalRetentionFailure() {
    std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
    terminal_retention_failure_owner_ = 0;
    terminal_retention_failure_request_.clear();
}

bool ManeuverReferenceClient::ReportTerminalRetentionFailure(
    const std::string & request_identity
) {
    std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
    if (!isValidManeuverRequestIdentity(request_identity) ||
        reference_control_owner_ == 0 ||
        (active_request_identity_ != request_identity &&
         (!pending_goal_handoff_ ||
          pending_goal_handoff_->request_identity != request_identity))) return false;
    terminal_retention_failure_owner_ = reference_control_owner_;
    terminal_retention_failure_request_ = request_identity;
    return true;
}

bool ManeuverReferenceClient::TerminalRetentionFailed() {
    std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
    return terminal_retention_failure_owner_ != 0 &&
        terminal_retention_failure_owner_ == reference_control_owner_ &&
        !terminal_retention_failure_request_.empty();
}

bool ManeuverReferenceClient::ReleaseReferenceControl(uint64_t owner_generation) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    if (owner_generation == 0 || owner_generation != reference_control_owner_) {
        return false;
    }
    SetReferenceModeHover(true);
    reference_control_owner_ = 0;
    return true;
}

bool ManeuverReferenceClient::ReleaseConsumerControl(uint8_t reason, uint8_t px4_nav_state) {
    {
        std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
        // Hover also drops any pending goal handoff and its stream guard; a
        // late acceptance of that goal can no longer adopt a generation.
        SetReferenceModeHover(true);
        reference_control_owner_ = 0;
        if (*stop_maneuver_timer_ != nullptr) {
            (*stop_maneuver_timer_)->cancel();
            stop_maneuver_timer_.Store(nullptr);
        }
        ++stop_maneuver_timer_generation_;
    }

    const ManeuverRequestScope scope = processManeuverRequestScope();
    if (!scope.valid()) {
        RCLCPP_INFO(logger_,
            "ManeuverReferenceClient::ReleaseConsumerControl(): No maneuver request was issued; nothing to release in Core");
        return false;
    }
    if (!release_consumer_control_client_ ||
        !release_consumer_control_client_->service_is_ready()) {
        RCLCPP_WARN(logger_,
            "ManeuverReferenceClient::ReleaseConsumerControl(): Core release service unavailable; "
            "Core falls back to PX4 native-navigation retirement");
        return false;
    }
    auto request = std::make_shared<iii_drone_interfaces::srv::ReleaseConsumerControl::Request>();
    request->producer_epoch = scope.epoch;
    request->last_request_counter = scope.last_counter;
    request->reason = reason;
    request->px4_nav_state = px4_nav_state;
    auto logger = logger_;
    release_consumer_control_client_->async_send_request(
        request,
        [logger](rclcpp::Client<iii_drone_interfaces::srv::ReleaseConsumerControl>::SharedFuture future) {
            const auto response = future.get();
            if (response->accepted) {
                RCLCPP_INFO(logger,
                    "ManeuverReferenceClient::ReleaseConsumerControl(): Core released consumer: "
                    "%u queued cleared, %u executing released, %u retained owner retired",
                    response->cleared_queued_count, response->released_active_count,
                    response->retired_owner_count);
            } else {
                RCLCPP_WARN(logger,
                    "ManeuverReferenceClient::ReleaseConsumerControl(): Core refused release: %s",
                    response->reason.c_str());
            }
        });
    auto event = iii_drone::diagnostics::HilTrace::event("consumer_control_release_requested");
    event.text("producer_epoch", scope.epoch);
    event.number("last_request_counter", scope.last_counter);
    event.number("reason", reason);
    event.number("px4_nav_state", px4_nav_state);
    event.commit();
    return true;
}

bool ManeuverReferenceClient::hoverIfFailureEpochUnchanged(uint64_t observed_epoch) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    if (maneuver_failure_epoch_ != observed_epoch) {
        return false;
    }
    if (currentTerminalStreamState() || ownsObjectStoppedHold()) {
        // An exact, applied Core terminal owner can outlive a successor that
        // never produced its first sample. A timeout must not turn that
        // retained finite command into a newly measured Hover reference.
        // Explicit mode release and Core's terminal lifecycle remain able to
        // retire this owner.
        return false;
    }
    if (isManeuverMode(reference_mode_.Load())) {
        SetReferenceModeHover(true);
    }
    if (reference_mode_.Load() == reference_mode_t::HOVER) {
        failed_attempts_ = 0;
        return true;
    }
    return false;
}

std::string ManeuverReferenceClient::currentReferenceModeLabel() const {
    switch (reference_mode_.Load()) {
        case reference_mode_t::PASSTHROUGH: return "passthrough";
        case reference_mode_t::HOVER: return "hover";
        case reference_mode_t::WAIT_FOR_MANEUVER_START: return "wait_for_maneuver_start";
        case reference_mode_t::MANEUVER: return "maneuver";
        case reference_mode_t::WAIT_FOR_MANEUVER_STOP: return "wait_for_maneuver_stop";
        case reference_mode_t::REFERENCE_LOSS_STOP: return "reference_loss_stopping";
    }
    return "hover";
}

bool ManeuverReferenceClient::StartManeuver() {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (pending_goal_handoff_) {
        RCLCPP_ERROR(
            logger_,
            "ManeuverReferenceClient::StartManeuver(): Cannot replace a pending goal handoff."
        );
        return false;
    }
    auto reference_mode = reference_mode_.Load();

    if (reference_mode == WAIT_FOR_MANEUVER_START || reference_mode == MANEUVER) {
        RCLCPP_ERROR(logger_, "ManeuverReferenceClient::StartManeuver(): Cannot start maneuver while a maneuver mode is already active");
        return false;
    }

    if (reference_mode == WAIT_FOR_MANEUVER_STOP) {
        RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::StartManeuver(): Stopping currently waiting maneuver.");
        stopManeuverPrematurely();
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StartManeuver(): Starting maneuver."
    );

    if (reference_mode == PASSTHROUGH) {
        UpdateReference();
    } else if (reference_mode != HOVER && reference_mode != WAIT_FOR_MANEUVER_STOP) {
        RCLCPP_ERROR(logger_, "ManeuverReferenceClient::StartManeuver(): Reference mode is not PASSTHROUGH or HOVER and not a MANEUVER mode.");
        return false;
    }

    clearPendingReferenceRequest();
    resetReferenceSafety();
    const bool has_prior_generation = !reference_stream_guard_.streamId().empty();
    if (has_prior_generation) {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        latest_stream_message_.reset();
    } else {
        resetReferenceStreamState();
    }
    active_stream_id_.clear();
    active_request_identity_.clear();
    candidate_request_identity_.clear();
    prepared_stream_id_.clear();
    prepared_reference_anchor_.reset();
    last_applied_sequence_ = 0;
    candidate_sequence_ = 0;
    if (terminal_hold_continuity_required_) {
        startup_reference_policy_.reset();
    } else {
        startup_reference_policy_.arm();
    }
    fault_stream_identity_.reset();
    recovery_phase_ = RecoveryPhase::None;
    pending_rebase_request_.reset();
    pending_commit_request_.reset();
    if (has_prior_generation) {
        // Keep the completed generation as the explicit predecessor.  Late
        // samples from it may still be in DDS queues after StopManeuver(); if
        // the guard were reset, one such sample could become the new baseline
        // and make the real successor look like an unauthorized generation.
        reference_stream_guard_.expectSuccessorGeneration();
    }
    object_stop_.reset();
    object_stop_failure_hold_ = false;
    reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_START);
    ++maneuver_failure_epoch_;
    maneuver_reference_valid_.Store(false);

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
    }

    maneuver_start_time_.Store(rclcpp::Clock().now());

    return true;

}

bool ManeuverReferenceClient::PrepareManeuverStreamHandoff() {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    const auto reference_mode = reference_mode_.Load();
    if (
        reference_mode != reference_mode_t::WAIT_FOR_MANEUVER_START &&
        reference_mode != reference_mode_t::MANEUVER &&
        reference_mode != reference_mode_t::WAIT_FOR_MANEUVER_STOP
    ) {
        RCLCPP_ERROR(
            logger_,
            "ManeuverReferenceClient::PrepareManeuverStreamHandoff(): "
            "Cannot prepare handoff without an active maneuver reference stream."
        );
        return false;
    }

    reference_stream_guard_.expectSuccessorGeneration(true);
    RCLCPP_DEBUG(
        logger_,
        "ManeuverReferenceClient::PrepareManeuverStreamHandoff(): "
        "Expecting one continuity-checked successor reference generation."
    );
    return true;
}

bool ManeuverReferenceClient::BeginManeuverGoalHandoff(
    const std::string & request_identity,
    bool preserve_active_predecessor_on_cancel
) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (!isValidManeuverRequestIdentity(request_identity)) {
        RCLCPP_ERROR(
            logger_,
            "ManeuverReferenceClient::BeginManeuverGoalHandoff(): Refusing malformed request identity."
        );
        return false;
    }
    if (pending_goal_handoff_ || object_stop_ ||
        reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_ERROR(
            logger_,
            "ManeuverReferenceClient::BeginManeuverGoalHandoff(): Another handoff or bounded recovery owns the stream."
        );
        return false;
    }

    const auto reference_mode = reference_mode_.Load();
    PendingManeuverGoalHandoff handoff;
    handoff.request_identity = request_identity;
    handoff.predecessor_stream_id = reference_stream_guard_.streamId();
    handoff.predecessor_was_running = reference_mode == reference_mode_t::MANEUVER;
    handoff.preserve_active_predecessor_on_cancel = preserve_active_predecessor_on_cancel;
    pending_goal_handoff_ = std::move(handoff);
    ++maneuver_failure_epoch_;

    // When an active predecessor exists, retain it as a valid G1 producer but
    // grant exactly one G2 only to this request identity. For an initial goal,
    // the empty guard will adopt only this already-bound identity at ingress.
    if (!reference_stream_guard_.streamId().empty()) {
        reference_stream_guard_.expectSuccessorGeneration(true);
    }
    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }
    ++stop_maneuver_timer_generation_;

    auto event = iii_drone::diagnostics::HilTrace::event("reference_goal_handoff_begin");
    event.text("request_identity", request_identity);
    event.text("predecessor_stream_id", pending_goal_handoff_->predecessor_stream_id);
    event.text("current_stream_id", reference_stream_guard_.streamId());
    event.boolean("early_successor_consumed", false);
    event.boolean("preserve_active_predecessor_on_cancel", preserve_active_predecessor_on_cancel);
    event.commit();
    return true;
}

bool ManeuverReferenceClient::ConfirmManeuverGoalHandoff(const std::string & request_identity) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (!pending_goal_handoff_ ||
        pending_goal_handoff_->request_identity != request_identity) {
        return false;
    }
    if (pending_goal_handoff_->goal_accepted) {
        return false;
    }
    pending_goal_handoff_->goal_accepted = true;
    const PendingManeuverGoalHandoff handoff = *pending_goal_handoff_;
    const bool owned_stop_waiting_for_completion =
        reference_mode_.Load() == reference_mode_t::WAIT_FOR_MANEUVER_STOP &&
        *stop_maneuver_timer_ != nullptr;
    if (handoff.successor_consumed) {
        maneuver_reference_valid_.Store(true);
        if (!owned_stop_waiting_for_completion) {
            reference_mode_.Store(reference_mode_t::MANEUVER);
        }
        pending_goal_handoff_.reset();
    } else if (owned_stop_waiting_for_completion) {
        // A terminal action may schedule its bounded stop before Confirm.
        // Keep that owned timer and WAIT_STOP mode so its original expiry
        // remains authoritative while this request awaits its first sample.
    } else if (
        handoff.preserve_active_predecessor_on_cancel &&
        handoff.predecessor_was_running
    ) {
        // A blended goal attaches to a deliberately preserved maneuver. Its
        // G1 remains the active control stream until G2 commits.
        reference_mode_.Store(reference_mode_t::MANEUVER);
    } else {
        // Keep G1's committed identity and safety history live while the
        // accepted goal awaits its explicitly authorized G2. G1 can be
        // acknowledged, but readReferenceStream classifies it separately so
        // it cannot satisfy this new maneuver start.
        reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_START);
    }
    maneuver_start_time_.Store(rclcpp::Clock().now());

    auto event = iii_drone::diagnostics::HilTrace::event("reference_goal_handoff_confirm");
    event.text("request_identity", handoff.request_identity);
    event.text("predecessor_stream_id", handoff.predecessor_stream_id);
    event.text("current_stream_id", reference_stream_guard_.streamId());
    event.boolean("early_successor_consumed", handoff.successor_consumed);
    event.commit();
    return true;
}

bool ManeuverReferenceClient::CancelManeuverGoalHandoff(const std::string & request_identity) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        return false;
    }
    // A pending successor has priority over its predecessor's still active
    // stream. Once Confirm has consumed G2, the pending slot is gone and the
    // committed active identity is the remaining terminal owner.
    if (!pending_goal_handoff_) {
        if (!isValidManeuverRequestIdentity(request_identity) ||
            active_request_identity_ != request_identity) {
            return false;
        }
        if (beginObjectStopLocked(request_identity)) return true;
        StopManeuver();
        return true;
    }
    if (pending_goal_handoff_->request_identity != request_identity) {
        return false;
    }
    const PendingManeuverGoalHandoff handoff = *pending_goal_handoff_;
    pending_goal_handoff_.reset();
    retireInadmissibleCachedStreamLocked();

    auto event = iii_drone::diagnostics::HilTrace::event("reference_goal_handoff_cancel");
    event.text("request_identity", handoff.request_identity);
    event.text("predecessor_stream_id", handoff.predecessor_stream_id);
    event.text("current_stream_id", reference_stream_guard_.streamId());
    event.boolean("early_successor_consumed", handoff.successor_consumed);
    event.commit();

    if (!handoff.successor_consumed) {
        reference_stream_guard_.cancelSuccessorGenerationExpectation();
        if (ownsObjectStoppedHold() &&
            handoff.predecessor_stream_id == object_stopped_hold_->stream_id) {
            // The unstarted successor never displaced this exact certified
            // rest. Resume its local Hold and continuing producer ACKs.
            reference_mode_.Store(reference_mode_t::HOVER);
            return true;
        }
        if (
            !handoff.goal_accepted &&
            handoff.preserve_active_predecessor_on_cancel &&
            handoff.predecessor_was_running
        ) {
            reference_mode_.Store(reference_mode_t::MANEUVER);
            return true;
        }
    }

    // An ordinary successor, a delayed-stop predecessor, or any successor
    // that already affected control takes the existing terminal safe path.
    StopManeuver();
    return true;
}

bool ManeuverReferenceClient::CompleteManeuverGoalHandoff(
    const std::string & request_identity,
    Reference final_reference
) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    if (!isValidManeuverRequestIdentity(request_identity) ||
        reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP ||
        (pending_goal_handoff_
            ? pending_goal_handoff_->request_identity != request_identity
            : active_request_identity_ != request_identity)) {
        return false;
    }
    if (isManeuverMode()) {
        if (!pending_goal_handoff_ &&
            currentAppliedObjectTrackingStream(request_identity)) {
            // The result's nominal target is metadata. The marked, applied
            // moving command remains owned until the next request takes it.
            return true;
        }
        StopManeuver(final_reference);
        // A retained predecessor terminal correction can defer this stop.
        // The reported nominal object target is metadata, not authority to
        // discard that command or the still-pending successor identity.
        if ((pending_goal_handoff_ &&
             pending_goal_handoff_->request_identity == request_identity) ||
            (active_request_identity_ == request_identity && isManeuverMode())) {
            RCLCPP_ERROR(logger_,
                "ManeuverReferenceClient::CompleteManeuverGoalHandoff(): "
                "Stop was deferred; request %s still owns a pending or active stream",
                request_identity.c_str());
            return false;
        }
    } else {
        // An action may complete before its first reference is consumed.
        // Preserve its reported target while retiring the pending identity.
        SetReferenceModeHover(true);
        SetReference(final_reference);
    }
    return true;
}

bool ManeuverReferenceClient::CompleteManeuverGoalHandoff(
    const std::string & request_identity
) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    if (!isValidManeuverRequestIdentity(request_identity) ||
        reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP ||
        (pending_goal_handoff_
            ? pending_goal_handoff_->request_identity != request_identity
            : active_request_identity_ != request_identity)) {
        return false;
    }

    bool marked_generation =
        applied_object_tracking_request_identity_ == request_identity;
    {
        std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
        marked_generation = marked_generation ||
            (latest_stream_message_ &&
             latest_stream_message_->request_identity == request_identity &&
             latest_stream_message_->object_tracking_active);
    }
    if (marked_generation) {
        // A successful marked result may leave its moving command live only
        // when this same request has actually consumed a fresh marked sample.
        // The next goal then asks Core for its certified rest transition.
        if (pending_goal_handoff_ || object_stop_ || object_stop_failure_hold_ ||
            reference_mode_.Load() != reference_mode_t::MANEUVER ||
            !currentAppliedObjectTrackingStream(request_identity)) {
            RCLCPP_ERROR(logger_,
                "ManeuverReferenceClient::CompleteManeuverGoalHandoff(): "
                "Request %s lacks a fresh applied object generation",
                request_identity.c_str());
            return false;
        }
        return true;
    }

    // Nontracking HoverByObject keeps its historical no-reference cleanup.
    return CancelManeuverGoalHandoff(request_identity);
}

bool ManeuverReferenceClient::StopManeuverGoalHandoffAfterTimeout(
    const std::string & request_identity, int timeout_ms
) {
    return scheduleOwnedManeuverStop(request_identity, std::nullopt, timeout_ms);
}

bool ManeuverReferenceClient::StopManeuverGoalHandoffAfterTimeout(
    const std::string & request_identity, Reference final_reference, int timeout_ms
) {
    return scheduleOwnedManeuverStop(
        request_identity, std::move(final_reference), timeout_ms
    );
}

bool ManeuverReferenceClient::scheduleOwnedManeuverStop(
    const std::string & request_identity,
    std::optional<Reference> final_reference,
    int timeout_ms
) {
    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    if (!isValidManeuverRequestIdentity(request_identity) ||
        reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP ||
        (pending_goal_handoff_
            ? pending_goal_handoff_->request_identity != request_identity
            : active_request_identity_ != request_identity)) {
        return false;
    }

    // A nonpositive timeout remains immediate. A positive owned stop may
    // outlive the first reference from its accepted pending goal; WAIT_STOP
    // will consume and acknowledge that authorized sample under this owner.
    if (timeout_ms <= 0 || !isManeuverMode()) {
        if (beginObjectStopLocked(request_identity)) return true;
        return final_reference
            ? CompleteManeuverGoalHandoff(request_identity, *final_reference)
            : CancelManeuverGoalHandoff(request_identity);
    }

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }
    const uint64_t generation = ++stop_maneuver_timer_generation_;
    reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_STOP);

    // The callback carries its original owner and generation. A canceled
    // timer already dispatched by ROS cannot borrow a later timer's callback.
    const std::function<void()> callback = [this, request_identity, generation, final_reference]() {
        std::lock_guard<std::recursive_mutex> callback_lock(transition_mutex_);
        if (generation != stop_maneuver_timer_generation_ ||
            reference_mode_.Load() != reference_mode_t::WAIT_FOR_MANEUVER_STOP ||
            (pending_goal_handoff_
                ? pending_goal_handoff_->request_identity != request_identity
                : active_request_identity_ != request_identity)) {
            return;
        }
        if (beginObjectStopLocked(request_identity)) return;
        if (final_reference) {
            CompleteManeuverGoalHandoff(request_identity, *final_reference);
        } else {
            CancelManeuverGoalHandoff(request_identity);
        }
    };
    stop_maneuver_timer_callback_ = callback;
    stop_maneuver_timer_ = create_wall_timer_(
        std::chrono::milliseconds(timeout_ms), callback
    );
    return true;
}

bool ManeuverReferenceClient::IsManeuverActive() {

    return isManeuverMode();

}

std::optional<uint8_t> ManeuverReferenceClient::currentTerminalStreamState() {
    if (!applied_terminal_stream_state_ ||
        applied_terminal_stream_id_ != active_stream_id_ ||
        applied_terminal_request_identity_ != active_request_identity_ ||
        last_applied_sequence_ == 0) {
        return std::nullopt;
    }
    return applied_terminal_stream_state_;
}

bool ManeuverReferenceClient::currentAppliedObjectTrackingStream(
    const std::string & request_identity) {
    if (request_identity.empty() || active_request_identity_ != request_identity ||
        active_stream_id_.empty() || last_applied_sequence_ == 0 ||
        applied_object_tracking_stream_id_ != active_stream_id_ ||
        applied_object_tracking_request_identity_ != request_identity ||
        applied_object_tracking_sequence_ != last_applied_sequence_ ||
        applied_object_tracking_state_ !=
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE ||
        reference_stream_guard_.streamId() != active_stream_id_ ||
        reference_stream_guard_.lastAppliedSequence() != last_applied_sequence_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    if (!latest_stream_message_ || !latest_stream_message_->is_valid ||
        !latest_stream_message_->object_tracking_active ||
        latest_stream_message_->terminal_hold_active ||
        latest_stream_message_->state !=
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE ||
        latest_stream_message_->stream_id != active_stream_id_ ||
        latest_stream_message_->request_identity != request_identity ||
        latest_stream_message_->sequence < last_applied_sequence_ ||
        rosTimeNs(latest_stream_message_->valid_until) <= clock_->now().nanoseconds()) {
        return false;
    }
    const auto timeout = std::chrono::milliseconds(configuration_->GetParameter(
        "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
    const auto now = std::chrono::steady_clock::now();
    return now >= latest_stream_received_at_ &&
        now - latest_stream_received_at_ <= timeout &&
        now >= applied_object_tracking_at_ &&
        now - applied_object_tracking_at_ <= timeout;
}

bool ManeuverReferenceClient::beginObjectStopLocked(
    const std::string & request_identity) {
    // transition_mutex_ is held by the caller. The sequence is the command
    // actually consumed, never a newer publication seen only by DDS.
    if (object_stop_) {
        return object_stop_->identity.request_identity == request_identity;
    }
    if (pending_goal_handoff_ || active_request_identity_ != request_identity ||
        active_stream_id_.empty() || last_applied_sequence_ == 0 ||
        applied_object_tracking_stream_id_ != active_stream_id_ ||
        applied_object_tracking_request_identity_ != request_identity ||
        applied_object_tracking_sequence_ != last_applied_sequence_ ||
        applied_object_tracking_state_ !=
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE) {
        return false;
    }
    ObjectStop stop;
    stop.identity = ReferenceStreamIdentity{
        active_stream_id_, request_identity, last_applied_sequence_};
    stop.requested_at = std::chrono::steady_clock::now();
    if (!currentAppliedObjectTrackingStream(request_identity) ||
        vehicle_odometry_adapter_history_->empty()) {
        stop.failure_reason = "object stop lacks a fresh actually applied marked command";
    }
    object_stop_ = std::move(stop);
    reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_STOP);
    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }
    if (object_stop_->failure_reason.empty()) {
        publishReferenceAckForStream(
            object_stop_->identity.stream_id,
            object_stop_->identity.last_applied_sequence,
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_OBJECT_STOP_REQUESTED,
            "owned object command transition stop requested");
    }
    return true;
}

bool ManeuverReferenceClient::ownsObjectStoppedHold() const {
    return object_stopped_hold_ &&
        object_stopped_hold_->control_owner_generation == reference_control_owner_ &&
        object_stopped_hold_->stream_id == active_stream_id_ &&
        object_stopped_hold_->stream_id == reference_stream_guard_.streamId() &&
        object_stopped_hold_->request_identity == active_request_identity_;
}

bool ManeuverReferenceClient::appliedObjectStopRestLocked(
    const Reference & reference) const {
    if (!object_stop_ || !object_stop_->admitted ||
        applied_object_tracking_stream_id_ != object_stop_->identity.stream_id ||
        applied_object_tracking_request_identity_ !=
            object_stop_->identity.request_identity ||
        applied_object_tracking_sequence_ != last_applied_sequence_ ||
        applied_object_tracking_state_ !=
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED) return false;
    return reference.position().allFinite() && reference.velocity().allFinite() &&
        reference.acceleration().allFinite() && std::isfinite(reference.yaw()) &&
        std::isfinite(reference.yaw_rate()) &&
        std::isfinite(reference.yaw_acceleration()) &&
        reference.velocity().norm() <= 1.0e-5 &&
        reference.acceleration().norm() <= 1.0e-5 &&
        std::abs(reference.yaw_rate()) <= 1.0e-5 &&
        std::abs(reference.yaw_acceleration()) <= 1.0e-5;
}

void ManeuverReferenceClient::finishObjectStopLocked(const Reference & rest) {
    const auto stopped_identity = object_stop_->identity;
    {
        std::lock_guard<std::mutex> lock(reference_mutex_);
        reference_ = rest;
    }
    object_stopped_hold_ = ObjectStoppedHold{
        stopped_identity.stream_id,
        stopped_identity.request_identity,
        reference_control_owner_,
        rest,
        std::chrono::steady_clock::now(),
        false};
    reference_mode_.Store(reference_mode_t::HOVER);
    maneuver_reference_valid_.Store(false);
    clearPendingReferenceRequest();
    terminal_consumer_identity_.clear();
    terminal_hold_continuity_required_ = false;
    object_stop_failure_hold_ = false;
    object_stop_.reset();
    ++stop_maneuver_timer_generation_;
    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }
}

void ManeuverReferenceClient::StopManeuver() {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    ++stop_maneuver_timer_generation_;

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::StopManeuver(): Ignoring ordinary stop while a reference-loss bounded stop owns the reference stream."
        );
        return;
    }

    if (ownsObjectStoppedHold()) {
        reference_stream_guard_.cancelSuccessorGenerationExpectation();
        pending_goal_handoff_.reset();
        reference_mode_.Store(reference_mode_t::HOVER);
        return;
    }

    if (!isManeuverMode()) {
        retireInadmissibleCachedStreamLocked();
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuver(): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    if (beginObjectStopLocked(active_request_identity_)) return;

    if (const auto terminal_state = currentTerminalStreamState()) {
        if (*terminal_state == iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE) {
            RCLCPP_WARN(logger_, "Deferring terminal stop until Core finishes its bounded command segment");
            return;
        }
        if (*terminal_state ==
            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_TERMINAL_DEGRADED) {
            terminal_degraded_hold_ = true;
        } else {
            return;
        }
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuver(): Stopping maneuver."
    );

    reference_mode_.Store(reference_mode_t::HOVER);
    maneuver_reference_valid_.Store(false);
    resetReferenceSafety();
    clearPendingReferenceRequest();
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        latest_stream_message_.reset();
    }
    active_stream_id_.clear();
    active_request_identity_.clear();
    candidate_request_identity_.clear();
    prepared_stream_id_.clear();
    prepared_reference_anchor_.reset();
    last_applied_sequence_ = 0;
    candidate_sequence_ = 0;
    startup_reference_policy_.reset();
    fault_stream_identity_.reset();
    pending_goal_handoff_.reset();
    recovery_phase_ = RecoveryPhase::None;
    pending_rebase_request_.reset();
    pending_commit_request_.reset();

    if (!terminal_degraded_hold_ && !object_stop_failure_hold_) {
        UpdateReference();
    }

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

}

void ManeuverReferenceClient::StopManeuver(Reference reference) {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
    ++stop_maneuver_timer_generation_;

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::StopManeuver(Reference): Ignoring ordinary stop while a reference-loss bounded stop owns the reference stream."
        );
        return;
    }

    if (ownsObjectStoppedHold()) {
        // Nominal action-result metadata cannot replace a certified rest
        // still owned by the predecessor's stopped stream.
        return;
    }

    if (!isManeuverMode()) {
        retireInadmissibleCachedStreamLocked();
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuver(Reference): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    if (beginObjectStopLocked(active_request_identity_)) return;

    if (currentTerminalStreamState()) {
        // A nominal action result must not overwrite the last accepted
        // terminal command. Core owns its analytic stop and final hold.
        StopManeuver();
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuver(Reference): Stopping maneuver with given reference."
    );

    {
        std::lock_guard<std::mutex> lock(reference_mutex_);
        reference_ = reference;
    }

    reference_mode_.Store(reference_mode_t::HOVER);
    maneuver_reference_valid_.Store(false);
    resetReferenceSafety();
    clearPendingReferenceRequest();
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        latest_stream_message_.reset();
    }
    active_stream_id_.clear();
    active_request_identity_.clear();
    candidate_request_identity_.clear();
    prepared_stream_id_.clear();
    prepared_reference_anchor_.reset();
    last_applied_sequence_ = 0;
    candidate_sequence_ = 0;
    startup_reference_policy_.reset();
    fault_stream_identity_.reset();
    pending_goal_handoff_.reset();
    recovery_phase_ = RecoveryPhase::None;
    pending_rebase_request_.reset();
    pending_commit_request_.reset();

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

}

void ManeuverReferenceClient::StopManeuverAfterTimeout(int timeout_ms) {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::StopManeuverAfterTimeout(): Ignoring delayed stop while a reference-loss bounded stop owns the reference stream."
        );
        return;
    }

    if (!isManeuverMode()) {
        retireInadmissibleCachedStreamLocked();
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuverAfterTimeout(): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(): Stopping maneuver after %d milliseconds.", 
        timeout_ms
    );

    if (*stop_maneuver_timer_ != nullptr && !(*stop_maneuver_timer_)->is_canceled()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuverAfterTimeout(): Timer already running. Resetting timer.");
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(): Storing mode WAIT_FOR_MANEUVER_STOP."
    );
    reference_mode_.Store(WAIT_FOR_MANEUVER_STOP);

    stop_maneuver_timer_callback_ = [this]() -> void {
        RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::StopManeuverAfterTimeout(): Timer expired. Stopping maneuver.");
        StopManeuver();
    };

    stop_maneuver_timer_ = create_wall_timer_(
        std::chrono::milliseconds(timeout_ms),
        [this]() -> void {
            stopManeuverPrematurely();
        }
    );

}

void ManeuverReferenceClient::StopManeuverAfterTimeout(
    Reference reference, 
    int timeout_ms
) {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Ignoring delayed stop while a reference-loss bounded stop owns the reference stream."
        );
        return;
    }

    if (!isManeuverMode()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Cannot stop maneuver while not in MANEUVER mode.");
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Stopping maneuver after %d milliseconds with given reference.", 
        timeout_ms
    );

    if (*stop_maneuver_timer_ != nullptr && !(*stop_maneuver_timer_)->is_canceled()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Timer already running. Resetting timer.");
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuverAfterTimeout(): Storing mode WAIT_FOR_MANEUVER_STOP."
    );
    reference_mode_.Store(WAIT_FOR_MANEUVER_STOP);

    stop_maneuver_timer_callback_ = [this, reference]() -> void {
        RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::StopManeuverAfterTimeout(Reference): Timer expired. Stopping maneuver with given reference.");
        StopManeuver(reference);
    };

    stop_maneuver_timer_ = create_wall_timer_(
        std::chrono::milliseconds(timeout_ms),
        [this]() -> void {
            stopManeuverPrematurely();
        }
    );

}

Reference ManeuverReferenceClient::GetReference(
    double dt_s,
    std::function<void()> on_fail_during_maneuver
) {

    Reference reference;
    (void)dt_s;

    iii_drone_interfaces::msg::StringStamped reference_mode_msg;

    reference_mode_t observed_mode;
    uint64_t failure_epoch;
    {
        std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
        observed_mode = reference_mode_.Load();
        failure_epoch = maneuver_failure_epoch_;
    }

    switch(observed_mode) {
        case reference_mode_t::PASSTHROUGH:

            failed_attempts_ = 0;
            reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());

            reference_mode_msg.data = "passthrough";

            break;

        case reference_mode_t::HOVER: {

            failed_attempts_ = 0;
            std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);
            if (object_stopped_hold_ &&
                object_stopped_hold_->control_owner_generation == reference_control_owner_ &&
                object_stopped_hold_->stream_id == active_stream_id_ &&
                object_stopped_hold_->request_identity == active_request_identity_) {
                const auto consumption = consumeReferenceCandidate(
                    reference, reference_mode_t::HOVER, false, false);
                if (consumption.applied_ack_identity) {
                    object_stopped_hold_->last_applied_at =
                        std::chrono::steady_clock::now();
                    publishReferenceAckForStream(
                        consumption.applied_ack_identity->stream_id,
                        consumption.applied_ack_identity->last_applied_sequence,
                        iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                        "owned stopped-object rest applied");
                } else {
                    std::lock_guard<std::mutex> reference_lock(reference_mutex_);
                    reference = reference_;
                }
                const auto timeout = std::chrono::milliseconds(
                    configuration_->GetParameter(
                        "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
                if (!object_stopped_hold_->failure_reported &&
                    (consumption.began_reference_loss_stop ||
                     std::chrono::steady_clock::now() -
                         object_stopped_hold_->last_applied_at > timeout)) {
                    object_stopped_hold_->failure_reported = true;
                    object_stop_failure_hold_ = true;
                    on_fail_during_maneuver();
                }
                reference_mode_msg.data = "hover_object_stopped";
                break;
            }
            {
                std::lock_guard<std::mutex> reference_lock(reference_mutex_);
                reference = reference_;
            }
            reference_mode_msg.data = "hover";

            break;

        }
        case WAIT_FOR_MANEUVER_START: {
    
            failed_attempts_ = 0;

            rclcpp::Duration elapsed_time_since_start = rclcpp::Clock().now() - *maneuver_start_time_;

            int elapsed_ms = elapsed_time_since_start.nanoseconds() / 1e6;

            int start_timeout_ms = configuration_->GetParameter(
                "/mission/wait_for_maneuver_start_timeout_ms").as_int();
            if (terminal_hold_continuity_required_ &&
                !terminal_consumer_identity_.empty() && last_applied_sequence_ > 0) {
                std::lock_guard<std::mutex> stream_lock(reference_stream_mutex_);
                if (latest_stream_message_ && latest_stream_message_->is_valid &&
                    latest_stream_message_->stream_id == active_stream_id_ &&
                    latest_stream_message_->request_identity == active_request_identity_ &&
                    latest_stream_message_->state ==
                        iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE &&
                    rosTimeNs(latest_stream_message_->valid_until) > clock_->now().nanoseconds()) {
                    // A quiescent analytic segment can take 17.5 s at the
                    // configured 0.4 m / 0.1 m/s bounds. Keep the ordinary
                    // 3 s budget unless this exact predecessor is fresh.
                    start_timeout_ms = std::max(start_timeout_ms, 25000);
                }
            }
            if (elapsed_ms > start_timeout_ms) {

                RCLCPP_ERROR(
                    logger_,
                    "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Timeout while waiting for maneuver start after %d milliseconds. Calling on fail callback and hovering if this maneuver still owns the client.",
                    elapsed_ms
                );

                on_fail_during_maneuver();
                hoverIfFailureEpochUnchanged(failure_epoch);

                std::lock_guard<std::mutex> lock(reference_mutex_);
                reference = reference_;

                reference_mode_msg.data = currentReferenceModeLabel();
                break;

            }
            
            const auto consumption = consumeReferenceCandidate(
                reference,
                reference_mode_t::WAIT_FOR_MANEUVER_START,
                true,
                true
            );
            if (consumption.pause_identity) {
                requestProducerPause(*consumption.pause_identity, "reference continuity fault");
            }
            if (consumption.applied_ack_identity) {
                publishReferenceAckForStream(
                    consumption.applied_ack_identity->stream_id,
                    consumption.applied_ack_identity->last_applied_sequence,
                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                    "first maneuver reference applied"
                );
            }

            if (consumption.object_unrecoverable) {
                // This is a failed owned generation, not a stationary hold.
                // Keep the last actually accepted finite command through the
                // mode-failure callback and require explicit recovery.
                object_stop_failure_hold_ = true;
                on_fail_during_maneuver();
                reference_mode_msg.data = "object_unrecoverable_hold";
                break;
            }

            if (consumption.terminal_degraded) {
                terminal_degraded_hold_ = true;
                on_fail_during_maneuver();
                hoverIfFailureEpochUnchanged(failure_epoch);
                reference_mode_msg.data = "terminal_degraded_hold";
                break;
            }

            if (consumption.began_reference_loss_stop) {
                reference_mode_msg.data = "reference_loss_stopping";
                break;
            }

            if (!consumption.accepted) {

                RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Reference is not yet valid, returning hover reference.");

                reference = reference_;

                reference_mode_msg.data = "wait_for_maneuver_start";

                break;

            }

            if (consumption.stream_result == StreamReadResult::PredecessorActive) {
                reference_mode_msg.data = "wait_for_maneuver_start";
                break;
            }

            RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Reference is valid, switching to MANEUVER mode.");
            reference_mode_msg.data = "maneuver";

            break;

        }
        case reference_mode_t::MANEUVER: {

            const auto consumption = consumeReferenceCandidate(
                reference,
                reference_mode_t::MANEUVER,
                true,
                false
            );
            if (consumption.pause_identity) {
                requestProducerPause(*consumption.pause_identity, "reference continuity fault");
            }
            if (consumption.applied_ack_identity) {
                publishReferenceAckForStream(
                    consumption.applied_ack_identity->stream_id,
                    consumption.applied_ack_identity->last_applied_sequence,
                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                    "maneuver reference applied"
                );
            }
            if (consumption.object_unrecoverable) {
                object_stop_failure_hold_ = true;
                on_fail_during_maneuver();
                reference_mode_msg.data = "object_unrecoverable_hold";
                break;
            }
            if (consumption.terminal_degraded) {
                terminal_degraded_hold_ = true;
                on_fail_during_maneuver();
                hoverIfFailureEpochUnchanged(failure_epoch);
                reference_mode_msg.data = "terminal_degraded_hold";
                break;
            }
            if (consumption.began_reference_loss_stop) {
                reference_mode_msg.data = "reference_loss_stopping";
                break;
            }

            const StreamReadResult stream_result = consumption.stream_result;
            const bool success =
                consumption.accepted ||
                stream_result == StreamReadResult::FreshHeld;

            // Check if mode is hovering:
            if (reference_mode_.Load() == reference_mode_t::HOVER) {
                RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::GetReference(): MANEUVER: Mode switched to HOVER while waiting, returning hover reference");
                reference = reference_;
                reference_mode_msg.data = "hover";
                break;
            }

            if (!success) {

                failed_attempts_++;

                const bool has_valid_maneuver_reference = maneuver_reference_valid_.Load();

                if (has_valid_maneuver_reference) {
                    ManeuverReferenceSafetyEvaluation safety_evaluation;
                    {
                        std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                        safety_evaluation = reference_safety_guard_->observeMiss();
                    }
                    if (safety_evaluation.decision == ManeuverReferenceSafetyDecision::BEGIN_STOP) {
                        auto loss_stop = beginReferenceLossStop(safety_evaluation);
                        reference = loss_stop.reference;
                        if (loss_stop.pause_identity) {
                            requestProducerPause(*loss_stop.pause_identity, safety_evaluation.reason);
                        }
                        reference_mode_msg.data = "reference_loss_stopping";
                        break;
                    }
                }

                if (
                    !has_valid_maneuver_reference &&
                    failed_attempts_ >= configuration_->GetParameter("/mission/max_failed_attempts_during_maneuver").as_int()
                ) {

                    RCLCPP_ERROR(
                        logger_,
                        "ManeuverReferenceClient::GetReference(): MANEUVER: Failed to acquire first valid reference after %d attempts. Calling on failed callback and hovering if this maneuver still owns the client.",
                        failed_attempts_
                    );

                    on_fail_during_maneuver();
                    hoverIfFailureEpochUnchanged(failure_epoch);

                    std::lock_guard<std::mutex> lock(reference_mutex_);
                    reference = reference_;

                    reference_mode_msg.data = currentReferenceModeLabel();

                    break;

                }

                if (has_valid_maneuver_reference) {
                    RCLCPP_WARN(
                        logger_,
                        "ManeuverReferenceClient::GetReference(): MANEUVER: Failed to acquire valid reference for %d consecutive attempt(s). Holding last valid maneuver reference.",
                        failed_attempts_
                    );

                    std::lock_guard<std::mutex> lock(reference_mutex_);
                    reference = reference_;
                } else {
                    RCLCPP_WARN(
                        logger_,
                        "ManeuverReferenceClient::GetReference(): MANEUVER: Failed to acquire valid reference before any maneuver reference was received. Holding current state."
                    );

                    if (!vehicle_odometry_adapter_history_->empty()) {
                        reference = Reference((*vehicle_odometry_adapter_history_)[0].ToState());
                    } else {
                        std::lock_guard<std::mutex> lock(reference_mutex_);
                        reference = reference_;
                    }
                }


                reference_mode_msg.data = "hold_on_reference_timeout";

                break;

            } 

            if (stream_result == StreamReadResult::FreshHeld) {
                failed_attempts_ = 0;
                reference_mode_msg.data = "maneuver";
                break;
            }

            failed_attempts_ = 0;
            reference_mode_msg.data = "maneuver";

            break;

        }
        case WAIT_FOR_MANEUVER_STOP: {
            const auto consumption = consumeReferenceCandidate(
                reference,
                reference_mode_t::WAIT_FOR_MANEUVER_STOP,
                false,
                false
            );
            if (consumption.pause_identity) {
                requestProducerPause(*consumption.pause_identity, "reference continuity fault");
            }
            if (consumption.applied_ack_identity) {
                publishReferenceAckForStream(
                    consumption.applied_ack_identity->stream_id,
                    consumption.applied_ack_identity->last_applied_sequence,
                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                    "handoff reference applied"
                );
            }
            bool object_stop_handled = false;
            bool object_stop_failed = false;
            {
                std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
                if (object_stop_ && reference_mode_.Load() ==
                        reference_mode_t::WAIT_FOR_MANEUVER_STOP &&
                    object_stop_->identity.stream_id == active_stream_id_ &&
                    object_stop_->identity.request_identity == active_request_identity_) {
                    object_stop_handled = true;
                    auto & stop = *object_stop_;
                    const auto now = std::chrono::steady_clock::now();
                    const bool phase_applied = consumption.accepted &&
                        applied_object_tracking_stream_id_ == stop.identity.stream_id &&
                        applied_object_tracking_request_identity_ ==
                            stop.identity.request_identity &&
                        applied_object_tracking_sequence_ == last_applied_sequence_ &&
                        (applied_object_tracking_state_ ==
                            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPING ||
                         applied_object_tracking_state_ ==
                            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED);
                    if (phase_applied && !stop.admitted) {
                        stop.admitted = true;
                        stop.completion_deadline = now + std::chrono::milliseconds(
                            static_cast<int>(
                                KinematicStopTrajectory::MaximumCertifiedDurationS * 1000.0) +
                            configuration_->GetParameter(
                                "/control/maneuver_controller/maneuver_execution_period_ms").as_int() +
                            configuration_->GetParameter(
                                "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
                    }
                    if (stop.failure_reason.empty() && phase_applied &&
                        applied_object_tracking_state_ ==
                            iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_OBJECT_STOPPED) {
                        if (appliedObjectStopRestLocked(reference)) {
                            finishObjectStopLocked(reference);
                            reference_mode_msg.data = "hover_object_stopped";
                        } else {
                            stop.failure_reason = "object STOPPED reference was not finite rest";
                        }
                    }
                    if (object_stop_ && object_stop_->failure_reason.empty() &&
                        (consumption.began_reference_loss_stop || consumption.terminal_degraded)) {
                        object_stop_->failure_reason =
                            "object stop stream entered an explicit reference failure";
                    }
                    if (object_stop_ && object_stop_->failure_reason.empty()) {
                        const auto admission = std::chrono::milliseconds(
                            configuration_->GetParameter(
                                "/mission/wait_for_maneuver_start_timeout_ms").as_int());
                        if (!object_stop_->admitted && now - object_stop_->requested_at > admission) {
                            object_stop_->failure_reason =
                                "object stop was not applied within the start budget";
                        } else if (object_stop_->completion_deadline &&
                                   now > *object_stop_->completion_deadline) {
                            object_stop_->failure_reason =
                                "object certified stop exceeded its completion deadline";
                        }
                    }
                    if (object_stop_ && !object_stop_->failure_reason.empty() &&
                        !object_stop_->failure_reported) {
                        object_stop_->failure_reported = true;
                        object_stop_failure_hold_ = true;
                        object_stop_failed = true;
                        RCLCPP_ERROR(logger_, "Owned object stop failed: %s (request=%s stream=%s)",
                            object_stop_->failure_reason.c_str(),
                            object_stop_->identity.request_identity.c_str(),
                            object_stop_->identity.stream_id.c_str());
                    }
                    if (object_stop_ && !consumption.accepted) {
                        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
                        reference = reference_;
                    }
                    if (object_stop_ && reference_mode_msg.data.empty()) {
                        reference_mode_msg.data = object_stop_->failure_reason.empty()
                            ? "object_stop_waiting" : "object_stop_failed_hold";
                    }
                }
            }
            if (object_stop_handled) {
                if (object_stop_failed) on_fail_during_maneuver();
                break;
            }
            if (consumption.began_reference_loss_stop) {
                reference_mode_msg.data = "reference_loss_stopping";
                break;
            }

            const StreamReadResult stream_result = consumption.stream_result;
            const bool success =
                consumption.accepted ||
                stream_result == StreamReadResult::FreshHeld;
            if (!success) {
                failed_attempts_++;
                ManeuverReferenceSafetyEvaluation safety_evaluation;
                {
                    std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                    safety_evaluation = reference_safety_guard_->observeMiss();
                }
                if (safety_evaluation.decision == ManeuverReferenceSafetyDecision::BEGIN_STOP) {
                    auto loss_stop = beginReferenceLossStop(safety_evaluation);
                    reference = loss_stop.reference;
                    if (loss_stop.pause_identity) {
                        requestProducerPause(*loss_stop.pause_identity, safety_evaluation.reason);
                    }
                    reference_mode_msg.data = "reference_loss_stopping";
                    break;
                }
                // A single miss inside the delivery deadline is the normal gap
                // while a just-completed maneuver's stream is being created;
                // repeated misses are worth a warning (the safety guard above
                // still owns the stop decision either way).
                if (failed_attempts_ > 1) {
                    RCLCPP_WARN(
                        logger_,
                        "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_STOP: Failed to acquire valid reference for %d consecutive attempt(s). Holding last valid maneuver reference within delivery deadline.",
                        failed_attempts_
                    );
                } else {
                    RCLCPP_DEBUG(
                        logger_,
                        "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_STOP: Failed to acquire valid reference for %d consecutive attempt(s). Holding last valid maneuver reference within delivery deadline.",
                        failed_attempts_
                    );
                }
                {
                    std::lock_guard<std::mutex> lock(reference_mutex_);
                    reference = reference_;
                }
                reference_mode_msg.data = "hold_on_reference_timeout";
                break;
            }

            if (stream_result == StreamReadResult::FreshHeld) {
                failed_attempts_ = 0;
                reference_mode_msg.data = "wait_for_maneuver_stop";
                break;
            }

            failed_attempts_ = 0;
            reference_mode_msg.data = "wait_for_maneuver_stop";
            break;
        }
        case REFERENCE_LOSS_STOP: {
            reference = sampleReferenceLossStop(
                std::move(on_fail_during_maneuver),
                reference_mode_msg.data
            );
            break;
        }
    }

    reference_mode_msg.stamp = clock_->now();

    reference_mode_publisher_->publish(reference_mode_msg);

    return reference;

}

void ManeuverReferenceClient::stopManeuverPrematurely() {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (
        pending_goal_handoff_ ||
        reference_mode_.Load() != reference_mode_t::WAIT_FOR_MANEUVER_STOP
    ) {
        RCLCPP_DEBUG(
            logger_,
            "ManeuverReferenceClient::stopManeuverPrematurely(): Ignoring stale delayed-stop callback after reference mode changed."
        );
        return;
    }

    const auto timer = *stop_maneuver_timer_;
    const auto callback = *stop_maneuver_timer_callback_;

    if (timer != nullptr) {
        timer->cancel();
    }

    if (callback) {
        callback();
    } else {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::stopManeuverPrematurely(): Delayed-stop callback is unavailable; falling back to hover."
        );
        SetReferenceModeHover(true);
    }
}

bool ManeuverReferenceClient::isManeuverMode() {

    auto reference_mode = reference_mode_.Load();

    return isManeuverMode(reference_mode);

}

bool ManeuverReferenceClient::isManeuverMode(reference_mode_t reference_mode) {

    switch(reference_mode) {
        case WAIT_FOR_MANEUVER_START:
        case MANEUVER:
        case WAIT_FOR_MANEUVER_STOP:
        case REFERENCE_LOSS_STOP:
            return true;
        default:
            return false;
    }

}

int ManeuverReferenceClient::boundedGetReferenceTimeoutMs(double dt_s) {

    const int configured_timeout_ms = configuration_->GetParameter("/mission/get_reference_timeout_ms").as_int();
    if (configured_timeout_ms <= 1) {
        return 1;
    }

    int update_budget_ms = configured_timeout_ms;
    if (std::isfinite(dt_s) && dt_s > 0.0) {
        update_budget_ms = std::max(20, static_cast<int>(std::round(dt_s * 1000.0 * 0.80)));
    }

    return std::max(1, std::min(configured_timeout_ms, update_budget_ms));

}

bool ManeuverReferenceClient::getReferenceFromServer(Reference & reference, int timeout_ms) {

    Reference nan_ref(
        point_t::Constant(NAN),
        NAN,
        vector_t::Constant(NAN),
        NAN,
        vector_t::Constant(NAN),
        NAN
    );

    if (!get_reference_client_->wait_for_service(std::chrono::nanoseconds(static_cast<int64_t>(5e6)))) {
        RCLCPP_ERROR(logger_, "ManeuverReferenceClient::getReferenceFromServer(): Service not available within 5 milliseconds, using nan reference.");
        reference = nan_ref;
        return false;

    }

    if (!get_reference_client_->service_is_ready()) {
        RCLCPP_ERROR(logger_, "ManeuverReferenceClient::getReferenceFromServer(): Service not ready, using nan reference.");
        reference = nan_ref;
        return false;
    }

    std::shared_ptr<iii_drone_interfaces::srv::GetReference::Response> response;

    {
        std::lock_guard<std::mutex> request_lock(get_reference_request_mutex_);

        if (!pending_get_reference_request_) {
            auto request = std::make_shared<iii_drone_interfaces::srv::GetReference::Request>();
            pending_get_reference_request_.emplace(
                get_reference_client_->async_send_request(request)
            );
        }

        if (
            pending_get_reference_request_->wait_for(std::chrono::milliseconds(timeout_ms)) ==
            std::future_status::ready
        ) {
            response = pending_get_reference_request_->get();
            pending_get_reference_request_.reset();
        } else {
            RCLCPP_DEBUG(
                logger_,
                "ManeuverReferenceClient::GetReference(): Outstanding request has not responded after another %d ms.",
                timeout_ms
            );
        }
    }

    if (!response) {
        RCLCPP_DEBUG(
            logger_,
            "ManeuverReferenceClient::GetReference(): Reference response is still pending after %d ms.",
            timeout_ms
        );
        reference = nan_ref;
        return false;
    }

    if(!response->is_valid) {

        RCLCPP_WARN(logger_, "ManeuverReferenceClient::getReferenceFromServer(): Received invalid reference.");
        reference = nan_ref;
        return false;

    }

    reference = ReferenceAdapter(response->reference).reference();

    return true;

}

void ManeuverReferenceClient::clearPendingReferenceRequest() {

    std::lock_guard<std::mutex> request_lock(get_reference_request_mutex_);

    if (!pending_get_reference_request_) {
        return;
    }

    get_reference_client_->remove_pending_request(
        pending_get_reference_request_->request_id
    );
    pending_get_reference_request_.reset();

}
