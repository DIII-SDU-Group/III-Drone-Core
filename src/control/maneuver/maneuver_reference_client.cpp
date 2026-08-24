/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

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

}  // namespace

void ManeuverReferenceClient::receiveReferenceStream(
    const iii_drone_interfaces::msg::ManeuverReferenceStream::SharedPtr message
) {
    std::lock_guard<std::mutex> lock(reference_stream_mutex_);
    if (
        latest_stream_message_ &&
        latest_stream_message_->stream_id == message->stream_id &&
        message->sequence <= latest_stream_message_->sequence
    ) {
        return;
    }
    latest_stream_message_ = *message;
    latest_stream_received_at_ = std::chrono::steady_clock::now();
}

ManeuverReferenceClient::StreamReadResult
ManeuverReferenceClient::readReferenceStream(Reference & reference) {
    iii_drone_interfaces::msg::ManeuverReferenceStream message;
    std::chrono::steady_clock::time_point received_at;
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        if (!latest_stream_message_) {
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
    if (
        std::chrono::steady_clock::now() - received_at > timeout
    ) {
        return StreamReadResult::Unavailable;
    }
    if (successor_generation_handoff_requested_.exchange(false)) {
        reference_stream_guard_.expectSuccessorGeneration();
    }
    const auto decision = reference_stream_guard_.observe(message, clock_->now());
    if (decision == ManeuverReferenceStreamDecision::Prepared) {
        reference = ReferenceAdapter(message.reference).reference();
        return StreamReadResult::Prepared;
    }
    if (decision == ManeuverReferenceStreamDecision::Paused) {
        return StreamReadResult::Paused;
    }
    if (decision == ManeuverReferenceStreamDecision::FreshHeld) {
        std::lock_guard<std::mutex> lock(reference_mutex_);
        reference = reference_;
        return StreamReadResult::FreshHeld;
    }
    if (decision != ManeuverReferenceStreamDecision::NewActive) {
        return StreamReadResult::Unavailable;
    }
    active_stream_id_ = reference_stream_guard_.streamId();
    reference = ReferenceAdapter(message.reference).reference();
    candidate_sequence_ = reference_stream_guard_.candidateSequence();
    return StreamReadResult::NewActive;
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
    ack.vehicle_state = StateAdapter(
        (*vehicle_odometry_adapter_history_)[0].ToState()
    ).ToMsg();
    ack.consumer_status = status;
    ack.detail = detail;
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
    ack.last_applied_sequence = last_applied_sequence;
    ack.applied_at = clock_->now();
    ack.vehicle_state = StateAdapter(
        (*vehicle_odometry_adapter_history_)[0].ToState()
    ).ToMsg();
    ack.consumer_status = status;
    ack.detail = detail;
    reference_ack_publisher_->publish(ack);
}

void ManeuverReferenceClient::requestProducerPause(const std::string & reason) {
    publishReferenceAck(
        iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_PAUSING,
        reason
    );
    if (active_stream_id_.empty() || !pause_reference_stream_client_->service_is_ready()) {
        return;
    }
    auto request = std::make_shared<iii_drone_interfaces::srv::PauseReferenceStream::Request>();
    request->stream_id = active_stream_id_;
    request->last_applied_sequence = last_applied_sequence_;
    request->reason = reason;
    pause_reference_stream_client_->async_send_request(request);
}

void ManeuverReferenceClient::resetReferenceStreamState() {
    {
        std::lock_guard<std::mutex> lock(reference_stream_mutex_);
        latest_stream_message_.reset();
    }
    active_stream_id_.clear();
    prepared_stream_id_.clear();
    prepared_reference_anchor_.reset();
    last_applied_sequence_ = 0;
    candidate_sequence_ = 0;
    successor_generation_handoff_requested_.store(false);
    reference_stream_guard_.reset();
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

Reference ManeuverReferenceClient::beginReferenceLossStop(
    const ManeuverReferenceSafetyEvaluation & evaluation
) {
    const ControlledCancellationConfig stop_config = referenceLossStopConfig();
    Reference initial;
    {
        std::lock_guard<std::mutex> reference_lock(reference_mutex_);
        initial = reference_;
    }
    if (!finiteReference(initial) && !vehicle_odometry_adapter_history_->empty()) {
        initial = Reference((*vehicle_odometry_adapter_history_)[0].ToState());
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
        reference_loss_stop_start_time_ = std::chrono::steady_clock::now();
        reference_loss_below_threshold_since_.reset();
        reference_loss_failure_reported_ = false;
        reference_loss_reason_ = evaluation.reason;
    }
    clearPendingReferenceRequest();
    recovery_phase_ = RecoveryPhase::Stopping;
    recovery_phase_started_ = std::chrono::steady_clock::now();
    requestProducerPause(evaluation.reason);
    reference_mode_.Store(reference_mode_t::REFERENCE_LOSS_STOP);

    RCLCPP_ERROR(
        logger_,
        "ManeuverReferenceClient::beginReferenceLossStop(): %s after %.3f s; "
        "position %.3f/%.3f m, velocity %.3f/%.3f m/s, acceleration %.3f/%.3f m/s^2. "
        "Rejecting further server references and starting %.3f s local bounded stop.",
        evaluation.reason.c_str(),
        evaluation.reference_age_s,
        evaluation.position_error_m,
        evaluation.position_limit_m,
        evaluation.velocity_error_m_s,
        evaluation.velocity_limit_m_s,
        evaluation.acceleration_error_m_s2,
        evaluation.acceleration_limit_m_s2,
        stop_duration_s
    );
    return initial;
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

bool ManeuverReferenceClient::advanceReferenceRecovery(
    Reference & reference,
    std::function<void()> on_fail_during_maneuver,
    std::string & reference_mode
) {
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
        if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
            SetReferenceModeHover(true);
        }
        failed_attempts_ = 0;
        reference_mode = "hover_after_reference_loss";
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
        if (!rebase_reference_stream_client_->service_is_ready()) {
            return fail_recovery("rebase service unavailable after bounded stop");
        }
        publishReferenceAck(
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_STOPPED,
            "bounded stop settled"
        );
        auto request = std::make_shared<iii_drone_interfaces::srv::RebaseReferenceStream::Request>();
        request->stream_id = active_stream_id_;
        request->last_applied_sequence = last_applied_sequence_;
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
            const std::string stopped_stream_id = active_stream_id_;
            uint64_t stopped_sequence = last_applied_sequence_;
            {
                std::lock_guard<std::mutex> lock(reference_stream_mutex_);
                if (
                    latest_stream_message_ &&
                    latest_stream_message_->stream_id == stopped_stream_id
                ) {
                    stopped_sequence = latest_stream_message_->sequence;
                }
            }
            RCLCPP_WARN(
                logger_,
                "ManeuverReferenceClient: producer requested action abort after bounded stop: %s",
                response->reason.c_str()
            );
            SetReferenceModeHover(true);
            publishReferenceAckForStream(
                stopped_stream_id,
                stopped_sequence,
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
        if (readReferenceStream(resumed) != StreamReadResult::NewActive) {
            reference_mode = "wait_for_rebased_reference";
            return true;
        }
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
            return fail_recovery("first committed reference failed continuity validation");
        }
        reference = resumed;
        last_applied_sequence_ = candidate_sequence_;
        reference_stream_guard_.commitCandidate();
        {
            std::lock_guard<std::mutex> reference_lock(reference_mutex_);
            reference_ = resumed;
        }
        publishReferenceAck(
            iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
            "rebased reference applied"
        );
        recovery_phase_ = RecoveryPhase::None;
        reference_loss_stop_trajectory_.reset();
        reference_loss_failure_reported_ = false;
        reference_mode_.Store(reference_mode_t::MANEUVER);
        reference_mode = "maneuver_rebased";
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient: committed rebased stream %s; maneuver resumed from stopped state.",
            active_stream_id_.c_str()
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

    UpdateReference(true);

    if (reference_mode == reference_mode_t::HOVER) {
        resetReferenceSafety();
        clearPendingReferenceRequest();
        resetReferenceStreamState();
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

}

bool ManeuverReferenceClient::StartManeuver() {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

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
    resetReferenceStreamState();
    reference_mode_.Store(reference_mode_t::WAIT_FOR_MANEUVER_START);
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

    successor_generation_handoff_requested_.store(true);
    RCLCPP_DEBUG(
        logger_,
        "ManeuverReferenceClient::PrepareManeuverStreamHandoff(): "
        "Expecting one continuity-checked successor reference generation."
    );
    return true;
}

bool ManeuverReferenceClient::IsManeuverActive() {

    return isManeuverMode();

}

void ManeuverReferenceClient::StopManeuver() {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::StopManeuver(): Ignoring ordinary stop while a reference-loss bounded stop owns the reference stream."
        );
        return;
    }

    if (!isManeuverMode()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuver(): Cannot stop maneuver while a maneuver mode is not active.");
        return;
    }

    RCLCPP_DEBUG(
        logger_, 
        "ManeuverReferenceClient::StopManeuver(): Stopping maneuver."
    );

    reference_mode_.Store(reference_mode_t::HOVER);
    maneuver_reference_valid_.Store(false);
    resetReferenceSafety();
    clearPendingReferenceRequest();
    resetReferenceStreamState();

    UpdateReference();

    if (*stop_maneuver_timer_ != nullptr) {
        (*stop_maneuver_timer_)->cancel();
        stop_maneuver_timer_.Store(nullptr);
    }

}

void ManeuverReferenceClient::StopManeuver(Reference reference) {

    std::lock_guard<std::recursive_mutex> transition_lock(transition_mutex_);

    if (reference_mode_.Load() == reference_mode_t::REFERENCE_LOSS_STOP) {
        RCLCPP_WARN(
            logger_,
            "ManeuverReferenceClient::StopManeuver(Reference): Ignoring ordinary stop while a reference-loss bounded stop owns the reference stream."
        );
        return;
    }

    if (!isManeuverMode()) {
        RCLCPP_WARN(logger_, "ManeuverReferenceClient::StopManeuver(Reference): Cannot stop maneuver while a maneuver mode is not active.");
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
    resetReferenceStreamState();

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

    switch(reference_mode_.Load()) {
        case reference_mode_t::PASSTHROUGH:

            failed_attempts_ = 0;
            reference = vehicle_odometry_adapter_history_->empty() ? Reference() : Reference((*vehicle_odometry_adapter_history_)[0].ToState());

            reference_mode_msg.data = "passthrough";

            break;

        case reference_mode_t::HOVER: {

            failed_attempts_ = 0;

            std::lock_guard<std::mutex> lock(reference_mutex_);

            reference = reference_;

            reference_mode_msg.data = "hover";

            break;

        }
        case WAIT_FOR_MANEUVER_START: {
    
            failed_attempts_ = 0;

            rclcpp::Duration elapsed_time_since_start = rclcpp::Clock().now() - *maneuver_start_time_;

            int elapsed_ms = elapsed_time_since_start.nanoseconds() / 1e6;

            if (elapsed_ms > configuration_->GetParameter("/mission/wait_for_maneuver_start_timeout_ms").as_int()) {

                RCLCPP_ERROR(
                    logger_,
                    "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Timeout while waiting for maneuver start after %d milliseconds. Calling on fail callback and switching to HOVER mode.",
                    elapsed_ms
                );

                on_fail_during_maneuver();

                SetReferenceModeHover(true);

                std::lock_guard<std::mutex> lock(reference_mutex_);
                reference = reference_;

                reference_mode_msg.data = "hover";
                break;

            }
            
            const StreamReadResult stream_result = readReferenceStream(reference);
            const bool success = stream_result == StreamReadResult::NewActive;

            if (!success) {

                RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Reference is not yet valid, returning hover reference.");

                reference = reference_;

                reference_mode_msg.data = "wait_for_maneuver_start";

                break;

            }

            if (reference_mode_.Load() == reference_mode_t::WAIT_FOR_MANEUVER_START) {

                ManeuverReferenceSafetyEvaluation safety_evaluation;
                {
                    std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                    if (
                        !reference_safety_guard_->hasAcceptedReference() &&
                        !vehicle_odometry_adapter_history_->empty()
                    ) {
                        safety_evaluation = reference_safety_guard_->observeReference(
                            Reference((*vehicle_odometry_adapter_history_)[0].ToState())
                        );
                    }
                    if (
                        safety_evaluation.decision !=
                            ManeuverReferenceSafetyDecision::BEGIN_STOP
                    ) {
                        safety_evaluation = reference_safety_guard_->observeReference(reference);
                    }
                }
                if (safety_evaluation.decision != ManeuverReferenceSafetyDecision::ACCEPT) {
                    reference = beginReferenceLossStop(safety_evaluation);
                    reference_mode_msg.data = "reference_loss_stopping";
                    break;
                }

                RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Reference is valid, switching to MANEUVER mode.");

                {
                    std::lock_guard<std::mutex> lock(reference_mutex_);
                    reference_ = reference;
                }
                maneuver_reference_valid_.Store(true);
                last_applied_sequence_ = candidate_sequence_;
                reference_stream_guard_.commitCandidate();
                publishReferenceAck(
                    iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                    "first maneuver reference applied"
                );

                reference_mode_.Store(reference_mode_t::MANEUVER);

                reference_mode_msg.data = "maneuver";

            } else if (reference_mode_.Load() == reference_mode_t::WAIT_FOR_MANEUVER_STOP) {

                RCLCPP_DEBUG(logger_, "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Mode switched to WAIT_FOR_MANEUVER_STOP while waiting.");

                reference_mode_msg.data = "wait_for_maneuver_stop";

            } else {

                RCLCPP_ERROR(logger_, "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_START: Mode switched to %d while waiting.", reference_mode_.Load());

                reference_mode_msg.data = "error";

            }

            break;

        }
        case reference_mode_t::MANEUVER: {

            const StreamReadResult stream_result = readReferenceStream(reference);
            const bool success =
                stream_result == StreamReadResult::NewActive ||
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
                        reference = beginReferenceLossStop(safety_evaluation);
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
                        "ManeuverReferenceClient::GetReference(): MANEUVER: Failed to acquire first valid reference after %d attempts. Calling on failed callback and switching to HOVER mode.",
                        failed_attempts_
                    );

                    on_fail_during_maneuver();

                    SetReferenceModeHover(true);

                    failed_attempts_ = 0;

                    std::lock_guard<std::mutex> lock(reference_mutex_);
                    reference = reference_;

                    reference_mode_msg.data = "hover";

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

            ManeuverReferenceSafetyEvaluation safety_evaluation;
            {
                std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                safety_evaluation = reference_safety_guard_->observeReference(reference);
            }
            if (safety_evaluation.decision != ManeuverReferenceSafetyDecision::ACCEPT) {
                reference = beginReferenceLossStop(safety_evaluation);
                reference_mode_msg.data = "reference_loss_stopping";
                break;
            }

            failed_attempts_ = 0;
            {
                std::lock_guard<std::mutex> lock(reference_mutex_);
                reference_ = reference;
            }
            maneuver_reference_valid_.Store(true);
            last_applied_sequence_ = candidate_sequence_;
            reference_stream_guard_.commitCandidate();
            publishReferenceAck(
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                "maneuver reference applied"
            );

            reference_mode_msg.data = "maneuver";

            break;

        }
        case WAIT_FOR_MANEUVER_STOP: {
            const StreamReadResult stream_result = readReferenceStream(reference);
            const bool success =
                stream_result == StreamReadResult::NewActive ||
                stream_result == StreamReadResult::FreshHeld;
            if (!success) {
                failed_attempts_++;
                ManeuverReferenceSafetyEvaluation safety_evaluation;
                {
                    std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                    safety_evaluation = reference_safety_guard_->observeMiss();
                }
                if (safety_evaluation.decision == ManeuverReferenceSafetyDecision::BEGIN_STOP) {
                    reference = beginReferenceLossStop(safety_evaluation);
                    reference_mode_msg.data = "reference_loss_stopping";
                    break;
                }
                RCLCPP_WARN(
                    logger_,
                    "ManeuverReferenceClient::GetReference(): WAIT_FOR_MANEUVER_STOP: Failed to acquire valid reference for %d consecutive attempt(s). Holding last valid maneuver reference within delivery deadline.",
                    failed_attempts_
                );
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

            ManeuverReferenceSafetyEvaluation safety_evaluation;
            {
                std::lock_guard<std::mutex> safety_lock(reference_safety_mutex_);
                safety_evaluation = reference_safety_guard_->observeReference(reference);
            }
            if (safety_evaluation.decision != ManeuverReferenceSafetyDecision::ACCEPT) {
                reference = beginReferenceLossStop(safety_evaluation);
                reference_mode_msg.data = "reference_loss_stopping";
                break;
            }

            failed_attempts_ = 0;
            {
                std::lock_guard<std::mutex> lock(reference_mutex_);
                reference_ = reference;
            }
            last_applied_sequence_ = candidate_sequence_;
            reference_stream_guard_.commitCandidate();
            publishReferenceAck(
                iii_drone_interfaces::msg::ManeuverReferenceAck::STATUS_APPLIED,
                "handoff reference applied"
            );
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

    if (reference_mode_.Load() != reference_mode_t::WAIT_FOR_MANEUVER_STOP) {
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
