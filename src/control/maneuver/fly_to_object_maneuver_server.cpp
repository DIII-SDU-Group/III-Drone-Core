/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::adapters;

namespace {

double shortestYawError(double current_yaw, double target_yaw) {
    return std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));
}

double shortestCableAxisYawError(double current_yaw, double target_yaw) {
    double error = shortestYawError(current_yaw, target_yaw);
    if (error > M_PI_2) {
        error -= M_PI;
    } else if (error < -M_PI_2) {
        error += M_PI;
    }
    return error;
}

Reference referenceWithCableAxisYawClosestTo(const Reference & reference, double current_yaw) {
    return Reference(
        reference.position(),
        current_yaw + shortestCableAxisYawError(current_yaw, reference.yaw()),
        reference.velocity(),
        reference.yaw_rate(),
        reference.acceleration(),
        reference.yaw_acceleration(),
        reference.stamp()
    );
}

constexpr double kFinalReferencePositionToleranceM = 1.0e-3;
constexpr double kFinalReferenceYawToleranceRad = 1.0e-3;
constexpr double kFinalReferenceVelocityToleranceMps = 1.0e-3;
constexpr double kFinalReferenceYawRateToleranceRadps = 1.0e-3;
constexpr double kFinalReferenceAccelerationToleranceMps2 = 1.0e-3;
constexpr double kFinalReferenceYawAccelerationToleranceRadps2 = 1.0e-3;

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlyToObjectManeuverServer::FlyToObjectManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    iii_drone::configuration::Configuration::SharedPtr parameters,
    iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client
) : ManeuverServer(
    node,
    combined_drone_awareness_handler,
    action_name,
    wait_for_execute_poll_ms,
    evaluate_done_poll_ms
),  configuration_(parameters),
    trajectory_generator_client_(trajectory_generator_client) {

    createServer<FlyToObject>();

}

void FlyToObjectManeuverServer::RegisterFirstReferenceAppliedCallback(
    std::function<bool(const std::string &)> callback
) {
    first_reference_applied_ = std::move(callback);
}

bool FlyToObjectManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_FLY_TO_OBJECT) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::CanExecuteManeuver(): Maneuver type is not MANEUVER_TYPE_FLY_TO_OBJECT, returning false.");
        return false;
    }

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    if (!validateAwarenessAndParameters(
        drone_awareness,
        params
    )) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::CanExecuteManeuver(): Awareness and parameters are not valid, returning false.");
        return false;
    }

    return true;

}

iii_drone::adapters::CombinedDroneAwarenessAdapter FlyToObjectManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    TargetAdapter target_adapter = params.target_adapter;

    State target_state = awareness_handler()->ComputeTargetState(target_adapter);

    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness_after;

    awareness_after.armed() = true;
    awareness_after.offboard() = true;
    awareness_after.target_adapter() = target_adapter;
    awareness_after.target_position_known() = true;
    awareness_after.drone_location() = DRONE_LOCATION_IN_FLIGHT;
    awareness_after.state() = target_state;

    return awareness_after;

}

maneuver_type_t FlyToObjectManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FLY_TO_OBJECT;
}

void FlyToObjectManeuverServer::startExecution(Maneuver & maneuver) {

    RCLCPP_INFO(node()->get_logger(), "FlyToObjectManeuverServer::startExecution(): Starting execution of maneuver.");

    auto cda_handler = awareness_handler();

    fly_to_object_maneuver_params_t params(maneuver.maneuver_params());

    target_adapter_ = params.target_adapter;

    const auto target_transform = target_adapter_->target_transform();
    RCLCPP_DEBUG(
        node()->get_logger(), 
        "FlyToObjectManeuverServer::startExecution(): target_type=%d target_id=%d reference_frame=%s target_transform_translation=[%.3f, %.3f, %.3f]",
        target_adapter_->target_type(),
        target_adapter_->target_id(),
        target_adapter_->reference_frame_id().c_str(),
        target_transform(0, 3),
        target_transform(1, 3),
        target_transform(2, 3)
    );

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "FlyToObjectManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    terminal_start_reference_ = consumeTerminalStartReference(maneuver.requestIdentity());
    object_tracking_session_.reset();
    published_object_tracking_session_.Store(nullptr);
    object_failure_hold_.reset();
    object_hover_ready_ = false;
    has_failed_ = false;
    active_target_reference_valid_ = false;
    nominal_target_observation_.Store(nullptr);
    mpc_settle_active_ = false;
    mpc_settle_first_iteration_ = false;
    target_position_filter_initialized_ = false;
    filtered_target_position_ = point_t::Zero();
    last_target_position_filter_update_time_ = node()->now();
    last_target_observation_ns_.store(0);
    maneuver_start_time_ = node()->now();
    threshold_reached_logged_ = false;
    settle_threshold_reached_logged_ = false;
    final_reference_streamed_logged_ = false;
    success_timing_logged_ = false;

    cda_handler->SetTarget(target_adapter_);

    if (supportsObjectTracking()) {
        const auto binding = currentReferenceBinding();
        if (binding.request_identity != maneuver.requestIdentity() ||
            binding.execution_id == 0 || binding.reference_provider_name != action_name()) {
            throw std::runtime_error("fly-to-object tracking has no exact execution owner");
        }
        const Reference seed = terminal_start_reference_.value_or(
            Reference(cda_handler->GetState())).CopyWithNewStamp(node()->now());
        const double minimum_altitude_m = cda_handler->ground_altitude_estimate() +
            configuration_->GetParameter(
                "/control/maneuver_controller/minimum_target_altitude").as_double();
        ObjectTrackingSession::Limits tracking_limits;
        tracking_limits.cancellation_config = controlledCancellationConfigFrom(configuration_);
        object_tracking_session_ = std::make_shared<ObjectTrackingSession>(
            [client = trajectory_generator_client_](
                const Reference & start, const Reference & target, bool reset) {
                return client->ComputeReference(start, target, true, reset,
                    trajectory_mode_t::bounded_positional);
            }, seed, maneuver.requestIdentity(), binding.execution_id,
            node()->now(), std::min(static_cast<double>(seed.position()(2)),
                minimum_altitude_m), tracking_limits);
        published_object_tracking_session_.Store(object_tracking_session_);
        PrimeOwnedManagedReference(seed);
    }

}

bool FlyToObjectManeuverServer::supportsObjectTracking() const {
    return !configuration_->GetParameter(
        "/control/maneuver_controller/fly_to_object_use_mpc").as_bool() &&
        configuration_->GetParameter(
        "/control/maneuver_controller/cable_landing_controller_type").as_string() ==
            "line_pid";
}

bool FlyToObjectManeuverServer::RetainsTrackedSource(
    const ReferenceCallbackBinding & source) const {
    const auto tracking = published_object_tracking_session_.Load();
    return source.callback && !source.request_identity.empty() &&
        source.execution_id != 0 && tracking &&
        tracking->owns(source.request_identity, source.execution_id);
}

bool FlyToObjectManeuverServer::TrackedSourceUnrecoverable(
    const ReferenceCallbackBinding & source) const {
    const auto tracking = published_object_tracking_session_.Load();
    return tracking && source.callback && !source.request_identity.empty() &&
        source.execution_id != 0 &&
        tracking->owns(source.request_identity, source.execution_id) &&
        tracking->unrecoverable();
}

bool FlyToObjectManeuverServer::RequestTrackedTransitionStop(
    const ReferenceCallbackBinding & source) {
    const auto tracking = published_object_tracking_session_.Load();
    return tracking && source.callback && !source.request_identity.empty() &&
        source.execution_id != 0 &&
        tracking->owns(source.request_identity, source.execution_id) &&
        tracking->RequestTransitionStop(
            source.request_identity, source.execution_id);
}

bool FlyToObjectManeuverServer::TrackedTransitionStopping(
    const ReferenceCallbackBinding & source) const {
    const auto tracking = published_object_tracking_session_.Load();
    return tracking && source.callback && !source.request_identity.empty() &&
        source.execution_id != 0 &&
        tracking->owns(source.request_identity, source.execution_id) &&
        tracking->transitionStopping();
}

std::optional<Reference> FlyToObjectManeuverServer::TrackedTransitionRest(
    const ReferenceCallbackBinding & source) const {
    const auto tracking = published_object_tracking_session_.Load();
    if (!tracking || !source.callback || source.request_identity.empty() ||
        source.execution_id == 0 ||
        !tracking->owns(source.request_identity, source.execution_id) ||
        !tracking->transitionRest()) return std::nullopt;
    return tracking->lastCommand();
}

Reference FlyToObjectManeuverServer::initializationReference(const State & state) const {
    const auto binding = currentReferenceBinding();
    if (supportsObjectTracking()) {
        const Reference seed = terminalStartReferenceFor(binding.request_identity)
            .value_or(Reference(state)).CopyWithNewStamp(node()->now());
        const double minimum_altitude_m = awareness_handler()->ground_altitude_estimate() +
            configuration_->GetParameter(
                "/control/maneuver_controller/minimum_target_altitude").as_double();
        std::string reason;
        if (!ObjectTrackingSession::CanCertifyInitialSeed(seed,
                std::min(static_cast<double>(seed.position()(2)), minimum_altitude_m),
                controlledCancellationConfigFrom(configuration_), reason)) {
            throw std::runtime_error("fly-to-object startup seed rejected: " + reason);
        }
        return seed;
    }
    if (const auto seed = terminalStartReferenceFor(binding.request_identity)) {
        return seed->CopyWithNewStamp(node()->now());
    }
    return ManeuverServer::initializationReference(state);
}

bool FlyToObjectManeuverServer::canCancel() {
    return true;
}

void FlyToObjectManeuverServer::RegisterAppliedRestReferenceCallback(
    std::function<bool(const std::string &, const Reference &)> callback) {
    applied_rest_reference_ = std::move(callback);
}

std::optional<ControlledCancellationConfig>
FlyToObjectManeuverServer::controlledCancellationConfig() const {
    return object_tracking_session_ ?
        std::optional(object_tracking_session_->cancellationConfig()) : std::nullopt;
}

bool FlyToObjectManeuverServer::validateControlledCancellationStop(
    const Reference & initial, const KinematicStopTrajectory & candidate,
    std::string & reason) {
    if (!object_tracking_session_) return true;
    if (object_tracking_session_->CertifiesCancellationStop(initial, candidate, reason)) return true;
    object_tracking_session_->RejectUnsafeCancellation(reason);
    return false;
}

bool FlyToObjectManeuverServer::controlledCancellationComplete(
    const ControlledCancellationConfig &) {
    if (!object_tracking_session_) return false;
    const bool profile_complete = controlledCancellationProfileComplete();
    const auto rest = controlledCancellationFinalReference();
    const std::string owner = current_maneuver().Load().requestIdentity();
    const bool exact_applied_rest = profile_complete && rest && applied_rest_reference_ &&
        applied_rest_reference_(owner, *rest);
    const bool proved = object_tracking_session_->ObserveCancellationProof(
        profile_complete, exact_applied_rest,
        awareness_handler()->GetMeasuredOdometry(), node()->now());
    if (proved && !object_failure_hold_) {
        try {
            object_failure_hold_ = std::make_shared<TerminalTrackingHold>(
                *rest, awareness_handler(), node()->get_clock(),
                TerminalTrackingHold::Clearance{}, 0.0);
            if (!object_failure_hold_->RequestQuiescence() ||
                !object_failure_hold_->isQuiescent()) {
                throw std::runtime_error("object cancellation rest cannot be retained");
            }
            auto hover = std::static_pointer_cast<HoverManeuverServer>(
                registered_maneuvers().at(MANEUVER_TYPE_HOVER));
            hover->AdoptTerminalHold(object_failure_hold_, owner);
        } catch (const std::exception & error) {
            object_tracking_session_->RejectUnsafeCancellation(error.what());
            return false;
        }
    }
    return proved;
}

bool FlyToObjectManeuverServer::controlledCancellationFailure() const {
    return object_tracking_session_ && object_tracking_session_->cancellationProofFailed();
}

bool FlyToObjectManeuverServer::rebaseExecution(
    const State & stopped_state,
    std::string & reason
) {
    if (trajectory_generator_client_->busy()) {
        reason = "trajectory generator is busy";
        return false;
    }
    first_iteration_ = true;
    has_failed_ = false;
    terminal_start_reference_.reset();
    object_tracking_session_.reset();
    published_object_tracking_session_.Store(nullptr);
    object_failure_hold_.reset();
    object_hover_ready_ = false;
    active_target_reference_valid_ = false;
    nominal_target_observation_.Store(nullptr);
    mpc_settle_active_ = false;
    mpc_settle_first_iteration_ = false;
    target_position_filter_initialized_ = false;
    filtered_target_position_ = stopped_state.position();
    last_target_position_filter_update_time_ = node()->now();
    last_target_observation_ns_.store(0);
    maneuver_start_time_ = node()->now();
    reason = "replanned fly-to-object from stopped state";
    return true;
}

Reference FlyToObjectManeuverServer::computeReference(const State & state) {

    if (object_tracking_session_ && object_tracking_session_->transitionStopping()) {
        return object_tracking_session_->TransitionReference(node()->now());
    }

    Reference target_reference;
    const bool use_mpc = configuration_->GetParameter("/control/maneuver_controller/fly_to_object_use_mpc").as_bool();
    bool reset = first_iteration_;
    bool set_reference = true;
    bool compute_with_mpc = use_mpc;

    if (mpc_settle_active_) {
        target_reference = mpc_settle_target_reference_;
        reset = set_reference = mpc_settle_first_iteration_;
        compute_with_mpc = false;
        RCLCPP_DEBUG_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "FlyToObjectManeuverServer::computeReference(): Using post-MPC interpolation settle"
        );
    } else {
        try {

            target_reference = updateLiveTargetReference(
                state, object_tracking_session_ ? currentReferenceBinding()
                    : ReferenceCallbackBinding{});

        } catch (const std::runtime_error &e) {
            nominal_target_observation_.Store(nullptr);

            if (active_target_reference_valid_ && targetLossWithinGrace()) {
                target_reference = active_target_reference_.Load();
                RCLCPP_WARN_THROTTLE(
                    node()->get_logger(),
                    *node()->get_clock(),
                    1000,
                    "FlyToObjectManeuverServer::computeReference(): Target temporarily unavailable; retaining last valid target during grace window: %s",
                    e.what()
                );
            } else {
                if (object_tracking_session_) {
                    return object_tracking_session_->FailureReference(
                        node()->now(), std::string("object target lost: ") + e.what());
                }
                has_failed_ = true;
                return Reference(state);
            }

        }
    }

    Reference ref;
    
    try {
        if (!compute_with_mpc && !mpc_settle_active_ && object_tracking_session_) {
            const auto capture_started = std::chrono::steady_clock::now();
            const auto measured = awareness_handler()->GetMeasuredOdometry();
            const auto capture_finished = std::chrono::steady_clock::now();
            if (!measured) {
                throw std::runtime_error("object tracking measured odometry unavailable");
            }
            const auto binding = currentReferenceBinding();
            const double minimum_altitude_m = awareness_handler()->ground_altitude_estimate() +
                configuration_->GetParameter(
                    "/control/maneuver_controller/minimum_target_altitude").as_double();
            std::string reason;
            const auto emission_stamp = node()->now();
            const auto clock_finished = std::chrono::steady_clock::now();
            bool first_timing_fault = false;
            if (!object_tracking_session_->Compute(target_reference, *measured,
                    emission_stamp, binding.request_identity, binding.execution_id,
                    minimum_altitude_m, 0.4, ref, reason, &first_timing_fault)) {
                if (first_timing_fault) {
                    // Compute has released its session lock. This best-effort
                    // ingress snapshot must not change the failed command.
                    try {
                        const auto failure_observed = std::chrono::steady_clock::now();
                        const auto ingress = awareness_handler()->TryGetOdometryIngressDiagnostics();
                        const auto ingress_copy_finished = std::chrono::steady_clock::now();
                        const auto steady_ns = [](std::chrono::steady_clock::time_point time) {
                            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                                time.time_since_epoch()).count();
                        };
                        const auto & continuity = measured->position_continuity;
                        std::ostringstream evidence;
                        evidence << "request=" << binding.request_identity
                            << " execution=" << binding.execution_id
                            << " captured_source_us=" << measured->source_sample_timestamp_us
                            << " captured_receipt_ros_ns=" << measured->receipt_stamp.nanoseconds()
                            << " captured_raw_reset=" << static_cast<unsigned>(measured->reset_counter)
                            << " captured_source_epoch=" << continuity.source_epoch
                            << " captured_position_epoch=" << continuity.position_epoch
                            << " captured_continuity_qualified=" << continuity.source_qualified
                            << " emission_ros_ns=" << emission_stamp.nanoseconds()
                            << " emission_clock_type=" << static_cast<int>(emission_stamp.get_clock_type())
                            << " capture_started_steady_ns=" << steady_ns(capture_started)
                            << " capture_finished_steady_ns=" << steady_ns(capture_finished)
                            << " clock_finished_steady_ns=" << steady_ns(clock_finished)
                            << " failure_observed_steady_ns=" << steady_ns(failure_observed)
                            << " ingress_copy_finished_steady_ns=" << steady_ns(ingress_copy_finished);
                        if (!ingress.available) {
                            evidence << " ingress=" << (ingress.busy ? "busy" : "unavailable");
                        } else {
                            evidence << " ingress=available"
                                << " latest_available=" << ingress.latest_available
                                << " latest_source_us=" << ingress.latest_source_sample_timestamp_us
                                << " latest_raw_reset=" << static_cast<unsigned>(ingress.latest_reset_counter)
                                << " latest_receipt_ros_ns=" << ingress.latest_receipt_ros_ns
                                << " latest_accepted_steady_ns=" << ingress.latest_accepted_steady_ns
                                << " ingress_total=" << ingress.total_callbacks
                                << " ingress_count=" << ingress.history_count
                                << " ingress_fields=source_us,raw_reset,callback_ros_ns,entry_steady_ns,lock_steady_ns,accepted_steady_ns,done_steady_ns,accepted,pending_before,pending_after"
                                << " ingress_history=[";
                            for (size_t i = 0; i < ingress.history_count; ++i) {
                                const auto & event = ingress.history[i];
                                if (i) evidence << ';';
                                evidence << event.source_sample_timestamp_us << ','
                                    << static_cast<unsigned>(event.reset_counter) << ','
                                    << event.callback_receipt_ros_ns << ','
                                    << event.callback_entry_steady_ns << ','
                                    << event.lock_acquired_steady_ns << ','
                                    << event.accepted_steady_ns << ','
                                    << event.completed_steady_ns << ','
                                    << event.accepted << ',' << event.pending_before << ','
                                    << event.pending_after;
                            }
                            evidence << ']';
                        }
                        RCLCPP_ERROR(node()->get_logger(),
                            "Object approach timing-fault evidence: %s", evidence.str().c_str());
                    } catch (...) {
                        // Diagnostic collection cannot change the failed control result.
                    }
                }
                throw std::runtime_error(reason);
            }
            RCLCPP_INFO_THROTTLE(node()->get_logger(), *node()->get_clock(), 5000,
                "Object approach tracking request=%s nominal=[%.3f,%.3f,%.3f] "
                "command_error_m=%.3f correction_m=%.3f saturated=%s",
                binding.request_identity.c_str(),
                target_reference.position()(0), target_reference.position()(1),
                target_reference.position()(2),
                (ref.position() - measured->state.position()).norm(),
                object_tracking_session_->correction().norm(),
                object_tracking_session_->saturated() ? "true" : "false");
        } else if (first_iteration_ && terminal_start_reference_ && !compute_with_mpc) {
            ref = trajectory_generator_client_->ComputeReference(
                *terminal_start_reference_, target_reference, set_reference, reset,
                trajectory_mode_t::positional);
        } else {
            State planner_state = state;
            if (first_iteration_ && terminal_start_reference_) {
                const auto & seed = *terminal_start_reference_;
                planner_state = State(seed.position(), seed.velocity(), seed.yaw(),
                    vector_t(0.0, 0.0, seed.yaw_rate()), seed.stamp());
            }
            ref = trajectory_generator_client_->ComputeReference(
                planner_state, target_reference, set_reference, reset,
                trajectory_mode_t::positional, compute_with_mpc);
        }

    } catch (const std::runtime_error &e) {

        RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::computeReference(): Failed to compute reference, exception: %s", e.what());
        if (object_tracking_session_) {
            ref = object_tracking_session_->FailureReference(node()->now(), e.what());
        } else {
            has_failed_ = true;
            ref = Reference(state,true,true);
        }

    }

    if (first_iteration_) {

        first_iteration_ = false;
        terminal_start_reference_.reset();

    }

    if (mpc_settle_first_iteration_) {

        mpc_settle_first_iteration_ = false;

    }

    return ref;

}

bool FlyToObjectManeuverServer::hasSucceeded(Maneuver & maneuver) {

    if (object_tracking_session_ && object_tracking_session_->failed()) return false;

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    if (!active_target_reference_valid_) {
        return false;
    }

    Reference target_reference = active_target_reference_.Load();
    Reference filtered_reference = target_reference;

    if (hasFailed(maneuver)) {
        return false;
    }
    if (object_tracking_session_ && object_tracking_session_->failed()) return false;

    const bool use_mpc = configuration_->GetParameter(
        "/control/maneuver_controller/fly_to_object_use_mpc").as_bool();
    if (object_tracking_session_ && !use_mpc) {
        const auto observation = nominal_target_observation_.Load();
        const auto binding = currentReferenceBinding();
        const auto now = node()->now();
        const auto steady_now = std::chrono::steady_clock::now();
        const auto max_age = std::chrono::milliseconds(configuration_->GetParameter(
            "/control/maneuver_controller/reference_stream_timeout_ms").as_int());
        const auto max_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            max_age).count();
        if (!observation ||
            observation->request_identity != maneuver.requestIdentity() ||
            observation->request_identity != binding.request_identity ||
            observation->execution_id == 0 ||
            observation->execution_id != binding.execution_id ||
            binding.reference_provider_name != action_name() ||
            !object_tracking_session_->owns(
                observation->request_identity, observation->execution_id) ||
            observation->observed_at.get_clock_type() != now.get_clock_type() ||
            observation->observed_at.nanoseconds() > now.nanoseconds() ||
            now.nanoseconds() - observation->observed_at.nanoseconds() > max_age_ns ||
            observation->received_at > steady_now ||
            steady_now - observation->received_at > max_age) {
            return false;
        }
        target_reference = observation->nominal;
        filtered_reference = observation->filtered;
    }

    const double position_distance = (state.position() - target_reference.position()).norm();
    const double yaw_error = std::abs(shortestCableAxisYawError(state.yaw(), target_reference.yaw()));
    const double distance = std::hypot(position_distance, yaw_error);

    const double filtered_distance = std::hypot(
        (state.position() - filtered_reference.position()).norm(),
        std::abs(shortestCableAxisYawError(state.yaw(), filtered_reference.yaw())));
    const bool vehicle_reached_target = distance < configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double();
    bool succeeded = vehicle_reached_target;
    bool final_reference_streamed = false;
    bool first_reference_applied = false;

    if (vehicle_reached_target && !threshold_reached_logged_) {
        threshold_reached_logged_ = true;
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasSucceeded(): timing: vehicle reached threshold after %.3f s. target_id=%d nominal_distance=%.3f filtered_distance=%.3f position_distance=%.3f yaw_error=%.3f",
            (node()->now() - maneuver_start_time_).seconds(),
            target_adapter_->target_id(),
            distance,
            filtered_distance,
            position_distance,
            yaw_error
        );
    }

    if (use_mpc) {
        if (!mpc_settle_active_) {
            if (!vehicle_reached_target) {
                return false;
            }

            // Freeze the same target that satisfied the terminal threshold.
            // Re-querying the live target here can block against the concurrent
            // reference callback and can also move the terminal goal between the
            // threshold check and interpolation settle.
            mpc_settle_target_reference_ = target_reference;
            mpc_settle_active_ = true;
            mpc_settle_first_iteration_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToObjectManeuverServer::hasSucceeded(): MPC reached threshold; entering final interpolation settle to target_id=%d position=[%.3f, %.3f, %.3f], yaw=%.3f",
                target_adapter_->target_id(),
                target_reference.position()(0),
                target_reference.position()(1),
                target_reference.position()(2),
                target_reference.yaw()
            );
            return false;
        }

        target_reference = mpc_settle_target_reference_;
        const double settle_position_distance = (state.position() - target_reference.position()).norm();
        const double settle_yaw_error = std::abs(shortestCableAxisYawError(state.yaw(), target_reference.yaw()));
        const double settle_distance = std::hypot(settle_position_distance, settle_yaw_error);
        const bool vehicle_reached_settle_target =
            settle_distance < configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double();
        final_reference_streamed = interpolationFinalReferenceStreamed(target_reference);
        if (vehicle_reached_settle_target && !settle_threshold_reached_logged_) {
            settle_threshold_reached_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToObjectManeuverServer::hasSucceeded(): timing: post-MPC settle reached threshold after %.3f s. target_id=%d distance=%.3f position_distance=%.3f yaw_error=%.3f final_reference_streamed=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                target_adapter_->target_id(),
                settle_distance,
                settle_position_distance,
                settle_yaw_error,
                final_reference_streamed ? "true" : "false"
            );
        }
        if (final_reference_streamed && !final_reference_streamed_logged_) {
            final_reference_streamed_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToObjectManeuverServer::hasSucceeded(): timing: interpolation final reference streamed after %.3f s. target_id=%d distance=%.3f position_distance=%.3f yaw_error=%.3f vehicle_reached_threshold=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                target_adapter_->target_id(),
                settle_distance,
                settle_position_distance,
                settle_yaw_error,
                vehicle_reached_settle_target ? "true" : "false"
            );
        }
        succeeded = vehicle_reached_settle_target && final_reference_streamed;
        if (!succeeded) {
            RCLCPP_DEBUG_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "FlyToObjectManeuverServer::hasSucceeded(): waiting for post-MPC interpolation settle. distance=%.3f position_distance=%.3f yaw_error=%.3f",
                settle_distance,
                settle_position_distance,
                settle_yaw_error
            );
        }
    } else {
        // A non-MPC fly-to-object reference is recomputed continuously from
        // the moving object.  Once the vehicle reaches that live target,
        // querying trajectory history here is both redundant and unsafe: the
        // reference callback may be updating the same history concurrently.
        // The success handoff immediately installs HoverByObject, which keeps
        // streaming an object-relative reference without releasing control.
        first_reference_applied = first_reference_applied_ &&
            first_reference_applied_(maneuver.requestIdentity());
        succeeded = vehicle_reached_target && first_reference_applied;
        if (succeeded && object_tracking_session_ && !object_hover_ready_) {
            const auto maneuvers = registered_maneuvers();
            const auto hover_entry = maneuvers.find(
                MANEUVER_TYPE_HOVER_BY_OBJECT);
            const auto binding = currentReferenceBinding();
            object_hover_ready_ = hover_entry != maneuvers.end() &&
                std::static_pointer_cast<HoverByObjectManeuverServer>(hover_entry->second)
                    ->UpdateTracked(target_adapter_.Load(), object_tracking_session_,
                        maneuver.requestIdentity(), binding.execution_id,
                        configuration_->GetParameter(
                            "/control/maneuver_controller/minimum_target_altitude").as_double());
            if (!object_hover_ready_) {
                object_tracking_session_->Fail(
                    "object hover continuation could not validate current target");
                succeeded = false;
            }
        }
    }

    if (succeeded && !success_timing_logged_) {
        success_timing_logged_ = true;
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasSucceeded(): timing: succeeded after %.3f s. target_id=%d distance=%.3f position_distance=%.3f yaw_error=%.3f final_reference_streamed=%s first_reference_applied=%s",
            (node()->now() - maneuver_start_time_).seconds(),
            target_adapter_->target_id(),
            distance,
            position_distance,
            yaw_error,
            final_reference_streamed ? "true" : "false",
            first_reference_applied ? "true" : "false"
        );
    }

    return succeeded;

}

bool FlyToObjectManeuverServer::hasFailed(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    if (object_tracking_session_ && object_tracking_session_->failed()) {
        if (!object_tracking_session_->stopComplete() &&
            !object_tracking_session_->unrecoverable()) return false;
        if (object_tracking_session_->stopComplete() && !object_failure_hold_) {
            try {
                const Reference rest = object_tracking_session_->lastCommand();
                object_failure_hold_ = std::make_shared<TerminalTrackingHold>(
                    rest, cda_handler, node()->get_clock(),
                    TerminalTrackingHold::Clearance{}, 0.0);
                object_failure_hold_->Fail("object approach failed after bounded command stop");
                auto hover = std::static_pointer_cast<HoverManeuverServer>(
                    registered_maneuvers().at(MANEUVER_TYPE_HOVER));
                hover->AdoptTerminalHold(object_failure_hold_, maneuver.requestIdentity());
            } catch (const std::exception & error) {
                RCLCPP_ERROR(node()->get_logger(),
                    "Object approach could not retain its bounded failure stop: %s", error.what());
            }
        }
        return true;
    }

    bool is_in_flight_or_on_current_cable = cda_handler->in_flight() || (
        cda_handler->on_cable() && 
        cda_handler->on_cable_id() == target_adapter_->target_id() &&
        target_adapter_->target_type() == TARGET_TYPE_CABLE
    );

    if (!is_in_flight_or_on_current_cable) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Drone is not in flight, and not on the target cable, returning true.");
        return true;
    }

    if (!cda_handler->offboard()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Drone is not in offboard mode, returning true.");
        return true;
    }

    if (!cda_handler->armed()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): Drone is not armed, returning true.");
        return true;
    }

    TargetAdapter active_target_adapter = cda_handler->target_adapter();
    if (active_target_adapter != *target_adapter_) {
        if (object_tracking_session_) {
            object_tracking_session_->Fail("object target adapter changed");
            return false;
        }
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasFailed(): Target adapter has changed, returning true. expected(type=%d,id=%d,frame=%s) active(type=%d,id=%d,frame=%s)",
            target_adapter_->target_type(),
            target_adapter_->target_id(),
            target_adapter_->reference_frame_id().c_str(),
            active_target_adapter.target_type(),
            active_target_adapter.target_id(),
            active_target_adapter.reference_frame_id().c_str()
        );
        return true;
    }

    try {
        (void)cda_handler->ComputeTargetTransform(*target_adapter_);
        markTargetObserved();
    } catch (const std::runtime_error & e) {
        nominal_target_observation_.Store(nullptr);
        if (active_target_reference_valid_ && targetLossWithinGrace()) {
            RCLCPP_WARN_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "FlyToObjectManeuverServer::hasFailed(): Target temporarily unavailable; retaining last valid target during grace window: %s",
                e.what()
            );
            return false;
        }
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasFailed(): Target transform is not currently computable, returning true: %s",
            e.what()
        );
        if (object_tracking_session_) {
            object_tracking_session_->Fail(std::string("object transform lost: ") + e.what());
            return false;
        }
        return true;
    }

    if (has_failed_) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::hasFailed(): The has_failed_ flag is set, returning true.");
        return true;
    }

    return false;

}

std::shared_ptr<void> FlyToObjectManeuverServer::getFeedback(Maneuver &) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::FlyToObject::Feedback>();

    ReferenceTrajectory reference_trajectory;

    try {
        reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    } catch (const std::runtime_error &e) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::getFeedback(): Failed to get reference trajectory, exception: %s", e.what());
        return std::static_pointer_cast<void>(feedback);
    }

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    State state = awareness_handler()->GetState();
    Reference target_reference;
    
    try {
        target_reference = getUpdatedTargetReference(state);
    } catch (const std::runtime_error &e) {
        if (active_target_reference_valid_ && targetLossWithinGrace()) {
            target_reference = active_target_reference_.Load();
            RCLCPP_WARN_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "FlyToObjectManeuverServer::getFeedback(): Target temporarily unavailable; reporting last valid target during grace window: %s",
                e.what()
            );
        } else {
            RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::getFeedback(): Failed to get updated target reference, exception: %s", e.what());
            has_failed_ = true;
            return std::static_pointer_cast<void>(feedback);
        }
    }

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    feedback->distance_vehicle_to_target = (target_reference.position() - state.position()).norm();

    return std::static_pointer_cast<void>(feedback);

}

void FlyToObjectManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::FlyToObject::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToObject>(maneuver.goal_handle());
    auto set_best_known_target_reference = [&]() {
        if (mpc_settle_active_) {
            result->target_reference = ReferenceAdapter(mpc_settle_target_reference_.Load()).ToMsg();
            return;
        }
        if (active_target_reference_valid_) {
            result->target_reference = ReferenceAdapter(active_target_reference_.Load()).ToMsg();
            return;
        }
        try {
            result->target_reference = ReferenceAdapter(getUpdatedTargetReference(awareness_handler()->GetState())).ToMsg();
        } catch (const std::runtime_error &e) {
            RCLCPP_WARN(
                node()->get_logger(),
                "FlyToObjectManeuverServer::publishResultAndFinalize(): Could not set target_reference for failed result, exception: %s",
                e.what()
            );
        }
    };

    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            result->success = true;
            if (mpc_settle_active_) {
                result->target_reference = ReferenceAdapter(mpc_settle_target_reference_).ToMsg();
            } else if (active_target_reference_valid_) {
                result->target_reference = ReferenceAdapter(active_target_reference_.Load()).ToMsg();
            } else {
                Reference target_reference;
                try {
                    target_reference = getUpdatedTargetReference(awareness_handler()->GetState());
                } catch (const std::runtime_error &e) {
                    RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::publishResultAndFinalize(): Failed to get updated target reference, exception: %s", e.what());
                    result->success = false;
                    goal_handle->abort(result);
                    awareness_handler()->ClearTarget();
                    break;
                }
                result->target_reference = ReferenceAdapter(target_reference).ToMsg();
            }
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            result->success = false;
            set_best_known_target_reference();
            goal_handle->abort(result);
            awareness_handler()->ClearTarget();
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            result->success = false;
            set_best_known_target_reference();
            goal_handle->canceled(result);
            awareness_handler()->ClearTarget();
            break;
    }

}

void FlyToObjectManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    auto registered_hover_by_object_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER_BY_OBJECT);

    std::shared_ptr<HoverByObjectManeuverServer> hover_by_object_maneuver_server = std::static_pointer_cast<HoverByObjectManeuverServer>(registered_hover_by_object_maneuver->second);

    const auto binding = currentReferenceBinding();
    const bool hover_ready = object_tracking_session_
        ? object_hover_ready_ &&
            hover_by_object_maneuver_server->RetainsTrackedSource(binding)
        : hover_by_object_maneuver_server->Update(target_adapter_.Load());
    if (hover_ready) {

        registerCallback(
            std::bind(
                &HoverByObjectManeuverServer::GetReference,
                hover_by_object_maneuver_server,
                std::placeholders::_1
            )
        );

        return;

    }

    if (object_tracking_session_) {
        auto session = object_tracking_session_;
        session->Fail("object hover continuation lost before callback publication");
        registerCallback([session, clock = node()->get_clock()](const State &) {
            return session->FailureReference(clock->now(),
                "object hover continuation lost before callback publication");
        });
        RCLCPP_ERROR(node()->get_logger(),
            "Tracked object continuation failed; retaining its bounded command stop");
        return;
    }

    RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::registerReferenceCallbackOnSuccess(): Failed to register hover by object reference callback on success, registering hover maneuver instead.");

    auto registered_hover_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER);

    std::shared_ptr<HoverManeuverServer> hover_maneuver_server = std::static_pointer_cast<HoverManeuverServer>(registered_hover_maneuver->second);

    hover_maneuver_server->Update(awareness_handler()->GetState());

    registerCallback(
        std::bind(
            &HoverManeuverServer::GetReference,
            hover_maneuver_server,
            std::placeholders::_1
        )
    );

}

Reference FlyToObjectManeuverServer::updateLiveTargetReference(
    const State & state, const ReferenceCallbackBinding & binding
) {
    // The nominal reference is normalized and altitude-clamped by the normal
    // target lookup. Only its position is filtered for command generation.
    // A failed lookup must not make the previous nominal observation fresh.
    Reference nominal;
    try {
        nominal = getUpdatedTargetReference(state);
    } catch (...) {
        nominal_target_observation_.Store(nullptr);
        throw;
    }
    const Reference filtered = filterTargetPositionReference(nominal, state);
    active_target_reference_ = filtered;
    active_target_reference_valid_ = true;
    if (object_tracking_session_ &&
        object_tracking_session_->owns(binding.request_identity, binding.execution_id)) {
        auto observation = std::make_shared<NominalTargetObservation>();
        observation->nominal = nominal;
        observation->filtered = filtered;
        observation->request_identity = binding.request_identity;
        observation->execution_id = binding.execution_id;
        observation->observed_at = node()->now();
        observation->received_at = std::chrono::steady_clock::now();
        nominal_target_observation_.Store(observation);
    } else {
        nominal_target_observation_.Store(nullptr);
    }
    return filtered;
}

Reference FlyToObjectManeuverServer::getUpdatedTargetReference(const iii_drone::control::State & state) {

    auto cda_handler = awareness_handler();

    Reference reference = enforceMinimumTargetAltitude(referenceWithCableAxisYawClosestTo(
        Reference(cda_handler->ComputeTargetState(target_adapter_)),
        state.yaw()
    ));
    markTargetObserved();

    RCLCPP_DEBUG_THROTTLE(
        node()->get_logger(),
        *node()->get_clock(),
        1000,
        "FlyToObjectManeuverServer::getUpdatedTargetReference(): target_id=%d reference=[%.3f, %.3f, %.3f, yaw=%.3f]",
        target_adapter_->target_id(),
        reference.position()(0),
        reference.position()(1),
        reference.position()(2),
        reference.yaw()
    );

    return reference;

}

void FlyToObjectManeuverServer::markTargetObserved() {
    last_target_observation_ns_.store(node()->now().nanoseconds());
}

bool FlyToObjectManeuverServer::targetLossWithinGrace() const {
    const double grace_s = configuration_->GetParameter(
        "/control/maneuver_controller/fly_to_object_target_loss_grace_s"
    ).as_double();
    const int64_t last_seen_ns = last_target_observation_ns_.load();
    if (grace_s <= 0.0 || last_seen_ns <= 0) {
        return false;
    }
    const int64_t elapsed_ns = node()->now().nanoseconds() - last_seen_ns;
    return elapsed_ns >= 0 && static_cast<double>(elapsed_ns) <= grace_s * 1.0e9;
}

Reference FlyToObjectManeuverServer::enforceMinimumTargetAltitude(
    const iii_drone::control::Reference & reference
) const {

    const double minimum_z = awareness_handler()->ground_altitude_estimate()
        + configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double();

    point_t position = reference.position();
    if (position(2) >= minimum_z) {
        return reference;
    }

    RCLCPP_WARN(
        node()->get_logger(),
        "FlyToObjectManeuverServer::enforceMinimumTargetAltitude(): Clamping fly-to-object target z from %.3f to %.3f",
        position(2),
        minimum_z
    );
    position(2) = minimum_z;

    return Reference(
        position,
        reference.yaw(),
        reference.velocity(),
        reference.yaw_rate(),
        reference.acceleration(),
        reference.yaw_acceleration(),
        reference.stamp()
    );

}

Reference FlyToObjectManeuverServer::filterTargetPositionReference(
    const iii_drone::control::Reference & raw_reference,
    const iii_drone::control::State & state
) {

    const double time_constant_s = configuration_->GetParameter(
        "/control/maneuver_controller/fly_to_object_target_low_pass_time_constant_s"
    ).as_double();

    if (time_constant_s <= 0.0) {
        target_position_filter_initialized_ = false;
        return raw_reference;
    }

    const rclcpp::Time now = node()->now();

    double dt_s = 0.0;
    if (!target_position_filter_initialized_) {
        filtered_target_position_ = state.position();
        target_position_filter_initialized_ = true;
    } else {
        dt_s = (now - last_target_position_filter_update_time_).seconds();
    }

    if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 1.0) {
        dt_s = static_cast<double>(
            configuration_->GetParameter("/control/maneuver_controller/maneuver_execution_period_ms").as_int()
        ) / 1000.0;
    }

    last_target_position_filter_update_time_ = now;

    const double alpha = std::clamp(dt_s / (time_constant_s + dt_s), 0.0, 1.0);
    const point_t previous_filtered_position = filtered_target_position_;
    const point_t raw_position = raw_reference.position();
    filtered_target_position_ = previous_filtered_position + alpha * (raw_position - previous_filtered_position);

    RCLCPP_DEBUG_THROTTLE(
        node()->get_logger(),
        *node()->get_clock(),
        1000,
        "FlyToObjectManeuverServer::filterTargetPositionReference(): target_id=%d raw=[%.3f, %.3f, %.3f] filtered=[%.3f, %.3f, %.3f] alpha=%.3f tau=%.3f dt=%.3f",
        target_adapter_->target_id(),
        raw_position(0),
        raw_position(1),
        raw_position(2),
        filtered_target_position_(0),
        filtered_target_position_(1),
        filtered_target_position_(2),
        alpha,
        time_constant_s,
        dt_s
    );

    return Reference(
        filtered_target_position_,
        raw_reference.yaw(),
        raw_reference.velocity(),
        raw_reference.yaw_rate(),
        raw_reference.acceleration(),
        raw_reference.yaw_acceleration(),
        raw_reference.stamp()
    );

}

bool FlyToObjectManeuverServer::interpolationFinalReferenceStreamed(
    const iii_drone::control::Reference & target_reference
) const {

    ReferenceTrajectory trajectory;
    try {
        trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    } catch (const std::runtime_error & e) {
        RCLCPP_DEBUG_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "FlyToObjectManeuverServer::interpolationFinalReferenceStreamed(): No trajectory available yet: %s",
            e.what()
        );
        return false;
    }

    if (trajectory.references().empty()) {
        return false;
    }

    const Reference streamed_reference = trajectory.references().front();

    const double position_error = (streamed_reference.position() - target_reference.position()).norm();
    const double yaw_error = std::abs(shortestCableAxisYawError(streamed_reference.yaw(), target_reference.yaw()));
    const double velocity_norm = streamed_reference.velocity().norm();
    const double yaw_rate_abs = std::abs(streamed_reference.yaw_rate());
    const double acceleration_norm = streamed_reference.acceleration().norm();
    const double yaw_acceleration_abs = std::abs(streamed_reference.yaw_acceleration());

    const bool final_reference_streamed =
        position_error <= kFinalReferencePositionToleranceM &&
        yaw_error <= kFinalReferenceYawToleranceRad &&
        velocity_norm <= kFinalReferenceVelocityToleranceMps &&
        yaw_rate_abs <= kFinalReferenceYawRateToleranceRadps &&
        acceleration_norm <= kFinalReferenceAccelerationToleranceMps2 &&
        yaw_acceleration_abs <= kFinalReferenceYawAccelerationToleranceRadps2;

    if (!final_reference_streamed) {
        const TargetAdapter target_adapter = target_adapter_.Load();
        RCLCPP_DEBUG_THROTTLE(
            node()->get_logger(),
            *node()->get_clock(),
            1000,
            "FlyToObjectManeuverServer::interpolationFinalReferenceStreamed(): target_id=%d position_error=%.4f yaw_error=%.4f velocity_norm=%.4f yaw_rate=%.4f acceleration_norm=%.4f yaw_acceleration=%.4f",
            target_adapter.target_id(),
            position_error,
            yaw_error,
            velocity_norm,
            yaw_rate_abs,
            acceleration_norm,
            yaw_acceleration_abs
        );
    }

    return final_reference_streamed;

}

bool FlyToObjectManeuverServer::validateAwarenessAndParameters(
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness,
    const fly_to_object_maneuver_params_t & params
) const {

    RCLCPP_DEBUG(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Validating awareness and parameters.");

    auto cda_handler = awareness_handler();

    if (params.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Target type is not TARGET_TYPE_CABLE, returning false.");
        return false;
    }

    if (!drone_awareness.offboard()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Drone is not in offboard mode, returning false.");
        return false;
    }

    if (!drone_awareness.armed()) {
        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Drone is not armed, returning false.");
        return false;
    }

    bool is_in_flight_or_on_current_cable = drone_awareness.in_flight() || (
        drone_awareness.on_cable() && 
        drone_awareness.on_cable_id() == params.target_adapter.target_id() &&
        params.target_adapter.target_type() == TARGET_TYPE_CABLE
    );

    if (!is_in_flight_or_on_current_cable) {
        RCLCPP_WARN(
            node()->get_logger(), 
            "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Drone is not in flight, and not on the target cable, returning false."
        );
        return false;
    }

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(params.target_adapter);

    } catch (const std::runtime_error &e) {

        RCLCPP_WARN(node()->get_logger(), "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Failed to compute target transform, returning false. Exception: %s", e.what());

        return false;

    }

    Reference target_reference = enforceMinimumTargetAltitude(Reference(State(
        target_transform.block<3, 1>(0, 3),
        vector_t::Zero(),
        matToQuat(target_transform.block<3, 3>(0, 0)),
        vector_t::Zero()
    )));
    point_t target_position_in_world_frame = target_reference.position();

    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate() >= configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double();

    if (!target_position_valid) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToObjectManeuverServer::validateAwarenessAndParameters(): Target position is not valid after minimum-altitude clamp, target_z=%.3f ground=%.3f min_altitude=%.3f, returning false.",
            target_position_in_world_frame[2],
            cda_handler->ground_altitude_estimate(),
            configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double()
        );
        return false;
    }

    return true;

}
