/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

using namespace iii_drone::control::maneuver;
using namespace iii_drone::control;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

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

}  // namespace

/*****************************************************************************/
// Impementation:
/*****************************************************************************/

HoverByObjectManeuverServer::HoverByObjectManeuverServer(
    rclcpp_lifecycle::LifecycleNode * node,
    CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
    const std::string & action_name,
    unsigned int wait_for_execute_poll_ms,
    unsigned int evaluate_done_poll_ms,
    bool use_nans,
    double max_euc_dist
) : ManeuverServer(
    node, 
    combined_drone_awareness_handler, 
    action_name, 
    wait_for_execute_poll_ms, 
    evaluate_done_poll_ms
),  has_target_(false),
    has_on_fail_callback_(false),
    use_nans_(use_nans), 
    max_euc_dist_(max_euc_dist) {

    createServer<HoverByObject>();

}

bool HoverByObjectManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_HOVER_BY_OBJECT) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverByObjectManeuverServer::CanExecuteManeuver(): Maneuver type is not HoverByObject"
        );
        return false;
    }

    hover_by_object_maneuver_params_t params(maneuver.maneuver_params());

    if (params.target_adapter.target_type() != TARGET_TYPE_CABLE) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverByObjectManeuverServer::CanExecuteManeuver(): Target type is not CABLE"
        );
        return false;
    }

    if (!drone_awareness.offboard()) {
        logNotOffboard("HoverByObjectManeuverServer::CanExecuteManeuver(): Drone is not offboard");
        return false;
    }

    if (!drone_awareness.armed()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverByObjectManeuverServer::CanExecuteManeuver(): Drone is not armed"
        );
        return false;
    }

    if (!drone_awareness.in_flight()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverByObjectManeuverServer::CanExecuteManeuver(): Drone is not in flight"
        );
        return false;
    }

    if (!validateAwareness(drone_awareness)) {
        RCLCPP_WARN(
            node()->get_logger(),
            "HoverByObjectManeuverServer::CanExecuteManeuver(): Target is not currently valid; accepting maneuver with hover fallback"
        );
    }

    return true;

}

iii_drone::adapters::CombinedDroneAwarenessAdapter HoverByObjectManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {

    hover_by_object_maneuver_params_t params(maneuver.maneuver_params());

    TargetAdapter target_adapter = params.target_adapter;

    State target_state = awareness_handler()->ComputeTargetState(target_adapter);

    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness_after = awareness_handler()->adapter();
    awareness_after.armed() = true;
    awareness_after.offboard() = true;
    awareness_after.target_adapter() = target_adapter_;
    awareness_after.target_position_known() = true;
    awareness_after.drone_location() = DRONE_LOCATION_IN_FLIGHT;
    awareness_after.state() = target_state;
                      
    return awareness_after;

}

void HoverByObjectManeuverServer::RegisterOnFailCallback(std::function<void()> on_fail_callback) {

    on_fail_callback_ = on_fail_callback;
    has_on_fail_callback_ = true;

}

void HoverByObjectManeuverServer::RegisterFirstReferenceAppliedCallback(
    std::function<bool(const std::string &)> callback) {
    first_reference_applied_ = std::move(callback);
}

void HoverByObjectManeuverServer::RegisterAppliedRestReferenceCallback(
    std::function<bool(const std::string &, const Reference &)> callback) {
    applied_rest_reference_ = std::move(callback);
}

bool HoverByObjectManeuverServer::UpdateTracked(
    const TargetAdapter & target_adapter,
    std::shared_ptr<ObjectTrackingSession> session,
    std::string source_request_identity,
    uint64_t source_execution_id,
    double minimum_altitude_above_ground_m) {
    if (!session || source_request_identity.empty() || source_execution_id == 0 ||
        !std::isfinite(minimum_altitude_above_ground_m) ||
        !session->owns(source_request_identity, source_execution_id) ||
        !Update(target_adapter)) return false;
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    object_tracking_session_ = std::move(session);
    object_owner_request_identity_ = std::move(source_request_identity);
    object_owner_execution_id_ = source_execution_id;
    object_minimum_altitude_above_ground_m_ = minimum_altitude_above_ground_m;
    return true;
}

bool HoverByObjectManeuverServer::CanAdoptTrackedSession(
    const Maneuver & successor, const ReferenceCallbackBinding & predecessor) const {
    if (successor.maneuver_type() != MANEUVER_TYPE_HOVER_BY_OBJECT ||
        predecessor.request_identity.empty() || predecessor.execution_id == 0) return false;
    const hover_by_object_maneuver_params_t params(successor.maneuver_params());
    const TargetAdapter current_target = target_adapter_.Load();
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    return object_tracking_session_ &&
        object_tracking_session_->owns(predecessor.request_identity, predecessor.execution_id) &&
        object_owner_request_identity_ == predecessor.request_identity &&
        object_owner_execution_id_ == predecessor.execution_id &&
        params.target_adapter.target_type() == current_target.target_type() &&
        params.target_adapter.target_id() == current_target.target_id();
}

bool HoverByObjectManeuverServer::HasMatchingTrackedSession(
    const Maneuver & successor) const {
    if (successor.maneuver_type() != MANEUVER_TYPE_HOVER_BY_OBJECT) return false;
    const hover_by_object_maneuver_params_t params(successor.maneuver_params());
    const TargetAdapter current_target = target_adapter_.Load();
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    return object_tracking_session_ &&
        params.target_adapter.target_type() == current_target.target_type() &&
        params.target_adapter.target_id() == current_target.target_id();
}

bool HoverByObjectManeuverServer::HasTrackedSourceIdentity(
    const ReferenceCallbackBinding & source) const {
    if (!source.callback || source.request_identity.empty() || source.execution_id == 0) {
        return false;
    }
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    return object_tracking_session_ &&
        object_owner_request_identity_ == source.request_identity &&
        object_owner_execution_id_ == source.execution_id;
}

bool HoverByObjectManeuverServer::RetainsTrackedSource(
    const ReferenceCallbackBinding & source) const {
    if (!source.callback || source.request_identity.empty() || source.execution_id == 0) {
        return false;
    }
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    return object_tracking_session_ &&
        object_owner_request_identity_ == source.request_identity &&
        object_owner_execution_id_ == source.execution_id &&
        object_tracking_session_->owns(source.request_identity, source.execution_id);
}

bool HoverByObjectManeuverServer::TrackedSourceUnrecoverable(
    const ReferenceCallbackBinding & source) const {
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    return object_tracking_session_ && source.callback &&
        object_owner_request_identity_ == source.request_identity &&
        object_owner_execution_id_ == source.execution_id &&
        object_tracking_session_->owns(source.request_identity, source.execution_id) &&
        object_tracking_session_->unrecoverable();
}

bool HoverByObjectManeuverServer::TrackedSourceFailed(
    const ReferenceCallbackBinding & source) const {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        if (!object_tracking_session_ || !source.callback ||
            object_owner_request_identity_ != source.request_identity ||
            object_owner_execution_id_ != source.execution_id) return false;
        tracking = object_tracking_session_;
    }
    return tracking->owns(source.request_identity, source.execution_id) &&
        tracking->failed();
}

std::optional<Reference> HoverByObjectManeuverServer::TrackedFailureRest(
    const ReferenceCallbackBinding & source) const {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        if (!object_tracking_session_ || !source.callback ||
            object_owner_request_identity_ != source.request_identity ||
            object_owner_execution_id_ != source.execution_id) return std::nullopt;
        tracking = object_tracking_session_;
    }
    if (!tracking->owns(source.request_identity, source.execution_id) ||
        !tracking->stopComplete()) return std::nullopt;
    return tracking->lastCommand();
}

bool HoverByObjectManeuverServer::HasTrackedSession() const {
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    return static_cast<bool>(object_tracking_session_);
}

bool HoverByObjectManeuverServer::RequestTrackedTransitionStop(
    const ReferenceCallbackBinding & source) {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        if (!object_tracking_session_ || !source.callback ||
            source.request_identity != object_owner_request_identity_ ||
            source.execution_id != object_owner_execution_id_) return false;
        tracking = object_tracking_session_;
    }
    return tracking->RequestTransitionStop(source.request_identity, source.execution_id);
}

bool HoverByObjectManeuverServer::TrackedTransitionStopping(
    const ReferenceCallbackBinding & source) const {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        if (!object_tracking_session_ || !source.callback ||
            source.request_identity != object_owner_request_identity_ ||
            source.execution_id != object_owner_execution_id_) return false;
        tracking = object_tracking_session_;
    }
    return tracking->transitionStopping();
}

std::optional<Reference> HoverByObjectManeuverServer::TrackedTransitionRest(
    const ReferenceCallbackBinding & source) const {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        if (!object_tracking_session_ || !source.callback ||
            source.request_identity != object_owner_request_identity_ ||
            source.execution_id != object_owner_execution_id_) return std::nullopt;
        tracking = object_tracking_session_;
    }
    if (!tracking->transitionRest()) return std::nullopt;
    return tracking->lastCommand();
}

void HoverByObjectManeuverServer::RetireTrackedSource(
    const ReferenceCallbackBinding & source) {
    std::lock_guard<std::mutex> lock(object_tracking_mutex_);
    if (source.request_identity != object_owner_request_identity_ ||
        source.execution_id != object_owner_execution_id_) return;
    object_tracking_session_.reset();
    object_owner_request_identity_.clear();
    object_owner_execution_id_ = 0;
}

bool HoverByObjectManeuverServer::Update(const iii_drone::adapters::TargetAdapter &target_adapter) {

    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        object_tracking_session_.reset();
        object_failure_hold_.reset();
        object_owner_request_identity_.clear();
        object_owner_execution_id_ = 0;
    }

    transform_matrix_t target_transform;
    
    if (
        !validateAwareness(
            awareness_handler()->adapter(),
            target_adapter
        )
    ) {

        RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
            "HoverByObjectManeuverServer::Update(): Failed to validate awareness for target_id=%d",
            target_adapter.target_id());

        has_target_ = false;

        return false;

    }

    has_target_ = true;

    target_adapter_ = target_adapter;

    return true;

}

iii_drone::control::Reference HoverByObjectManeuverServer::GetReference(const iii_drone::control::State & state) {

    // A successor owns only the already certified bounded stop. Target loss or
    // fresh perception during this transition cannot restart correction.
    std::shared_ptr<ObjectTrackingSession> transition_tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        transition_tracking = object_tracking_session_;
    }
    // An entered Compute may hold the session mutex while planning. Do not
    // hold the outer owner mutex while waiting for either session operation:
    // the scheduler must still read exact owner identity on its timer tick.
    if (transition_tracking && transition_tracking->transitionStopping()) {
        return transition_tracking->TransitionReference(node()->now());
    }

    if (!has_on_fail_callback_) {

        throw std::runtime_error("No on fail callback registered for HoverByObjectManeuverServer");

    }

    // if (!has_target_) {

    //     throw std::runtime_error("No target set for HoverByObjectManeuverServer");

    // }

    transform_matrix_t target_transform;
    
    try {

        target_transform = awareness_handler()->ComputeTargetTransform(target_adapter_);

    } catch (const std::runtime_error &e) {

        has_target_ = false;

    }

    if (!validateAwareness(awareness_handler()->adapter())) {

        has_target_ = false;

    }

    vector_t position;
    double yaw;
    vector_t velocity;
    double yaw_rate;
    vector_t acceleration;
    double yaw_acceleration;

    std::shared_ptr<ObjectTrackingSession> tracking;
    std::string owner_request;
    uint64_t owner_execution = 0;
    double minimum_altitude_above_ground = 0.0;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        tracking = object_tracking_session_;
        owner_request = object_owner_request_identity_;
        owner_execution = object_owner_execution_id_;
        minimum_altitude_above_ground = object_minimum_altitude_above_ground_m_;
    }
    if (tracking) {
        if (!tracking->owns(owner_request, owner_execution)) {
            return tracking->lastCommand();
        }
        if (tracking->failed()) {
            return tracking->FailureReference(node()->now(), tracking->failureReason());
        }
        if (!has_target_) {
            const Reference stopping = tracking->FailureReference(
                node()->now(), "object hover target is unavailable");
            on_fail_callback_();
            return stopping;
        }
        const pose_t pose = poseFromTransformMatrix(target_transform);
        const double raw_yaw = quatToEul(pose.orientation)(2);
        const Reference nominal(pose.position,
            state.yaw() + shortestCableAxisYawError(state.yaw(), raw_yaw));
        const auto measured = awareness_handler()->GetMeasuredOdometry();
        if (!measured) {
            return tracking->FailureReference(
                node()->now(), "object hover measured odometry unavailable");
        }
        Reference output;
        std::string reason;
        const double minimum_altitude_m = awareness_handler()->ground_altitude_estimate() +
            minimum_altitude_above_ground;
        if (!tracking->Compute(nominal, *measured, node()->now(), owner_request,
                owner_execution, minimum_altitude_m, 0.4, output, reason)) {
            if (!tracking->owns(owner_request, owner_execution)) return output;
            RCLCPP_ERROR_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
                "Object hover tracking failed: %s", reason.c_str());
            on_fail_callback_();
        } else {
            RCLCPP_INFO_THROTTLE(node()->get_logger(), *node()->get_clock(), 5000,
                "Object hover tracking request=%s nominal=[%.3f,%.3f,%.3f] "
                "command_error_m=%.3f correction_m=%.3f saturated=%s",
                owner_request.c_str(), nominal.position()(0), nominal.position()(1),
                nominal.position()(2),
                (output.position() - measured->state.position()).norm(),
                tracking->correction().norm(),
                tracking->saturated() ? "true" : "false");
        }
        return output;
    }

    if (has_target_) {

        pose_t pose = poseFromTransformMatrix(target_transform);

        position = pose.position;
        const double raw_yaw = quatToEul(pose.orientation)(2);
        yaw = state.yaw() + shortestCableAxisYawError(state.yaw(), raw_yaw);
        
    } else {

        on_fail_callback_();

        position = state.position();
        yaw = state.yaw();

    }

    if (use_nans_) {

        velocity = vector_t::Constant(NAN);
        yaw_rate = NAN;
        acceleration = vector_t::Constant(NAN);
        yaw_acceleration = NAN;

    } else {

        velocity = vector_t::Zero();
        yaw_rate = 0;
        acceleration = vector_t::Zero();
        yaw_acceleration = 0;

    }

    return iii_drone::control::Reference(
        position,
        yaw,
        velocity,
        yaw_rate,
        acceleration,
        yaw_acceleration
    );

}

maneuver_type_t HoverByObjectManeuverServer::maneuver_type() const {

    return MANEUVER_TYPE_HOVER_BY_OBJECT;

}

void HoverByObjectManeuverServer::startExecution(Maneuver & maneuver) {

    RCLCPP_INFO(
        node()->get_logger(),
        "HoverByObjectManeuverServer::startExecution(): Starting execution of HoverByObject maneuver"
    );

    if (!has_on_fail_callback_) {

        throw std::runtime_error("No on fail callback registered for HoverByObjectManeuverServer");

    }

    hover_by_object_maneuver_params_t params(maneuver.maneuver_params());

    const TargetAdapter previous_target = target_adapter_.Load();
    target_adapter_ = params.target_adapter;
    has_target_ = true;

    std::optional<Reference> adopted_seed;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        if (object_tracking_session_) {
            if (params.target_adapter.target_type() != previous_target.target_type() ||
                params.target_adapter.target_id() != previous_target.target_id()) {
                object_tracking_session_.reset();
                object_failure_hold_.reset();
                object_owner_request_identity_.clear();
                object_owner_execution_id_ = 0;
            }
        }
        if (object_tracking_session_) {
            const auto binding = currentReferenceBinding();
            const auto seed = consumeTerminalStartReference(maneuver.requestIdentity());
            std::string reason;
            if (!seed || !object_tracking_session_->Adopt(
                    object_owner_request_identity_, object_owner_execution_id_,
                    maneuver.requestIdentity(), binding.execution_id, *seed,
                    node()->now(), reason)) {
                throw std::runtime_error("object hover cannot adopt exact finite seed: " + reason);
            }
            object_owner_request_identity_ = maneuver.requestIdentity();
            object_owner_execution_id_ = binding.execution_id;
            adopted_seed = *seed;
        }
    }
    if (adopted_seed) PrimeOwnedManagedReference(*adopted_seed);

    awareness_handler()->SetTarget(target_adapter_);

    hover_duration_s_ = params.duration_s;

    sustain_action_ = params.sustain_action;

    hover_start_time_ = rclcpp::Clock().now();

}

bool HoverByObjectManeuverServer::canCancel() {

    return true;

}

std::optional<ControlledCancellationConfig>
HoverByObjectManeuverServer::controlledCancellationConfig() const {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        tracking = object_tracking_session_;
    }
    return tracking ? std::optional(tracking->cancellationConfig()) : std::nullopt;
}

bool HoverByObjectManeuverServer::validateControlledCancellationStop(
    const Reference & initial, const KinematicStopTrajectory & candidate,
    std::string & reason) {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        tracking = object_tracking_session_;
    }
    if (!tracking) return true;
    if (tracking->CertifiesCancellationStop(initial, candidate, reason)) return true;
    tracking->RejectUnsafeCancellation(reason);
    return false;
}

bool HoverByObjectManeuverServer::controlledCancellationComplete(
    const ControlledCancellationConfig &) {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        tracking = object_tracking_session_;
    }
    if (!tracking) return false;
    const bool profile_complete = controlledCancellationProfileComplete();
    const auto rest = controlledCancellationFinalReference();
    const std::string owner = current_maneuver().Load().requestIdentity();
    const bool exact_applied_rest = profile_complete && rest && applied_rest_reference_ &&
        applied_rest_reference_(owner, *rest);
    const bool proved = tracking->ObserveCancellationProof(
        profile_complete, exact_applied_rest,
        awareness_handler()->GetMeasuredOdometry(), node()->now());
    if (proved) {
        try {
            auto hold = std::make_shared<TerminalTrackingHold>(
                *rest, awareness_handler(), node()->get_clock(),
                TerminalTrackingHold::Clearance{}, 0.0);
            if (!hold->RequestQuiescence() || !hold->isQuiescent()) {
                throw std::runtime_error("object hover cancellation rest cannot be retained");
            }
            auto hover = std::static_pointer_cast<HoverManeuverServer>(
                registered_maneuvers().at(MANEUVER_TYPE_HOVER));
            hover->AdoptTerminalHold(hold, owner);
            std::lock_guard<std::mutex> lock(object_tracking_mutex_);
            if (object_tracking_session_ == tracking) object_failure_hold_ = std::move(hold);
        } catch (const std::exception & error) {
            tracking->RejectUnsafeCancellation(error.what());
            return false;
        }
    }
    return proved;
}

bool HoverByObjectManeuverServer::controlledCancellationFailure() const {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        tracking = object_tracking_session_;
    }
    return tracking && tracking->cancellationProofFailed();
}

iii_drone::control::Reference HoverByObjectManeuverServer::computeReference(const iii_drone::control::State & state) {

    return GetReference(state);

}

Reference HoverByObjectManeuverServer::initializationReference(const State & state) const {
    const auto binding = currentReferenceBinding();
    if (const auto seed = terminalStartReferenceFor(binding.request_identity)) {
        return seed->CopyWithNewStamp(node()->now());
    }
    return ManeuverServer::initializationReference(state);
}

bool HoverByObjectManeuverServer::hasSucceeded(Maneuver & maneuver) {

    if (!sustain_action_) {
        std::shared_ptr<ObjectTrackingSession> tracking;
        std::string owner;
        {
            std::lock_guard<std::mutex> lock(object_tracking_mutex_);
            tracking = object_tracking_session_;
            owner = object_owner_request_identity_;
        }
        if (tracking) {
            return !tracking->failed() && owner == maneuver.requestIdentity() &&
                first_reference_applied_ &&
                first_reference_applied_(maneuver.requestIdentity());
        }
        return true;

    }

    if (hasFailed(maneuver)) {

        return false;

    }

    if (rclcpp::Clock().now() - hover_start_time_ >= rclcpp::Duration::from_seconds(hover_duration_s_)) {

        return true;

    }

    return false;

}

bool HoverByObjectManeuverServer::hasFailed(Maneuver & maneuver) {
    std::shared_ptr<ObjectTrackingSession> tracking;
    {
        std::lock_guard<std::mutex> lock(object_tracking_mutex_);
        tracking = object_tracking_session_;
    }
    if (tracking) {
        if (!validateAwareness(awareness_handler()->adapter())) {
            tracking->Fail("object hover awareness or target unavailable");
        }
        if (!tracking->failed()) return false;
        if (!tracking->stopComplete() && !tracking->unrecoverable()) return false;
        if (tracking->stopComplete()) {
            std::lock_guard<std::mutex> lock(object_tracking_mutex_);
            if (!object_failure_hold_ && tracking == object_tracking_session_) {
                try {
                    object_failure_hold_ = std::make_shared<TerminalTrackingHold>(
                        tracking->lastCommand(), awareness_handler(), node()->get_clock(),
                        TerminalTrackingHold::Clearance{}, 0.0);
                    object_failure_hold_->Fail(
                        "object hover failed after bounded command stop");
                    auto hover = std::static_pointer_cast<HoverManeuverServer>(
                        registered_maneuvers().at(MANEUVER_TYPE_HOVER));
                    hover->AdoptTerminalHold(object_failure_hold_, maneuver.requestIdentity());
                } catch (const std::exception & error) {
                    RCLCPP_ERROR(node()->get_logger(),
                        "Object hover could not retain bounded failure stop: %s", error.what());
                }
            }
        }
        return true;
    }
    return !validateAwareness(awareness_handler()->adapter());
}

void HoverByObjectManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto goal_handle = std::static_pointer_cast<rclcpp_action::ServerGoalHandle<HoverByObject>>(maneuver.goal_handle());
    auto result = std::make_shared<HoverByObject::Result>();

    switch (maneuver_result_type){

        case MANEUVER_RESULT_TYPE_SUCCEED:
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            goal_handle->abort(result);
            awareness_handler()->ClearTarget();
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            goal_handle->canceled(result);
            awareness_handler()->ClearTarget();
            break;
    }

}

void HoverByObjectManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    registerCallback(
        std::bind(
            &HoverByObjectManeuverServer::GetReference,
            this,
            std::placeholders::_1
        )
    );

}

bool HoverByObjectManeuverServer::validateTargetTransform(
    const iii_drone::types::transform_matrix_t &target_transform, 
    const iii_drone::control::State &state
) const {

    vector_t target_position = poseFromTransformMatrix(target_transform).position;
    vector_t drone_position = state.position();

    if ((target_position - drone_position).norm() > max_euc_dist_) {

        return false;

    }

    return true;    

}

bool HoverByObjectManeuverServer::validateAwareness(
    iii_drone::adapters::CombinedDroneAwarenessAdapter drone_awareness,
    const iii_drone::adapters::TargetAdapter &target_adapter
) const {

    if (!drone_awareness.offboard()) {
        if (operatorNativeControl()) {
            RCLCPP_INFO_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
                "HoverByObjectManeuverServer::validateAwareness(): target_id=%d offboard=false (operator native control)",
                target_adapter.target_id());
        } else {
            RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
                "HoverByObjectManeuverServer::validateAwareness(): target_id=%d offboard=false",
                target_adapter.target_id());
        }
        return false;
    }

    if (!drone_awareness.armed()) {
        RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
            "HoverByObjectManeuverServer::validateAwareness(): target_id=%d armed=false",
            target_adapter.target_id());
        return false;
    }

    if (!drone_awareness.in_flight()) {
        RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
            "HoverByObjectManeuverServer::validateAwareness(): target_id=%d in_flight=false",
            target_adapter.target_id());
        return false;
    }

    if (!drone_awareness.has_target()) {
        RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
            "HoverByObjectManeuverServer::validateAwareness(): target_id=%d has_target=false",
            target_adapter.target_id());
        return false;
    }

    transform_matrix_t target_transform;
    
    try {

        if (target_adapter.target_type() == iii_drone::adapters::target_type_t::TARGET_TYPE_NONE) {

            target_transform = awareness_handler()->ComputeTargetTransform(target_adapter_);
        
        } else {

            target_transform = awareness_handler()->ComputeTargetTransform(target_adapter);

        }


    } catch (const std::runtime_error &e) {
        RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
            "HoverByObjectManeuverServer::validateAwareness(): target_id=%d transform failed: %s",
            target_adapter.target_id(), e.what());
        return false;

    }

    if (!validateTargetTransform(
        target_transform,
        drone_awareness.state()
    )) {
        const double distance = (poseFromTransformMatrix(target_transform).position -
            drone_awareness.state().position()).norm();
        RCLCPP_WARN_THROTTLE(node()->get_logger(), *node()->get_clock(), 1000,
            "HoverByObjectManeuverServer::validateAwareness(): target_id=%d raw_target_distance_m=%.3f max_m=%.3f",
            target_adapter.target_id(), distance, max_euc_dist_);
        return false;

    }

    return true;

}
