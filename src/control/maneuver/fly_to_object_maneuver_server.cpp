/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp>

#include <algorithm>
#include <cmath>

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
    has_failed_ = false;
    active_target_reference_valid_ = false;
    mpc_settle_active_ = false;
    mpc_settle_first_iteration_ = false;
    target_position_filter_initialized_ = false;
    filtered_target_position_ = point_t::Zero();
    last_target_position_filter_update_time_ = node()->now();
    maneuver_start_time_ = node()->now();
    threshold_reached_logged_ = false;
    settle_threshold_reached_logged_ = false;
    final_reference_streamed_logged_ = false;
    success_timing_logged_ = false;

    cda_handler->SetTarget(target_adapter_);

}

bool FlyToObjectManeuverServer::canCancel() {
    return true;
}

Reference FlyToObjectManeuverServer::computeReference(const State & state) {

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

            target_reference = filterTargetPositionReference(
                getUpdatedTargetReference(state),
                state
            );
            active_target_reference_ = target_reference;
            active_target_reference_valid_ = true;

        } catch (const std::runtime_error &e) {

            has_failed_ = true;
            return Reference(state);

        }
    }

    Reference ref;
    
    try {

        ref = trajectory_generator_client_->ComputeReference(
            state,
            target_reference,
            set_reference,
            reset,
            trajectory_mode_t::positional,
            compute_with_mpc
        );

    } catch (const std::runtime_error &e) {

        RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::computeReference(): Failed to compute reference, exception: %s", e.what());
        has_failed_ = true;
        ref = Reference(state,true,true);

    }

    if (first_iteration_) {

        first_iteration_ = false;

    }

    if (mpc_settle_first_iteration_) {

        mpc_settle_first_iteration_ = false;

    }

    return ref;

}

bool FlyToObjectManeuverServer::hasSucceeded(Maneuver & maneuver) {

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    if (!active_target_reference_valid_) {
        return false;
    }

    Reference target_reference = active_target_reference_.Load();

    if (hasFailed(maneuver)) {
        return false;
    }

    const double position_distance = (state.position() - target_reference.position()).norm();
    const double yaw_error = std::abs(shortestCableAxisYawError(state.yaw(), target_reference.yaw()));
    const double distance = std::hypot(position_distance, yaw_error);

    const bool use_mpc = configuration_->GetParameter("/control/maneuver_controller/fly_to_object_use_mpc").as_bool();
    const bool vehicle_reached_target = distance < configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double();
    bool succeeded = vehicle_reached_target;
    bool final_reference_streamed = false;

    if (vehicle_reached_target && !threshold_reached_logged_) {
        threshold_reached_logged_ = true;
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasSucceeded(): timing: vehicle reached threshold after %.3f s. target_id=%d distance=%.3f position_distance=%.3f yaw_error=%.3f",
            (node()->now() - maneuver_start_time_).seconds(),
            target_adapter_->target_id(),
            distance,
            position_distance,
            yaw_error
        );
    }

    if (use_mpc) {
        if (!mpc_settle_active_) {
            if (!vehicle_reached_target) {
                return false;
            }

            try {
                target_reference = getUpdatedTargetReference(state);
            } catch (const std::runtime_error & e) {
                RCLCPP_WARN(
                    node()->get_logger(),
                    "FlyToObjectManeuverServer::hasSucceeded(): MPC reached threshold but final raw target reference is unavailable: %s",
                    e.what()
                );
                return false;
            }

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
        final_reference_streamed = interpolationFinalReferenceStreamed(target_reference);
        if (final_reference_streamed && !final_reference_streamed_logged_) {
            final_reference_streamed_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToObjectManeuverServer::hasSucceeded(): timing: interpolation final reference streamed after %.3f s. target_id=%d distance=%.3f position_distance=%.3f yaw_error=%.3f vehicle_reached_threshold=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                target_adapter_->target_id(),
                distance,
                position_distance,
                yaw_error,
                vehicle_reached_target ? "true" : "false"
            );
        }
        succeeded = vehicle_reached_target && final_reference_streamed;
        if (vehicle_reached_target && !succeeded) {
            RCLCPP_DEBUG_THROTTLE(
                node()->get_logger(),
                *node()->get_clock(),
                1000,
                "FlyToObjectManeuverServer::hasSucceeded(): vehicle is within threshold, waiting for interpolation final reference to stream."
            );
        }
    }

    if (succeeded && !success_timing_logged_) {
        success_timing_logged_ = true;
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasSucceeded(): timing: succeeded after %.3f s. target_id=%d distance=%.3f position_distance=%.3f yaw_error=%.3f final_reference_streamed=%s",
            (node()->now() - maneuver_start_time_).seconds(),
            target_adapter_->target_id(),
            distance,
            position_distance,
            yaw_error,
            final_reference_streamed ? "true" : "false"
        );
    }

    return succeeded;

}

bool FlyToObjectManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();

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
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToObjectManeuverServer::hasFailed(): Target transform is not currently computable, returning true: %s",
            e.what()
        );
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
        RCLCPP_ERROR(node()->get_logger(), "FlyToObjectManeuverServer::getFeedback(): Failed to get updated target reference, exception: %s", e.what());
        has_failed_ = true;
        return std::static_pointer_cast<void>(feedback);
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

    if (hover_by_object_maneuver_server->Update(target_adapter_)) {

        registerCallback(
            std::bind(
                &HoverByObjectManeuverServer::GetReference,
                hover_by_object_maneuver_server,
                std::placeholders::_1
            )
        );

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

Reference FlyToObjectManeuverServer::getUpdatedTargetReference(const iii_drone::control::State & state) {

    auto cda_handler = awareness_handler();

    Reference reference = enforceMinimumTargetAltitude(referenceWithCableAxisYawClosestTo(
        Reference(cda_handler->ComputeTargetState(target_adapter_)),
        state.yaw()
    ));

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
