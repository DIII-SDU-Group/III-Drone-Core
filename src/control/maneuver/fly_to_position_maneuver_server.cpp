/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp>

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

double yawClosestTo(double current_yaw, double target_yaw) {
    return current_yaw + shortestYawError(current_yaw, target_yaw);
}

constexpr double kFinalReferencePositionToleranceM = 1.0e-3;
constexpr double kFinalReferenceYawToleranceRad = 1.0e-3;
constexpr double kFinalReferenceVelocityToleranceMps = 1.0e-3;
constexpr double kFinalReferenceYawRateToleranceRadps = 1.0e-3;
constexpr double kFinalReferenceAccelerationToleranceMps2 = 1.0e-3;
constexpr double kFinalReferenceYawAccelerationToleranceRadps2 = 1.0e-3;
constexpr double kBlendStartReferenceMaxAgeS = 2.0;

bool isFlightCapableForPositionFlight(const CombinedDroneAwarenessAdapter & awareness) {
    return awareness.in_flight() || (
        awareness.armed()
        && awareness.on_cable()
        && awareness.gripper_open()
    );
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FlyToPositionManeuverServer::FlyToPositionManeuverServer(
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

    createServer<iii_drone_interfaces::action::FlyToPosition>();

}

bool FlyToPositionManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
) const {

    if (maneuver.maneuver_type() != MANEUVER_TYPE_FLY_TO_POSITION) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Maneuver type is not FlyToPosition"
        );
        return false;
    }

    if (!isFlightCapableForPositionFlight(drone_awareness)) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Drone is not flight-capable"
        );
        return false;
    }

    if (!drone_awareness.offboard()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Drone is not in offboard mode"
        );
        return false;
    }

    fly_to_position_maneuver_params_t fly_to_position_maneuver_params(maneuver.maneuver_params());

    bool maneuver_params_valid = validateManeuverParameters(fly_to_position_maneuver_params);

    if (!maneuver_params_valid) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::CanExecuteManeuver(): Maneuver parameters are invalid"
        );
        return false;
    }

    return true;

}

iii_drone::adapters::CombinedDroneAwarenessAdapter FlyToPositionManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver &maneuver) {

    iii_drone::adapters::CombinedDroneAwarenessAdapter awareness_after;

    awareness_after.armed() = true;
    awareness_after.offboard() = true;
    awareness_after.target_adapter() = TargetAdapter();
    awareness_after.target_position_known() = false;
    awareness_after.drone_location() = DRONE_LOCATION_IN_FLIGHT;

    fly_to_position_maneuver_params_t maneuver_params(maneuver.maneuver_params());

    auto cda_handler = awareness_handler();

    point_t target_position_in_world_frame = maneuver_params.transform_target_position(
        configuration_->GetParameter("/tf/world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );

    quaternion_t target_orientation = eulToQuat(
        euler_angles_t(
            0.0, 
            0.0, 
            maneuver_params.transform_target_yaw(
                configuration_->GetParameter("/tf/world_frame_id").as_string(),
                cda_handler->tf_buffer()
            )
        )
    );

    awareness_after.state() = State(
        target_position_in_world_frame,
        vector_t(0,0,0),
        target_orientation,
        vector_t(0,0,0)
    );

    return awareness_after;

}

maneuver_type_t FlyToPositionManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_FLY_TO_POSITION;
}

void FlyToPositionManeuverServer::startExecution(Maneuver & maneuver) {

    RCLCPP_INFO(
        node()->get_logger(),
        "FlyToPositionManeuverServer::startExecution(): Starting execution of maneuver."
    );

    auto cda_handler = awareness_handler();

    fly_to_position_maneuver_params_t fly_to_position_maneuver_params(maneuver.maneuver_params());

    State state = cda_handler->GetState();

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): State: %f, %f, %f, %f", state.position()[0], state.position()[1], state.position()[2], state.yaw());

    point_t target_position_in_world_frame = fly_to_position_maneuver_params.transform_target_position(
        configuration_->GetParameter("/tf/world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): Target position in world frame: %f, %f, %f", target_position_in_world_frame[0], target_position_in_world_frame[1], target_position_in_world_frame[2]);

    const double raw_target_yaw_in_world_frame = fly_to_position_maneuver_params.transform_target_yaw(
        configuration_->GetParameter("/tf/world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );
    const double target_yaw_in_world_frame = yawClosestTo(
        state.yaw(),
        raw_target_yaw_in_world_frame
    );

    // RCLCPP_DEBUG(node()->get_logger(), "FlyToPositionManeuverServer::startExecution(): Target yaw in world frame: %f", target_yaw_in_world_frame);

    target_reference_ = iii_drone::control::Reference(
        target_position_in_world_frame,
        target_yaw_in_world_frame
    );

    RCLCPP_INFO(
        node()->get_logger(),
        "FlyToPositionManeuverServer::startExecution(): Target position=[%.3f, %.3f, %.3f], raw target yaw=%.3f, shortest target yaw=%.3f, current yaw=%.3f, yaw error=%.3f",
        target_position_in_world_frame[0],
        target_position_in_world_frame[1],
        target_position_in_world_frame[2],
        raw_target_yaw_in_world_frame,
        target_yaw_in_world_frame,
        state.yaw(),
        shortestYawError(state.yaw(), target_yaw_in_world_frame)
    );

    if (trajectory_generator_client_->busy()) {

        std::string error_message = "FlyToPositionManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";

        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());

        throw std::runtime_error(error_message);

    }

    first_iteration_ = true;
    has_failed_ = false;
    mpc_settle_active_ = false;
    mpc_settle_first_iteration_ = false;
    active_blend_to_next_ = fly_to_position_maneuver_params.blend_to_next;
    maneuver_start_time_ = node()->now();
    threshold_reached_logged_ = false;
    settle_threshold_reached_logged_ = false;
    final_reference_streamed_logged_ = false;
    success_timing_logged_ = false;

    Reference blend_start_reference;
    {
        std::lock_guard<std::mutex> lock(blend_mutex_);
        latest_streamed_reference_.reset();
        blend_completion_reference_.reset();
        initial_blend_start_reference_.reset();
    }
    if (consumePendingBlendStartReference(blend_start_reference)) {
        std::lock_guard<std::mutex> lock(blend_mutex_);
        initial_blend_start_reference_ = blend_start_reference;
        latest_streamed_reference_ = blend_start_reference;
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToPositionManeuverServer::startExecution(): Consuming pending blend start reference. position=[%.3f, %.3f, %.3f], yaw=%.3f, velocity=[%.3f, %.3f, %.3f], yaw_rate=%.3f, acceleration=[%.3f, %.3f, %.3f], yaw_acceleration=%.3f",
            blend_start_reference.position()(0),
            blend_start_reference.position()(1),
            blend_start_reference.position()(2),
            blend_start_reference.yaw(),
            blend_start_reference.velocity()(0),
            blend_start_reference.velocity()(1),
            blend_start_reference.velocity()(2),
            blend_start_reference.yaw_rate(),
            blend_start_reference.acceleration()(0),
            blend_start_reference.acceleration()(1),
            blend_start_reference.acceleration()(2),
            blend_start_reference.yaw_acceleration()
        );
    }

    cda_handler->ClearTarget();

}

bool FlyToPositionManeuverServer::canCancel() {
    return true;
}

Reference FlyToPositionManeuverServer::computeReference(const State & state) {

    const bool use_mpc = configuration_->GetParameter("/control/maneuver_controller/fly_to_position_use_mpc").as_bool();
    bool reset, set_reference;
    Reference target_reference = target_reference_;
    bool compute_with_mpc = use_mpc;

    if (mpc_settle_active_) {
        reset = set_reference = mpc_settle_first_iteration_;
        target_reference = mpc_settle_target_reference_;
        compute_with_mpc = false;
        RCLCPP_DEBUG(
            node()->get_logger(),
            "FlyToPositionManeuverServer::computeReference(): Using post-MPC interpolation settle"
        );
    } else {
        reset = set_reference = first_iteration_;
    }

    if (!mpc_settle_active_ && use_mpc) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "FlyToPositionManeuverServer::computeReference(): Using MPC"
        );
    } else {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "FlyToPositionManeuverServer::computeReference(): Using interpolation"
        );
    }

    Reference ref;
    
    try {
        State computation_state = state;
        std::optional<Reference> computation_start_reference;
        {
            std::lock_guard<std::mutex> lock(blend_mutex_);
            if (first_iteration_ && initial_blend_start_reference_) {
                computation_start_reference = *initial_blend_start_reference_;
                initial_blend_start_reference_.reset();
            }
        }
        if (computation_start_reference && !compute_with_mpc) {
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::computeReference(): Generating blended interpolation from full start reference. acceleration=[%.3f, %.3f, %.3f], yaw_acceleration=%.3f",
                computation_start_reference->acceleration()(0),
                computation_start_reference->acceleration()(1),
                computation_start_reference->acceleration()(2),
                computation_start_reference->yaw_acceleration()
            );
            ref = trajectory_generator_client_->ComputeReference(
                *computation_start_reference,
                target_reference,
                set_reference,
                reset,
                trajectory_mode_t::positional
            );
        } else {
            if (computation_start_reference) {
                computation_state = stateFromReference(*computation_start_reference);
            }
            ref = trajectory_generator_client_->ComputeReference(
                computation_state,
                target_reference,
                set_reference,
                reset,
                trajectory_mode_t::positional,
                compute_with_mpc
            );
        }
    } catch (std::runtime_error & e) {
        RCLCPP_ERROR(
            node()->get_logger(),
            "FlyToPositionManeuverServer::computeReference(): %s",
            e.what()
        );

        has_failed_ = true;
        
        ref = Reference(state,true,true);
    }

    if (first_iteration_) {

        first_iteration_ = false;

    }

    if (mpc_settle_first_iteration_) {

        mpc_settle_first_iteration_ = false;

    }

    storeLatestStreamedReference(ref);

    return ref;

}

bool FlyToPositionManeuverServer::hasSucceeded(Maneuver &) {

    auto cda_handler = awareness_handler();

    State state = cda_handler->GetState();

    double distance = (state.position() - target_reference_->position()).norm();
    double yaw_error = std::abs(shortestYawError(state.yaw(), target_reference_->yaw()));

    const double reached_position_threshold =
        configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double();
    const double blend_radius =
        configuration_->GetParameter("/control/maneuver_controller/fly_to_position_blend_radius").as_double();
    const double active_position_threshold = active_blend_to_next_ ? blend_radius : reached_position_threshold;

    const bool vehicle_reached_target =
        distance < active_position_threshold
        && yaw_error < configuration_->GetParameter("/control/maneuver_controller/reached_yaw_error_threshold").as_double();

    const bool use_mpc = configuration_->GetParameter("/control/maneuver_controller/fly_to_position_use_mpc").as_bool();
    bool succeeded = vehicle_reached_target;
    Reference final_target_reference = target_reference_;
    bool final_reference_streamed = false;

    if (vehicle_reached_target && !threshold_reached_logged_) {
        threshold_reached_logged_ = true;
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasSucceeded(): timing: vehicle reached threshold after %.3f s. distance=%.3f threshold=%.3f yaw_error=%.3f blend_to_next=%s",
            (node()->now() - maneuver_start_time_).seconds(),
            distance,
            active_position_threshold,
            yaw_error,
            active_blend_to_next_ ? "true" : "false"
        );
    }

    if (active_blend_to_next_ && vehicle_reached_target) {
        prepareBlendCompletionReference(state);
        if (!success_timing_logged_) {
            success_timing_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): blend_to_next succeeded early after %.3f s. distance=%.3f yaw_error=%.3f final_reference_streamed=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                distance,
                yaw_error,
                final_reference_streamed ? "true" : "false"
            );
        }
        return true;
    }

    if (use_mpc) {
        if (!mpc_settle_active_) {
            if (!vehicle_reached_target) {
                return false;
            }

            mpc_settle_target_reference_ = final_target_reference;
            mpc_settle_active_ = true;
            mpc_settle_first_iteration_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): MPC reached threshold; entering final interpolation settle to target position=[%.3f, %.3f, %.3f], yaw=%.3f",
                final_target_reference.position()(0),
                final_target_reference.position()(1),
                final_target_reference.position()(2),
                final_target_reference.yaw()
            );
            return false;
        }

        final_target_reference = mpc_settle_target_reference_;
        const double settle_distance = (state.position() - final_target_reference.position()).norm();
        const double settle_yaw_error = std::abs(shortestYawError(state.yaw(), final_target_reference.yaw()));
        const bool vehicle_reached_settle_target =
            settle_distance < configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double()
            && settle_yaw_error < configuration_->GetParameter("/control/maneuver_controller/reached_yaw_error_threshold").as_double();
        final_reference_streamed = interpolationFinalReferenceStreamed(final_target_reference);
        if (vehicle_reached_settle_target && !settle_threshold_reached_logged_) {
            settle_threshold_reached_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): timing: post-MPC settle reached threshold after %.3f s. distance=%.3f yaw_error=%.3f final_reference_streamed=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                settle_distance,
                settle_yaw_error,
                final_reference_streamed ? "true" : "false"
            );
        }
        if (final_reference_streamed && !final_reference_streamed_logged_) {
            final_reference_streamed_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): timing: interpolation final reference streamed after %.3f s. distance=%.3f yaw_error=%.3f vehicle_reached_threshold=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                settle_distance,
                settle_yaw_error,
                vehicle_reached_settle_target ? "true" : "false"
            );
        }
        succeeded = vehicle_reached_settle_target && final_reference_streamed;
        if (!succeeded) {
            RCLCPP_DEBUG(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): waiting for post-MPC interpolation settle. distance=%.3f yaw_error=%.3f",
                settle_distance,
                settle_yaw_error
            );
        }
    } else {
        final_reference_streamed = interpolationFinalReferenceStreamed(final_target_reference);
        if (final_reference_streamed && !final_reference_streamed_logged_) {
            final_reference_streamed_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): timing: interpolation final reference streamed after %.3f s. distance=%.3f yaw_error=%.3f vehicle_reached_threshold=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                distance,
                yaw_error,
                vehicle_reached_target ? "true" : "false"
            );
        }
        succeeded = vehicle_reached_target && final_reference_streamed;
        if (vehicle_reached_target && !succeeded) {
            RCLCPP_DEBUG(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): vehicle is within threshold, waiting for interpolation final reference to stream."
            );
        }
    }

    if (succeeded) {
        if (!success_timing_logged_) {
            success_timing_logged_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "FlyToPositionManeuverServer::hasSucceeded(): timing: succeeded after %.3f s. distance=%.3f yaw_error=%.3f final_reference_streamed=%s",
                (node()->now() - maneuver_start_time_).seconds(),
                distance,
                yaw_error,
                final_reference_streamed ? "true" : "false"
            );
        }
    }

    return succeeded;

}

bool FlyToPositionManeuverServer::interpolationFinalReferenceStreamed(
    const iii_drone::control::Reference & target_reference
) const {

    ReferenceTrajectory trajectory;
    try {
        trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    } catch (const std::runtime_error & e) {
        RCLCPP_DEBUG(
            node()->get_logger(),
            "FlyToPositionManeuverServer::interpolationFinalReferenceStreamed(): No trajectory available yet: %s",
            e.what()
        );
        return false;
    }

    if (trajectory.references().empty()) {
        return false;
    }

    const Reference streamed_reference = trajectory.references().front();

    const double position_error = (streamed_reference.position() - target_reference.position()).norm();
    const double yaw_error = std::abs(shortestYawError(streamed_reference.yaw(), target_reference.yaw()));
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
        RCLCPP_DEBUG(
            node()->get_logger(),
            "FlyToPositionManeuverServer::interpolationFinalReferenceStreamed(): position_error=%.4f yaw_error=%.4f velocity_norm=%.4f yaw_rate=%.4f acceleration_norm=%.4f yaw_acceleration=%.4f",
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

bool FlyToPositionManeuverServer::hasFailed(Maneuver &) {

    auto cda_handler = awareness_handler();
    const auto drone_awareness = cda_handler->adapter();

    if (!isFlightCapableForPositionFlight(drone_awareness)) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Drone is not flight-capable (location=%d, armed=%d, gripper_open=%d)"
            ,
            static_cast<int>(drone_awareness.drone_location()),
            drone_awareness.armed(),
            drone_awareness.gripper_open()
        );
        return true;
    }

    if (!cda_handler->offboard()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Drone is not in offboard mode"
        );
        return true;
    }

    if (!cda_handler->armed()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Drone is not armed"
        );
        return true;
    }

    if (has_failed_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::hasFailed(): Maneuver failed flag is set"
        );
        return true;
    }

    return false;

}

std::shared_ptr<void> FlyToPositionManeuverServer::getFeedback(Maneuver &) {

    auto feedback = std::make_shared<iii_drone_interfaces::action::FlyToPosition::Feedback>();

    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();

    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);

    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());

    return std::static_pointer_cast<void>(feedback);

}

void FlyToPositionManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {

    auto result = std::make_shared<iii_drone_interfaces::action::FlyToPosition::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleFlyToPosition>(maneuver.goal_handle());

    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            result->success = true;
            result->target_reference = ReferenceAdapter(successReferenceForResult()).ToMsg();
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            result->success = false;
            result->target_reference = ReferenceAdapter(mpc_settle_active_ ? mpc_settle_target_reference_.Load() : target_reference_.Load()).ToMsg();
            goal_handle->abort(result);
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            result->success = false;
            result->target_reference = ReferenceAdapter(mpc_settle_active_ ? mpc_settle_target_reference_.Load() : target_reference_.Load()).ToMsg();
            goal_handle->canceled(result);
            break;
    }

}

void FlyToPositionManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {

    if (active_blend_to_next_) {
        RCLCPP_INFO(
            node()->get_logger(),
            "FlyToPositionManeuverServer::registerReferenceCallbackOnSuccess(): Keeping FTP reference callback active for blended successor."
        );
        return;
    }

    auto registered_hover_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER);

    std::shared_ptr<HoverManeuverServer> hover_maneuver_server = std::static_pointer_cast<HoverManeuverServer>(registered_hover_maneuver->second);

    hover_maneuver_server->Update(successReferenceForResult());

    registerCallback(
        std::bind(
            &HoverManeuverServer::GetReference,
            hover_maneuver_server,
            std::placeholders::_1
        )
    );

}

bool FlyToPositionManeuverServer::consumePendingBlendStartReference(Reference & start_reference) {

    std::lock_guard<std::mutex> lock(blend_mutex_);

    if (!pending_blend_start_reference_) {
        return false;
    }

    const double age_s = (node()->now() - pending_blend_start_time_).seconds();
    if (age_s > kBlendStartReferenceMaxAgeS) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::consumePendingBlendStartReference(): Discarding stale blend start reference. age=%.3f s",
            age_s
        );
        pending_blend_start_reference_.reset();
        return false;
    }

    start_reference = pending_blend_start_reference_->CopyWithNewStamp(node()->now());
    pending_blend_start_reference_.reset();
    return true;

}

Reference FlyToPositionManeuverServer::latestStreamedReferenceOrState(const State & state) const {

    std::lock_guard<std::mutex> lock(blend_mutex_);
    if (latest_streamed_reference_) {
        return latest_streamed_reference_->CopyWithNewStamp(node()->now());
    }
    return Reference(state).CopyWithNewStamp(node()->now());

}

void FlyToPositionManeuverServer::storeLatestStreamedReference(const Reference & reference) {

    std::lock_guard<std::mutex> lock(blend_mutex_);
    latest_streamed_reference_ = reference.CopyWithNewStamp(node()->now());

}

void FlyToPositionManeuverServer::prepareBlendCompletionReference(const State & state) {

    Reference blend_reference = latestStreamedReferenceOrState(state);

    std::lock_guard<std::mutex> lock(blend_mutex_);
    blend_completion_reference_ = blend_reference;
    pending_blend_start_reference_ = blend_reference;
    pending_blend_start_time_ = node()->now();

    RCLCPP_INFO(
        node()->get_logger(),
        "FlyToPositionManeuverServer::prepareBlendCompletionReference(): Stored blend handoff reference. position=[%.3f, %.3f, %.3f], yaw=%.3f, velocity=[%.3f, %.3f, %.3f], yaw_rate=%.3f",
        blend_reference.position()(0),
        blend_reference.position()(1),
        blend_reference.position()(2),
        blend_reference.yaw(),
        blend_reference.velocity()(0),
        blend_reference.velocity()(1),
        blend_reference.velocity()(2),
        blend_reference.yaw_rate()
    );

}

Reference FlyToPositionManeuverServer::successReferenceForResult() const {

    std::lock_guard<std::mutex> lock(blend_mutex_);
    if (blend_completion_reference_) {
        return blend_completion_reference_->CopyWithNewStamp(node()->now());
    }
    return mpc_settle_active_ ? mpc_settle_target_reference_.Load() : target_reference_.Load();

}

State FlyToPositionManeuverServer::stateFromReference(const Reference & reference) const {

    return State(
        reference.position(),
        reference.velocity(),
        reference.yaw(),
        vector_t(0.0, 0.0, reference.yaw_rate()),
        reference.stamp()
    );

}

bool FlyToPositionManeuverServer::validateManeuverParameters(const fly_to_position_maneuver_params_t & maneuver_params) const {

    auto cda_handler = awareness_handler();

    point_t target_position_in_world_frame;
    try {
        target_position_in_world_frame = maneuver_params.transform_target_position(
            configuration_->GetParameter("/tf/world_frame_id").as_string(),
            cda_handler->tf_buffer()
        );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::validateManeuverParameters(): Could not transform target from frame '%s': %s",
            maneuver_params.frame_id.c_str(),
            ex.what()
        );
        return false;
    }

    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate() >= configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double();
    if (!target_position_valid) {
        RCLCPP_WARN(
            node()->get_logger(),
            "FlyToPositionManeuverServer::validateManeuverParameters(): Target altitude %.3f m above ground estimate %.3f m is below minimum %.3f m",
            target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate(),
            cda_handler->ground_altitude_estimate(),
            configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double()
        );
    }

    return target_position_valid;

}
