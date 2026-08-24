#include <iii_drone_core/control/maneuver/cable_aware_fly_to_position_maneuver_server.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <limits>

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

double getDoubleParameter(
    const iii_drone::configuration::Configuration::SharedPtr & configuration,
    const std::string & name,
    double fallback
) {
    if (configuration->HasParameter(name)) {
        return configuration->GetParameter(name).as_double();
    }
    return fallback;
}

bool isFlightCapableForPositionFlight(const CombinedDroneAwarenessAdapter & awareness) {
    return awareness.in_flight() || (
        awareness.armed()
        && awareness.on_cable()
        && awareness.gripper_open()
    );
}

}  // namespace

CableAwareFlyToPositionManeuverServer::CableAwareFlyToPositionManeuverServer(
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

    powerline_overview_client_cb_group_ = node->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
    );
    get_powerline_overview_client_ = node->create_client<iii_drone_interfaces::srv::GetPowerlineOverview>(
        "/mission/powerline_overview_provider/get_powerline_overview",
        rclcpp::ServicesQoS(),
        powerline_overview_client_cb_group_
    );

    createServer<iii_drone_interfaces::action::CableAwareFlyToPosition>();
}

bool CableAwareFlyToPositionManeuverServer::CanExecuteManeuver(
    const Maneuver & maneuver,
    const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
) const {
    if (maneuver.maneuver_type() != MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION) {
        RCLCPP_WARN(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::CanExecuteManeuver(): Maneuver type is not CableAwareFlyToPosition");
        return false;
    }
    if (!isFlightCapableForPositionFlight(drone_awareness)) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::CanExecuteManeuver(): Drone is not flight-capable"
        );
        return false;
    }
    if (!drone_awareness.offboard()) {
        RCLCPP_WARN(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::CanExecuteManeuver(): Drone is not in offboard mode");
        return false;
    }

    fly_to_position_maneuver_params_t maneuver_params(maneuver.maneuver_params());
    if (!validateManeuverParameters(maneuver_params)) {
        RCLCPP_WARN(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::CanExecuteManeuver(): Maneuver parameters are invalid");
        return false;
    }
    const auto stored_powerline = storedPowerlineOverview();
    if (!stored_powerline.has_value()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::CanExecuteManeuver(): No stored powerline overview available; rejecting cable-aware fly-to-position"
        );
        return false;
    }
    if (!targetSatisfiesCableClearance(maneuver_params, stored_powerline.value())) {
        return false;
    }
    return true;
}

iii_drone::adapters::CombinedDroneAwarenessAdapter CableAwareFlyToPositionManeuverServer::ExpectedAwarenessAfterExecution(const Maneuver & maneuver) {
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
    quaternion_t target_orientation = eulToQuat(euler_angles_t(
        0.0,
        0.0,
        maneuver_params.transform_target_yaw(configuration_->GetParameter("/tf/world_frame_id").as_string(), cda_handler->tf_buffer())
    ));
    awareness_after.state() = State(target_position_in_world_frame, vector_t(0, 0, 0), target_orientation, vector_t(0, 0, 0));
    return awareness_after;
}

maneuver_type_t CableAwareFlyToPositionManeuverServer::maneuver_type() const {
    return MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION;
}

void CableAwareFlyToPositionManeuverServer::startExecution(Maneuver & maneuver) {
    RCLCPP_INFO(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::startExecution(): Starting execution of maneuver.");

    auto cda_handler = awareness_handler();
    fly_to_position_maneuver_params_t maneuver_params(maneuver.maneuver_params());
    State state = cda_handler->GetState();
    point_t target_position_in_world_frame = maneuver_params.transform_target_position(
        configuration_->GetParameter("/tf/world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );
    const double raw_target_yaw_in_world_frame = maneuver_params.transform_target_yaw(
        configuration_->GetParameter("/tf/world_frame_id").as_string(),
        cda_handler->tf_buffer()
    );
    const double target_yaw_in_world_frame = yawClosestTo(
        state.yaw(),
        raw_target_yaw_in_world_frame
    );
    target_reference_ = iii_drone::control::Reference(target_position_in_world_frame, target_yaw_in_world_frame);

    RCLCPP_INFO(
        node()->get_logger(),
        "CableAwareFlyToPositionManeuverServer::startExecution(): Target position=[%.3f, %.3f, %.3f], raw target yaw=%.3f, shortest target yaw=%.3f, current yaw=%.3f, yaw error=%.3f",
        target_position_in_world_frame[0],
        target_position_in_world_frame[1],
        target_position_in_world_frame[2],
        raw_target_yaw_in_world_frame,
        target_yaw_in_world_frame,
        state.yaw(),
        shortestYawError(state.yaw(), target_yaw_in_world_frame)
    );

    if (trajectory_generator_client_->busy()) {
        std::string error_message = "CableAwareFlyToPositionManeuverServer::startExecution(): Trajectory generator client is busy, cannot start execution of maneuver.";
        RCLCPP_FATAL(node()->get_logger(), error_message.c_str());
        throw std::runtime_error(error_message);
    }

    first_iteration_ = true;
    waiting_for_initial_plan_ = false;
    has_failed_ = false;
    cda_handler->ClearTarget();
}

bool CableAwareFlyToPositionManeuverServer::canCancel() {
    return true;
}

std::optional<ControlledCancellationConfig>
CableAwareFlyToPositionManeuverServer::controlledCancellationConfig() const {
    return controlledCancellationConfigFrom(configuration_);
}

bool CableAwareFlyToPositionManeuverServer::rebaseExecution(
    const State &,
    std::string & reason
) {
    if (trajectory_generator_client_->busy()) {
        reason = "cable-aware trajectory generator is busy";
        return false;
    }
    first_iteration_ = true;
    waiting_for_initial_plan_ = false;
    has_failed_ = false;
    reason = "replanned cable-aware flight from stopped state";
    return true;
}

Reference CableAwareFlyToPositionManeuverServer::computeReference(const State & state) {
    try {
        if (first_iteration_) {
            trajectory_generator_client_->Reset(state);
            trajectory_generator_client_->ComputeReferenceTrajectoryAsync(
                state,
                target_reference_,
                true,
                true,
                trajectory_mode_t::cable_aware,
                false
            );
            first_iteration_ = false;
            waiting_for_initial_plan_ = true;
            RCLCPP_INFO(
                node()->get_logger(),
                "CableAwareFlyToPositionManeuverServer::computeReference(): Launched initial cable-aware A* request; holding current position until trajectory is available."
            );
            return Reference(state);
        }

        if (waiting_for_initial_plan_) {
            if (!trajectory_generator_client_->done()) {
                RCLCPP_DEBUG(
                    node()->get_logger(),
                    "CableAwareFlyToPositionManeuverServer::computeReference(): Waiting for initial cable-aware trajectory; holding current position."
                );
                return Reference(state);
            }
            if (!trajectory_generator_client_->lastRequestSucceeded()) {
                throw std::runtime_error(
                    "Initial cable-aware trajectory generation failed: "
                    + trajectory_generator_client_->lastErrorMessage()
                );
            }
            waiting_for_initial_plan_ = false;
            RCLCPP_INFO(
                node()->get_logger(),
                "CableAwareFlyToPositionManeuverServer::computeReference(): Initial cable-aware trajectory ready; streaming planned references."
            );
        } else if (trajectory_generator_client_->done() && !trajectory_generator_client_->lastRequestSucceeded()) {
            throw std::runtime_error(
                "Cable-aware trajectory update failed: "
                + trajectory_generator_client_->lastErrorMessage()
            );
        }

        const ReferenceTrajectory trajectory = trajectory_generator_client_->GetReferenceTrajectory();
        if (trajectory.references().empty()) {
            throw std::runtime_error("Cable-aware trajectory response contained no references.");
        }

        Reference ref = trajectory.references().front();

        if (!trajectory_generator_client_->busy()) {
            trajectory_generator_client_->ComputeReferenceTrajectoryAsync(
                state,
                target_reference_,
                false,
                false,
                trajectory_mode_t::cable_aware,
                false
            );
        } else {
            RCLCPP_DEBUG(
                node()->get_logger(),
                "CableAwareFlyToPositionManeuverServer::computeReference(): Cable-aware trajectory update still running; reusing latest cable-aware reference."
            );
        }

        return ref;
    } catch (std::runtime_error & e) {
        RCLCPP_ERROR(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::computeReference(): %s", e.what());
        has_failed_ = true;
        return Reference(state);
    }
}

bool CableAwareFlyToPositionManeuverServer::hasSucceeded(Maneuver &) {
    auto cda_handler = awareness_handler();
    State state = cda_handler->GetState();
    double distance = (state.position() - target_reference_->position()).norm();
    double yaw_error = std::abs(shortestYawError(state.yaw(), target_reference_->yaw()));
    bool succeeded = distance < configuration_->GetParameter("/control/maneuver_controller/reached_position_euclidean_distance_threshold").as_double()
        && yaw_error < configuration_->GetParameter("/control/maneuver_controller/reached_yaw_error_threshold").as_double();
    if (succeeded) {
        RCLCPP_INFO(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::hasSucceeded(): distance=%.3f, yaw_error=%.3f", distance, yaw_error);
    }
    return succeeded;
}

bool CableAwareFlyToPositionManeuverServer::hasFailed(Maneuver &) {
    auto cda_handler = awareness_handler();
    const auto drone_awareness = cda_handler->adapter();
    if (!isFlightCapableForPositionFlight(drone_awareness)) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::hasFailed(): Drone is not flight-capable (location=%d, armed=%d, gripper_open=%d)",
            static_cast<int>(drone_awareness.drone_location()),
            drone_awareness.armed(),
            drone_awareness.gripper_open()
        );
        return true;
    }
    if (!cda_handler->offboard()) {
        RCLCPP_WARN(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::hasFailed(): Drone is not in offboard mode");
        return true;
    }
    if (!cda_handler->armed()) {
        RCLCPP_WARN(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::hasFailed(): Drone is not armed");
        return true;
    }
    if (has_failed_) {
        RCLCPP_WARN(node()->get_logger(), "CableAwareFlyToPositionManeuverServer::hasFailed(): Maneuver failed flag is set");
        return true;
    }
    return false;
}

std::shared_ptr<void> CableAwareFlyToPositionManeuverServer::getFeedback(Maneuver &) {
    auto feedback = std::make_shared<iii_drone_interfaces::action::CableAwareFlyToPosition::Feedback>();
    ReferenceTrajectory reference_trajectory = trajectory_generator_client_->GetReferenceTrajectory();
    ReferenceTrajectoryAdapter reference_trajectory_adapter(reference_trajectory);
    feedback->planned_path = reference_trajectory_adapter.ToPathMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    feedback->vehicle_pose = StateAdapter(awareness_handler()->GetState()).ToPoseStampedMsg(configuration_->GetParameter("/tf/world_frame_id").as_string());
    return std::static_pointer_cast<void>(feedback);
}

void CableAwareFlyToPositionManeuverServer::publishResultAndFinalize(
    Maneuver & maneuver,
    maneuver_result_type_t maneuver_result_type
) {
    auto result = std::make_shared<iii_drone_interfaces::action::CableAwareFlyToPosition::Result>();
    auto goal_handle = std::static_pointer_cast<GoalHandleCableAwareFlyToPosition>(maneuver.goal_handle());

    switch (maneuver_result_type) {
        case MANEUVER_RESULT_TYPE_SUCCEED:
            result->success = true;
            result->target_reference = ReferenceAdapter(target_reference_).ToMsg();
            goal_handle->succeed(result);
            break;
        case MANEUVER_RESULT_TYPE_ABORT:
            result->success = false;
            result->target_reference = ReferenceAdapter(target_reference_).ToMsg();
            goal_handle->abort(result);
            break;
        case MANEUVER_RESULT_TYPE_CANCEL:
            result->success = false;
            result->target_reference = ReferenceAdapter(
                controlledCancellationFinalReference().value_or(target_reference_.Load())
            ).ToMsg();
            goal_handle->canceled(result);
            break;
    }
}

void CableAwareFlyToPositionManeuverServer::registerReferenceCallbackOnSuccess(const Maneuver &) {
    auto registered_hover_maneuver = registered_maneuvers().find(MANEUVER_TYPE_HOVER);
    std::shared_ptr<HoverManeuverServer> hover_maneuver_server = std::static_pointer_cast<HoverManeuverServer>(registered_hover_maneuver->second);
    hover_maneuver_server->Update(target_reference_);
    registerCallback(std::bind(&HoverManeuverServer::GetReference, hover_maneuver_server, std::placeholders::_1));
}

bool CableAwareFlyToPositionManeuverServer::validateManeuverParameters(const fly_to_position_maneuver_params_t & maneuver_params) const {
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
            "CableAwareFlyToPositionManeuverServer::validateManeuverParameters(): Could not transform target from frame '%s': %s",
            maneuver_params.frame_id.c_str(),
            ex.what()
        );
        return false;
    }
    if (maneuver_params.ignore_altitude) {
        RCLCPP_INFO(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::validateManeuverParameters(): Minimum target altitude check bypassed by goal"
        );
        return true;
    }
    bool target_position_valid = target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate()
        >= configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double();
    if (!target_position_valid) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::validateManeuverParameters(): Target altitude %.3f m above ground estimate %.3f m is below minimum %.3f m",
            target_position_in_world_frame[2] - cda_handler->ground_altitude_estimate(),
            cda_handler->ground_altitude_estimate(),
            configuration_->GetParameter("/control/maneuver_controller/minimum_target_altitude").as_double()
        );
    }
    return target_position_valid;
}

bool CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview() const {
    return storedPowerlineOverview().has_value();
}

std::optional<PowerlineAdapter> CableAwareFlyToPositionManeuverServer::storedPowerlineOverview() const {
    if (!get_powerline_overview_client_) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Powerline overview client is not initialized."
        );
        return std::nullopt;
    }

    if (!get_powerline_overview_client_->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Stored powerline overview service is unavailable."
        );
        return std::nullopt;
    }

    auto request = std::make_shared<iii_drone_interfaces::srv::GetPowerlineOverview::Request>();
    auto future = get_powerline_overview_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
        if (std::chrono::steady_clock::now() >= deadline) {
            RCLCPP_WARN(
                node()->get_logger(),
                "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Timed out waiting for stored powerline overview."
            );
            return std::nullopt;
        }
    }

    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Stored powerline overview request did not complete."
        );
        return std::nullopt;
    }

    auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Stored powerline overview provider returned success=false."
        );
        return std::nullopt;
    }
    if (response->stored_powerline.lines.empty()) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Stored powerline overview has no lines."
        );
        return std::nullopt;
    }

    RCLCPP_DEBUG(
        node()->get_logger(),
        "CableAwareFlyToPositionManeuverServer::hasStoredPowerlineOverview(): Stored powerline overview available with %zu line(s).",
        response->stored_powerline.lines.size()
    );
    return PowerlineAdapter(response->stored_powerline);
}

bool CableAwareFlyToPositionManeuverServer::targetSatisfiesCableClearance(
    const fly_to_position_maneuver_params_t & maneuver_params,
    const PowerlineAdapter & powerline
) const {
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
            "CableAwareFlyToPositionManeuverServer::targetSatisfiesCableClearance(): Could not transform target from frame '%s': %s",
            maneuver_params.frame_id.c_str(),
            ex.what()
        );
        return false;
    }

    const double clearance_m = getDoubleParameter(
        configuration_,
        "/control/trajectory_generator/cable_aware_clearance_m",
        1.0
    );
    if (!configuration_->HasParameter("/control/trajectory_generator/cable_aware_clearance_m")) {
        RCLCPP_WARN_ONCE(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::targetSatisfiesCableClearance(): Missing /control/trajectory_generator/cable_aware_clearance_m in maneuver configuration; using fallback 1.000 m. Check node parameter bundle wiring."
        );
    }
    double minimum_distance_m = std::numeric_limits<double>::infinity();
    int closest_line_id = -1;
    for (const auto & line : powerline.single_line_adapters()) {
        const double distance_m = distanceToCable(target_position_in_world_frame, powerline, line);
        if (distance_m < minimum_distance_m) {
            minimum_distance_m = distance_m;
            closest_line_id = line.id();
        }
    }

    if (minimum_distance_m < clearance_m) {
        RCLCPP_WARN(
            node()->get_logger(),
            "CableAwareFlyToPositionManeuverServer::targetSatisfiesCableClearance(): Rejecting target [%.3f, %.3f, %.3f]; nearest stored conductor id=%d distance=%.3f m is below required clearance %.3f m",
            target_position_in_world_frame.x(),
            target_position_in_world_frame.y(),
            target_position_in_world_frame.z(),
            closest_line_id,
            minimum_distance_m,
            clearance_m
        );
        return false;
    }

    return true;
}

vector_t CableAwareFlyToPositionManeuverServer::cableDirection(
    const PowerlineAdapter & powerline,
    const SingleLineAdapter & line
) const {
    vector_t direction = powerline.projection_plane().normal;
    direction.z() = 0.0;
    if (direction.norm() > 1.0e-6) {
        return direction.normalized();
    }

    direction = iii_drone::math::quatToMat(line.quaternion()).col(0);
    if (direction.norm() < 1.0e-6) {
        return vector_t(1.0, 0.0, 0.0);
    }
    return direction.normalized();
}

double CableAwareFlyToPositionManeuverServer::distanceToCable(
    const point_t & point,
    const PowerlineAdapter & powerline,
    const SingleLineAdapter & line
) const {
    const vector_t direction = cableDirection(powerline, line);
    const vector_t delta = point - line.position();
    return (delta - delta.dot(direction) * direction).norm();
}
