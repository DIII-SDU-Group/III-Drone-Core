#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <optional>

#include <iii_drone_configuration/configuration.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/reference_trajectory_adapter.hpp>
#include <iii_drone_core/adapters/state_adapter.hpp>
#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>
#include <iii_drone_core/control/maneuver/hover_maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver.hpp>
#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/control/maneuver/maneuver_types.hpp>
#include <iii_drone_core/control/trajectory_generator_client.hpp>
#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_interfaces/action/cable_aware_fly_to_position.hpp>
#include <iii_drone_interfaces/srv/get_powerline_overview.hpp>

namespace iii_drone {
namespace control {
namespace maneuver {

    class CableAwareFlyToPositionManeuverServer : public ManeuverServer {
        using CableAwareFlyToPosition = iii_drone_interfaces::action::CableAwareFlyToPosition;
        using GoalHandleCableAwareFlyToPosition = rclcpp_action::ServerGoalHandle<CableAwareFlyToPosition>;

    public:
        CableAwareFlyToPositionManeuverServer(
            rclcpp_lifecycle::LifecycleNode * node,
            CombinedDroneAwarenessHandler::SharedPtr combined_drone_awareness_handler,
            const std::string & action_name,
            unsigned int wait_for_execute_poll_ms,
            unsigned int evaluate_done_poll_ms,
            iii_drone::configuration::Configuration::SharedPtr parameters,
            iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client
        );

        bool CanExecuteManeuver(
            const Maneuver & maneuver,
            const iii_drone::adapters::CombinedDroneAwarenessAdapter & drone_awareness
        ) const override;

        iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(const Maneuver & maneuver) override;

    private:
        maneuver_type_t maneuver_type() const override;
        void startExecution(Maneuver & maneuver) override;
        bool canCancel() override;
        std::optional<ControlledCancellationConfig> controlledCancellationConfig() const override;
        bool rebaseExecution(const State & stopped_state, std::string & reason) override;
        iii_drone::control::Reference computeReference(const iii_drone::control::State & state) override;
        bool hasSucceeded(Maneuver & maneuver) override;
        bool hasFailed(Maneuver & maneuver) override;
        std::shared_ptr<void> getFeedback(Maneuver & maneuver) override;
        void publishResultAndFinalize(Maneuver & maneuver, maneuver_result_type_t maneuver_result_type) override;
        void registerReferenceCallbackOnSuccess(const Maneuver & maneuver) override;
        bool validateManeuverParameters(const fly_to_position_maneuver_params_t & maneuver_params) const;
        bool hasStoredPowerlineOverview() const;
        std::optional<iii_drone::adapters::PowerlineAdapter> storedPowerlineOverview() const;
        bool targetSatisfiesCableClearance(
            const fly_to_position_maneuver_params_t & maneuver_params,
            const iii_drone::adapters::PowerlineAdapter & powerline
        ) const;
        double distanceToCable(
            const iii_drone::types::point_t & point,
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::adapters::SingleLineAdapter & line
        ) const;
        iii_drone::types::vector_t cableDirection(
            const iii_drone::adapters::PowerlineAdapter & powerline,
            const iii_drone::adapters::SingleLineAdapter & line
        ) const;

        iii_drone::configuration::Configuration::SharedPtr configuration_;
        iii_drone::control::TrajectoryGeneratorClient::SharedPtr trajectory_generator_client_;
        iii_drone::utils::Atomic<iii_drone::control::Reference> target_reference_;
        iii_drone::utils::Atomic<bool> first_iteration_ = true;
        iii_drone::utils::Atomic<bool> waiting_for_initial_plan_ = false;
        iii_drone::utils::Atomic<bool> has_failed_ = false;
        rclcpp::CallbackGroup::SharedPtr powerline_overview_client_cb_group_;
        rclcpp::Client<iii_drone_interfaces::srv::GetPowerlineOverview>::SharedPtr get_powerline_overview_client_;
    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
