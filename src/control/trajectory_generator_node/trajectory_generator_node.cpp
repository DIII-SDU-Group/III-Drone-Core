/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator_node/trajectory_generator_node.hpp>

#include <chrono>
#include <future>

using namespace iii_drone::control::trajectory_generator_node;

namespace {

using LifecycleConfigurator = iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>;
using ParameterType = rclcpp::ParameterType;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

void DeclareManagedParameters(LifecycleConfigurator & configurator)
{
    const auto bool_t = ParameterType::PARAMETER_BOOL;
    const auto int_t = ParameterType::PARAMETER_INTEGER;
    const auto double_t = ParameterType::PARAMETER_DOUBLE;
    const auto string_t = ParameterType::PARAMETER_STRING;

    configurator.DeclareParameter("/control/trajectory_generator/MPC_use_state_feedback", bool_t);
    configurator.DeclareParameter("/control/trajectory_generator/MPC_N", int_t);
    configurator.DeclareParameter("/control/dt", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/interpolation_avg_velocity_m_s", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/interpolation_max_velocity_m_s", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", double_t);
    configurator.DeclareParameter("/control/trajectory_interpolator/reference_trajectory_length_N", int_t);
    configurator.DeclareParameter("/control/trajectory_generator/cable_aware_grid_resolution_m", double_t);
    configurator.DeclareParameter("/control/trajectory_generator/cable_aware_grid_margin_m", double_t);
    configurator.DeclareParameter("/control/trajectory_generator/cable_aware_clearance_m", double_t);
    configurator.DeclareParameter("/control/trajectory_generator/cable_aware_max_astar_expansions", int_t);
    configurator.DeclareParameter("/tf/world_frame_id", string_t);
    configurator.DeclareParameter("/tf/drone_frame_id", string_t);

    const std::vector<std::string> mpc_prefixes = {"position_MPC", "cable_landing_MPC", "cable_takeoff_MPC"};
    const std::vector<std::string> weighted_suffixes = {
        "vx_max", "vy_max", "vz_max",
        "ax_max", "ay_max", "az_max",
        "wx", "wy", "wz",
        "wvx", "wvy", "wvz",
        "wax", "way", "waz",
        "wjx", "wjy", "wjz",
    };

    for (const auto & prefix : mpc_prefixes) {
        for (const auto & suffix : weighted_suffixes) {
            configurator.DeclareParameter("/control/trajectory_generator/" + prefix + "_" + suffix, double_t);
        }
    }

    for (const auto & prefix : mpc_prefixes) {
        std::vector<ConfigurationEntry> entries;
        entries.reserve(5 + weighted_suffixes.size());
        entries.emplace_back("/control/trajectory_generator/MPC_use_state_feedback", bool_t);
        entries.emplace_back("/control/trajectory_generator/MPC_N", int_t);
        entries.emplace_back("/control/dt", double_t);
        entries.emplace_back("/tf/world_frame_id", string_t);
        entries.emplace_back("/tf/drone_frame_id", string_t);
        for (const auto & suffix : weighted_suffixes) {
            entries.emplace_back("/control/trajectory_generator/" + prefix + "_" + suffix, double_t);
        }
        configurator.CreateConfiguration(prefix, entries);
    }

    configurator.CreateConfiguration("trajectory_interpolator", {
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_avg_velocity_m_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_velocity_m_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/reference_trajectory_length_N", int_t),
        ConfigurationEntry("/control/dt", double_t),
    });

    configurator.CreateConfiguration("cable_aware_trajectory_planner", {
        ConfigurationEntry("/control/trajectory_generator/cable_aware_grid_resolution_m", double_t),
        ConfigurationEntry("/control/trajectory_generator/cable_aware_grid_margin_m", double_t),
        ConfigurationEntry("/control/trajectory_generator/cable_aware_clearance_m", double_t),
        ConfigurationEntry("/control/trajectory_generator/cable_aware_max_astar_expansions", int_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_avg_velocity_m_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_avg_yaw_rate_rad_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_velocity_m_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_acceleration_m_s2", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_yaw_rate_rad_s", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/interpolation_max_yaw_acceleration_rad_s2", double_t),
        ConfigurationEntry("/control/trajectory_interpolator/reference_trajectory_length_N", int_t),
        ConfigurationEntry("/control/dt", double_t),
    });
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectoryGeneratorNode::TrajectoryGeneratorNode(
    const std::string node_name,
    const std::string node_namespace,
    const rclcpp::NodeOptions & options
) : LifecycleNode(node_name, node_namespace, options) {
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	const char * log_level_env = std::getenv("TRAJECTORY_GENERATOR_LOG_LEVEL");
	std::string log_level = log_level_env == nullptr ? "" : log_level_env;

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			set_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			set_logger_level(RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			set_logger_level(RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			set_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			set_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

    trajectory_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "trajectory_path",
        10
    );

    target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose",
        10
    );

    trajectory_compute_time_publisher_ = this->create_publisher<iii_drone_interfaces::msg::TrajectoryComputeTime>(
        "trajectory_compute_time",
        10
    );

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::TrajectoryGeneratorNode(): Ready.");

}

TrajectoryGeneratorNode::~TrajectoryGeneratorNode() {

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::~TrajectoryGeneratorNode(): Terminating.");

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryGeneratorNode::on_configure(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::on_configure(): Configuring.");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "TrajectoryGeneratorNode::on_configure(): Failed to configure parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "TrajectoryGeneratorNode::on_configure(): Initializing configurator."
    );

    configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>>(
        this,
        "trajectory_generator"
    );
    DeclareManagedParameters(*configurator_);
    configurator_->validate();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryGeneratorNode::on_cleanup(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::on_cleanup(): Cleaning up.");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "TrajectoryGeneratorNode::on_cleanup(): Failed to cleanup parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "TrajectoryGeneratorNode::on_cleanup(): Cleaning up configurator."
    );

    configurator_.reset();
    configurator_ = nullptr;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryGeneratorNode::on_activate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::on_activate(): Activating.");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "TrajectoryGeneratorNode::on_activate(): Failed to activate parent class."
        );
        return parent_return;
    }

    trajectory_generator_ = std::make_shared<TrajectoryGenerator> (
        configurator_->GetConfiguration("position_MPC"),
        configurator_->GetConfiguration("cable_landing_MPC"),
        configurator_->GetConfiguration("cable_takeoff_MPC"),
        this
    );

    trajectory_interpolator_ = std::make_shared<TrajectoryInterpolator> (
        configurator_->GetConfiguration("trajectory_interpolator"),
        this
    );

    cable_aware_trajectory_planner_ = std::make_shared<CableAwareTrajectoryPlanner> (
        configurator_->GetConfiguration("cable_aware_trajectory_planner"),
        this
    );

    powerline_overview_client_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
    );
    get_powerline_overview_client_ = this->create_client<iii_drone_interfaces::srv::GetPowerlineOverview>(
        "/mission/powerline_overview_provider/get_powerline_overview",
        rclcpp::ServicesQoS(),
        powerline_overview_client_cb_group_
    );

    compute_reference_trajectory_service_ = this->create_service<iii_drone_interfaces::srv::ComputeReferenceTrajectory>(
        "compute_reference_trajectory",
        std::bind(
            &TrajectoryGeneratorNode::computeReferenceTrajectoryCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2
        )
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryGeneratorNode::on_deactivate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::on_deactivate(): Deactivating.");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "TrajectoryGeneratorNode::on_deactivate(): Failed to deactivate parent class."
        );
        return parent_return;
    }

    compute_reference_trajectory_service_->clear_on_new_request_callback();
    compute_reference_trajectory_service_.reset();
    compute_reference_trajectory_service_ = nullptr;

    trajectory_interpolator_.reset();
    trajectory_interpolator_ = nullptr;

    cable_aware_trajectory_planner_.reset();
    cable_aware_trajectory_planner_ = nullptr;

    get_powerline_overview_client_.reset();
    get_powerline_overview_client_ = nullptr;

    powerline_overview_client_cb_group_.reset();
    powerline_overview_client_cb_group_ = nullptr;

    trajectory_generator_.reset();
    trajectory_generator_ = nullptr;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryGeneratorNode::on_shutdown(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::on_shutdown(): Shutting down.");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "TrajectoryGeneratorNode::on_shutdown(): Failed to shutdown parent class."
        );
        return parent_return;
    }

    // Create and start thread detached which sleeps for 1 second, then shuts down rclcpp
    std::thread shutdown_thread([this](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });
    shutdown_thread.detach();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TrajectoryGeneratorNode::on_error(const rclcpp_lifecycle::State & state) {

    RCLCPP_FATAL(this->get_logger(), "TrajectoryGeneratorNode::on_error(): Lifecycle transition failed.");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_error(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "TrajectoryGeneratorNode::on_error(): Failed to error parent class."
        );
        return parent_return;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

void TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::ComputeReferenceTrajectory::Response> response
) {

    adapters::StateAdapter state_adapter(request->state);
    adapters::ReferenceAdapter reference_adapter(request->reference);
    adapters::ReferenceAdapter start_reference_adapter(request->start_reference);

    bool use_mpc = request->use_mpc;

    // Compute the reference trajectory
    ReferenceTrajectory ref_traj;

    // Measure time
    uint64_t nanoseconds;
    std::string type;
    
    try {
    if ((trajectory_mode_t)request->trajectory_mode.mode == trajectory_mode_t::cable_aware) {

        type = "CableAwareAStarLLS";

        RCLCPP_DEBUG(
            this->get_logger(),
            "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Using cable-aware A* + LLS smoothing."
        );

        auto start = std::chrono::high_resolution_clock::now();

        iii_drone::adapters::PowerlineAdapter powerline = getStoredPowerlineOverview();

        ref_traj = cable_aware_trajectory_planner_->ComputeReferenceTrajectory(
            state_adapter.state(),
            reference_adapter.reference(),
            powerline,
            request->reset
        );

        auto end = std::chrono::high_resolution_clock::now();

        nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    } else if (use_mpc) {

        type = "MPC";

        RCLCPP_DEBUG(
            this->get_logger(), 
            "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Using MPC."
        );

        auto start = std::chrono::high_resolution_clock::now();

        ref_traj = trajectory_generator_->ComputeReferenceTrajectory(
            state_adapter.state(),
            reference_adapter.reference(),
            request->set_reference,
            request->reset,
            (trajectory_mode_t)request->trajectory_mode.mode
        );

        auto end = std::chrono::high_resolution_clock::now();

        nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    } else {

        type = "Interpolation";

        RCLCPP_DEBUG(
            this->get_logger(), 
            "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Using interpolation."
        );

        auto start = std::chrono::high_resolution_clock::now();

        if (request->use_start_reference) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Using full start reference for interpolation."
            );
            ref_traj = trajectory_interpolator_->ComputeReferenceTrajectory(
                start_reference_adapter.reference(),
                reference_adapter.reference(),
                request->set_reference,
                request->reset
            );
        } else {
            ref_traj = trajectory_interpolator_->ComputeReferenceTrajectory(
                state_adapter.state(),
                reference_adapter.reference(),
                request->set_reference,
                request->reset
            );
        }

        auto end = std::chrono::high_resolution_clock::now();

        nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "TrajectoryGeneratorNode::computeReferenceTrajectoryCallback(): Trajectory computation failed: %s",
            ex.what()
        );
        ref_traj = ReferenceTrajectory({Reference(state_adapter.state())});
        nanoseconds = 0;
        type = "FailedFallback";
        response->success = false;
        response->error_message = ex.what();
    }

    if (type != "FailedFallback") {
        response->success = true;
        response->error_message = "";
    }

    // Set the response
    adapters::ReferenceTrajectoryAdapter ref_traj_adapter(ref_traj);
    iii_drone_interfaces::msg::ReferenceTrajectory msg = ref_traj_adapter.ToMsg();

    response->reference_trajectory = msg;

    publishTrajectoryPath(ref_traj_adapter);
    publishTargetPose(reference_adapter);

    iii_drone_interfaces::msg::TrajectoryComputeTime compute_time_msg;
    compute_time_msg.nanoseconds = nanoseconds;
    compute_time_msg.trajectory_type = type;

    trajectory_compute_time_publisher_->publish(compute_time_msg);

}

void TrajectoryGeneratorNode::publishTrajectoryPath(const adapters::ReferenceTrajectoryAdapter & reference_trajectory_adapter) {

    nav_msgs::msg::Path path_msg = reference_trajectory_adapter.ToPathMsg(configurator_->GetParameter("/tf/world_frame_id").as_string());

    trajectory_path_publisher_->publish(path_msg);

}

void TrajectoryGeneratorNode::publishTargetPose(const adapters::ReferenceAdapter & reference_adapter) {

    geometry_msgs::msg::PoseStamped pose_msg = reference_adapter.ToPoseStampedMsg(configurator_->GetParameter("/tf/world_frame_id").as_string());
    target_pose_publisher_->publish(pose_msg);

}

iii_drone::adapters::PowerlineAdapter TrajectoryGeneratorNode::getStoredPowerlineOverview() {

    if (!get_powerline_overview_client_) {
        throw std::runtime_error("TrajectoryGeneratorNode: powerline overview client is not initialized.");
    }

    if (!get_powerline_overview_client_->wait_for_service(std::chrono::milliseconds(500))) {
        throw std::runtime_error(
            "TrajectoryGeneratorNode: cable-aware trajectory requested, but stored powerline overview service is unavailable."
        );
    }

    auto request = std::make_shared<iii_drone_interfaces::srv::GetPowerlineOverview::Request>();
    auto future = get_powerline_overview_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error(
                "TrajectoryGeneratorNode: timed out waiting for stored powerline overview."
            );
        }
    }

    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        throw std::runtime_error("TrajectoryGeneratorNode: stored powerline overview request did not complete.");
    }

    auto response = future.get();
    if (!response->success) {
        throw std::runtime_error(
            "TrajectoryGeneratorNode: cable-aware trajectory rejected because no stored powerline overview is available."
        );
    }

    if (response->stored_powerline.lines.empty()) {
        throw std::runtime_error(
            "TrajectoryGeneratorNode: cable-aware trajectory rejected because stored powerline overview has no lines."
        );
    }

    RCLCPP_DEBUG(
        this->get_logger(),
        "TrajectoryGeneratorNode::getStoredPowerlineOverview(): Using stored powerline overview with %zu line(s).",
        response->stored_powerline.lines.size()
    );

    return iii_drone::adapters::PowerlineAdapter(response->stored_powerline);

}

int main(int argc, char * argv[]) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<TrajectoryGeneratorNode>();

    executor.add_node(node->get_node_base_interface());

    try {
        
        executor.spin();

    } catch(const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "TrajectoryGeneratorNode main loop failed: %s", e.what());
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;

}
