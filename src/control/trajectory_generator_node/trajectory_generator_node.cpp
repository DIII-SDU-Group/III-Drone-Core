/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/trajectory_generator_node/trajectory_generator_node.hpp>

using namespace iii_drone::control::trajectory_generator_node;

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

	std::string log_level = std::getenv("TRAJECTORY_GENERATOR_LOG_LEVEL");

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
        configurator_->GetParameterBundle("position_MPC"),
        configurator_->GetParameterBundle("cable_landing_MPC"),
        configurator_->GetParameterBundle("cable_takeoff_MPC"),
        this
    );

    trajectory_interpolator_ = std::make_shared<TrajectoryInterpolator> (
        configurator_->GetParameterBundle("trajectory_interpolator"),
        this
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

    RCLCPP_INFO(this->get_logger(), "TrajectoryGeneratorNode::on_error(): Error.");

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

    bool use_mpc = request->use_mpc;

    // Compute the reference trajectory
    ReferenceTrajectory ref_traj;

    // Measure time
    uint64_t nanoseconds;
    std::string type;
    
    if (use_mpc) {

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

        ref_traj = trajectory_interpolator_->ComputeReferenceTrajectory(
            state_adapter.state(),
            reference_adapter.reference(),
            request->set_reference,
            request->reset
        );

        auto end = std::chrono::high_resolution_clock::now();

        nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

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

    nav_msgs::msg::Path path_msg = reference_trajectory_adapter.ToPathMsg(configurator_->GetParameter("world_frame_id").as_string());

    trajectory_path_publisher_->publish(path_msg);

}

void TrajectoryGeneratorNode::publishTargetPose(const adapters::ReferenceAdapter & reference_adapter) {

    geometry_msgs::msg::PoseStamped pose_msg = reference_adapter.ToPoseStampedMsg(configurator_->GetParameter("world_frame_id").as_string());
    target_pose_publisher_->publish(pose_msg);

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
        
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;

}
