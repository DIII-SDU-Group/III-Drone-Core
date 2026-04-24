/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/pl_mapper_node/pl_mapper_node.hpp"

using namespace iii_drone::perception::pl_mapper_node;
using namespace iii_drone::math;
using namespace iii_drone::types;

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

    configurator.DeclareParameter("/perception/begin_running", bool_t);
    configurator.DeclareParameter("/perception/pl_mapper/kf_r", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/kf_q", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/alive_cnt_low_thresh", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/alive_cnt_high_thresh", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/alive_cnt_ceiling", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/min_point_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/max_point_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/view_cone_slope", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/strict_min_point_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/strict_max_point_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/strict_view_cone_slope", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/matching_line_max_dist", double_t);
    configurator.DeclareParameter("/perception/pl_mapper/predict_callback_period_ms", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/max_lines", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/use_inter_line_positions", bool_t);
    configurator.DeclareParameter("/perception/pl_mapper/inter_pos_window_size", int_t);
    configurator.DeclareParameter("/perception/pl_mapper/overwrite_non_FOV_line_positions_from_inter_pos", bool_t);
    configurator.DeclareParameter("/tf/drone_frame_id", string_t);
    configurator.DeclareParameter("/tf/world_frame_id", string_t);
    configurator.DeclareParameter("/tf/cable_gripper_frame_id", string_t);
    configurator.DeclareParameter("/tf/mmwave_frame_id", string_t);

    configurator.CreateConfiguration("powerline", {
        ConfigurationEntry("/perception/pl_mapper/kf_r", double_t),
        ConfigurationEntry("/perception/pl_mapper/kf_q", double_t),
        ConfigurationEntry("/perception/pl_mapper/alive_cnt_low_thresh", int_t),
        ConfigurationEntry("/perception/pl_mapper/alive_cnt_high_thresh", int_t),
        ConfigurationEntry("/perception/pl_mapper/alive_cnt_ceiling", int_t),
        ConfigurationEntry("/perception/pl_mapper/matching_line_max_dist", double_t),
        ConfigurationEntry("/perception/pl_mapper/max_lines", int_t),
        ConfigurationEntry("/perception/pl_mapper/min_point_dist", double_t),
        ConfigurationEntry("/perception/pl_mapper/max_point_dist", double_t),
        ConfigurationEntry("/perception/pl_mapper/view_cone_slope", double_t),
        ConfigurationEntry("/perception/pl_mapper/strict_min_point_dist", double_t),
        ConfigurationEntry("/perception/pl_mapper/strict_max_point_dist", double_t),
        ConfigurationEntry("/perception/pl_mapper/strict_view_cone_slope", double_t),
        ConfigurationEntry("/perception/pl_mapper/inter_pos_window_size", int_t),
        ConfigurationEntry("/perception/pl_mapper/overwrite_non_FOV_line_positions_from_inter_pos", bool_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
        ConfigurationEntry("/tf/mmwave_frame_id", string_t),
    });
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineMapperNode::PowerlineMapperNode(
    const std::string & node_name, 
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(
    node_name, 
    node_namespace,
    options
) {
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	const char * log_level_env = std::getenv("PL_MAPPER_LOG_LEVEL");
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

    powerline_pub_ = this->create_publisher<iii_drone_interfaces::msg::Powerline>(
        "powerline", 
        10
    );
    points_est_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "points_est",
        10
    );
    transformed_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "transformed_points", 
        10
    );
    projected_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "projected_points", 
        10
    );

    system_command_clients_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    hough_transformer_command_client_ = this->create_client<iii_drone_interfaces::srv::SystemCommand>(
        "/perception/hough_transformer/command",
        rclcpp::ServicesQoS(),
        system_command_clients_callback_group_
    );

    pl_dir_computer_command_client_ = this->create_client<iii_drone_interfaces::srv::SystemCommand>(
        "/perception/pl_dir_computer/command",
        rclcpp::ServicesQoS(),
        system_command_clients_callback_group_
    );

    state_pub_ = this->create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "state", 
        10
    );

    state_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), 
        [this]() {
            auto msg = std::make_unique<iii_drone_interfaces::msg::StringStamped>();
            msg->stamp = this->now();
            
            switch(pl_mapper_state_) {
                case pl_mapper_state_running:
                    msg->data = "Running";
                    break;
                case pl_mapper_state_paused:
                    msg->data = "Paused";
                    break;
                case pl_mapper_state_frozen:
                    msg->data = "Frozen";
                    break;
                default:
                case pl_mapper_state_idle:
                    msg->data = "Idle";
                    break;
            }

            state_pub_->publish(std::move(msg));
        }
    );

    RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::PowerlineMapperNode(): PL mapper ready");

}

PowerlineMapperNode::~PowerlineMapperNode() {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::~PowerlineMapperNode(): Destroying PL mapper");

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PowerlineMapperNode::on_configure(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::on_configure(): Configuring PL mapper");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineMapperNode::on_configure(): Failed to configure parent class."
        );
        return parent_return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_configure(): Initializing configurator object"
    );

    configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>>(
        this,
        "pl_mapper"
    );
    DeclareManagedParameters(*configurator_);
    configurator_->validate();

    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_configure(): Setting internal state"
    );

    pl_mapper_state_ = configurator_->GetParameter("/perception/begin_running").as_bool() ? pl_mapper_state_running : pl_mapper_state_idle;

    // Tf:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_configure(): Setting up tf2 buffer and listener"
    );

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineMapperNode::on_cleanup(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::on_cleanup(): Cleaning up PL mapper");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineMapperNode::on_cleanup(): Failed to cleanup parent class."
        );
        return parent_return;
    }

    // Tf:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_cleanup(): Cleaning up tf2 buffer and listener"
    );

    transform_listener_.reset();
    transform_listener_ = nullptr;

    tf_buffer_->clear();
    tf_buffer_.reset();
    tf_buffer_ = nullptr;

    // Configurator:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_cleanup(): Cleaning up configurator object"
    );

    configurator_.reset();
    configurator_ = nullptr;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineMapperNode::on_activate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::on_activate(): Activating PL mapper");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineMapperNode::on_activate(): Failed to activate parent class."
        );
        return parent_return;
    }

    // Powerline object:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_activate(): Initializing powerline object"
    );

    powerline_ = std::make_shared<Powerline>(
        configurator_->GetConfiguration("powerline"),
        tf_buffer_
    );

    // Subscribers:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_activate(): Initializing subscribers"
    );

    pl_direction_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/perception/pl_dir_computer/powerline_direction_quat", 
        10, 
        std::bind(
            &PowerlineMapperNode::plDirectionCallback, 
            this, 
            std::placeholders::_1
        )
    );

    mmwave_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor/mmwave/pcl", 
        10, 
        std::bind(
            &PowerlineMapperNode::mmWaveCallback, 
            this,
            std::placeholders::_1
        )
    );

    // PL mapper command service:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_activate(): Creating PL mapper command service"
    );

    pl_mapper_command_srv_ = this->create_service<iii_drone_interfaces::srv::PLMapperCommand>(
        "pl_mapper_command", 
        std::bind(
            &PowerlineMapperNode::plMapperCommandCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2
        )
    );

    // Prediction timer:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_activate(): Setting up prediction timer"
    );

    std::chrono::milliseconds predict_callback_ms(
        configurator_->GetParameter("/perception/pl_mapper/predict_callback_period_ms").as_int()
    );

    pl_predict_timer_ = this->create_wall_timer(
        predict_callback_ms, 
        std::bind(
            &PowerlineMapperNode::predictCallback, 
            this
        )
    );

    // Getting initial transforms:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_activate(): Getting initial transforms"
    );

    geometry_msgs::msg::TransformStamped mmw_tf;

    rclcpp::Rate tf_poll_rate(std::chrono::milliseconds(500));

    while(rclcpp::ok()) {

        try {

            mmw_tf = tf_buffer_->lookupTransform(
                configurator_->GetParameter("/tf/drone_frame_id").as_string(), 
                configurator_->GetParameter("/tf/mmwave_frame_id").as_string(), 
                tf2::TimePointZero
            );

            break;

        } catch(tf2::TransformException & ex) {

            RCLCPP_DEBUG(
                this->get_logger(), 
                "Could not get mmWave transform, frame drone to iwr6843_frame, trying again..."
            );

        }

        tf_poll_rate.sleep();

    }

    quaternion_t mmw_quat = quaternionFromTransformMsg(mmw_tf.transform);

    R_drone_to_mmw_ = quatToMat(mmw_quat);
    v_drone_to_mmw_ = vectorFromTransformMsg(mmw_tf.transform);


    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineMapperNode::on_deactivate(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::on_deactivate(): Deactivating PL mapper");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineMapperNode::on_deactivate(): Failed to deactivate parent class."
        );
        return parent_return;
    }

    // Prediction timer:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_deactivate(): Stopping prediction timer"
    );

    pl_predict_timer_->cancel();
    pl_predict_timer_.reset();
    pl_predict_timer_ = nullptr;

    // PL mapper command service:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_deactivate(): Stopping PL mapper command service"
    );

    pl_mapper_command_srv_->clear_on_new_request_callback();
    pl_mapper_command_srv_.reset();
    pl_mapper_command_srv_ = nullptr;

    //Subscribers:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_deactivate(): Cleaning up publishers and subscribers"
    );

    pl_direction_sub_->clear_on_new_message_callback();
    pl_direction_sub_.reset();
    pl_direction_sub_ = nullptr;

    mmwave_sub_->clear_on_new_message_callback();
    mmwave_sub_.reset();
    mmwave_sub_ = nullptr;

    // Powerline object:
    RCLCPP_DEBUG(
        this->get_logger(), 
        "PowerlineMapperNode::on_deactivate(): Cleaning up powerline object"
    );

    powerline_.reset();
    powerline_ = nullptr;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineMapperNode::on_shutdown(const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::on_shutdown(): Shutting down PL mapper");

    CallbackReturn parent_return = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (parent_return != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            this->get_logger(), 
            "PowerlineMapperNode::on_shutdown(): Failed to shutdown parent class."
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

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PowerlineMapperNode::on_error(const rclcpp_lifecycle::State & state) {
    RCLCPP_FATAL(this->get_logger(), "PowerlineMapperNode::on_error(): Lifecycle transition failed.");

    return rclcpp_lifecycle::LifecycleNode::on_error(state);

}

void PowerlineMapperNode::plMapperCommandCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::PLMapperCommand::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::PLMapperCommand::Response> response
) {

    auto send_system_command = [this](
        bool start_nstop,
        rclcpp::Client<iii_drone_interfaces::srv::SystemCommand>::SharedPtr command_client
    ) {

        if (!command_client->wait_for_service(std::chrono::seconds(1))) {
            std::string fatal_msg = "PowerlineMapperNode::plMapperCommandCallback(): " + std::string(command_client->get_service_name()) + " service not available";

            RCLCPP_FATAL(
                this->get_logger(), 
                fatal_msg.c_str()
            );

            throw std::runtime_error(fatal_msg);

        }

        auto req = std::make_shared<iii_drone_interfaces::srv::SystemCommand::Request>();
        req->command = start_nstop ? req->SYSTEM_COMMAND_START : req->SYSTEM_COMMAND_STOP;

        bool done = false;

        auto cb = [&done](
            rclcpp::Client<iii_drone_interfaces::srv::SystemCommand>::SharedFuture
        ) {
            done = true;
        };

        auto res = command_client->async_send_request(
            req,
            cb
        );

        rclcpp::Rate rate(10);

        while (!done) {
            rate.sleep();
        }

        return;

    };

    pl_mapper_state_t previous_state = pl_mapper_state_;

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Received command");

    if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_START) {

        RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Starting PL mapper");

        send_system_command(
            true,
            hough_transformer_command_client_
        );
        
        send_system_command(
            true,
            pl_dir_computer_command_client_
        );

        pl_mapper_state_ = pl_mapper_state_running;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

    } else if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_STOP) {

        RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Stopping PL mapper");

        send_system_command(
            false,
            hough_transformer_command_client_
        );
        
        send_system_command(
            false,
            pl_dir_computer_command_client_
        );

        pl_mapper_state_ = pl_mapper_state_idle;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

        publishPowerline(
            Powerline(
                configurator_->GetConfiguration("powerline"),
                tf_buffer_
            )
        );

    } else if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_PAUSE) {

        RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Pausing PL mapper");

        send_system_command(
            true,
            hough_transformer_command_client_
        );
        
        send_system_command(
            true,
            pl_dir_computer_command_client_
        );

        pl_mapper_state_ = pl_mapper_state_paused;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

    } else if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_FREEZE) {

        RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Freezing PL mapper");

        send_system_command(
            false,
            hough_transformer_command_client_
        );
        
        send_system_command(
            false,
            pl_dir_computer_command_client_
        );

        pl_mapper_state_ = pl_mapper_state_frozen;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

    } else {

        RCLCPP_WARN(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Invalid command");

        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_INVALID_CMD;

        return;

    }

    if (request->pl_mapper_cmd.reset || (previous_state == pl_mapper_state_idle && pl_mapper_state_ == pl_mapper_state_running)) {

        RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Resetting PL mapper");

        powerline_->Reset();

    }

}

void PowerlineMapperNode::predictCallback() {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::predictCallback(): Running predict step");

    pl_mapper_state_t pl_mapper_state = pl_mapper_state_;

    bool only_orientation = false;

    switch(pl_mapper_state) {

        case pl_mapper_state_idle:
            return;

        case pl_mapper_state_frozen:
            only_orientation = true;
            [[fallthrough]];
        case pl_mapper_state_running:
        case pl_mapper_state_paused: {

            geometry_msgs::msg::TransformStamped tf;

            try {

                tf = tf_buffer_->lookupTransform(
                    configurator_->GetParameter("/tf/world_frame_id").as_string(), 
                    configurator_->GetParameter("/tf/drone_frame_id").as_string(), 
                    tf2::TimePointZero
                );

            } catch(tf2::TransformException & ex) {

                RCLCPP_DEBUG(
                    this->get_logger(), 
                    "Could not get odometry transform, frame drone to world"
                );

                return;

            }

            pose_t drone_pose = poseFromTransformMsg(tf.transform);

            powerline_->UpdateOdometry(
                drone_pose,
                only_orientation
            );

            break;

        }

    }

    publishPowerline(*powerline_);

}

void PowerlineMapperNode::mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::mmWaveCallback(): Received mmWave point cloud, updating powerline");

    if (pl_mapper_state_ != pl_mapper_state_running) {

        return;

    }

    // RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::mmWaveCallback(): Processing %u points", msg->width);

    iii_drone::adapters::PointCloudAdapter pcl_adapter(msg);

    std::vector<point_t> transformed_points;
    std::vector<point_t> projected_points;

    std::vector<point_t> pcl_points = pcl_adapter.points();

    int n_skipped = 0;

    for (size_t i = 0; i < pcl_points.size(); i++) {

        auto line = SingleLine(
            -1,
            pcl_points[i],
            pl_direction_,
            configurator_->GetParameter("/tf/mmwave_frame_id").as_string(),
            tf_buffer_,
            configurator_->GetConfiguration("powerline")
        );

        if(!line.IsInFOV()) {

            // RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::mmWaveCallback(): Point not in FOV, skipping");

            n_skipped++;

            continue;

        }

        geometry_msgs::msg::PointStamped pt;
        pt.header.frame_id = msg->header.frame_id;
        pt.point = pointMsgFromPoint(pcl_points[i]);

        try {
            pt = tf_buffer_->transform(
                pt, 
                configurator_->GetParameter("/tf/drone_frame_id").as_string()
            );
        } catch (tf2::TransformException & ex) {
            n_skipped++;
            continue;
        }

        point_t transformed_point = pointFromPointMsg(pt.point);

        // RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::mmWaveCallback(): Point in FOV, updating powerline");

        point_t projected_point = powerline_->UpdateLine(transformed_point);

        transformed_points.push_back(transformed_point);
        projected_points.push_back(projected_point);

    }   

    // RCLCPP_INFO(this->get_logger(), "PowerlineMapperNode::mmWaveCallback(): Skipped %d points", n_skipped);

    powerline_->CleanupLines();

    powerline_->ComputeInterLinePositions();

    if (configurator_->GetParameter("/perception/pl_mapper/use_inter_line_positions").as_bool())
        powerline_->UpdateNonFOVLines();

    publishPoints(
        transformed_points, 
        transformed_points_pub_
    );

    publishPoints(
        projected_points, 
        projected_points_pub_
    );



}

void PowerlineMapperNode::plDirectionCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plDirectionCallback(): Received powerline direction, updating powerline");

    if (pl_mapper_state_ != pl_mapper_state_running && pl_mapper_state_ != pl_mapper_state_paused) {

        return;

    }

    quaternion_t quat = quaternionFromQuaternionMsg(msg->quaternion);

    pl_direction_ = quat;

    powerline_->UpdateDirection(quat);
}

void PowerlineMapperNode::publishPowerline(const Powerline & powerline) const {

    auto msg = powerline.ToAdapter(true).ToMsg();

    iii_drone::adapters::PointCloudAdapter pcl_adapter = powerline.ToPointCloudAdapter(true);

    powerline_pub_->publish(msg);
    points_est_pub_->publish(pcl_adapter.ToMsg());

}

void PowerlineMapperNode::publishPoints(
    std::vector<point_t> points, 
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub
) const {

    iii_drone::adapters::PointCloudAdapter pcl_adapter(
        powerline_->stamp(),
        configurator_->GetParameter("/tf/drone_frame_id").as_string(),
        points
    );

    pub->publish(pcl_adapter.ToMsg());

}

int main(int argc, char *argv[]) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<PowerlineMapperNode>();

    executor.add_node(node->get_node_base_interface());

    try {
        
        executor.spin();

    } catch(const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "PowerlineMapperNode main loop failed: %s", e.what());
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;

}
