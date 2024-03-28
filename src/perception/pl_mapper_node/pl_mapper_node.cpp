/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/perception/pl_mapper_node/pl_mapper_node.hpp"

using namespace iii_drone::perception::pl_mapper_node;
using namespace iii_drone::math;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineMapperNode::PowerlineMapperNode(
    const std::string & node_name, 
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp::Node(
        node_name, 
        node_namespace,
        options
    ), 
    configurator_(this),
    pl_mapper_state_(configurator_.GetParameter("pl_mapper_begin_running").as_bool() ? pl_mapper_state_running : pl_mapper_state_idle) {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::PowerlineMapperNode(): Initializing PL mapper");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    powerline_ = std::make_shared<Powerline>(
        configurator_.GetParameterBundle("powerline"),
        tf_buffer_
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

    pl_mapper_command_srv_ = this->create_service<iii_drone_interfaces::srv::PLMapperCommand>(
        "pl_mapper_command", 
        std::bind(
            &PowerlineMapperNode::plMapperCommandCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2
        )
    );

    geometry_msgs::msg::TransformStamped mmw_tf;

    rclcpp::Rate tf_poll_rate(std::chrono::milliseconds(500));

    while(true) {

        try {

            mmw_tf = tf_buffer_->lookupTransform(
                configurator_.GetParameter("drone_frame_id").as_string(), 
                configurator_.GetParameter("mmwave_frame_id").as_string(), 
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

    std::chrono::milliseconds sleep_ms(configurator_.GetParameter("init_sleep_time_ms").as_int());
	rclcpp::Rate init_sleep_rate(sleep_ms);
	init_sleep_rate.sleep();

    // Call on_timer function every second
    std::chrono::milliseconds odom_callback_ms(configurator_.GetParameter("odometry_callback_period_ms").as_int());
    drone_tf_timer_ = this->create_wall_timer(
        odom_callback_ms, 
        std::bind(
            &PowerlineMapperNode::odometryCallback, 
            this
        )
    );

    quaternion_t mmw_quat = quaternionFromTransformMsg(mmw_tf.transform);

    R_drone_to_mmw_ = quatToMat(mmw_quat);
    v_drone_to_mmw_ = vectorFromTransformMsg(mmw_tf.transform);

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::PowerlineMapperNode(): PL mapper initialized");

}

void PowerlineMapperNode::plMapperCommandCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::PLMapperCommand::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::PLMapperCommand::Response> response
) {

    pl_mapper_state_t previous_state = pl_mapper_state_;

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Received command");

    if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_START) {

        RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Starting PL mapper");

        pl_mapper_state_ = pl_mapper_state_running;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

    } else if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_STOP) {

        RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Stopping PL mapper");

        pl_mapper_state_ = pl_mapper_state_idle;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

        publishPowerline(
            Powerline(
                configurator_.GetParameterBundle("powerline"),
                tf_buffer_
            )
        );

    } else if (request->pl_mapper_cmd.command == iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_PAUSE) {

        RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Pausing PL mapper");

        pl_mapper_state_ = pl_mapper_state_paused;
        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;

    } else {

        RCLCPP_ERROR(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Invalid command");

        response->pl_mapper_ack = iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_INVALID_CMD;

        return;

    }

    if (request->pl_mapper_cmd.reset || (previous_state == pl_mapper_state_idle && pl_mapper_state_ == pl_mapper_state_running)) {

        RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::plMapperCommandCallback(): Resetting PL mapper");

        powerline_->Reset();

    }

}

void PowerlineMapperNode::odometryCallback() {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::odometryCallback(): Updating powerline by odometry");

    if (pl_mapper_state_ == pl_mapper_state_idle) {

        return;

    }

    geometry_msgs::msg::TransformStamped tf;

    try {

        tf = tf_buffer_->lookupTransform(
            configurator_.GetParameter("world_frame_id").as_string(), 
            configurator_.GetParameter("drone_frame_id").as_string(), 
            tf2::TimePointZero
        );

    } catch(tf2::TransformException & ex) {

        RCLCPP_WARN(
            this->get_logger(), 
            "Could not get odometry transform, frame drone to world"
        );

        return;

    }

    pose_t drone_pose = poseFromTransformMsg(tf.transform);

    powerline_->UpdateOdometry(drone_pose);

    publishPowerline(*powerline_);

}

void PowerlineMapperNode::mmWaveCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    RCLCPP_DEBUG(this->get_logger(), "PowerlineMapperNode::mmWaveCallback(): Received mmWave point cloud, updating powerline");

    if (pl_mapper_state_ != pl_mapper_state_running) {

        return;

    }

    iii_drone::adapters::PointCloudAdapter pcl_adapter(msg);

    std::vector<point_t> transformed_points;
    std::vector<point_t> projected_points;

    std::vector<point_t> pcl_points = pcl_adapter.points();

    for (size_t i = 0; i < pcl_points.size(); i++) {

        auto line = SingleLine(
            -1,
            pcl_points[i],
            pl_direction_,
            configurator_.GetParameter("mmwave_frame_id").as_string(),
            tf_buffer_,
            configurator_.GetParameterBundle("powerline")
        );

        if(!line.IsInFOV()) {

            continue;

        }

        geometry_msgs::msg::PointStamped pt;
        pt.header.frame_id = msg->header.frame_id;
        pt.point = pointMsgFromPoint(pcl_points[i]);

        pt = tf_buffer_->transform(
            pt, 
            configurator_.GetParameter("drone_frame_id").as_string()
        );

        point_t transformed_point = pointFromPointMsg(pt.point);

        point_t projected_point = powerline_->UpdateLine(transformed_point);

        transformed_points.push_back(transformed_point);
        projected_points.push_back(projected_point);

    }   

    powerline_->CleanupLines();

    powerline_->ComputeInterLinePositions();

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

    if (pl_mapper_state_ != pl_mapper_state_running) {

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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub
) const {

    iii_drone::adapters::PointCloudAdapter pcl_adapter(
        powerline_->stamp(),
        configurator_.GetParameter("drone_frame_id").as_string(),
        points
    );

    pub->publish(pcl_adapter.ToMsg());

}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<PowerlineMapperNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
