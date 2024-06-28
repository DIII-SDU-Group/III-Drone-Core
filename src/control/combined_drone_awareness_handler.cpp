/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/combined_drone_awareness_handler.hpp>

using namespace iii_drone::control;
using namespace iii_drone::utils;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::adapters;

using VehicleStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleStatusAdapter>;
using VehicleOdometryAdapterHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>;
using PowerlineAdapterHistory = iii_drone::utils::History<iii_drone::adapters::PowerlineAdapter>;
using GripperStatusAdapterHistory = iii_drone::utils::History<iii_drone::adapters::GripperStatusAdapter>;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CombinedDroneAwarenessHandler::CombinedDroneAwarenessHandler(
    iii_drone::configuration::ParameterBundle::SharedPtr params,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    rclcpp_lifecycle::LifecycleNode * node,
    bool debug
) : params_(params),
    tf_buffer_(tf_buffer),
    node_(node) {

    debug_ = debug;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::CombinedDroneAwarenessHandler(): Creating publishers");

    combined_drone_awareness_pub_ = node_->create_publisher<iii_drone_interfaces::msg::CombinedDroneAwareness>(
        "combined_drone_awareness",
        10
    );

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    target_pub_ = node_->create_publisher<iii_drone_interfaces::msg::Target>(
        "target",
        10
    );

    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_pose",
        10
    );

    target_drone_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_drone_pose",
        10
    );


}

CombinedDroneAwarenessHandler::~CombinedDroneAwarenessHandler() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::~CombinedDroneAwarenessHandler(): Destroying CombinedDroneAwarenessHandler");

    if (is_started_) {
        Stop();
    }

}

void CombinedDroneAwarenessHandler::Start() {

    if (is_started_) {

        RCLCPP_ERROR(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): CombinedDroneAwarenessHandler is already started");

        return;

    }

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Starting CombinedDroneAwarenessHandler");

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating tf_broadcaster_");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating history and atomic member objects");
    vehicle_status_adapter_history_ = std::make_shared<VehicleStatusAdapterHistory>(1);
    vehicle_odometry_adapter_history_ = std::make_shared<VehicleOdometryAdapterHistory>(2);
    powerline_adapter_history_ = std::make_shared<PowerlineAdapterHistory>(1);
    gripper_status_adapter_history_ = std::make_shared<GripperStatusAdapterHistory>(1);
    ground_altitude_estimate_ = std::make_shared<iii_drone::utils::Atomic<double>>(0.0);
    target_adapter_ = std::make_shared<iii_drone::utils::Atomic<TargetAdapter>>();

    
    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating Register Offboard Mode service");

    register_offboard_mode_srv_ = node_->create_service<iii_drone_interfaces::srv::RegisterOffboardMode>(
        "register_offboard_mode",
        [this](
            const std::shared_ptr<iii_drone_interfaces::srv::RegisterOffboardMode::Request> request, 
            std::shared_ptr<iii_drone_interfaces::srv::RegisterOffboardMode::Response>
        ) {
            if (request->deregister) {
                deregisterOffboardMode(request->mode_id);
            } else {
                registerOffboardMode(request->mode_id);
            }
        }
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating ground altitude estimate");
    int ground_estimate_window_size = params_->GetParameter("ground_estimate_window_size").as_int();
    ground_altitudes_history_ = std::make_shared<History<double>>(0, ground_estimate_window_size);

    ground_altitude_update_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(params_->GetParameter("ground_estimate_update_period_ms").as_int()),
        [this]() {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ground_altitude_update_timer_: Updating ground altitude estimate");
            ground_altitude_update_timer_->cancel();
        }
    );

    ground_altitude_update_timer_->cancel();

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating combined_drone_awareness_");
    combined_drone_awareness_ = std::make_shared<Atomic<combined_drone_awareness_t>>();

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Updating combined drone awareness");
    updateCombinedDroneAwareness();

    if (debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating combined drone awareness publish timer");
    combined_drone_awareness_pub_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(params_->GetParameter("combined_drone_awareness_pub_period_ms").as_int()),
        [this]() {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::combined_drone_awareness_pub_timer_: Publishing combined drone awareness");

            combined_drone_awareness_t cda = combined_drone_awareness();
            
            iii_drone_interfaces::msg::CombinedDroneAwareness msg;

            msg.state = StateAdapter(cda.state).ToMsg();
            msg.armed = cda.armed;
            msg.offboard = cda.offboard;
            msg.has_target = cda.has_target();
            msg.target = cda.target_adapter.ToMsg();
            msg.target_position_known = cda.target_position_known;
            msg.drone_location = (uint8_t)cda.drone_location;
            msg.on_cable_id = cda.on_cable_id;
            msg.ground_altitude_estimate = cda.ground_altitude_estimate;
            msg.gripper_open = cda.gripper_open;

            combined_drone_awareness_pub_->publish(msg);

        }
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating subscribers");
	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        px4_sub_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::vehicle_status_sub_: Vehicle status received");
            iii_drone::adapters::px4::VehicleStatusAdapter adapter(*msg);
            vehicle_status_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromVehicleStatus();
        }
    );

    vehicle_odometry_sub_ = node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        px4_sub_qos,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::vehicle_odometry_sub_: Vehicle odometry received");
            iii_drone::adapters::px4::VehicleOdometryAdapter adapter(*msg);
            vehicle_odometry_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromVehicleOdometry();
        }
    );

    powerline_sub_ = node_->create_subscription<iii_drone_interfaces::msg::Powerline>(
        "/perception/pl_mapper/powerline",
        10,
        [this](const iii_drone_interfaces::msg::Powerline::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::powerline_sub_: Powerline received");
            iii_drone::adapters::PowerlineAdapter adapter(*msg);
            powerline_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromPowerline();
        }
    );

    gripper_status_sub_ = node_->create_subscription<iii_drone_interfaces::msg::GripperStatus>(
        "/payload/charger_gripper/gripper_status",
        10,
        [this](const iii_drone_interfaces::msg::GripperStatus::SharedPtr msg) {
            if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::gripper_status_sub_: Gripper status received");
            iii_drone::adapters::GripperStatusAdapter adapter(*msg);
            gripper_status_adapter_history_->Store(adapter);
            updateCombinedDroneAwarenessFromGripperStatus();
        }
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Creating publish timer");
    publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
            &CombinedDroneAwarenessHandler::publishMembers,
            this
        )
    );

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Start(): Started CombinedDroneAwarenessHandler");

    is_started_ = true;

}

void CombinedDroneAwarenessHandler::Stop() {

    if (!is_started_) {

        RCLCPP_ERROR(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): CombinedDroneAwarenessHandler is already stopped");

        return;

    }

    is_started_ = false;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping CombinedDroneAwarenessHandler");

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping publish timer");
    publish_timer_->cancel();
    publish_timer_.reset();
    publish_timer_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping subscribers");
    vehicle_status_sub_->clear_on_new_message_callback();
    vehicle_status_sub_.reset();
    vehicle_status_sub_ = nullptr;

    vehicle_odometry_sub_->clear_on_new_message_callback();
    vehicle_odometry_sub_.reset();
    vehicle_odometry_sub_ = nullptr;

    powerline_sub_->clear_on_new_message_callback();
    powerline_sub_.reset();
    powerline_sub_ = nullptr;

    gripper_status_sub_->clear_on_new_message_callback();
    gripper_status_sub_.reset();
    gripper_status_sub_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping combined_drone_awareness_pub_timer_");
    combined_drone_awareness_pub_timer_->cancel();
    combined_drone_awareness_pub_timer_.reset();
    combined_drone_awareness_pub_timer_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Resetting combined_drone_awareness_");
    combined_drone_awareness_.reset();
    combined_drone_awareness_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping ground altitude estimate");
    ground_altitude_update_timer_->cancel();
    ground_altitude_update_timer_.reset();
    ground_altitude_update_timer_ = nullptr;

    ground_altitudes_history_->clear();
    ground_altitudes_history_.reset();
    ground_altitudes_history_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopping Register Offboard Mode service");
    register_offboard_mode_srv_->clear_on_new_request_callback();
    register_offboard_mode_srv_.reset();
    register_offboard_mode_srv_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Clearing histories and atomics");
    vehicle_status_adapter_history_->clear();
    vehicle_status_adapter_history_.reset();
    vehicle_status_adapter_history_ = nullptr;

    vehicle_odometry_adapter_history_->clear();
    vehicle_odometry_adapter_history_.reset();
    vehicle_odometry_adapter_history_ = nullptr;

    powerline_adapter_history_->clear();
    powerline_adapter_history_.reset();
    powerline_adapter_history_ = nullptr;

    gripper_status_adapter_history_->clear();
    gripper_status_adapter_history_.reset();
    gripper_status_adapter_history_ = nullptr;

    ground_altitude_estimate_.reset();
    ground_altitude_estimate_ = nullptr;

    target_adapter_.reset();
    target_adapter_ = nullptr;

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Destroying tf_broadcaster_");
    tf_broadcaster_.reset();
    tf_broadcaster_ = nullptr;

    if (debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::Stop(): Stopped CombinedDroneAwarenessHandler");

}

iii_drone::control::State CombinedDroneAwarenessHandler::GetState() const {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::GetState(): Getting state");

    if (vehicle_status_adapter_history_->empty()) {
        return iii_drone::control::State();
    }

    return (*vehicle_odometry_adapter_history_)[0].ToState();

}

iii_drone::control::State CombinedDroneAwarenessHandler::ComputeTargetState(const iii_drone::adapters::TargetAdapter & target_adapter) const {

    transform_matrix_t target_transform = ComputeTargetTransform(target_adapter);

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target adapter id: %d",
    //     target_adapter.target_id()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target adapter type: %d",
    //     target_adapter.target_type()
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target adapter transform:\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f",
    //     target_adapter.target_transform()(0, 0),
    //     target_adapter.target_transform()(0, 1),
    //     target_adapter.target_transform()(0, 2),
    //     target_adapter.target_transform()(0, 3),
    //     target_adapter.target_transform()(1, 0),
    //     target_adapter.target_transform()(1, 1),
    //     target_adapter.target_transform()(1, 2),
    //     target_adapter.target_transform()(1, 3),
    //     target_adapter.target_transform()(2, 0),
    //     target_adapter.target_transform()(2, 1),
    //     target_adapter.target_transform()(2, 2),
    //     target_adapter.target_transform()(2, 3),
    //     target_adapter.target_transform()(3, 0),
    //     target_adapter.target_transform()(3, 1),
    //     target_adapter.target_transform()(3, 2),
    //     target_adapter.target_transform()(3, 3)
    // );

    // RCLCPP_DEBUG(
    //     node_->get_logger(),
    //     "Target transform:\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f",
    //     target_transform(0, 0),
    //     target_transform(0, 1),
    //     target_transform(0, 2),
    //     target_transform(0, 3),
    //     target_transform(1, 0),
    //     target_transform(1, 1),
    //     target_transform(1, 2),
    //     target_transform(1, 3),
    //     target_transform(2, 0),
    //     target_transform(2, 1),
    //     target_transform(2, 2),
    //     target_transform(2, 3),
    //     target_transform(3, 0),
    //     target_transform(3, 1),
    //     target_transform(3, 2),
    //     target_transform(3, 3)
    // );

    return State(
        target_transform.block<3, 1>(0, 3),
        vector_t::Zero(),
        matToQuat(target_transform.block<3, 3>(0, 0)),
        vector_t::Zero()
    );

}

iii_drone::types::transform_matrix_t CombinedDroneAwarenessHandler::ComputeTargetTransform(const iii_drone::adapters::TargetAdapter & target_adapter) const {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ComputeTargetTransform(): Computing target transform");

    iii_drone::control::Reference reference;

    if (target_adapter.target_type() == iii_drone::adapters::TARGET_TYPE_CABLE) {

        iii_drone::adapters::PowerlineAdapter powerline_adapter = (*powerline_adapter_history_)[0];

        iii_drone::adapters::SingleLineAdapter target_line;

        try {

            target_line = powerline_adapter.GetLine(target_adapter.target_id());

        } catch (std::runtime_error & e) {

            if (debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ComputeTargetTransform: target line with id %d not found", target_adapter.target_id());

            throw std::runtime_error("CombinedDroneAwarenessHandler::ComputeTargetTransform(): target line with id " + std::to_string(target_adapter.target_id()) + " not found");
        }

        geometry_msgs::msg::PoseStamped target_object_pose_stamped = target_line.ToPoseStampedMsg();

        geometry_msgs::msg::PoseStamped target_object_pose_stamped_world;

        rclcpp::Time now = rclcpp::Clock().now();

        try {

            target_object_pose_stamped_world = tf_buffer_->transform(
                target_object_pose_stamped,
                params_->GetParameter("world_frame_id").as_string(),
                tf2::durationFromSec(1.)
            );

        } catch (tf2::TransformException & e) {

            if (debug_) RCLCPP_DEBUG(
                node_->get_logger(), 
                "CombinedDroneAwarenessHandler::ComputeTargetTransform: could not transform target object pose to world frame after %f seconds: %s", 
                (now - rclcpp::Clock().now()).seconds(),
                e.what()
            );

            throw std::runtime_error("CombinedDroneAwarenessHandler::ComputeTargetTransform(): could not transform target object pose to world frame: " + std::string(e.what()));

        }

        // RCLCPP_DEBUG(
        //     node_->get_logger(),
        //     "CombinedDroneAwarenessHandler::ComputeTargetTransform: target object pose in world frame: %f, %f, %f, %f, %f, %f, %f",
        //     target_object_pose_stamped_world.pose.position.x,
        //     target_object_pose_stamped_world.pose.position.y,
        //     target_object_pose_stamped_world.pose.position.z,
        //     target_object_pose_stamped_world.pose.orientation.w,
        //     target_object_pose_stamped_world.pose.orientation.x,
        //     target_object_pose_stamped_world.pose.orientation.y,
        //     target_object_pose_stamped_world.pose.orientation.z
        // );

        iii_drone::types::transform_matrix_t ref_T_c = target_adapter.target_transform();
        iii_drone::types::pose_t w_T_c_pose = poseFromPoseMsg(target_object_pose_stamped_world.pose);
        // RCLCPP_DEBUG()
        iii_drone::types::transform_matrix_t w_T_c = createTransformMatrix(
            w_T_c_pose.position,
            w_T_c_pose.orientation
        );
        iii_drone::types::transform_matrix_t w_T_ref = w_T_c * ref_T_c.inverse();
        std::string target_reference_frame_id = target_adapter.reference_frame_id();
        geometry_msgs::msg::TransformStamped ref_T_drone_msg = tf_buffer_->lookupTransform(
            target_reference_frame_id,
            params_->GetParameter("drone_frame_id").as_string(),
            tf2::TimePointZero
        );
        iii_drone::types::transform_matrix_t ref_T_drone = transformMatrixFromTransformMsg(ref_T_drone_msg.transform);
        iii_drone::types::transform_matrix_t w_T_d = w_T_ref * ref_T_drone;

        return w_T_d;

    } else {

        std::string msg = "CombinedDroneAwarenessHandler::ComputeTargetTransform(): target type " + std::to_string(target_adapter.target_type()) + " not implemented";

        throw std::runtime_error(msg);

    }
}

iii_drone::types::pose_t CombinedDroneAwarenessHandler::GetPoseOfTarget(const iii_drone::adapters::TargetAdapter & target_adapter) const {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::GetPoseOfTarget(): Getting the pose of the target");

    iii_drone::types::pose_t pose;

    if (target_adapter.target_type() == iii_drone::adapters::TARGET_TYPE_CABLE) {

        iii_drone::adapters::PowerlineAdapter powerline_adapter = (*powerline_adapter_history_)[0];

        iii_drone::adapters::SingleLineAdapter target_line;

        try {

            target_line = powerline_adapter.GetLine(target_adapter.target_id());

        } catch (std::runtime_error & e) {

            throw std::runtime_error("CombinedDroneAwarenessHandler::GetPoseOfTarget(): target line with id " + std::to_string(target_adapter.target_id()) + " not found");
        }

        geometry_msgs::msg::PoseStamped target_object_pose_stamped = target_line.ToPoseStampedMsg();

        geometry_msgs::msg::PoseStamped target_object_pose_stamped_world = tf_buffer_->transform(
            target_object_pose_stamped,
            params_->GetParameter("world_frame_id").as_string()
        );

        pose = poseFromPoseMsg(target_object_pose_stamped_world.pose);

    } else {

        throw std::runtime_error("CombinedDroneAwarenessHandler::GetPoseOfTarget(): target type not implemented");

    }

    return pose;

}

void CombinedDroneAwarenessHandler::SetTarget(iii_drone::adapters::TargetAdapter target_adapter) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::SetTarget(): Setting target");
    
    if (target_adapter.target_type() != iii_drone::adapters::TARGET_TYPE_CABLE) {
        
        throw std::runtime_error("CombinedDroneAwarenessHandler::SetTarget: target type not implemented");

    }

    target_adapter_->Store(target_adapter);

    updateCombinedDroneAwarenessFromTarget();

}

void CombinedDroneAwarenessHandler::ClearTarget() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::ClearTarget(): Clearing target");

    target_adapter_->Store(TargetAdapter());

    updateCombinedDroneAwarenessFromTarget();

}

const combined_drone_awareness_t CombinedDroneAwarenessHandler::combined_drone_awareness() const {
    return *combined_drone_awareness_;
}

bool CombinedDroneAwarenessHandler::armed() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).armed;
}

bool CombinedDroneAwarenessHandler::offboard() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).offboard;
}

bool CombinedDroneAwarenessHandler::has_target() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).has_target();
}

bool CombinedDroneAwarenessHandler::target_position_known() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).target_position_known;
}

TargetAdapter CombinedDroneAwarenessHandler::target_adapter() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).target_adapter;
}

bool CombinedDroneAwarenessHandler::on_ground() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).on_ground();
}

bool CombinedDroneAwarenessHandler::on_cable() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).on_cable();
}

bool CombinedDroneAwarenessHandler::in_flight() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).in_flight();
}

int CombinedDroneAwarenessHandler::on_cable_id() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).on_cable_id;
}

double CombinedDroneAwarenessHandler::ground_altitude_estimate() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).ground_altitude_estimate;
}

drone_location_t CombinedDroneAwarenessHandler::drone_location(int &on_cable_id) const {
    on_cable_id = ((combined_drone_awareness_t)*combined_drone_awareness_).on_cable_id;
    return ((combined_drone_awareness_t)*combined_drone_awareness_).drone_location;
}

bool CombinedDroneAwarenessHandler::gripper_open() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).gripper_open;
}

drone_location_t CombinedDroneAwarenessHandler::drone_location() const {
    return ((combined_drone_awareness_t)*combined_drone_awareness_).drone_location;
}

tf2_ros::Buffer::SharedPtr CombinedDroneAwarenessHandler::tf_buffer() const {
    return tf_buffer_;
}

void CombinedDroneAwarenessHandler::registerOffboardMode(int navigation_state_id) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::RegisterOffboardMode(): Registering offboard mode with id %d", navigation_state_id);

    std::vector<int> offboard_states = offboard_nav_state_ids_.Load();

    if (std::find(offboard_states.begin(), offboard_states.end(), navigation_state_id) == offboard_states.end()) {
        offboard_states.push_back(navigation_state_id);
        offboard_nav_state_ids_.Store(offboard_states);
    }

}

void CombinedDroneAwarenessHandler::deregisterOffboardMode(int navigation_state_id) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::DeregisterOffboardMode(): Deregistering offboard mode with id %d", navigation_state_id);

    std::vector<int> offboard_states = offboard_nav_state_ids_.Load();

    offboard_states.erase(std::remove(offboard_states.begin(), offboard_states.end(), navigation_state_id), offboard_states.end());

    offboard_nav_state_ids_.Store(offboard_states);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwareness() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwareness(): Updating combined drone awareness");

    // Update the combined drone awareness
    combined_drone_awareness_t combined_drone_awareness = *combined_drone_awareness_;

    updateCombinedDroneAwarenessFromVehicleStatus(combined_drone_awareness);
    updateCombinedDroneAwarenessFromVehicleOdometry(combined_drone_awareness);
    updateCombinedDroneAwarenessFromPowerline(combined_drone_awareness);
    updateCombinedDroneAwarenessFromGripperStatus(combined_drone_awareness);
    updateCombinedDroneAwarenessFromTarget(combined_drone_awareness);

    // Update the combined drone awareness
    *combined_drone_awareness_ = combined_drone_awareness;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleStatus() {

    combined_drone_awareness_t combined_drone_awareness = *combined_drone_awareness_;

    updateCombinedDroneAwarenessFromVehicleStatus(combined_drone_awareness);

    *combined_drone_awareness_ = combined_drone_awareness;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleStatus(combined_drone_awareness_t & combined_drone_awareness) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleStatus(): Updating combined drone awareness from vehicle status");

    // Update the armed and offboard states
    if (vehicle_status_adapter_history_->empty()) {
        combined_drone_awareness.armed = false;
        combined_drone_awareness.offboard = false;
    } else {
        combined_drone_awareness.armed = (*vehicle_status_adapter_history_)[0].arming_state() == iii_drone::adapters::px4::ARMING_STATE_ARMED;

        int navigation_state_id = (*vehicle_status_adapter_history_)[0].nav_state();

        std::vector<int> offboard_states = offboard_nav_state_ids_.Load();

        combined_drone_awareness.offboard = (std::find(offboard_states.begin(), offboard_states.end(), navigation_state_id) != offboard_states.end());

    }

    // Update the drone location:
    updateDroneLocation(combined_drone_awareness);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleOdometry() {

    combined_drone_awareness_t combined_drone_awareness = *combined_drone_awareness_;

    updateCombinedDroneAwarenessFromVehicleOdometry(combined_drone_awareness);

    *combined_drone_awareness_ = combined_drone_awareness;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleOdometry(combined_drone_awareness_t & combined_drone_awareness) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromVehicleOdometry(): Updating combined drone awareness from vehicle odometry");

    // Update the ground altitude estimate
    updateGroundAltitudeEstimate(
        combined_drone_awareness.armed,
        combined_drone_awareness.offboard,
        combined_drone_awareness.has_target()
    );

    combined_drone_awareness.ground_altitude_estimate = ground_altitude_estimate_->Load();

    // Update the drone location
    updateDroneLocation(combined_drone_awareness);

    // Update state
    combined_drone_awareness.state = GetState();

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromPowerline() {

    combined_drone_awareness_t combined_drone_awareness = *combined_drone_awareness_;

    updateCombinedDroneAwarenessFromPowerline(combined_drone_awareness);

    *combined_drone_awareness_ = combined_drone_awareness;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromPowerline(combined_drone_awareness_t & combined_drone_awareness) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromPowerline(): Updating combined drone awareness from powerline");

    // Update the target cable position known
    if (powerline_adapter_history_->empty() || !combined_drone_awareness.has_target()) {
        return;
    } else if (combined_drone_awareness.target_adapter.target_type() == TARGET_TYPE_CABLE) {
        int target_cable_id = combined_drone_awareness.target_adapter.target_id();
        combined_drone_awareness.target_position_known = (*powerline_adapter_history_)[0].HasLine(target_cable_id);
    }

    // Update the drone location
    updateDroneLocation(combined_drone_awareness);

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromGripperStatus() {

    combined_drone_awareness_t combined_drone_awareness = *combined_drone_awareness_;

    updateCombinedDroneAwarenessFromGripperStatus(combined_drone_awareness);

    *combined_drone_awareness_ = combined_drone_awareness;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromGripperStatus(combined_drone_awareness_t & combined_drone_awareness) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromGripperStatus(): Updating combined drone awareness from gripper status");

    // Update the gripper open flag
    if (gripper_status_adapter_history_->empty()) {
        combined_drone_awareness.gripper_open = true;
    } else {
        combined_drone_awareness.gripper_open = (*gripper_status_adapter_history_)[0].open();
    }

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromTarget() {

    combined_drone_awareness_t combined_drone_awareness = *combined_drone_awareness_;

    updateCombinedDroneAwarenessFromTarget(combined_drone_awareness);

    *combined_drone_awareness_ = combined_drone_awareness;

}

void CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromTarget(combined_drone_awareness_t & combined_drone_awareness) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateCombinedDroneAwarenessFromTarget(): Updating combined drone awareness from target");

    // Update the target cable id
    combined_drone_awareness.target_adapter = target_adapter_->Load();

    // Update the target cable position known
    if (combined_drone_awareness.target_adapter.target_type() == TARGET_TYPE_CABLE) {
        if (powerline_adapter_history_->empty()) {
            combined_drone_awareness.target_position_known = false;
        } else {
            int target_cable_id = combined_drone_awareness.target_adapter.target_id();
            combined_drone_awareness.target_position_known = (*powerline_adapter_history_)[0].HasLine(target_cable_id);
        }
    } else {
        combined_drone_awareness.target_position_known = false;
    }

}

void CombinedDroneAwarenessHandler::updateGroundAltitudeEstimate(
    bool armed,
    bool offboard,
    bool has_target_cable
) {

    if (!has_found_initial_location_) {
        return;
    }

    bool is_on_ground = !armed && !has_target_cable;

    if (!is_on_ground) {
        ground_altitude_update_timer_->cancel();
        return;
    }

    if (vehicle_odometry_adapter_history_->empty()) {
        return;
    }

    if (!ground_altitude_update_timer_->is_canceled()) {
        return;
    }

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateGroundAltitudeEstimate(): Updating ground altitude estimate");

    double ground_altitude = (*vehicle_odometry_adapter_history_)[0].position()[2];

    ground_altitudes_history_->Store(ground_altitude);

    std::vector<double> ground_altitudes = ground_altitudes_history_->vector();

    double ground_altitude_estimate = std::accumulate(ground_altitudes.begin(), ground_altitudes.end(), 0.0) / ground_altitudes.size();

    ground_altitude_estimate_->Store(ground_altitude_estimate);

    ground_altitude_update_timer_->reset();

    geometry_msgs::msg::TransformStamped ground_tf;

    ground_tf.header.stamp = node_->now();
    ground_tf.header.frame_id = params_->GetParameter("world_frame_id").as_string();
    ground_tf.child_frame_id = params_->GetParameter("ground_frame_id").as_string();

    ground_tf.transform = transformMsgFromTransform(
        vector_t(0, 0, ground_altitude_estimate),
        quaternion_t(1, 0, 0, 0)
    );

    tf_broadcaster_->sendTransform(ground_tf);

}

void CombinedDroneAwarenessHandler::updateDroneLocation(combined_drone_awareness_t & combined_drone_awareness) {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::updateDroneLocation(): Updating drone location");

    if (vehicle_odometry_adapter_history_->empty() || vehicle_status_adapter_history_->empty()) {
        combined_drone_awareness.drone_location = DRONE_LOCATION_UNKNOWN;
        combined_drone_awareness.on_cable_id = -1;
        return;
    }

    iii_drone::adapters::px4::VehicleOdometryAdapter vehicle_odometry_adapter = (*vehicle_odometry_adapter_history_)[0];

    point_t drone_position = vehicle_odometry_adapter.position();

    // Check if on ground:
    if (drone_position[2] - ground_altitude_estimate_->Load() < params_->GetParameter("landed_altitude_threshold").as_double()) {
        combined_drone_awareness.drone_location = DRONE_LOCATION_ON_GROUND;
        combined_drone_awareness.on_cable_id = -1;
        has_found_initial_location_ = true;
        return;
    }

    // Check if on cable:
    if (!powerline_adapter_history_->empty()) {

        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
            params_->GetParameter("drone_frame_id").as_string(),
            params_->GetParameter("cable_gripper_frame_id").as_string(),
            tf2::TimePointZero
        );

        vector_t v_drone_to_gripper = vectorFromTransformMsg(transform_stamped.transform);

        point_t gripper_position = drone_position + v_drone_to_gripper;

        iii_drone::adapters::PowerlineAdapter powerline_adapter = (*powerline_adapter_history_)[0];

        iii_drone::adapters::SingleLineAdapter closest_line;

        bool found_closest_line = false;

        try {
            closest_line = powerline_adapter.GetClosestLine(gripper_position);
            found_closest_line = true;
        } catch (std::exception & e) {
            found_closest_line = false;
        }

        if (params_->GetParameter("use_gripper_status_condition").as_bool() && !gripper_open()) {
            combined_drone_awareness.drone_location = DRONE_LOCATION_ON_CABLE;
            combined_drone_awareness.on_cable_id = -1;
            has_found_initial_location_ = true;
        }

        if (found_closest_line) {

            point_t closest_line_position = closest_line.position();

            geometry_msgs::msg::PointStamped closest_line_position_stamped;
            closest_line_position_stamped.header.frame_id = closest_line.frame_id();
            closest_line_position_stamped.point = pointMsgFromPoint(closest_line_position);

            closest_line_position_stamped = tf_buffer_->transform(
                closest_line_position_stamped,
                params_->GetParameter("drone_frame_id").as_string()
            );

            closest_line_position = pointFromPointMsg(closest_line_position_stamped.point);

            float closest_line_distance = (closest_line_position - v_drone_to_gripper).norm();

            if (closest_line_distance <= params_->GetParameter("on_cable_max_euc_distance").as_double()) {
                
                combined_drone_awareness.drone_location = DRONE_LOCATION_ON_CABLE;
                combined_drone_awareness.on_cable_id = closest_line.id();

                has_found_initial_location_ = true;

                return;

            }
        }
    }

    // Check if in flight:
    if (combined_drone_awareness.armed) {
        combined_drone_awareness.drone_location = DRONE_LOCATION_IN_FLIGHT;
        has_found_initial_location_ = true;
        return;

    } else if (!has_found_initial_location_) {
        // Drone is unarmed and initial location has not been found and drone is not on cable,
        // assume drone is on ground

        combined_drone_awareness.drone_location = DRONE_LOCATION_ON_GROUND;
        combined_drone_awareness.on_cable_id = -1;
        has_found_initial_location_ = true;
        return;

    }

    // Assume drone is on ground:
    combined_drone_awareness.drone_location = DRONE_LOCATION_ON_GROUND;
    combined_drone_awareness.on_cable_id = -1;
    has_found_initial_location_ = false;

    // // Throw error if no location found
    // std::string error_message = "CombinedDroneAwarenessHandler::updateDroneLocation(): Could not determine drone location.";

    // if (params_->GetParameter("fail_on_unable_to_locate").as_bool()) {
    //     RCLCPP_FATAL(node_->get_logger(), error_message.c_str());
    //     throw std::runtime_error(error_message);
    // } else {
    //     RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
    //     combined_drone_awareness.drone_location = DRONE_LOCATION_UNKNOWN;
    //     combined_drone_awareness.on_cable_id = -1;
    // }

}

void CombinedDroneAwarenessHandler::publishMembers() {

    if(debug_) RCLCPP_DEBUG(node_->get_logger(), "CombinedDroneAwarenessHandler::publishMembers(): Publishing members");

    iii_drone::adapters::TargetAdapter target_adapter = target_adapter_->Load();

    if (target_adapter.target_type() != iii_drone::adapters::TARGET_TYPE_CABLE) {
        return;
    }

    iii_drone_interfaces::msg::Target target_msg = target_adapter.ToMsg();

    target_pub_->publish(target_msg);

    try {
        pose_t target_pose = GetPoseOfTarget(target_adapter);

        geometry_msgs::msg::PoseStamped target_pose_stamped;
        target_pose_stamped.header.stamp = node_->now();
        target_pose_stamped.header.frame_id = params_->GetParameter("world_frame_id").as_string();
        target_pose_stamped.pose = poseMsgFromPose(target_pose);

        target_pose_pub_->publish(target_pose_stamped);

    } catch (std::runtime_error & e) {
        
    }

    try {
        transform_matrix_t target_world_to_drone = ComputeTargetTransform(target_adapter);

        pose_t target_pose_world_to_drone = poseFromTransformMatrix(target_world_to_drone);

        geometry_msgs::msg::PoseStamped target_pose_world_to_drone_stamped;
        target_pose_world_to_drone_stamped.header.stamp = node_->now();
        target_pose_world_to_drone_stamped.header.frame_id = params_->GetParameter("world_frame_id").as_string();
        target_pose_world_to_drone_stamped.pose = poseMsgFromPose(target_pose_world_to_drone);

        target_drone_pose_pub_->publish(target_pose_world_to_drone_stamped);

    } catch (std::runtime_error & e) {
        
    }

}