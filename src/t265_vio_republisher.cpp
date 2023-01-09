// Include ros2 and px4 packages
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/timesync.hpp>

#include <nav_msgs/msg/odometry.hpp>

// Includes for ros2  tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "geometry.h"

class T265VIORepublisher : public rclcpp::Node
{
public:
    // Constructor
    T265VIORepublisher() : Node("t265_vio_republisher")
    {
        this->declare_parameter<std::string>("t265_world_frame_id", "odom_frame");
        this->declare_parameter<std::string>("t265_camera_frame_id", "camera_pose_frame");
        this->declare_parameter<std::string>("drone_frame_id", "drone");
        this->declare_parameter<std::string>("world_frame_id", "world");

        std::string drone_frame_id;
        std::string camera_frame_id;
        this->get_parameter("drone_frame_id", drone_frame_id);
        this->get_parameter("t265_camera_frame_id", camera_frame_id);

        // tf
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // ROS2 sleep with rate 1 second
        rclcpp::Rate loop_rate(1);
        while (rclcpp::ok()) {
            // Check if transform is available
            try {
                tf_buffer_->lookupTransform(drone_frame_id, camera_frame_id, tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "Transforms are ready");
                break;
            }
            catch (tf2::LookupException &ex) {
                RCLCPP_INFO(this->get_logger(), "Waiting for transform");
                loop_rate.sleep();
            }
        }

        // Create a publisher
        pub_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry_out", 10);

        // get common timestamp
        timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
            "/fmu/timesync/out",
            10,
            [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                timestamp_.store(msg->timestamp);
            }
        );

        // Create a subscriber
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/camera/pose/sample", 
            rclcpp::QoS(10).best_effort(),
            std::bind(&T265VIORepublisher::callback, this, std::placeholders::_1)
        );
    }

private:
    // Callback function
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Callback function called");

        // // Get t265_camera_frame_id parameter
        // std::string t265_camera_frame_id;
        // this->get_parameter("t265_camera_frame_id", t265_camera_frame_id);

        // // Get drone_frame_id parameter
        // std::string drone_frame_id;
        // this->get_parameter("drone_frame_id", drone_frame_id);

        // // Get world_frame_id parameter
        // std::string world_frame_id;
        // this->get_parameter("world_frame_id", world_frame_id);

        // // Get t265_world_frame_id parameter
        // std::string t265_world_frame_id;
        // this->get_parameter("t265_world_frame_id", t265_world_frame_id);

        // // Get transform from t265_camera_frame_id to drone_frame_id
        // geometry_msgs::msg::TransformStamped T_t265_to_drone;
        // try
        // {
        //     T_t265_to_drone = tf_buffer_->lookupTransform(t265_camera_frame_id, drone_frame_id, tf2::TimePointZero);
        // }
        // catch (std::exception &ex)
        // // catch (tf2::LookupException &ex)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Could not get transform from %s to %s: %s", drone_frame_id.c_str(), t265_camera_frame_id.c_str(), ex.what());
        //     return;
        // }

        // nav_msgs::msg::Odometry odom_t265_world_to_t265_camera = *msg;

        // nav_msgs::msg::Odometry odom_t265_world_to_drone = odom_t265_world_to_t265_camera;

        // // Get transform world_frame_id to t265_world_frame_id
        // geometry_msgs::msg::TransformStamped T_world_to_t265_world;
        // try
        // {
        //     T_world_to_t265_world = tf_buffer_->lookupTransform(world_frame_id, t265_world_frame_id, rclcpp::Time(0));
        // }
        // catch (tf2::LookupException &ex)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Could not get transform from %s to %s: %s", t265_world_frame_id.c_str(), world_frame_id.c_str(), ex.what());
        //     return;
        // }

        // odom_t265_world_to_drone.child_frame_id = drone_frame_id;

        // odom_t265_world_to_drone.pose.pose.position.x = odom_t265_world_to_t265_camera.pose.pose.position.x + T_t265_to_drone.transform.translation.x;
        // odom_t265_world_to_drone.pose.pose.position.y = odom_t265_world_to_t265_camera.pose.pose.position.y + T_t265_to_drone.transform.translation.y;
        // odom_t265_world_to_drone.pose.pose.position.z = odom_t265_world_to_t265_camera.pose.pose.position.z + T_t265_to_drone.transform.translation.z;

        // quat_t q_t265_world_to_t265_camera(
        //     odom_t265_world_to_t265_camera.pose.pose.orientation.w,
        //     odom_t265_world_to_t265_camera.pose.pose.orientation.x,
        //     odom_t265_world_to_t265_camera.pose.pose.orientation.y,
        //     odom_t265_world_to_t265_camera.pose.pose.orientation.z
        // );

        // quat_t q_t265_camera_to_drone(
        //     T_t265_to_drone.transform.rotation.w,
        //     T_t265_to_drone.transform.rotation.x,
        //     T_t265_to_drone.transform.rotation.y,
        //     T_t265_to_drone.transform.rotation.z
        // );

        // quat_t q_t265_world_to_drone = quatMultiply(q_t265_world_to_t265_camera, q_t265_camera_to_drone);

        // // odom_t265_world_to_drone.pose.pose.orientation.w = odom_t265_world_to_t265_camera.pose.pose.orientation.w * T_t265_to_drone.transform.rotation.w - odom_t265_world_to_t265_camera.pose.pose.orientation.x * T_t265_to_drone.transform.rotation.x - odom_t265_world_to_t265_camera.pose.pose.orientation.y * T_t265_to_drone.transform.rotation.y - odom_t265_world_to_t265_camera.pose.pose.orientation.z * T_t265_to_drone.transform.rotation.z;
        // // odom_t265_world_to_drone.pose.pose.orientation.x = odom_t265_world_to_t265_camera.pose.pose.orientation.w * T_t265_to_drone.transform.rotation.x + odom_t265_world_to_t265_camera.pose.pose.orientation.x * T_t265_to_drone.transform.rotation.w + odom_t265_world_to_t265_camera.pose.pose.orientation.y * T_t265_to_drone.transform.rotation.z - odom_t265_world_to_t265_camera.pose.pose.orientation.z * T_t265_to_drone.transform.rotation.y;
        // // odom_t265_world_to_drone.pose.pose.orientation.y = odom_t265_world_to_t265_camera.pose.pose.orientation.w * T_t265_to_drone.transform.rotation.y - odom_t265_world_to_t265_camera.pose.pose.orientation.x * T_t265_to_drone.transform.rotation.z + odom_t265_world_to_t265_camera.pose.pose.orientation.y * T_t265_to_drone.transform.rotation.w + odom_t265_world_to_t265_camera.pose.pose.orientation.z * T_t265_to_drone.transform.rotation.x;
        // // odom_t265_world_to_drone.pose.pose.orientation.z = odom_t265_world_to_t265_camera.pose.pose.orientation.w * T_t265_to_drone.transform.rotation.z + odom_t265_world_to_t265_camera.pose.pose.orientation.x * T_t265_to_drone.transform.rotation.y - odom_t265_world_to_t265_camera.pose.pose.orientation.y * T_t265_to_drone.transform.rotation.x + odom_t265_world_to_t265_camera.pose.pose.orientation.z * T_t265_to_drone.transform.rotation.w;

        // odom_t265_world_to_drone.pose.pose.orientation.w = q_t265_world_to_drone(0);
        // odom_t265_world_to_drone.pose.pose.orientation.x = q_t265_world_to_drone(1);
        // odom_t265_world_to_drone.pose.pose.orientation.y = q_t265_world_to_drone(2);
        // odom_t265_world_to_drone.pose.pose.orientation.z = q_t265_world_to_drone(3);

        // // Transform odom_t265_world_to_drone twist to drone frame
        // vector_t odom_t265_world_to_drone_twist_linear(
        //     odom_t265_world_to_drone.twist.twist.linear.x,
        //     odom_t265_world_to_drone.twist.twist.linear.y,
        //     odom_t265_world_to_drone.twist.twist.linear.z
        // );

        // vector_t odom_t265_world_to_drone_twist_angular(
        //     odom_t265_world_to_drone.twist.twist.angular.x,
        //     odom_t265_world_to_drone.twist.twist.angular.y,
        //     odom_t265_world_to_drone.twist.twist.angular.z
        // );

        // quat_t quat_t265_to_drone(
        //     T_t265_to_drone.transform.rotation.w,
        //     T_t265_to_drone.transform.rotation.x,
        //     T_t265_to_drone.transform.rotation.y,
        //     T_t265_to_drone.transform.rotation.z
        // );

        // rotation_matrix_t R_t265_to_drone = quatToMat(quat_t265_to_drone);

        // rotation_matrix_t R_drone_to_t265 = R_t265_to_drone.transpose();

        // odom_t265_world_to_drone_twist_linear = R_drone_to_t265 * odom_t265_world_to_drone_twist_linear;

        // odom_t265_world_to_drone_twist_angular = R_drone_to_t265 * odom_t265_world_to_drone_twist_angular;


        // odom_t265_world_to_drone.twist.twist.linear.x = odom_t265_world_to_drone_twist_linear(0);
        // odom_t265_world_to_drone.twist.twist.linear.y = odom_t265_world_to_drone_twist_linear(1);
        // odom_t265_world_to_drone.twist.twist.linear.z = odom_t265_world_to_drone_twist_linear(2);

        // odom_t265_world_to_drone.twist.twist.angular.x = odom_t265_world_to_drone_twist_angular(0);
        // odom_t265_world_to_drone.twist.twist.angular.y = odom_t265_world_to_drone_twist_angular(1);
        // odom_t265_world_to_drone.twist.twist.angular.z = odom_t265_world_to_drone_twist_angular(2);

        // nav_msgs::msg::Odometry odom_world_to_drone = odom_t265_world_to_drone;

        // odom_world_to_drone.header.frame_id = world_frame_id;

        // // Transform odom_world_to_drone pose to world frame
        // odom_world_to_drone.pose.pose.position.x = odom_world_to_drone.pose.pose.position.x + T_world_to_t265_world.transform.translation.x;
        // odom_world_to_drone.pose.pose.position.y = odom_world_to_drone.pose.pose.position.y + T_world_to_t265_world.transform.translation.y;
        // odom_world_to_drone.pose.pose.position.z = odom_world_to_drone.pose.pose.position.z + T_world_to_t265_world.transform.translation.z;

        // // odom_world_to_drone.pose.pose.orientation.w = odom_world_to_drone.pose.pose.orientation.w * T_world_to_t265_world.transform.rotation.w - odom_world_to_drone.pose.pose.orientation.x * T_world_to_t265_world.transform.rotation.x - odom_world_to_drone.pose.pose.orientation.y * T_world_to_t265_world.transform.rotation.y - odom_world_to_drone.pose.pose.orientation.z * T_world_to_t265_world.transform.rotation.z;
        // // odom_world_to_drone.pose.pose.orientation.x = odom_world_to_drone.pose.pose.orientation.w * T_world_to_t265_world.transform.rotation.x + odom_world_to_drone.pose.pose.orientation.x * T_world_to_t265_world.transform.rotation.w + odom_world_to_drone.pose.pose.orientation.y * T_world_to_t265_world.transform.rotation.z - odom_world_to_drone.pose.pose.orientation.z * T_world_to_t265_world.transform.rotation.y;
        // // odom_world_to_drone.pose.pose.orientation.y = odom_world_to_drone.pose.pose.orientation.w * T_world_to_t265_world.transform.rotation.y - odom_world_to_drone.pose.pose.orientation.x * T_world_to_t265_world.transform.rotation.z + odom_world_to_drone.pose.pose.orientation.y * T_world_to_t265_world.transform.rotation.w + odom_world_to_drone.pose.pose.orientation.z * T_world_to_t265_world.transform.rotation.x;
        // // odom_world_to_drone.pose.pose.orientation.z = odom_world_to_drone.pose.pose.orientation.w * T_world_to_t265_world.transform.rotation.z + odom_world_to_drone.pose.pose.orientation.x * T_world_to_t265_world.transform.rotation.y - odom_world_to_drone.pose.pose.orientation.y * T_world_to_t265_world.transform.rotation.x + odom_world_to_drone.pose.pose.orientation.z * T_world_to_t265_world.transform.rotation.w;

        // quat_t q_world_to_t265_world(
        //     T_world_to_t265_world.transform.rotation.w,
        //     T_world_to_t265_world.transform.rotation.x,
        //     T_world_to_t265_world.transform.rotation.y,
        //     T_world_to_t265_world.transform.rotation.z
        // );

        // quat_t q_world_to_drone = quatMultiply(q_world_to_t265_world, q_t265_world_to_drone);

        // odom_world_to_drone.pose.pose.orientation = msg->pose.pose.orientation;

        // odom_pub_->publish(odom_world_to_drone);

        // Create a message
        nav_msgs::msg::Odometry odom = *msg;

        // orientation_t eul_NWU_to_NED(-M_PI,0,0);
        // rotation_matrix_t R_NWU_to_NED = eulToR(eul_NWU_to_NED);
        // quat_t q_NWU_to_NED = eulToQuat(eul_NWU_to_NED);

        // vector_t pos_NWU(
        //     odom.pose.pose.position.x,
        //     odom.pose.pose.position.y,
        //     odom.pose.pose.position.z
        // );

        // vector_t pos_NED = R_NWU_to_NED * pos_NWU;

        // quat_t q_NWU(
        //     odom.pose.pose.orientation.w,
        //     odom.pose.pose.orientation.x,
        //     odom.pose.pose.orientation.y,
        //     odom.pose.pose.orientation.z
        // );

        // quat_t q_NED = quatMultiply(q_NWU_to_NED, q_NWU);

        rotation_matrix_t R_fmu_to_T265 = eulToR(orientation_t(0,M_PI,0));

        vector_t pos_FMU(
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z
        );

        pos_FMU = R_fmu_to_T265 * pos_FMU;

        quat_t q_FMU(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z
        );

        q_FMU = matToQuat(R_fmu_to_T265) * q_FMU;

        vector_t vel_FMU(
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z
        );

        vel_FMU = R_fmu_to_T265 * vel_FMU;

        orientation_t ang_rate_FMU(
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z
        );

        ang_rate_FMU = quatToEul(matToQuat(R_fmu_to_T265 * eulToR(ang_rate_FMU)));

        // Fill the message

        px4_msgs::msg::VehicleVisualOdometry msg_out = px4_msgs::msg::VehicleVisualOdometry();

        msg_out.timestamp = timestamp_.load();
        msg_out.local_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;

        msg_out.x = pos_FMU(0);
        msg_out.y = pos_FMU(1);
        msg_out.z = pos_FMU(2);

        msg_out.q[0] = q_FMU(0);
        msg_out.q[1] = q_FMU(1);
        msg_out.q[2] = q_FMU(2);
        msg_out.q[3] = q_FMU(3);

        for (int i = 0; i < 21; i++) {
            msg_out.pose_covariance[i] = NAN;
        }

        msg_out.vx = vel_FMU(0);
        msg_out.vy = vel_FMU(1);
        msg_out.vz = vel_FMU(2);

        msg_out.rollspeed = ang_rate_FMU(0);
        msg_out.pitchspeed = ang_rate_FMU(1);
        msg_out.yawspeed = ang_rate_FMU(2);
        
        for (int i = 0; i < 21; i++) {
            msg_out.velocity_covariance[i] = NAN;
        }

        // Publish the message
        pub_->publish(msg_out);

    }

    // Publisher
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

    // ROS2 tf members
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<T265VIORepublisher>());
    rclcpp::shutdown();
    return 0;
}