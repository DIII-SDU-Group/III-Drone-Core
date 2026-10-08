#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <iii_drone_core/perception/single_line.hpp>
#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

using iii_drone::types::point_t;

// pl_mapper moves radar points into the drone frame with the static mount it
// caches at activation (a tf2::Transform) instead of a TF transform per point,
// and quatToMat must be the same rotation as tf2: all three must agree.
TEST(RadarExtrinsic, CachedMountMatchesTfPerPoint) {
    auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped mount;
    mount.header.frame_id = "drone";
    mount.child_frame_id = "mmwave";
    mount.transform.translation.x = 0.12;
    mount.transform.translation.y = -0.03;
    mount.transform.translation.z = 0.08;
    // A mount rotated about two axes (not just the identity).
    tf2::Quaternion rotation;
    rotation.setRPY(0.3, -1.2, 0.7);
    mount.transform.rotation = tf2::toMsg(rotation);
    ASSERT_TRUE(buffer.setTransform(mount, "test", true));

    const auto lookup = buffer.lookupTransform("drone", "mmwave", tf2::TimePointZero);
    const auto R = iii_drone::math::quatToMat(
        iii_drone::types::quaternionFromTransformMsg(lookup.transform));
    const auto v = iii_drone::types::vectorFromTransformMsg(lookup.transform);

    for (const point_t & point : std::vector<point_t>{
            point_t(1.0, 0.0, 0.0), point_t(0.2, 3.5, -1.0), point_t(-2.0, 0.4, 7.25)}) {
        geometry_msgs::msg::PointStamped stamped;
        stamped.header.frame_id = "mmwave";
        stamped.point = iii_drone::types::pointMsgFromPoint(point);
        const auto expected = iii_drone::types::pointFromPointMsg(buffer.transform(stamped, "drone").point);
        const point_t via_quat_to_mat = R * point + v;
        EXPECT_NEAR((via_quat_to_mat - expected).norm(), 0.0, 1e-5) << point.transpose();
        tf2::Transform drone_from_mmwave;
        tf2::fromMsg(lookup.transform, drone_from_mmwave);
        const tf2::Vector3 cached = drone_from_mmwave * tf2::Vector3(point.x(), point.y(), point.z());
        EXPECT_NEAR((point_t(cached.x(), cached.y(), cached.z()) - expected).norm(), 0.0, 1e-5) << point.transpose();
    }
}

// eulToMat is R = Rz(yaw) Ry(pitch) Rx(roll) for every angle combination
// (its (1,2) entry used cos(pitch) where cos(yaw) belongs).
TEST(RadarExtrinsic, EulerMatrixIsTheZyxRotation) {
    for (double roll : {-2.5, -0.4, 0.0, 0.05, 1.3}) {
        for (double pitch : {-1.2, -0.1, 0.0, 0.6}) {
            for (double yaw : {-3.0, -1.57, 0.0, 0.7, 2.2}) {
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                const tf2::Matrix3x3 expected(q);
                const auto R = iii_drone::math::eulToMat(
                    iii_drone::types::euler_angles_t(roll, pitch, yaw));
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 3; ++column) {
                        EXPECT_NEAR(R(row, column), expected[row][column], 1e-5)
                            << "roll " << roll << " pitch " << pitch << " yaw " << yaw
                            << " entry " << row << "," << column;
                    }
                }
            }
        }
    }
}

// The static check is what the per-line check applies after moving a point
// into the radar frame.
TEST(RadarExtrinsic, StaticFieldOfViewCheckIsDistanceAndConeBound) {
    EXPECT_FALSE(iii_drone::perception::SingleLine::IsInFOV(point_t(0.1, 0.0, 0.0), 0.5, 10.0, 0.2));
    EXPECT_FALSE(iii_drone::perception::SingleLine::IsInFOV(point_t(20.0, 0.0, 0.0), 0.5, 10.0, 0.2));
}
