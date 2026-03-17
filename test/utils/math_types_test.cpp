#include <gtest/gtest.h>

#include <cmath>

#include <geometry_msgs/msg/transform.hpp>

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

namespace {

constexpr float kTolerance = 1e-4F;
constexpr float kRotationTolerance = 2e-1F;

void ExpectVectorNear(
  const iii_drone::types::vector_t & actual,
  const iii_drone::types::vector_t & expected,
  float tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectPointNear(
  const iii_drone::types::point_t & actual,
  const iii_drone::types::point_t & expected,
  float tolerance = kTolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectQuaternionNear(
  const iii_drone::types::quaternion_t & actual,
  const iii_drone::types::quaternion_t & expected,
  float tolerance = kTolerance)
{
  EXPECT_NEAR(actual(0), expected(0), tolerance);
  EXPECT_NEAR(actual(1), expected(1), tolerance);
  EXPECT_NEAR(actual(2), expected(2), tolerance);
  EXPECT_NEAR(actual(3), expected(3), tolerance);
}

void ExpectRotationMatrixNear(
  const iii_drone::types::rotation_matrix_t & actual,
  const iii_drone::types::rotation_matrix_t & expected,
  float tolerance = kTolerance)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), tolerance);
    }
  }
}

}  // namespace

TEST(MathTypesTest, PoseDefaultConstructorCreatesIdentityOrientation)
{
  iii_drone::types::pose_t pose;

  ExpectPointNear(pose.position, iii_drone::types::point_t::Zero());
  ExpectQuaternionNear(
    pose.orientation,
    iii_drone::types::quaternion_t(1.0F, 0.0F, 0.0F, 0.0F));
}

TEST(MathTypesTest, PointVectorQuaternionConversionsRoundTrip)
{
  const iii_drone::types::point_t point(1.5F, -2.0F, 3.25F);
  const iii_drone::types::vector_t vector(-4.0F, 5.5F, 6.25F);
  const iii_drone::types::quaternion_t quaternion(0.5F, -0.25F, 0.75F, -1.0F);

  ExpectPointNear(
    iii_drone::types::pointFromPointMsg(iii_drone::types::pointMsgFromPoint(point)),
    point);
  ExpectVectorNear(
    iii_drone::types::vectorFromVectorMsg(iii_drone::types::vectorMsgFromVector(vector)),
    vector);
  ExpectQuaternionNear(
    iii_drone::types::quaternionFromQuaternionMsg(
      iii_drone::types::quaternionMsgFromQuaternion(quaternion)),
    quaternion);
}

TEST(MathTypesTest, PoseAndTransformConversionsPreserveData)
{
  const iii_drone::types::point_t position(3.0F, 4.0F, -1.0F);
  const iii_drone::types::quaternion_t quaternion =
    iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(0.1F, -0.2F, 0.3F));

  const auto pose_msg = iii_drone::types::poseMsgFromPose(position, quaternion);
  const auto pose = iii_drone::types::poseFromPoseMsg(pose_msg);
  ExpectPointNear(pose.position, position);
  ExpectQuaternionNear(pose.orientation, quaternion);

  const auto transform_msg = iii_drone::types::transformMsgFromTransform(position, quaternion);
  const auto transform_pose = iii_drone::types::poseFromTransformMsg(transform_msg);
  ExpectPointNear(transform_pose.position, position);
  ExpectQuaternionNear(transform_pose.orientation, quaternion);

  ExpectPointNear(iii_drone::types::pointFromPoseMsg(pose_msg), position);
  ExpectQuaternionNear(iii_drone::types::quaternionFromPoseMsg(pose_msg), quaternion);
  ExpectVectorNear(iii_drone::types::vectorFromTransformMsg(transform_msg), position);
  ExpectQuaternionNear(
    iii_drone::types::quaternionFromTransformMsg(transform_msg),
    quaternion);
}

TEST(MathTypesTest, TransformMatrixConversionsRoundTrip)
{
  const iii_drone::types::vector_t translation(0.25F, -3.5F, 7.0F);
  const iii_drone::types::quaternion_t quaternion =
    iii_drone::math::eulToQuat(iii_drone::types::euler_angles_t(-0.4F, 0.2F, 1.1F));

  const auto matrix = iii_drone::math::createTransformMatrix(translation, quaternion);
  const auto transform_msg = iii_drone::types::transformMsgFromTransformMatrix(matrix);
  const auto reconstructed = iii_drone::types::transformMatrixFromTransformMsg(transform_msg);
  const auto pose = iii_drone::types::poseFromTransformMatrix(reconstructed);
  const auto expected_rotation =
    iii_drone::math::quatToMat(iii_drone::types::quaternionFromTransformMsg(transform_msg));

  ExpectVectorNear(reconstructed.block<3, 1>(0, 3), translation);
  ExpectPointNear(pose.position, translation);
  ExpectRotationMatrixNear(
    reconstructed.block<3, 3>(0, 0),
    expected_rotation,
    kRotationTolerance);
  ExpectRotationMatrixNear(
    iii_drone::math::quatToMat(iii_drone::types::quaternionFromTransformMatrix(reconstructed)),
    expected_rotation,
    kRotationTolerance);
  ExpectRotationMatrixNear(
    iii_drone::math::quatToMat(pose.orientation),
    expected_rotation,
    kRotationTolerance);
}

TEST(MathTypesTest, CreatePlaneStoresPointAndNormal)
{
  const iii_drone::types::point_t point(1.0F, 2.0F, 3.0F);
  const iii_drone::types::vector_t normal(0.0F, 1.0F, 0.0F);

  const auto plane = iii_drone::types::createPlane(point, normal);

  ExpectPointNear(plane.p, point);
  ExpectVectorNear(plane.normal, normal);
}

TEST(MathTypesTest, QuaternionMathHelpersRemainConsistent)
{
  const iii_drone::types::euler_angles_t euler(0.3F, -0.1F, 0.5F);
  const auto quaternion = iii_drone::math::eulToQuat(euler);
  const auto reconstructed_euler = iii_drone::math::quatToEul(quaternion);
  const auto rotation = iii_drone::math::eulToMat(euler);
  const auto rotated = iii_drone::math::rotateVector(rotation, iii_drone::types::vector_t::UnitX());
  const auto round_trip_quaternion = iii_drone::math::matToQuat(iii_drone::math::quatToMat(quaternion));
  const auto product = iii_drone::math::quatMultiply(quaternion, iii_drone::math::quatInv(quaternion));

  ExpectVectorNear(reconstructed_euler, euler);
  EXPECT_NEAR(rotated.norm(), 1.0F, kTolerance);
  ExpectRotationMatrixNear(
    iii_drone::math::quatToMat(round_trip_quaternion),
    iii_drone::math::quatToMat(quaternion),
    kRotationTolerance);
  ExpectQuaternionNear(product, iii_drone::types::quaternion_t(1.0F, 0.0F, 0.0F, 0.0F));
}

TEST(MathTypesTest, ProjectPointOnPlaneMovesPointOntoPlane)
{
  const auto plane = iii_drone::types::createPlane(
    iii_drone::types::point_t(0.0F, 0.0F, 2.0F),
    iii_drone::types::vector_t(0.0F, 0.0F, 1.0F));
  const iii_drone::types::point_t point(4.0F, -1.0F, 7.0F);

  const auto projected = iii_drone::math::projectPointOnPlane(point, plane);

  ExpectPointNear(projected, iii_drone::types::point_t(4.0F, -1.0F, 2.0F));
  EXPECT_NEAR((projected - plane.p).dot(plane.normal), 0.0F, kTolerance);
}
