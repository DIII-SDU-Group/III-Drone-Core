#include <gtest/gtest.h>

#include <cmath>

#include <iii_drone_core/control/waypoint_path_planner.hpp>

using iii_drone::control::Reference;
using iii_drone::control::WaypointPathConstraints;
using iii_drone::control::WaypointPathPlanner;
using iii_drone::control::WaypointPathWaypoint;
using iii_drone::control::WaypointTransition;
using iii_drone::types::point_t;

namespace {

WaypointPathConstraints constraints() {
    return WaypointPathConstraints{1.5, 0.8, 1.5, 0.02};
}

WaypointPathWaypoint blend(double x, double y, double z, double radius = 1.0) {
    return {point_t(x, y, z), 0.0, WaypointTransition::Blend, radius, 0.0};
}

}  // namespace

TEST(WaypointPathPlannerTest, CornerBlendStaysInsideConfiguredEnvelope) {
    const point_t corner(5.0, 0.0, 5.0);
    const auto plan = WaypointPathPlanner().plan(
        Reference(point_t(0.0, 0.0, 5.0), 0.0),
        {
            blend(corner.x(), corner.y(), corner.z()),
            {point_t(5.0, 8.0, 5.0), 0.0, WaypointTransition::Stop, 0.0, 0.0},
        },
        false,
        0,
        constraints()
    );

    bool observed_blend = false;
    for (std::size_t index = 0; index < plan.prefix.references.size(); ++index) {
        if (plan.prefix.primitive_indices[index] != 1) {
            continue;
        }
        observed_blend = true;
        EXPECT_LE((plan.prefix.references[index].position() - corner).norm(), 1.0 + 1.0e-6);
    }
    EXPECT_TRUE(observed_blend);
}

TEST(WaypointPathPlannerTest, LongSuccessorDoesNotChangeLocalBlendGeometry) {
    const Reference start(point_t(0.0, 0.0, 5.0), 0.0);
    const WaypointPathWaypoint corner = blend(5.0, 0.0, 5.0);
    const auto short_plan = WaypointPathPlanner().plan(
        start,
        {corner, {point_t(5.0, 8.0, 5.0), 0.0, WaypointTransition::Stop, 0.0, 0.0}},
        false,
        0,
        constraints()
    );
    const auto long_plan = WaypointPathPlanner().plan(
        start,
        {corner, {point_t(5.0, 80.0, 5.0), 0.0, WaypointTransition::Stop, 0.0, 0.0}},
        false,
        0,
        constraints()
    );

    std::vector<point_t> short_blend;
    std::vector<point_t> long_blend;
    for (std::size_t index = 0; index < short_plan.prefix.references.size(); ++index) {
        if (short_plan.prefix.primitive_indices[index] == 1) {
            short_blend.push_back(short_plan.prefix.references[index].position());
        }
    }
    for (std::size_t index = 0; index < long_plan.prefix.references.size(); ++index) {
        if (long_plan.prefix.primitive_indices[index] == 1) {
            long_blend.push_back(long_plan.prefix.references[index].position());
        }
    }
    ASSERT_EQ(short_blend.size(), long_blend.size());
    for (std::size_t index = 0; index < short_blend.size(); ++index) {
        EXPECT_NEAR((short_blend[index] - long_blend[index]).norm(), 0.0, 1.0e-9);
    }
}

TEST(WaypointPathPlannerTest, RepeatingPathHasContinuousLoopSeamAndBoundedDynamics) {
    const auto limits = constraints();
    const auto plan = WaypointPathPlanner().plan(
        Reference(point_t(-2.0, -2.0, 5.0), 0.0),
        {
            {point_t(0.0, 0.0, 5.0), 0.0, WaypointTransition::Stop, 0.0, 0.0},
            blend(4.0, 0.0, 5.0),
            blend(4.0, 4.0, 5.0),
            blend(0.0, 4.0, 5.0),
            blend(0.0, 0.0, 5.0),
        },
        true,
        1,
        limits
    );

    ASSERT_FALSE(plan.loop.empty());
    EXPECT_LT(plan.loopDurationS(), 32.0);
    EXPECT_NEAR(
        (plan.loop.references.front().position() - plan.loop.references.back().position()).norm(),
        0.0,
        1.0e-6
    );
    EXPECT_NEAR(
        (plan.loop.references.front().velocity() - plan.loop.references.back().velocity()).norm(),
        0.0,
        0.05
    );

    for (const auto & reference : plan.previewReferences()) {
        EXPECT_LE(reference.velocity().norm(), limits.nominal_speed_m_s + 1.0e-6);
        EXPECT_LE(reference.acceleration().norm(), limits.max_acceleration_m_s2 + 1.0e-6);
        EXPECT_NEAR(reference.position().z(), 5.0, 1.0e-6);
    }
    for (std::size_t index = 1; index < plan.loop.references.size(); ++index) {
        const double interval = plan.loop.times_s[index] - plan.loop.times_s[index - 1];
        const double jerk = (
            (plan.loop.references[index].acceleration() -
                plan.loop.references[index - 1].acceleration()) /
            interval
        ).norm();
        EXPECT_LE(jerk, limits.max_jerk_m_s3 + 1.0e-5);
    }

    const auto before = plan.sample(plan.prefixDurationS() + plan.loopDurationS() - 1.0e-4);
    const auto after = plan.sample(plan.prefixDurationS() + plan.loopDurationS() + 1.0e-4);
    EXPECT_LT((before.reference.position() - after.reference.position()).norm(), 0.01);
}

TEST(WaypointPathPlannerTest, RepeatingLoopReportsOriginalGoalWaypointIndices) {
    const auto plan = WaypointPathPlanner().plan(
        Reference(point_t(-2.0, -2.0, 5.0), 0.0),
        {
            {point_t(0.0, 0.0, 5.0), 0.0, WaypointTransition::Stop, 0.0, 0.0},
            blend(4.0, 0.0, 5.0),
            blend(4.0, 4.0, 5.0),
            blend(0.0, 4.0, 5.0),
            blend(0.0, 0.0, 5.0),
        },
        true,
        1,
        constraints()
    );

    ASSERT_FALSE(plan.loop.waypoint_indices.empty());
    EXPECT_EQ(plan.loop.waypoint_indices.front(), 2U);
    EXPECT_EQ(plan.loop.waypoint_indices.back(), 1U);
    for (const uint32_t index : plan.loop.waypoint_indices) {
        EXPECT_GE(index, 1U);
        EXPECT_LT(index, 5U);
    }
}

TEST(WaypointPathPlannerTest, RepeatingPrefixReportsOnlyOriginalGoalWaypointIndices) {
    constexpr uint32_t repeat_from_index = 1;
    constexpr uint32_t waypoint_count = 5;
    const auto plan = WaypointPathPlanner().plan(
        Reference(point_t(-2.0, -2.0, 5.0), 0.0),
        {
            {point_t(0.0, 0.0, 5.0), 0.0, WaypointTransition::Stop, 0.0, 0.0},
            blend(4.0, 0.0, 5.0),
            blend(4.0, 4.0, 5.0),
            blend(0.0, 4.0, 5.0),
            blend(0.0, 0.0, 5.0),
        },
        true,
        repeat_from_index,
        constraints()
    );

    ASSERT_FALSE(plan.prefix.waypoint_indices.empty());
    EXPECT_EQ(plan.prefix.waypoint_indices.back(), repeat_from_index);
    for (const uint32_t index : plan.prefix.waypoint_indices) {
        EXPECT_LT(index, waypoint_count);
    }
}
