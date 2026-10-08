#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <iii_drone_core/utils/opti_track_pose_relay.hpp>

using namespace iii_drone::utils::opti_track_pose_relay;

namespace {

constexpr int64_t kMs = 1'000'000;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

using Vector = std::array<double, 3>;
using Quaternion = std::array<double, 4>;  // w, x, y, z

// v' = q v q*
Vector rotate(const Quaternion & q, const Vector & v) {
    const double w = q[0];
    const Vector u{q[1], q[2], q[3]};
    const Vector t{
        2.0 * (u[1] * v[2] - u[2] * v[1]),
        2.0 * (u[2] * v[0] - u[0] * v[2]),
        2.0 * (u[0] * v[1] - u[1] * v[0]),
    };
    return {
        v[0] + w * t[0] + (u[1] * t[2] - u[2] * t[1]),
        v[1] + w * t[1] + (u[2] * t[0] - u[0] * t[2]),
        v[2] + w * t[2] + (u[0] * t[1] - u[1] * t[0]),
    };
}

// The 180 degree rotation about x between the lab and the PX4 frames.
Vector flipYZ(const Vector & v) {
    return {v[0], -v[1], -v[2]};
}

void expectNear(const Vector & actual, const Vector & expected, double tolerance = 1.0e-12) {
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual[i], expected[i], tolerance) << "component " << i;
    }
}

void expectNear(const Quaternion & actual, const Quaternion & expected, double tolerance = 1.0e-12) {
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(actual[i], expected[i], tolerance) << "component " << i;
    }
}

PoseRelayParameters validParameters() {
    PoseRelayParameters parameters;
    parameters.rigid_body_id = 7;
    return parameters;
}

bool mentions(const std::vector<std::string> & errors, const std::string & text) {
    for (const auto & error : errors) {
        if (error.find(text) != std::string::npos) {
            return true;
        }
    }
    return false;
}

LabPose labPose(const Vector & position, const Quaternion & orientation) {
    LabPose pose;
    pose.position = position;
    pose.orientation = orientation;
    return pose;
}

}  // namespace

TEST(PoseRelayParameters, DefaultsAreValidOnceTheRigidBodyIsSet) {
    EXPECT_TRUE(ValidatePoseRelayParameters(validParameters()).empty());
}

TEST(PoseRelayParameters, UnsetRigidBodyIdIsReported) {
    const auto errors = ValidatePoseRelayParameters(PoseRelayParameters());
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_NE(errors[0].find("/opti_track/pose_relay/rigid_body_id is not configured"), std::string::npos)
        << errors[0];
}

TEST(PoseRelayParameters, EveryInvalidValueIsNamed) {
    PoseRelayParameters parameters;
    parameters.rigid_body_id = -2;
    parameters.lab_ros_domain_id = 233;
    parameters.output_rate_hz = 0.0;
    parameters.stale_timeout_s = 0.0;
    parameters.position_variance_m2 = kNan;
    parameters.orientation_variance_rad2 = -1.0;
    parameters.origin_latitude_deg = 90.5;
    parameters.origin_longitude_deg = -180.5;
    parameters.origin_altitude_m = std::numeric_limits<double>::infinity();
    const auto errors = ValidatePoseRelayParameters(parameters);
    EXPECT_EQ(errors.size(), 9u);
    for (const std::string name : {
            "rigid_body_id must be >= 0", "lab_ros_domain_id", "output_rate_hz", "stale_timeout_s",
            "position_variance_m2", "orientation_variance_rad2", "origin_latitude_deg",
            "origin_longitude_deg", "origin_altitude_m"}) {
        EXPECT_TRUE(mentions(errors, "/opti_track/pose_relay/" + name)) << name;
    }
}

TEST(PoseRelayParameters, RangeBoundaries) {
    struct Case {
        const char * name;
        void (*apply)(PoseRelayParameters &);
        bool valid;
    };
    const std::vector<Case> cases{
        {"domain 0", [](PoseRelayParameters & p) { p.lab_ros_domain_id = 0; }, true},
        {"domain 232", [](PoseRelayParameters & p) { p.lab_ros_domain_id = 232; }, true},
        {"domain -1", [](PoseRelayParameters & p) { p.lab_ros_domain_id = -1; }, false},
        {"rate 1", [](PoseRelayParameters & p) { p.output_rate_hz = 1.0; }, true},
        {"rate 200", [](PoseRelayParameters & p) { p.output_rate_hz = 200.0; }, true},
        {"rate 200.1", [](PoseRelayParameters & p) { p.output_rate_hz = 200.1; }, false},
        {"rate nan", [](PoseRelayParameters & p) { p.output_rate_hz = kNan; }, false},
        {"stale 1", [](PoseRelayParameters & p) { p.stale_timeout_s = 1.0; }, true},
        {"stale 1.5", [](PoseRelayParameters & p) { p.stale_timeout_s = 1.5; }, false},
        {"stale -0.1", [](PoseRelayParameters & p) { p.stale_timeout_s = -0.1; }, false},
        {"position variance 0", [](PoseRelayParameters & p) { p.position_variance_m2 = 0.0; }, false},
        {"position variance 1", [](PoseRelayParameters & p) { p.position_variance_m2 = 1.0; }, true},
        {"latitude -90", [](PoseRelayParameters & p) { p.origin_latitude_deg = -90.0; }, true},
        {"longitude 180", [](PoseRelayParameters & p) { p.origin_longitude_deg = 180.0; }, true},
        {"altitude -501", [](PoseRelayParameters & p) { p.origin_altitude_m = -501.0; }, false},
        {"rigid body 0", [](PoseRelayParameters & p) { p.rigid_body_id = 0; }, true},
    };
    for (const auto & scenario : cases) {
        SCOPED_TRACE(scenario.name);
        PoseRelayParameters parameters = validParameters();
        scenario.apply(parameters);
        EXPECT_EQ(ValidatePoseRelayParameters(parameters).empty(), scenario.valid);
    }
}

TEST(PoseRelayParameters, LabTopicCarriesTheRigidBodyId) {
    EXPECT_EQ(LabPoseTopic(7), "/body_splitter/body_7/pose");
    EXPECT_EQ(LabPoseTopic(1203), "/body_splitter/body_1203/pose");
}

TEST(LabPoseToNed, IdentityFlipsTheVerticalAndLateralAxes) {
    const auto ned = LabPoseToNed(labPose({1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}));
    ASSERT_TRUE(ned.has_value());
    expectNear(ned->position, {1.0, -2.0, -3.0});
    expectNear(ned->orientation, {1.0, 0.0, 0.0, 0.0});
}

TEST(LabPoseToNed, NinetyDegreeYawFacesTheOtherWay) {
    // Lab: nose along lab +y (left of lab x), a +90 degree yaw about Z up.
    const double c = std::cos(M_PI / 4.0);
    const double s = std::sin(M_PI / 4.0);
    const auto ned = LabPoseToNed(labPose({0.5, -1.0, 1.5}, {c, 0.0, 0.0, s}));
    ASSERT_TRUE(ned.has_value());
    expectNear(ned->position, {0.5, 1.0, -1.5});
    // NED: nose along -east, a -90 degree yaw about down.
    expectNear(ned->orientation, {c, 0.0, 0.0, -s});
    expectNear(rotate(ned->orientation, {1.0, 0.0, 0.0}), {0.0, -1.0, 0.0});
    // The right wing (FRD y) points north (lab +x).
    expectNear(rotate(ned->orientation, {0.0, 1.0, 0.0}), {1.0, 0.0, 0.0});
}

TEST(LabPoseToNed, RollAndPitchKeepTheirPhysicalSense) {
    const double half = M_PI / 12.0;  // 30 degree rotations
    // Lab roll about forward x lifts the left side: in FRD the right side drops.
    const auto roll = LabPoseToNed(labPose({0.0, 0.0, 0.0}, {std::cos(half), std::sin(half), 0.0, 0.0}));
    ASSERT_TRUE(roll.has_value());
    expectNear(roll->orientation, {std::cos(half), std::sin(half), 0.0, 0.0});
    EXPECT_GT(rotate(roll->orientation, {0.0, 1.0, 0.0})[2], 0.0);  // right wing below (down positive)
    // Lab rotation about left y lowers the nose: in NED the nose points down.
    const auto pitch = LabPoseToNed(labPose({0.0, 0.0, 0.0}, {std::cos(half), 0.0, std::sin(half), 0.0}));
    ASSERT_TRUE(pitch.has_value());
    EXPECT_GT(rotate(pitch->orientation, {1.0, 0.0, 0.0})[2], 0.0);
}

TEST(LabPoseToNed, AnyRotationMapsBodyAxesThroughTheFlip) {
    // R_ned(q') * flip(v) == flip(R_lab(q) * v) for every body vector v.
    std::mt19937 generator(7);
    std::normal_distribution<double> normal(0.0, 1.0);
    for (int sample = 0; sample < 200; ++sample) {
        Quaternion q{normal(generator), normal(generator), normal(generator), normal(generator)};
        const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (auto & component : q) {
            component /= norm;
        }
        const Vector position{normal(generator), normal(generator), normal(generator)};
        const auto ned = LabPoseToNed(labPose(position, q));
        ASSERT_TRUE(ned.has_value());
        expectNear(ned->position, flipYZ(position));
        for (const Vector & body : {Vector{1.0, 0.0, 0.0}, Vector{0.0, 1.0, 0.0}, Vector{0.0, 0.0, 1.0}}) {
            expectNear(rotate(ned->orientation, flipYZ(body)), flipYZ(rotate(q, body)), 1.0e-9);
        }
    }
}

TEST(LabPoseToNed, QuaternionIsNormalised) {
    const double c = std::cos(0.3);
    const double s = std::sin(0.3);
    const double scale = 1.07;
    const auto ned = LabPoseToNed(labPose({0.0, 0.0, 0.0}, {scale * c, 0.0, 0.0, scale * s}));
    ASSERT_TRUE(ned.has_value());
    expectNear(ned->orientation, {c, 0.0, 0.0, -s});
    const auto & q = ned->orientation;
    EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1.0e-12);
}

TEST(LabPoseToNed, NonFiniteOrDegenerateInputIsRejected) {
    const double infinity = std::numeric_limits<double>::infinity();
    struct Case {
        LabPose pose;
        const char * reason;
    };
    const std::vector<Case> cases{
        {labPose({kNan, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}), "non-finite position"},
        {labPose({0.0, infinity, 0.0}, {1.0, 0.0, 0.0, 0.0}), "non-finite position"},
        {labPose({0.0, 0.0, 0.0}, {kNan, 0.0, 0.0, 0.0}), "non-finite orientation"},
        {labPose({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, -infinity}), "non-finite orientation"},
        {labPose({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}), "degenerate orientation"},
        {labPose({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0, 0.0}), "degenerate orientation"},
        {labPose({0.0, 0.0, 0.0}, {0.5, 0.0, 0.0, 0.0}), "degenerate orientation"},
    };
    for (const auto & scenario : cases) {
        SCOPED_TRACE(scenario.reason);
        std::string reason;
        EXPECT_FALSE(LabPoseToNed(scenario.pose, &reason).has_value());
        EXPECT_NE(reason.find(scenario.reason), std::string::npos) << reason;
    }
    EXPECT_FALSE(LabPoseToNed(labPose({kNan, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0})).has_value());
}

TEST(OutputGate, FirstPoseIsForwarded) {
    OutputGate gate(50.0, 0.15);
    EXPECT_TRUE(gate.Offer(1'000 * kMs, 1'000 * kMs));
}

TEST(OutputGate, FastInputIsDecimatedToTheOutputRate) {
    OutputGate gate(50.0, 0.15);
    const int64_t input_period_ns = 1'000'000'000 / 120;
    std::vector<int64_t> forwarded;
    for (int i = 0; i < 1200; ++i) {  // 10 s at 120 Hz
        const int64_t t = 1'000 * kMs + i * input_period_ns;
        if (gate.Offer(t, t)) {
            forwarded.push_back(t);
        }
    }
    EXPECT_GE(forwarded.size(), 499u);
    EXPECT_LE(forwarded.size(), 501u);
    for (std::size_t i = 1; i < forwarded.size(); ++i) {
        EXPECT_GE(forwarded[i] - forwarded[i - 1], 10 * kMs);
    }
    // Never more than one period's worth in any one second.
    for (std::size_t i = 0; i + 51 < forwarded.size(); ++i) {
        EXPECT_GT(forwarded[i + 51] - forwarded[i], 1'000 * kMs);
    }
}

TEST(OutputGate, SlowInputIsForwardedCompletely) {
    OutputGate gate(50.0, 0.15);
    const int64_t input_period_ns = 1'000'000'000 / 30;
    for (int i = 0; i < 300; ++i) {
        const int64_t t = 1'000 * kMs + i * input_period_ns;
        EXPECT_TRUE(gate.Offer(t, t)) << "pose " << i;
    }
}

TEST(OutputGate, PoseOlderThanTheStaleTimeoutIsNeverForwarded) {
    OutputGate gate(50.0, 0.15);
    EXPECT_FALSE(gate.Offer(1'000 * kMs, 1'151 * kMs));
    EXPECT_TRUE(gate.Offer(1'000 * kMs, 1'150 * kMs));
}

TEST(OutputGate, PoseNoNewerThanTheLastForwardedIsDropped) {
    OutputGate gate(50.0, 0.15);
    ASSERT_TRUE(gate.Offer(1'000 * kMs, 1'000 * kMs));
    EXPECT_FALSE(gate.Offer(1'000 * kMs, 1'030 * kMs));
    EXPECT_FALSE(gate.Offer(990 * kMs, 1'030 * kMs));
    EXPECT_TRUE(gate.Offer(1'025 * kMs, 1'030 * kMs));
}

TEST(OutputGate, ResumedStreamIsNotCaughtUp) {
    OutputGate gate(50.0, 0.15);
    const int64_t input_period_ns = 1'000'000'000 / 120;
    int64_t t = 1'000 * kMs;
    for (int i = 0; i < 120; ++i, t += input_period_ns) {
        gate.Offer(t, t);
    }
    // 500 ms without poses (stale), then a burst of five delayed poses
    // arriving 1 ms apart, then the regular stream.
    t += 500 * kMs;
    const int64_t resume = t;
    std::vector<int64_t> forwarded;
    for (int i = 0; i < 5; ++i, t += kMs) {
        if (gate.Offer(t, t)) {
            forwarded.push_back(t);
        }
    }
    for (int i = 0; i < 24; ++i, t += input_period_ns) {
        if (gate.Offer(t, t)) {
            forwarded.push_back(t);
        }
    }
    ASSERT_FALSE(forwarded.empty());
    EXPECT_EQ(forwarded.front(), resume);  // the first fresh pose, immediately
    for (std::size_t i = 1; i < forwarded.size(); ++i) {
        EXPECT_GE(forwarded[i] - forwarded[i - 1], 10 * kMs);
    }
    // At most the output rate over the ~200 ms after the resume.
    EXPECT_LE(static_cast<double>(forwarded.size()), (t - resume) * 50.0 / 1.0e9 + 1.0);
}

TEST(RelayHealthMonitor, NoPoseYetIsAnError) {
    RelayHealthMonitor monitor(50.0, 0.15, 0);
    const auto report = monitor.Report(500 * kMs);
    EXPECT_EQ(report.level, HealthLevel::ERROR);
    EXPECT_EQ(report.message, "no pose received yet");
    EXPECT_TRUE(report.stale);
    EXPECT_TRUE(std::isnan(report.last_input_age_ms));
    EXPECT_TRUE(std::isnan(report.max_input_gap_ms));
    EXPECT_TRUE(std::isnan(report.lab_stamp_age_ms));
    EXPECT_DOUBLE_EQ(report.input_rate_hz, 0.0);
    EXPECT_FALSE(report.forwarding);
}

TEST(RelayHealthMonitor, ForwardingNeedsAPoseForwardedWithinTheStaleTimeout) {
    RelayHealthMonitor monitor(50.0, 0.15, 0);
    monitor.RecordInput(0, kNan);
    monitor.RecordOutput(0);
    EXPECT_TRUE(monitor.Report(150 * kMs).forwarding);
    EXPECT_FALSE(monitor.Report(151 * kMs).forwarding);
    // Received but not forwarded (e.g. decimated): not forwarding fresh poses.
    monitor.RecordInput(200 * kMs, kNan);
    const auto report = monitor.Report(210 * kMs);
    EXPECT_FALSE(report.stale);
    EXPECT_FALSE(report.forwarding);
}

TEST(RelayHealthMonitor, SteadyStreamIsOk) {
    RelayHealthMonitor monitor(50.0, 0.15, 0);
    const int64_t input_period_ns = 1'000'000'000 / 120;
    for (int i = 0; i < 60; ++i) {
        monitor.RecordInput(i * input_period_ns, 12.5);
        if (i % 12 == 0 || i % 12 == 5) {  // 20 Hz forwarded
            monitor.RecordOutput(i * input_period_ns);
        }
    }
    const auto report = monitor.Report(500 * kMs);
    EXPECT_EQ(report.level, HealthLevel::OK) << report.message;
    EXPECT_EQ(report.message, "ok");
    EXPECT_FALSE(report.stale);
    EXPECT_NEAR(report.input_rate_hz, 120.0, 0.1);
    EXPECT_NEAR(report.output_rate_hz, 20.0, 0.1);
    EXPECT_NEAR(report.last_input_age_ms, 500.0 - 59 * 1000.0 / 120.0, 1.0e-3);
    EXPECT_NEAR(report.max_input_gap_ms, 1000.0 / 120.0, 1.0e-3);
    EXPECT_DOUBLE_EQ(report.lab_stamp_age_ms, 12.5);
    EXPECT_EQ(report.rejected_samples, 0u);
    EXPECT_TRUE(report.forwarding);
}

TEST(RelayHealthMonitor, StalePoseIsAnError) {
    RelayHealthMonitor monitor(50.0, 0.15, 0);
    monitor.RecordInput(100 * kMs, kNan);
    const auto report = monitor.Report(400 * kMs);
    EXPECT_EQ(report.level, HealthLevel::ERROR);
    EXPECT_TRUE(report.stale);
    EXPECT_NE(report.message.find("stale: last pose 300 ms ago (timeout 150 ms)"), std::string::npos)
        << report.message;
    EXPECT_NEAR(report.max_input_gap_ms, 300.0, 1.0e-9);
    EXPECT_TRUE(std::isnan(report.lab_stamp_age_ms));
}

TEST(RelayHealthMonitor, RecoveredGapAndRejectedPosesWarnForOnePeriod) {
    RelayHealthMonitor monitor(50.0, 0.15, 0);
    const int64_t input_period_ns = 1'000'000'000 / 120;
    for (int i = 0; i < 12; ++i) {
        monitor.RecordInput(i * input_period_ns, kNan);
    }
    // 250 ms gap, then the stream resumes.
    const int64_t resume = 11 * input_period_ns + 250 * kMs;
    for (int i = 0; i < 20; ++i) {
        monitor.RecordInput(resume + i * input_period_ns, kNan);
    }
    monitor.RecordRejected();
    const int64_t first_end = resume + 19 * input_period_ns + 2 * kMs;
    const auto warned = monitor.Report(first_end);
    EXPECT_EQ(warned.level, HealthLevel::WARN);
    EXPECT_FALSE(warned.stale);
    EXPECT_NE(warned.message.find("input gap 250 ms"), std::string::npos) << warned.message;
    EXPECT_NE(warned.message.find("1 rejected poses"), std::string::npos) << warned.message;
    EXPECT_EQ(warned.rejected_samples, 1u);

    // A clean period afterwards is OK again; the rejected total is kept.
    int64_t t = resume + 20 * input_period_ns;
    for (int i = 0; i < 60; ++i, t += input_period_ns) {
        monitor.RecordInput(t, kNan);
    }
    const auto clean = monitor.Report(t);
    EXPECT_EQ(clean.level, HealthLevel::OK) << clean.message;
    EXPECT_EQ(clean.rejected_samples, 1u);
}

TEST(RelayHealthMonitor, InputBelowHalfTheOutputRateWarns) {
    RelayHealthMonitor monitor(50.0, 0.15, 0);
    for (int i = 0; i < 10; ++i) {  // 20 Hz
        monitor.RecordInput(i * 50 * kMs, kNan);
    }
    const auto report = monitor.Report(500 * kMs);
    EXPECT_EQ(report.level, HealthLevel::WARN);
    EXPECT_NE(report.message.find("input rate 20.0 Hz below half the output rate 50.0 Hz"), std::string::npos)
        << report.message;
}

TEST(OriginSender, SendsOnlyWhileDisarmedWithoutOriginAndFusingVision) {
    struct Case {
        const char * name;
        bool disarmed;
        bool xy_global;
        bool cs_ev_pos;
        bool due;
    };
    const std::vector<Case> cases{
        {"ready", true, false, true, true},
        {"armed", false, false, true, false},
        {"origin set", true, true, true, false},
        {"no vision position fusion", true, false, false, false},
        {"armed with origin", false, true, true, false},
    };
    for (const auto & scenario : cases) {
        SCOPED_TRACE(scenario.name);
        OriginSender sender(true, 5'000 * kMs, 3'000 * kMs);
        sender.UpdateDisarmed(scenario.disarmed, 0);
        sender.UpdateGlobalOrigin(scenario.xy_global, 0);
        sender.UpdateVisionPositionFusion(scenario.cs_ev_pos, 0);
        EXPECT_EQ(sender.Due(100 * kMs), scenario.due);
    }
}

TEST(OriginSender, NeedsEveryInputFresh) {
    OriginSender sender(true, 5'000 * kMs, 3'000 * kMs);
    EXPECT_FALSE(sender.Due(0));
    sender.UpdateDisarmed(true, 0);
    sender.UpdateGlobalOrigin(false, 0);
    EXPECT_FALSE(sender.Due(0));  // estimator flags missing
    sender.UpdateVisionPositionFusion(true, 0);
    EXPECT_TRUE(sender.Due(3'000 * kMs));
    EXPECT_FALSE(sender.Due(3'001 * kMs));  // all inputs older than 3 s
    sender.UpdateGlobalOrigin(false, 3'001 * kMs);
    sender.UpdateVisionPositionFusion(true, 3'001 * kMs);
    EXPECT_FALSE(sender.Due(3'001 * kMs));  // arming state stale: never assume disarmed
    sender.UpdateDisarmed(true, 3'001 * kMs);
    EXPECT_TRUE(sender.Due(3'001 * kMs));
}

TEST(OriginSender, ResendsAtMostEveryIntervalAndStopsOnceTheOriginIsSet) {
    OriginSender sender(true, 5'000 * kMs, 3'000 * kMs);
    const auto update = [&sender](int64_t t, bool xy_global) {
        sender.UpdateDisarmed(true, t);
        sender.UpdateGlobalOrigin(xy_global, t);
        sender.UpdateVisionPositionFusion(true, t);
    };
    EXPECT_FALSE(sender.sent());
    update(0, false);
    ASSERT_TRUE(sender.Due(0));
    sender.MarkSent(0);
    EXPECT_TRUE(sender.sent());
    update(4'999 * kMs, false);
    EXPECT_FALSE(sender.Due(4'999 * kMs));
    update(5'000 * kMs, false);
    ASSERT_TRUE(sender.Due(5'000 * kMs));
    sender.MarkSent(5'000 * kMs);
    update(10'500 * kMs, true);
    EXPECT_FALSE(sender.Due(10'500 * kMs));
    EXPECT_TRUE(sender.sent());
}

TEST(OriginSender, DisabledNeverSends) {
    OriginSender sender(false, 5'000 * kMs, 3'000 * kMs);
    sender.UpdateDisarmed(true, 0);
    sender.UpdateGlobalOrigin(false, 0);
    sender.UpdateVisionPositionFusion(true, 0);
    EXPECT_FALSE(sender.Due(0));
    EXPECT_FALSE(sender.sent());
}
