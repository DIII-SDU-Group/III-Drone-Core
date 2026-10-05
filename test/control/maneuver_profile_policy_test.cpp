#include <gtest/gtest.h>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcutils/logging.h>

#include <iii_drone_core/control/maneuver/maneuver_profile_policy.hpp>
#include <iii_drone_core/control/maneuver/maneuver_server.hpp>
#include <iii_drone_core/utils/runtime_profile.hpp>

using iii_drone::control::Reference;
using iii_drone::control::State;
using iii_drone::control::maneuver::Maneuver;
using iii_drone::control::maneuver::ManeuverAvailableInProfile;
using iii_drone::control::maneuver::ManeuverServer;
using iii_drone::control::maneuver::ManeuverUnavailableMessage;
using iii_drone::control::maneuver::maneuver_type_t;
using iii_drone::utils::ResolveRuntimeProfile;

namespace maneuver = iii_drone::control::maneuver;

namespace {

constexpr char kRequest[] = "mri1-00000000000000010000000000000001-0000000000000001";

class RclcppContext {
public:
    RclcppContext() : initialized_here_(!rclcpp::ok()) {
        if (initialized_here_) {
            rclcpp::init(0, nullptr);
        }
    }

    ~RclcppContext() {
        if (initialized_here_) {
            rclcpp::shutdown();
        }
    }

private:
    bool initialized_here_;
};

// Records every formatted log line and forwards it to the previous handler.
class LogCapture {
public:
    LogCapture() : previous_(rcutils_logging_get_output_handler()) {
        instance_ = this;
        rcutils_logging_set_output_handler(&LogCapture::handle);
    }

    ~LogCapture() {
        rcutils_logging_set_output_handler(previous_);
        instance_ = nullptr;
    }

    bool contains(int severity, const std::string & message) const {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & [entry_severity, entry_message] : entries_) {
            if (entry_severity == severity && entry_message == message) {
                return true;
            }
        }
        return false;
    }

private:
    static void handle(
        const rcutils_log_location_t * location,
        int severity,
        const char * name,
        rcutils_time_point_value_t timestamp,
        const char * format,
        va_list * args
    ) {
        LogCapture * capture = instance_;
        if (capture == nullptr) {
            return;
        }
        char buffer[1024];
        va_list copy;
        va_copy(copy, *args);
        std::vsnprintf(buffer, sizeof(buffer), format, copy);
        va_end(copy);
        {
            std::lock_guard<std::mutex> lock(capture->mutex_);
            capture->entries_.emplace_back(severity, buffer);
        }
        if (capture->previous_ != nullptr) {
            capture->previous_(location, severity, name, timestamp, format, args);
        }
    }

    static inline LogCapture * instance_ = nullptr;
    rcutils_logging_output_handler_t previous_;
    mutable std::mutex mutex_;
    std::vector<std::pair<int, std::string>> entries_;
};

// A real action server whose scheduler hooks admit every goal.
template <typename ActionT>
class StubManeuverServer final : public ManeuverServer {
public:
    StubManeuverServer(
        rclcpp_lifecycle::LifecycleNode * node,
        maneuver_type_t type,
        const std::string & action_name
    ) : ManeuverServer(node, nullptr, action_name, 1, 1), type_(type) {
        createServer<ActionT>();
    }

    bool CanExecuteManeuver(
        const Maneuver &,
        const iii_drone::adapters::CombinedDroneAwarenessAdapter &
    ) const override {
        return true;
    }

    iii_drone::adapters::CombinedDroneAwarenessAdapter ExpectedAwarenessAfterExecution(
        const Maneuver &
    ) override {
        return {};
    }

protected:
    maneuver_type_t maneuver_type() const override { return type_; }
    void startExecution(Maneuver &) override {}
    bool canCancel() override { return true; }
    Reference computeReference(const State &) override { return Reference(); }
    bool hasSucceeded(Maneuver &) override { return false; }
    bool hasFailed(Maneuver &) override { return false; }
    void publishResultAndFinalize(Maneuver &, maneuver_result_type_t) override {}
    void registerReferenceCallbackOnSuccess(const Maneuver &) override {}

private:
    maneuver_type_t type_;
};

// Sends one goal and returns whether the server accepted it.
template <typename ActionT>
std::optional<bool> goalAccepted(
    rclcpp::Executor & executor,
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client
) {
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        return std::nullopt;
    }
    typename ActionT::Goal goal;
    goal.request_identity = kRequest;
    auto future = client->async_send_goal(goal);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        if (std::chrono::steady_clock::now() > deadline) {
            return std::nullopt;
        }
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return future.get() != nullptr;
}

}  // namespace

TEST(RuntimeProfile, NonEmptyParameterTakesPrecedenceOverTheEnvironment) {
    EXPECT_EQ(ResolveRuntimeProfile("opti_track", "hil"), "opti_track");
    EXPECT_EQ(ResolveRuntimeProfile("sim", "opti_track"), "sim");
}

TEST(RuntimeProfile, EmptyParameterFallsBackToTheEnvironment) {
    EXPECT_EQ(ResolveRuntimeProfile("", "opti_track"), "opti_track");
    EXPECT_EQ(ResolveRuntimeProfile("  \t", "hil"), "hil");
    EXPECT_EQ(ResolveRuntimeProfile("", nullptr), "");
    EXPECT_EQ(ResolveRuntimeProfile("", ""), "");
}

TEST(RuntimeProfile, IsTrimmedAndLowerCased) {
    EXPECT_EQ(ResolveRuntimeProfile(" OPTI_TRACK\n", nullptr), "opti_track");
    EXPECT_EQ(ResolveRuntimeProfile("", " Opti_Track "), "opti_track");
}

TEST(ManeuverProfilePolicy, OptiTrackServesOnlyTheFlightBasics) {
    const std::vector<std::pair<maneuver_type_t, bool>> expected{
        {maneuver::MANEUVER_TYPE_HOVER, true},
        {maneuver::MANEUVER_TYPE_FLY_TO_POSITION, true},
        {maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH, true},
        {maneuver::MANEUVER_TYPE_CABLE_AWARE_FLY_TO_POSITION, false},
        {maneuver::MANEUVER_TYPE_FLY_TO_OBJECT, false},
        {maneuver::MANEUVER_TYPE_HOVER_BY_OBJECT, false},
        {maneuver::MANEUVER_TYPE_HOVER_ON_CABLE, false},
        {maneuver::MANEUVER_TYPE_CABLE_LANDING, false},
        {maneuver::MANEUVER_TYPE_CABLE_TAKEOFF, false},
        {maneuver::MANEUVER_TYPE_NONE, false},
    };
    for (const auto & [type, available] : expected) {
        SCOPED_TRACE(static_cast<int>(type));
        EXPECT_EQ(ManeuverAvailableInProfile(type, "opti_track"), available);
    }
}

TEST(ManeuverProfilePolicy, OptiTrackRejectsManeuversAddedLater) {
    // An allowlist: a maneuver type the policy does not know stays unavailable.
    EXPECT_FALSE(ManeuverAvailableInProfile(static_cast<maneuver_type_t>(9), "opti_track"));
    EXPECT_FALSE(ManeuverAvailableInProfile(static_cast<maneuver_type_t>(15), "opti_track"));
}

TEST(ManeuverProfilePolicy, OtherProfilesAreUnrestricted) {
    for (const std::string profile : {"sim", "hil", "real", "", "unknown_profile"}) {
        SCOPED_TRACE(profile);
        for (int type = maneuver::MANEUVER_TYPE_FLY_TO_POSITION;
             type <= maneuver::MANEUVER_TYPE_FOLLOW_WAYPOINT_PATH; ++type) {
            EXPECT_TRUE(ManeuverAvailableInProfile(static_cast<maneuver_type_t>(type), profile));
        }
    }
}

TEST(ManeuverProfilePolicy, RejectionUsesTheSharedWording) {
    EXPECT_EQ(
        ManeuverUnavailableMessage("cable_landing", "opti_track"),
        "Maneuver cable_landing is not available in the opti_track profile"
    );
}

TEST(ManeuverProfileRejection, UnavailableManeuverRejectsGoalsBeforeTheScheduler) {
    RclcppContext context;
    LogCapture log;
    const std::string space = "/maneuver_profile_rejection_test";
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("maneuver_controller", space);
    auto client_node = std::make_shared<rclcpp::Node>("maneuver_profile_rejection_client", space);

    auto landing = std::make_shared<StubManeuverServer<iii_drone_interfaces::action::CableLanding>>(
        node.get(), maneuver::MANEUVER_TYPE_CABLE_LANDING, "cable_landing");
    auto hover = std::make_shared<StubManeuverServer<iii_drone_interfaces::action::Hover>>(
        node.get(), maneuver::MANEUVER_TYPE_HOVER, "hover");
    const std::string reason = ManeuverUnavailableMessage("cable_landing", "opti_track");
    landing->SetUnavailable(reason);
    EXPECT_FALSE(landing->available());
    EXPECT_TRUE(hover->available());

    // Started like a registered server: the scheduler would admit any goal it
    // is asked about, and accepted goals are aborted at once.
    int registrations = 0;
    for (const auto & server : std::vector<ManeuverServer::SharedPtr>{landing, hover}) {
        server->Start(
            [&registrations](Maneuver, bool & executing_instantly) {
                ++registrations;
                executing_instantly = false;
                return true;
            },
            [](Maneuver) { return false; },
            [](Maneuver) { return true; },
            [](Maneuver) { return false; },
            [](Maneuver) { return false; },
            [](Maneuver) {},
            nullptr,
            {}
        );
    }

    auto landing_client = rclcpp_action::create_client<iii_drone_interfaces::action::CableLanding>(
        client_node, space + "/cable_landing");
    auto hover_client = rclcpp_action::create_client<iii_drone_interfaces::action::Hover>(
        client_node, space + "/hover");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(client_node);

    const auto landing_accepted = goalAccepted<iii_drone_interfaces::action::CableLanding>(
        executor, landing_client);
    ASSERT_TRUE(landing_accepted.has_value()) << "no goal response from cable_landing";
    EXPECT_FALSE(*landing_accepted);
    EXPECT_EQ(registrations, 0);
    EXPECT_TRUE(log.contains(RCUTILS_LOG_SEVERITY_ERROR, reason));

    // The same hooks admit a goal of an available maneuver.
    const auto hover_accepted = goalAccepted<iii_drone_interfaces::action::Hover>(
        executor, hover_client);
    ASSERT_TRUE(hover_accepted.has_value()) << "no goal response from hover";
    EXPECT_TRUE(*hover_accepted);
    EXPECT_EQ(registrations, 1);

    executor.remove_node(client_node);
    executor.remove_node(node->get_node_base_interface());
    landing->Stop();
    hover->Stop();
}
