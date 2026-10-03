#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <iii_drone_core/utils/multi_threaded_executor.hpp>

using namespace std::chrono_literals;

namespace {

class RclcppFixture : public ::testing::Test {
protected:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override {
        if (rclcpp::ok()) rclcpp::shutdown();
    }
};

}  // namespace

// ros2/rclcpp#3240: wait-set rebuilds that run while a default-group
// (MutuallyExclusive) callback executes drop the group; a lost rebuild request
// then leaves its timers and services, e.g. lifecycle get_state, unserviced
// for good. Churn rebuilds against a busy default group with frequent wait
// timeouts and require the group to stay serviced throughout.
TEST_F(RclcppFixture, DefaultGroupStaysServicedUnderRebuildChurn) {
    // The lifecycle services live in the default group, as in the field.
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("executor_rebuild_churn");

    std::atomic<int> default_timer_calls{0};
    auto busy_timer = node->create_wall_timer(1ms, [&]() {
        ++default_timer_calls;
        std::this_thread::sleep_for(500us);
    });

    // Another group keeps adding and removing entities: every change requests a
    // rebuild, mostly while the default group is busy.
    auto churn_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr churn_subscription;
    auto churn_timer = node->create_wall_timer(
        2ms,
        [&]() {
            if (churn_subscription) {
                churn_subscription.reset();
            } else {
                churn_subscription = node->create_subscription<std_msgs::msg::Empty>(
                    "churn", 1, [](std_msgs::msg::Empty::ConstSharedPtr) {});
            }
        },
        churn_group);

    iii_drone::utils::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node->get_node_base_interface());
    std::thread spinner([&]() { executor.spin(); });

    auto client_node = std::make_shared<rclcpp::Node>("executor_rebuild_probe");
    auto client = client_node->create_client<lifecycle_msgs::srv::GetState>(
        "executor_rebuild_churn/get_state");
    rclcpp::executors::SingleThreadedExecutor client_executor;
    client_executor.add_node(client_node);
    ASSERT_TRUE(client->wait_for_service(5s));

    const auto deadline = std::chrono::steady_clock::now() + 4s;
    int probes = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const int calls_before = default_timer_calls.load();
        auto future = client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
        ASSERT_EQ(client_executor.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS)
            << "lifecycle get_state unserviced after " << probes << " probes";
        std::this_thread::sleep_for(50ms);
        EXPECT_GT(default_timer_calls.load(), calls_before) << "default-group timer stopped";
        ++probes;
    }

    executor.cancel();
    spinner.join();
    EXPECT_GT(probes, 20);
}

namespace {

/**
 * Loses every interrupt the executor raises, as rmw_fastrtps does when the
 * trigger races an rmw_wait timeout (ros2/rclcpp#3240).
 */
template <class Executor>
class LosingInterruptsExecutor : public Executor {
public:
    using Executor::Executor;
    void LoseInterrupts() { this->interrupt_guard_condition_ = std::make_shared<rclcpp::GuardCondition>(); }
};

}  // namespace

// A rebuild while a default-group callback runs drops the group; the request
// to rebuild once it finishes must not depend on the interrupt arriving.
TEST_F(RclcppFixture, DefaultGroupReturnsAfterRebuildWhileBusyEvenIfTheInterruptIsLost) {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("executor_lost_interrupt");

    // Wakes the executor regularly, as the field node's other timers do.
    auto other_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto wake_timer = node->create_wall_timer(10ms, []() {}, other_group);

    // A default-group callback that adds an entity (requesting a rebuild) and
    // stays busy until the rebuild has run without its group, as on_activate
    // registering the maneuver servers does.
    std::atomic<bool> busy_done{false};
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr added_subscription;
    rclcpp::TimerBase::SharedPtr busy_timer;
    busy_timer = node->create_wall_timer(50ms, [&]() {
        busy_timer->cancel();
        added_subscription = node->create_subscription<std_msgs::msg::Empty>(
            "added", 1, [](std_msgs::msg::Empty::ConstSharedPtr) {});
        std::this_thread::sleep_for(300ms);
        busy_done = true;
    });

    LosingInterruptsExecutor<iii_drone::utils::MultiThreadedExecutor> executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node->get_node_base_interface());
    executor.LoseInterrupts();
    std::thread spinner([&]() { executor.spin(); });
    struct StopSpinner {
        std::thread & spinner;
        ~StopSpinner() {
            rclcpp::shutdown();
            spinner.join();
        }
    } stop_spinner{spinner};

    auto client_node = std::make_shared<rclcpp::Node>("executor_lost_interrupt_probe");
    auto client = client_node->create_client<lifecycle_msgs::srv::GetState>(
        "executor_lost_interrupt/get_state");
    rclcpp::executors::SingleThreadedExecutor client_executor;
    client_executor.add_node(client_node);
    ASSERT_TRUE(client->wait_for_service(5s));

    const auto busy_deadline = std::chrono::steady_clock::now() + 5s;
    while (!busy_done && std::chrono::steady_clock::now() < busy_deadline) {
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(busy_done.load()) << "default-group timer unserviced";

    auto future = client->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    EXPECT_EQ(client_executor.spin_until_future_complete(future, 3s), rclcpp::FutureReturnCode::SUCCESS)
        << "lifecycle get_state unserviced after its group was busy during a rebuild";
}

TEST_F(RclcppFixture, UsesRequestedThreadCount) {
    EXPECT_EQ(iii_drone::utils::MultiThreadedExecutor(rclcpp::ExecutorOptions(), 3).get_number_of_threads(), 3u);
    EXPECT_GE(iii_drone::utils::MultiThreadedExecutor().get_number_of_threads(), 2u);
}
