#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <algorithm>
#include <chrono>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace utils {

    /**
     * @brief rclcpp's MultiThreadedExecutor without Jazzy's lost wait-set
     * rebuild (ros2/rclcpp#3240).
     *
     * A wait-set rebuild that runs while a MutuallyExclusive callback executes
     * leaves that callback group's entities out of the wait set. Upstream only
     * requests the rebuild that brings them back through a guard condition,
     * and rmw_fastrtps clears that trigger when it races an rmw_wait timeout:
     * the group's timers, subscriptions and services (including lifecycle
     * get_state/change_state) then stay unserviced for the rest of the
     * process. This executor requests the rebuild directly after every
     * MutuallyExclusive callback, so it cannot be lost.
     *
     * Otherwise identical to rclcpp::executors::MultiThreadedExecutor
     * (rclcpp 28.1.x).
     */
    class MultiThreadedExecutor : public rclcpp::Executor {
    public:
        explicit MultiThreadedExecutor(
            const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
            size_t number_of_threads = 0,
            bool yield_before_execute = false,
            std::chrono::nanoseconds next_exec_timeout = std::chrono::nanoseconds(-1)
        ) : rclcpp::Executor(options),
            number_of_threads_(number_of_threads > 0 ?
                number_of_threads :
                std::max<size_t>(std::thread::hardware_concurrency(), 2)),
            yield_before_execute_(yield_before_execute),
            next_exec_timeout_(next_exec_timeout) {}

        void spin() override {
            if (spinning.exchange(true)) {
                throw std::runtime_error("spin() called while already spinning");
            }
            struct StopSpinning {
                MultiThreadedExecutor & executor;
                ~StopSpinning() {
                    executor.wait_result_.reset();
                    executor.spinning.store(false);
                }
            } stop_spinning{*this};

            std::vector<std::thread> threads;
            size_t thread_id = 0;
            {
                std::lock_guard<std::mutex> wait_lock{wait_mutex_};
                for (; thread_id < number_of_threads_ - 1; ++thread_id) {
                    threads.emplace_back([this]() { run(); });
                }
            }

            run();
            for (auto & thread : threads) {
                thread.join();
            }
        }

        size_t get_number_of_threads() const { return number_of_threads_; }

    private:
        void run() {
            while (rclcpp::ok(this->context_) && spinning.load()) {
                rclcpp::AnyExecutable any_exec;
                {
                    std::lock_guard<std::mutex> wait_lock{wait_mutex_};
                    if (!rclcpp::ok(this->context_) || !spinning.load()) {
                        return;
                    }
                    if (!get_next_executable(any_exec, next_exec_timeout_)) {
                        continue;
                    }
                }
                if (yield_before_execute_) {
                    std::this_thread::yield();
                }

                execute_any_executable(any_exec);

                if (any_exec.callback_group &&
                    any_exec.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive)
                {
                    // The group may have been left out of a rebuild while this
                    // callback ran: rebuild once it can be taken again.
                    entities_need_rebuild_.store(true);
                    interrupt_guard_condition_->trigger();
                }

                // Keep the AnyExecutable destructor from resetting the
                // group's can_be_taken_from.
                any_exec.callback_group.reset();
            }
        }

        std::mutex wait_mutex_;
        const size_t number_of_threads_;
        const bool yield_before_execute_;
        const std::chrono::nanoseconds next_exec_timeout_;
    };

} // namespace utils
} // namespace iii_drone
