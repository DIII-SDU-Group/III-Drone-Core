#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
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
     * rebuild (ros2/rclcpp#3240) and without a rebuild per callback.
     *
     * A wait-set rebuild that runs while a MutuallyExclusive callback executes
     * leaves that callback group's entities out of the wait set. Upstream only
     * requests the rebuild that brings them back through a guard condition,
     * and rmw_fastrtps clears that trigger when it races an rmw_wait timeout:
     * the group's timers, subscriptions and services (including lifecycle
     * get_state/change_state) then stay unserviced for the rest of the
     * process. This executor requests that rebuild directly, so it cannot be
     * lost.
     *
     * Upstream wakes a waiting thread and rebuilds the whole entity collection
     * after every MutuallyExclusive callback, which on the Pi cost more CPU
     * than the callbacks themselves. This executor rebuilds after a callback
     * only when a rebuild ran while it executed, the only way its group can
     * have been left out. A thread woken by nothing but entities of a busy
     * group rebuilds so the wait set leaves that group out until it is free,
     * instead of waking on them again and again.
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

        /// Wait-set rebuilds this executor requested itself.
        size_t get_rebuild_requests() const { return rebuild_requests_.load(); }

    private:
        void run() {
            while (rclcpp::ok(this->context_) && spinning.load()) {
                rclcpp::AnyExecutable any_exec;
                // Identifies the entity collection the callback was taken
                // from; every rebuild replaces it.
                std::shared_ptr<const void> collection_at_take;
                {
                    std::lock_guard<std::mutex> wait_lock{wait_mutex_};
                    if (!rclcpp::ok(this->context_) || !spinning.load()) {
                        return;
                    }
                    if (!get_next_executable(any_exec, next_exec_timeout_)) {
                        // Woken with nothing to take while a MutuallyExclusive
                        // callback runs: what woke us belongs to its busy
                        // group. Short callbacks end within a wake or two; for
                        // longer ones, leave the group out of the next wait.
                        if (exclusive_callbacks_running_.load() > 0 &&
                            ++empty_wakes_ >= kEmptyWakesBeforeExcludingBusyGroups)
                        {
                            empty_wakes_ = 0;
                            request_rebuild(false);
                        }
                        continue;
                    }
                    empty_wakes_ = 0;
                    if (any_exec.callback_group &&
                        any_exec.callback_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive)
                    {
                        ++exclusive_callbacks_running_;
                        collection_at_take = current_collection();
                    }
                }
                if (yield_before_execute_) {
                    std::this_thread::yield();
                }

                execute_any_executable(any_exec);

                if (collection_at_take) {
                    --exclusive_callbacks_running_;
                    // A rebuild while the callback ran left its group out of
                    // the wait set: rebuild now that it can be taken again.
                    if (current_collection() != collection_at_take) {
                        request_rebuild(true);
                    }
                }

                // Keep the AnyExecutable destructor from resetting the
                // group's can_be_taken_from.
                any_exec.callback_group.reset();
            }
        }

        // collect_entities() (rclcpp 28.1.x) replaces current_notify_waitable_
        // on every rebuild; the lost-interrupt test fails if that changes.
        std::shared_ptr<const void> current_collection() {
            std::lock_guard<std::mutex> guard(mutex_);
            return current_notify_waitable_;
        }

        void request_rebuild(bool interrupt) {
            ++rebuild_requests_;
            entities_need_rebuild_.store(true);
            if (interrupt) {
                interrupt_guard_condition_->trigger();
            }
        }

        static constexpr size_t kEmptyWakesBeforeExcludingBusyGroups = 2;

        std::mutex wait_mutex_;
        size_t empty_wakes_ = 0;  // guarded by wait_mutex_
        std::atomic<size_t> exclusive_callbacks_running_{0};
        std::atomic<size_t> rebuild_requests_{0};
        const size_t number_of_threads_;
        const bool yield_before_execute_;
        const std::chrono::nanoseconds next_exec_timeout_;
    };

} // namespace utils
} // namespace iii_drone
