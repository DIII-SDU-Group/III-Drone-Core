#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace utils {

    /**
     * @brief Ends ROS callbacks that capture an object's `this` together with
     * the object.
     *
     * rclcpp keeps a subscription, timer or pending service response alive
     * while an executor thread runs it, and destroying the owner does not
     * wait for that call. A callback that captured `this` then runs on freed
     * memory. Each callback holds a Token and runs its body only inside a
     * successful Enter(); the owner calls Close() first thing in its
     * destructor, which waits for bodies already running and rejects all
     * later ones.
     *
     * Close() must not be called from inside an entered callback of the same
     * lifetime (it would wait for itself).
     */
    class CallbackLifetime {
    private:
        struct State {
            std::shared_mutex mutex;
            bool alive = true;
        };

    public:
        class Token {
        public:
            /**
             * @brief Enters one callback body. The returned lock owns the
             * lifetime (and the body may run) only while the owner is alive.
             */
            std::shared_lock<std::shared_mutex> Enter() const {
                std::shared_lock<std::shared_mutex> lock(state_->mutex);
                if (!state_->alive) {
                    lock.unlock();
                }
                return lock;
            }

        private:
            friend class CallbackLifetime;
            explicit Token(std::shared_ptr<State> state) : state_(std::move(state)) {}
            std::shared_ptr<State> state_;
        };

        CallbackLifetime() : state_(std::make_shared<State>()) {}

        ~CallbackLifetime() {
            Close();
        }

        CallbackLifetime(const CallbackLifetime &) = delete;
        CallbackLifetime & operator=(const CallbackLifetime &) = delete;

        Token token() const {
            return Token(state_);
        }

        /**
         * @brief Wraps a callback with a fixed signature so its body runs
         * only while the owner is alive, e.g.
         * Guard<const Msg::SharedPtr>(std::bind(&Owner::onMsg, this, _1)).
         */
        template <typename... Args, typename Callback>
        std::function<void(Args...)> Guard(Callback callback) const {
            return [lifetime = token(), callback = std::move(callback)](Args... args) {
                const auto alive = lifetime.Enter();
                if (!alive.owns_lock()) return;
                callback(std::forward<Args>(args)...);
            };
        }

        /**
         * @brief Waits for running callback bodies and rejects later ones.
         */
        void Close() {
            std::unique_lock<std::shared_mutex> lock(state_->mutex);
            state_->alive = false;
        }

    private:
        std::shared_ptr<State> state_;

    };

} // namespace utils
} // namespace iii_drone
