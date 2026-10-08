#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <cstdint>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/token.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/control/reference.hpp>

/*****************************************************************************/
// Defines
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    using ReferenceCallback = std::function<Reference(const State &)>;

    class RetiredReferenceCallback final : public std::exception {
    public:
        const char * what() const noexcept override {
            return "reference callback was retired or quiesced";
        }
    };

    // A copied callable can outlive the token holder which installed it.
    // Quiescing excludes new entries while an action worker drains callbacks
    // already inside provider code; retirement is irreversible.
    class ReferenceCallbackLease {
    public:
        Reference invoke(const ReferenceCallback & callback, const State & state) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (retired_ || quiescing_) throw RetiredReferenceCallback();
                ++in_flight_;
            }
            struct Exit {
                ReferenceCallbackLease & lease;
                ~Exit() {
                    std::lock_guard<std::mutex> lock(lease.mutex_);
                    --lease.in_flight_;
                    lease.drained_.notify_all();
                }
            } exit{*this};
            return callback(state);
        }

        void requestQuiescence() {
            std::lock_guard<std::mutex> lock(mutex_);
            quiescing_ = true;
        }

        bool drained() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return in_flight_ == 0;
        }

        void waitUntilDrained() const {
            std::unique_lock<std::mutex> lock(mutex_);
            drained_.wait(lock, [this] { return in_flight_ == 0; });
        }

        bool resume() {
            std::lock_guard<std::mutex> lock(mutex_);
            if (retired_) return false;
            quiescing_ = false;
            return true;
        }

        void retire() {
            std::lock_guard<std::mutex> lock(mutex_);
            retired_ = true;
            quiescing_ = true;
        }

        bool retired() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return retired_;
        }

        bool quiescing() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return quiescing_;
        }

    private:
        mutable std::mutex mutex_;
        mutable std::condition_variable drained_;
        std::size_t in_flight_ = 0;
        bool quiescing_ = false;
        bool retired_ = false;
    };

    struct ReferenceCallbackBinding {
        ReferenceCallback callback;
        std::string reference_provider_name;
        uint64_t execution_id = 0;
        std::string request_identity;
        uint64_t revision = 0;
        std::shared_ptr<ReferenceCallbackLease> lease;
    };

    typedef struct reference_callback_struct {
        utils::Atomic<ReferenceCallbackBinding> binding;
        // Final reference publication takes stream -> publication_mutex_.
        // Replacement only takes publication_mutex_; neither path computes or
        // waits for a lease while holding either publication lock.
        mutable std::mutex publication_mutex_;

        reference_callback_struct(
            ReferenceCallback callback = nullptr,
            const std::string &reference_provider_name = ""
        ) {
            set(callback, reference_provider_name);
        }

        Reference operator()(const State &state) {
            const auto current_binding = snapshot();
            if (current_binding.callback) {
                return current_binding.callback(state);
            } else {
                throw std::runtime_error("ReferenceCallbackStruct(): Reference callback is not set.");
            }
        }

        ReferenceCallbackBinding snapshot() const {
            // Take the callable, provider, and execution identity under one
            // lock. The callable copy also keeps an in-flight invocation alive
            // while a successor replaces the binding.
            return binding.Load();
        }

        uint64_t beginExecution(
            const std::string & reference_provider_name,
            const std::string & request_identity,
            ReferenceCallback initial_callback = nullptr
        ) {
            std::lock_guard<std::mutex> lock(publication_mutex_);
            const auto previous = snapshot();
            const uint64_t execution_id = previous.execution_id + 1;
            if (previous.lease) previous.lease->retire();
            binding = makeBinding(std::move(initial_callback), reference_provider_name,
                execution_id, request_identity, previous.revision + 1);
            return execution_id;
        }

        void set(
            ReferenceCallback callback,
            const std::string & reference_provider_name,
            uint64_t execution_id,
            const std::string & request_identity
        ) {
            std::lock_guard<std::mutex> lock(publication_mutex_);
            const auto previous = snapshot();
            if (previous.lease) previous.lease->retire();
            binding = makeBinding(std::move(callback), reference_provider_name,
                execution_id, request_identity, previous.revision + 1);
        }

        // Compatibility for token-only callers. Empty identity remains
        // deliberately ineligible for native stream publication.
        uint64_t beginExecution(const std::string & reference_provider_name) {
            return beginExecution(reference_provider_name, "");
        }

        void set(
            ReferenceCallback callback,
            const std::string & reference_provider_name,
            uint64_t execution_id
        ) {
            set(
                std::move(callback),
                reference_provider_name,
                execution_id,
                snapshot().request_identity
            );
        }

        void set(ReferenceCallback callback, const std::string & reference_provider_name) {
            const auto current = snapshot();
            set(
                std::move(callback),
                reference_provider_name,
                current.execution_id,
                current.request_identity
            );
        }

        typedef std::shared_ptr<reference_callback_struct> SharedPtr;

    private:
        static ReferenceCallbackBinding makeBinding(
            ReferenceCallback callback, const std::string & provider,
            uint64_t execution_id, const std::string & request_identity,
            uint64_t revision
        ) {
            std::shared_ptr<ReferenceCallbackLease> lease;
            if (callback) {
                lease = std::make_shared<ReferenceCallbackLease>();
                callback = [lease, callback = std::move(callback)](const State & state) {
                    return lease->invoke(callback, state);
                };
            }
            return ReferenceCallbackBinding{std::move(callback), provider, execution_id,
                request_identity, revision, std::move(lease)};
        }

    } ReferenceCallbackStruct;

    typedef utils::Token<ReferenceCallbackStruct> ReferenceCallbackToken;

} // namespace maneuver
} // namespace control
} // namespace iii_drone_core
