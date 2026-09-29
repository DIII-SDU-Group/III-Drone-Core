#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include <iii_drone_core/utils/callback_lifetime.hpp>

using namespace std::chrono_literals;
using iii_drone::utils::CallbackLifetime;

TEST(CallbackLifetime, RejectsCallbacksAfterClose) {
    CallbackLifetime lifetime;
    const auto token = lifetime.token();
    int calls = 0;
    auto guarded = lifetime.Guard<int>([&calls](int increment) { calls += increment; });

    EXPECT_TRUE(token.Enter().owns_lock());
    guarded(2);
    EXPECT_EQ(calls, 2);

    lifetime.Close();
    EXPECT_FALSE(token.Enter().owns_lock());
    guarded(5);
    EXPECT_EQ(calls, 2);
}

TEST(CallbackLifetime, CloseWaitsForRunningCallback) {
    auto lifetime = std::make_unique<CallbackLifetime>();
    std::atomic<bool> entered{false};
    std::atomic<bool> finished{false};

    std::thread callback([&, token = lifetime->token()]() {
        const auto alive = token.Enter();
        ASSERT_TRUE(alive.owns_lock());
        entered = true;
        std::this_thread::sleep_for(100ms);
        finished = true;
    });

    while (!entered) std::this_thread::sleep_for(1ms);
    // Destroying the owner must not return while its callback still runs.
    lifetime.reset();
    EXPECT_TRUE(finished);
    callback.join();
}

TEST(CallbackLifetime, TokenOutlivesOwner) {
    CallbackLifetime::Token token = [] {
        CallbackLifetime lifetime;
        return lifetime.token();
    }();
    EXPECT_FALSE(token.Enter().owns_lock());
}
