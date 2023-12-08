/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/utils/timestamp.hpp>

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

using namespace iii_drone::utils;

Timestamp::Timestamp() {
    Update();
}

Timestamp::Timestamp(const rclcpp::Time & stamp) {
    stamp_ = stamp;
}

void Timestamp::Update() {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    stamp_ = rclcpp::Clock().now();
}

Timestamp & Timestamp::operator=(const rclcpp::Time & stamp) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    stamp_ = stamp;

    return *this;
}

Timestamp & Timestamp::operator=(const Timestamp & timestamp) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    stamp_ = timestamp;

    return *this;
}

Timestamp::operator rclcpp::Time() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    return stamp_;
}