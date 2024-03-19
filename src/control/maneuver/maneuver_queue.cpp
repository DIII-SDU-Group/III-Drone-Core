/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/control/maneuver/maneuver_queue.hpp>

using namespace iii_drone::control::maneuver;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverQueue::ManeuverQueue(unsigned int size) { 

    max_size_ = size;

    std::vector<Maneuver>::resize(0);

}

bool ManeuverQueue::Push(
    Maneuver item, 
    bool block
) {

    std::unique_lock<std::mutex> wlck(writer_mutex_);

    bool is_full = false;

    {
        std::unique_lock<std::mutex> lck(mutex_);

        is_full = max_size_ > 0 && std::vector<Maneuver>::size() >= max_size_;
    }

    if(is_full) {

        if (!block) 
            return false;

        is_not_full_.wait(wlck);

    }

    {
        std::unique_lock<std::mutex> lck(mutex_);

        std::vector<Maneuver>::push_back(item);
    }

    is_not_empty_.notify_all();

    return true;

}

bool ManeuverQueue::Pop(
    Maneuver &read_item,
    bool block
) {

    std::unique_lock<std::mutex> lck(reader_mutex_);

    bool empty = false;

    {
        std::unique_lock<std::mutex> lck(mutex_);

        empty = std::vector<Maneuver>::empty();
    }

    if(empty) {

        if (!block) {
            return false;
        };

        is_not_empty_.wait(lck);

    }

    {
        std::unique_lock<std::mutex> lck(mutex_);

        read_item = std::vector<Maneuver>::front();

        std::vector<Maneuver>::erase(std::vector<Maneuver>::begin());
    }

    is_not_full_.notify_all();

    return true;

}

bool ManeuverQueue::Peak(
    Maneuver &read_item,
    bool block
) const {

    std::unique_lock<std::mutex> lck(reader_mutex_);

    bool empty = false;

    {
        std::unique_lock<std::mutex> lck(mutex_);

        empty = std::vector<Maneuver>::empty();
    }

    if(empty) {

        if (!block) {
            return false;
        };

        is_not_empty_.wait(lck);

    }

    {
        std::unique_lock<std::mutex> lck(mutex_);

        read_item = std::vector<Maneuver>::front();
    }

    return true;

}

void ManeuverQueue::Clear() {

    std::unique_lock<std::mutex> wlck(writer_mutex_);

    {
        std::unique_lock<std::mutex> lck(mutex_);

        std::vector<Maneuver>::clear();
    }

    is_not_full_.notify_all();

}

void ManeuverQueue::ClearFrom(Maneuver maneuver) {

    std::unique_lock<std::mutex> wlck(writer_mutex_);

    std::unique_lock<std::mutex> lck(mutex_);

    std::vector<Maneuver>::iterator it = std::find(std::vector<Maneuver>::begin(), std::vector<Maneuver>::end(), maneuver);

    if(it != std::vector<Maneuver>::end()) {

        std::vector<Maneuver>::erase(std::vector<Maneuver>::begin(), it);

        lck.unlock();

        is_not_full_.notify_all();

    }
}

bool ManeuverQueue::Update(Maneuver maneuver) {

    std::unique_lock<std::mutex> wlck(writer_mutex_);

    std::unique_lock<std::mutex> lck(mutex_);

    std::vector<Maneuver>::iterator it = find(maneuver.uuid());

    if(it != std::vector<Maneuver>::end()) {

        *it = maneuver;

        lck.unlock();

        return true;

    }

    return false;

}

Maneuver ManeuverQueue::Find(const rclcpp_action::GoalUUID &uuid) {

    std::unique_lock<std::mutex> lck(mutex_);

    std::vector<Maneuver>::iterator it = find(uuid);

    if(it != std::vector<Maneuver>::end()) {

        return *it;

    }

    return Maneuver();

}

std::vector<Maneuver>::iterator ManeuverQueue::find(const rclcpp_action::GoalUUID &uuid) {

    std::unique_lock<std::mutex> lck(mutex_);

    return std::find_if(std::vector<Maneuver>::begin(), std::vector<Maneuver>::end(), [&uuid](const Maneuver &maneuver) {
        return maneuver.uuid() == uuid;
    });

}

std::vector<Maneuver> ManeuverQueue::vector() const {

    std::unique_lock<std::mutex> rlck(reader_mutex_);

    std::unique_lock<std::mutex> lck(mutex_);

    std::vector<Maneuver> maneuvers;

    for (auto it = std::vector<Maneuver>::begin(); it != std::vector<Maneuver>::end(); ++it) {

        maneuvers.push_back(*it);

    }

    return maneuvers;

}

int ManeuverQueue::size() const {

    std::unique_lock<std::mutex> rlck(reader_mutex_);
    std::unique_lock<std::mutex> lck(mutex_);

    return std::vector<Maneuver>::size();

}

bool ManeuverQueue::empty() const {

    std::unique_lock<std::mutex> rlck(reader_mutex_);
    std::unique_lock<std::mutex> lck(mutex_);

    return std::vector<Maneuver>::empty();

}

bool ManeuverQueue::full() const {

    std::unique_lock<std::mutex> rlck(reader_mutex_);
    std::unique_lock<std::mutex> lck(mutex_);

    return max_size_ > 0 && std::vector<Maneuver>::size() >= max_size_;

}