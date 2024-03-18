#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <mutex>
#include <vector>
#include <memory>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace control {
namespace maneuver {

    /**
     * @brief Class for handling a queue of maneuvers to be executed.
     */    
    class ManeuverQueue : private std::vector<Maneuver> {
    public:
        /**
         * @brief Constructor
         * 
         * @param size Maximum queue size. If 0, the queue is unbounded.
         */
        ManeuverQueue(unsigned int size = 0);

        /**
         * @brief Pushes an item to the queue.
         * 
         * @param item Item to push
         * @param block Whether to block if the queue is full, default false
         * 
         * @return true If the item was pushed, false otherwise
         */
        bool Push(
            Maneuver item, 
            bool block=false
        );

        /**
         * @brief Pops an item from the queue.
         * 
         * @param read_item Read item
         * @param block Whether to block if the queue is empty, default false
         * 
         * @return true If the item was read, false otherwise
         */
        bool Pop(
            Maneuver &read_item, 
            bool block=false
        );

        /**
         * @brief Peeks an item from the queue.
         * 
         * @param read_item Read item
         * @param block Whether to block if the queue is empty, default false
         * 
         * @return true If the item was read, false otherwise
         */
        bool Peak(
            Maneuver &read_item, 
            bool block=false
        ) const;

        /**
         * @brief Clears the queue.
         */
        void Clear();

        /**
         * @brief Clears the queue from a specific maneuver.
         * 
         * @param maneuver Maneuver from which to clear the queue
         */
        void ClearFrom(Maneuver maneuver);

        /**
         * @brief Updates a maneuver in the queue. Replace the first occurrence of the maneuver.
         * 
         * @param maneuver Maneuver to update
         * 
         * @return true If the maneuver was updated, false otherwise
        */
        bool Update(Maneuver maneuver);

        /**
         * @brief Finds a maneuver from UUID.
         * 
         * @param uuid UUID of the maneuver
         * 
         * @return Maneuver if found, empty maneuver otherwise
         */
        Maneuver Find(const rclcpp_action::GoalUUID &uuid);

        /**
         * @brief Gets the vector of maneuvers in the queue.
         * 
         * @return Vector of maneuvers in the queue
        */
        std::vector<Maneuver> vector() const;

        /**
         * @brief Get the current size of the queue.
         * 
         * @return Current size of the queue
         */
        int size() const;

        /**
         * @brief Whether the queue is empty.
         * 
         * @return true If the queue is empty, false otherwise
         */
        bool empty() const;

        /**
         * @brief Whether the queue is full.
         * 
         * @return true If the queue is full, false otherwise
         */
        bool full() const;

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<ManeuverQueue> SharedPtr;

        /**
         * @brief Unique pointer type.
         */
        typedef std::unique_ptr<ManeuverQueue> UniquePtr;

    private:
        /**
         * @brief Maximum size of the queue
         */
        unsigned int max_size_;

        /**
         * @brief Reader mutex for the queue.
         */
        mutable std::mutex reader_mutex_;

        /**
         * @brief Writer mutex for the queue.
         */
        std::mutex writer_mutex_;

        /**
         * @brief General mutex for the queue.
         */
        mutable std::mutex mutex_;

        /**
         * @brief Condition variable for the queue not being empty.
         */
        std::condition_variable is_not_full_;

        /**
         * @brief Condition variable for the queue not being full.
         */
        mutable std::condition_variable is_not_empty_;

        /**
         * @brief Finds a maneuver in the queue by its UUID.
         * 
         * @param uuid UUID of the maneuver
         * 
         * @return Iterator to the maneuver if found, end() otherwise
         */
        std::vector<Maneuver>::iterator find(const rclcpp_action::GoalUUID &uuid);


    };

} // namespace maneuver
} // namespace control
} // namespace iii_drone
    