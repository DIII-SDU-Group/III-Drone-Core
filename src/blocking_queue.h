#include <mutex>
#include <queue>
#include <condition_variable>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

template <class T> class BlockingQueue: public std::queue<T> {
public:
    BlockingQueue(int size) {

        max_size_ = size;

    }

    bool Push(T item, bool block) {

        std::unique_lock<std::mutex> wlck(writer_mutex_);

        if(Full()) {

            if (!block) 
                return false;

            is_not_full_.wait(wlck);

        }

        std::queue<T>::push(item);

        is_not_empty_.notify_all();

        return true;

    }

    bool Pop(T &read_item, bool block) {

        return readItem(read_item, block, true);

    }

    bool Peak(T &read_item, bool block) {

        return readItem(read_item, block, false);

    }

    bool PopSpecificItem(T compare_item) {

        if (std::queue<T>::empty())
            return false;

        T read_item = std::queue<T>::front();

        if (read_item == compare_item) {

            std::queue<T>::pop();
            
            return true;

        } else return false;

    }

    bool Empty() {

        return std::queue<T>::empty();

    }

    bool Full(){

        return std::queue<T>::size() >= max_size_;

    }

private:
    int max_size_;
    std::mutex reader_mutex_;
    std::mutex writer_mutex_;
    std::condition_variable is_not_full_;
    std::condition_variable is_not_empty_;

    bool readItem(T &read_item, bool block, bool pop) {

        std::unique_lock<std::mutex> lck(reader_mutex_);

        while(Empty()) {

            if (!block) {
                return false;
            };

            is_not_empty_.wait(lck);

        }

        read_item = std::queue<T>::front();

        if (pop)
            std::queue<T>::pop();

        if(!Full())
            is_not_full_.notify_all();

        return true;

    }
};