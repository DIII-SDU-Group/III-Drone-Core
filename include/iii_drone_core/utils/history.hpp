#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <mutex>
#include <shared_mutex>
#include <vector>

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define COMPATIBLE_T_ASSERTION(T) static_assert(std::is_default_constructible<T>::value, "T must be default constructible"); \
                                 static_assert(std::is_copy_constructible<T>::value, "T must be copy constructible");

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace utils {

    /**
     * @brief History class, storing the last N values of a type.
     * 
     * @tparam T The type of the history.
     */
    template<class T>
    class History {
    public:
        /**
         * @brief Constructor.
         * 
         * @param capacity The capacity of the history, default 2.
         */
        History(const size_t capacity = 2) : capacity_(capacity) {
                
            COMPATIBLE_T_ASSERTION(T)

            values_.resize(0);
        }

        /**
         * @brief Constructor from value.
         * 
         * @param value The value.
         * @param capacity The capacity of the history, default 2.
         */
        History(
            const T & value,
            const size_t capacity = 2
        ) : capacity_(capacity) {
                    
            COMPATIBLE_T_ASSERTION(T)

            values_.resize(0);
            values_.push_back(value);
        }

        /**
         * @brief Copy constructor.
         * 
         * @param other The other history.
         */
        History(const History<T> & other) : capacity_(other.capacity()) {
                        
            COMPATIBLE_T_ASSERTION(T)

            values_.resize(0);

            std::vector<T> values = other.Load();

            for (size_t i = 0; i < values.size(); i++) {
                values_.push_back(values[i]);
            }

        }

        /**
         * @brief Destructor.
         */
        ~History() { }

        /**
         * @brief Assignment operator.
         * 
         * @param other The other history.
         * 
         * @return The history.
         */
        History<T> & operator=(const History<T> & other) {

            if (this == &other) {
                return *this;
            }

            if (capacity_ != other.capacity()) {
                throw std::runtime_error("Cannot assign history with different capacity");
            }
                
            std::vector<T> values = other.Load();

            std::unique_lock<std::shared_mutex> lock(mutex_);

            values_.resize(0);

            for (size_t i = 0; i < values.size(); i++) {
                values_.push_back(values[i]);
            }

            return *this;   
        }

        /**
         * @brief Assignment operator, stores the value as the latest value in the history.
         * 
         * @param value The value.
         * 
         * @return The history.
         */
        History<T> & operator=(const T & value) {
                    
            Store(value);

            return *this;
        }

        /**
         * @brief Assignment operator, overwrites the values.
         * 
         * @param values The values.
         * 
         * @return The atomic.
         */
        History<T> & operator=(const std::vector<T> & values) {
                    
            std::unique_lock<std::shared_mutex> lock(mutex_);

            values_.resize(0);

            if (values.size() > capacity_) {
                for (size_t i = values.size() - capacity_; i < values.size(); i++) {
                    values_.push_back(values[i]);
                }
            } else {
                for (size_t i = 0; i < values.size(); i++) {
                    values_.push_back(values[i]);
                }
            }

            return *this;
        }

        /**
         * @brief Get the values.
         * 
         * @return The values vector.
         */
        std::vector<T> Load() const {
                        
            std::shared_lock<std::shared_mutex> lock(mutex_);

            if (values_.size() == 0) {
                throw std::runtime_error("History is empty");
            }

            return values_;
        }

        /**
         * @brief Store the latest value.
         * 
         * @param value The value.
         */
        void Store(const T & value) {
                        
            std::unique_lock<std::shared_mutex> lock(mutex_);

            if (values_.size() == capacity_) {
                values_.erase(values_.begin());
            }

            values_.push_back(value);
        }

        /**
         * @brief Get the latest value.
         * 
         * @return The value.
         */
        operator T() const {

            std::shared_lock<std::shared_mutex> lock(mutex_);

            if (values_.size() == 0) {
                throw std::runtime_error("History is empty");
            }

            return values_[values_.size() - 1];
        }

        /**
         * @brief Get the latest value.
         * 
         * @return The value.
         */
        T operator()() const {
                
            std::shared_lock<std::shared_mutex> lock(mutex_);

            if (values_.size() == 0) {
                throw std::runtime_error("History is empty");
            }

            return values_[values_.size() - 1];
        }

        /**
         * @brief Store the latest value.
         * 
         * @param value The value.
         */
        void operator()(const T & value) {
                            
            std::unique_lock<std::shared_mutex> lock(mutex_);

            if (values_.size() == capacity_) {
                values_.erase(values_.begin());
            }

            values_.push_back(value);

        }

        /**
         * @brief Get the value at negative index.
         * 
         * @param index The index [-(size-1):0].
         * 
         * @return The value.
         */
        T operator[](const int index) const {
                
            std::shared_lock<std::shared_mutex> lock(mutex_);

            if (values_.size() == 0) {
                throw std::runtime_error("History is empty");
            }

            int size = values_.size();

            if (index <= -size || index > 0) {
                throw std::runtime_error("Index out of range");
            }

            int i = size - 1 + index;

            return values_[i];

        }

        /**
         * @brief Get the size.
        */
        size_t size() const {

            std::shared_lock<std::shared_mutex> lock(mutex_);

            return values_.size();
        }

        /**
         * @brief Get the capacity.
        */
        size_t capacity() const {
            return capacity_;
        }

        /**
         * @brief Returns true if the history is empty.
         * 
         * @return True if the history is empty.
         */
        bool empty() const {
            std::shared_lock<std::shared_mutex> lock(mutex_);

            return values_.empty();
        }

        /**
         * @brief Returns true if the history is full.
         * 
         * @return True if the history is full.
         */
        bool full() const {
            std::shared_lock<std::shared_mutex> lock(mutex_);

            return values_.size() == capacity_;
        }

        /**
         * @brief Clear the history.
         */
        void clear() {
            std::unique_lock<std::shared_mutex> lock(mutex_);

            values_.clear();
        }

    private:
        /**
         * @brief The values.
         */
        std::vector<T> values_;

        /**
         * @brief The mutex.
         */
        mutable std::shared_mutex mutex_;

        /**
         * @brief The capacity.
         */
        const size_t capacity_;

    };


} // namespace utils
} // namespace iii_drone