#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <mutex>
#include <shared_mutex>

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

    template<class T>
    class Atomic {
    public:
        /**
         * @brief Constructor.
         */
        Atomic() {
                
            COMPATIBLE_T_ASSERTION(T)

            value_ = T();
        }

        /**
         * @brief Constructor from value.
         * 
         * @param value The value.
         */
        Atomic(const T & value) {
                    
            COMPATIBLE_T_ASSERTION(T)

            value_ = value;
        }

        /**
         * @brief Copy constructor.
         * 
         * @param other The other atomic.
         */
        Atomic(const Atomic<T> & other) {
                        
            COMPATIBLE_T_ASSERTION(T)

            value_ = other.Load();
        }

        /**
         * @brief Destructor.
         */
        ~Atomic() { }

        /**
         * @brief Assignment operator.
         * 
         * @param other The other atomic.
         * 
         * @return The atomic.
         */
        Atomic<T> & operator=(const Atomic<T> & other) {
                
            T value = other.Load();

            std::unique_lock<std::shared_mutex> lock(mutex_);

            value_ = value;

            return *this;   
        }

        /**
         * @brief Assignment operator.
         * 
         * @param value The value.
         * 
         * @return The atomic.
         */
        Atomic<T> & operator=(const T & value) {
                    
            std::unique_lock<std::shared_mutex> lock(mutex_);

            value_ = value;

            return *this;
        }

        /**
         * @brief Get the value.
         * 
         * @return The value.
         */
        T Load() const {
                        
            std::shared_lock<std::shared_mutex> lock(mutex_);

            return value_;
        }

        /**
         * @brief Set the value.
         * 
         * @param value The value.
         */
        void Store(const T & value) {
                        
            std::unique_lock<std::shared_mutex> lock(mutex_);

            value_ = value;
        }

        /**
         * @brief Get the value.
         * 
         * @return The value.
         */
        operator T() const {

            std::shared_lock<std::shared_mutex> lock(mutex_);

            return value_;
        }

        /**
         * @brief Get the value.
         * 
         * @return The value.
         */
        T operator()() const {
                
            std::shared_lock<std::shared_mutex> lock(mutex_);

            return value_;  
        }

        /**
         * @brief Set the value.
         * 
         * @param value The value.
         */
        void operator()(const T & value) {
                            
            std::unique_lock<std::shared_mutex> lock(mutex_);

            value_ = value;

        }

        /**
         * @brief -> operator overload.
         * 
         * @return Pointer to the object.
         */
        T * operator->() {
                
            std::unique_lock<std::shared_mutex> lock(mutex_);

            return &value_;

        }

        /** 
         * @brief Dereference operator overload.
         * 
         * @return Reference to the object.
         */
        T & operator*() {
                
            std::unique_lock<std::shared_mutex> lock(mutex_);

            return value_;
        }

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<Atomic<T>> SharedPtr;

    private:
        /**
         * @brief The value.
         */
        T value_;

        /**
         * @brief The mutex.
         */
        mutable std::shared_mutex mutex_;

    };

} // namespace utils
} // namespace iii_drone