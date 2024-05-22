#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <mutex>
#include <memory>
#include <vector>
#include <string>
#include <future>
#include <chrono>
#include <iostream>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/atomic.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace utils {

    /**
     * @brief Token class for managing access to a resource shared between threads from a master thread.
     * 
     * @tparam T Resource type
     */
    template <typename T>
    class Token {

    public:
        /**
         * @brief Copy constructor.
         * 
         * @param other The other token.
         */
        Token(const Token<T> & other) : logger_(other.logger_) {

            token_name_counter_ = other.token_name_counter_;

            // Increment token name counter
            auto it = token_name_counter_->find(other.name_);

            if (it != token_name_counter_->end()) {

                it->second++;

            } else {

                throw std::runtime_error("Token::Token(): Copy constructor: Token name not found in token name counter.");

            }
            
            resource_mutex_ = other.resource_mutex_;
            resource_local_lock_ = std::unique_lock<std::mutex>(*resource_mutex_, std::defer_lock);
            resource_ = other.resource_;

            name_ = other.name_;

            slaves_mutex_ = other.slaves_mutex_;
            slaves_ = other.slaves_;

            token_holder_ = other.token_holder_;

            slaves_request_queue_mutex_ = other.slaves_request_queue_mutex_;
            slaves_request_queue_ = other.slaves_request_queue_;

            master_acquired_token_callback_ = other.master_acquired_token_callback_;
            
        }

        /**
         * @brief Constructor from shared ptr.
         * 
         * @param resource Resource
         * @param master_acquired_token_callback Callback to be called when master acquires token
         * @param logger Logger,
         * @param name Name, default is "master"
         */
        Token(
            std::shared_ptr<T> resource,
            std::function<void()> master_acquired_token_callback,
            rclcpp::Logger logger,
            const std::string & name = "master"
        ) : logger_(logger) {

            token_name_counter_ = std::make_shared<std::map<std::string, unsigned int>>();

            token_name_counter_->insert(std::make_pair(name, 1));

            resource_mutex_ = std::make_shared<std::mutex>();
            resource_local_lock_ = std::unique_lock<std::mutex>(*resource_mutex_);
            resource_ = resource;

            name_ = name;

            slaves_mutex_ = std::make_shared<std::mutex>();
            slaves_ = std::make_shared<std::vector<std::string>>();

            token_holder_ = std::make_shared<iii_drone::utils::Atomic<std::string>>("master");

            slaves_request_queue_mutex_ = std::make_shared<std::mutex>();
            slaves_request_queue_ = std::make_shared<std::vector<slave_request_t>>();

            master_acquired_token_callback_ = master_acquired_token_callback;
        
        }

        /**
         * @brief Destructor
         */
        ~Token() {

            if (rclcpp::ok()) {

                RCLCPP_DEBUG(logger_, "Token::~Token(): Destroying token %s", name_.c_str());

            }

            if (is_master()) {

                std::unique_lock<std::mutex> lock(*slaves_mutex_);

                if (!slaves_->empty()) {

                    std::cerr << "Master destroyed with slaves still alive." << std::endl;

                }

                lock.unlock();

            } else {

                std::unique_lock<std::mutex> request_lock(*slaves_request_queue_mutex_);

                // Remove any request from slave
                auto it1 = std::find_if(
                    slaves_request_queue_->begin(),
                    slaves_request_queue_->end(),
                    [this](const slave_request_t & slave_request_queue_item) {
                        return slave_request_queue_item.slave_name == name_;
                    }
                );

                if (it1 != slaves_request_queue_->end()) {

                    slaves_request_queue_->erase(it1);

                }

                request_lock.unlock();

                std::unique_lock<std::mutex> slaves_lock(*slaves_mutex_);

                // Decrease token name counter
                bool remove_name = false;

                auto it2 = token_name_counter_->find(name_);

                if (it2 != token_name_counter_->end()) {

                    if (it2->second == 1) {

                        remove_name = true;

                        token_name_counter_->erase(it2);

                    } else {

                        it2->second--;

                    }

                } else {

                    std::cerr << "Slave destroyed but not in token name counter.";

                }

                // Remove slave from master
                auto it3 = std::find(slaves_->begin(), slaves_->end(), name_);

                if (it3 != slaves_->end()) {

                    if (remove_name) {

                        if (rclcpp::ok()) {

                            RCLCPP_DEBUG(logger_, "Token::~Token(): Token name counter reached 0, removing slave %s from master's slave list", name_.c_str());

                        }

                        slaves_->erase(it3);

                    }

                } else {

                    std::cerr << "Slave destroyed but not in master's slave list.";

                }

                slaves_lock.unlock();

                if (has_token()) {

                    Release();

                }
            }
        }

        /**
         * @brief Creates a slave handle.
         * 
         * @param slave_name Slave name
         * 
         * @return Slave handle
         */
        Token<T> CreateSlaveHandle(const std::string & slave_name) {

            RCLCPP_DEBUG(logger_, "Token::CreateSlaveHandle(): Creating slave handle %s", slave_name.c_str());
            
            std::unique_lock<std::mutex> lock(*slaves_mutex_);

            if (!is_master()) {

                throw std::runtime_error("Only master can create slave tokens.");

            }

            if (slave_name == "master") {

                throw std::runtime_error("Slave name cannot be \"master\".");

            }

            if (std::find(slaves_->begin(), slaves_->end(), slave_name) != slaves_->end()) {

                throw std::runtime_error("Slave name already exists.");

            }

            Token<T> slave_handle(
                resource_,
                master_acquired_token_callback_,
                logger_
            );

            slave_handle.resource_mutex_ = resource_mutex_;
            slave_handle.resource_local_lock_ = std::unique_lock<std::mutex>(*resource_mutex_, std::defer_lock);
            slave_handle.resource_ = resource_;

            slave_handle.name_ = slave_name;

            slave_handle.token_name_counter_ = token_name_counter_;

            auto it = token_name_counter_->find(slave_name);

            if (it != token_name_counter_->end()) {

                throw std::runtime_error("Slave name already exists.");

            } else {

                token_name_counter_->insert(std::make_pair(slave_name, 1));

            }

            slave_handle.slaves_mutex_ = slaves_mutex_;
            slave_handle.slaves_ = slaves_;

            RCLCPP_DEBUG(logger_, "Token::CreateSlaveHandle(): Adding slave %s to slave list", slave_name.c_str());

            slave_handle.slaves_->push_back(slave_name);

            RCLCPP_DEBUG(logger_, "Token::CreateSlaveHandle(): Verifying slave %s in slave list", slave_name.c_str());

            if (std::find(slaves_->begin(), slaves_->end(), slave_name) == slaves_->end()) {

                throw std::runtime_error("Slave " + slave_name + " was not added to slave list.");

            }

            std::string msg = "Token::CreateSlaveHandle(): Slave list: ";
            for (auto & slave : *slaves_) {
                msg += "\n" + slave + ", ";
            }
            // Remove last comma
            msg.pop_back();
            msg.pop_back();

            RCLCPP_DEBUG(logger_, msg.c_str());

            slave_handle.token_holder_ = token_holder_;

            slave_handle.slaves_request_queue_mutex_ = slaves_request_queue_mutex_;
            slave_handle.slaves_request_queue_ = slaves_request_queue_;

            slave_handle.master_acquired_token_callback_ = std::bind(
                &Token<T>::masterAcquiredTokenInternalCallback, 
                this
            );

            slave_handle.logger_ = logger_;
            
            return slave_handle;

        }

        /**
         * @brief Request to acquire the token, to be called by a slave.
         * 
         * @param timeout_ms Timeout in milliseconds, 0 for infinite timeout
         * 
         * @return True if token was acquired, false otherwise
         */
        bool Acquire(
            unsigned int timeout_ms = 0
        ) {

            RCLCPP_DEBUG(logger_, "Token::Acquire(): Acquiring token by slave %s", name_.c_str());

            if (is_master()) {

                throw std::runtime_error("Master cannot request token.");

            }

            if (has_token()) {

                RCLCPP_DEBUG(logger_, "Token::Acquire(): Already has token, returning true.");

                return true;

            }

            std::unique_lock<std::mutex> lock(*slaves_request_queue_mutex_);

            // Check if slave has already requested token:
            auto it1 = std::find_if(
                slaves_request_queue_->begin(),
                slaves_request_queue_->end(),
                [this](const slave_request_t & slave_request_queue_item) {
                    return slave_request_queue_item.slave_name == name_;
                }
            );

            if (it1 != slaves_request_queue_->end()) {

                throw std::runtime_error("Slave has already requested token.");

            }

            RCLCPP_DEBUG(logger_, "Token::Acquire(): Sending slave request.");

            slave_request_t slave_request {
                name_, 
                std::make_shared<std::promise<bool>>()
            };

            slaves_request_queue_->push_back(slave_request);

            auto acquired_token_future = slave_request.acquired_token_promise->get_future();

            lock.unlock();

            if (timeout_ms == 0) {

                acquired_token_future.wait();

                if (acquired_token_future.get()) {

                    if (!has_token()) {

                        throw std::runtime_error("Slave acquired token but does not have it.");

                    }

                    if (!resource_local_lock_.try_lock()) {

                        throw std::runtime_error("Slave acquired token but cannot lock resource mutex.");

                    }

                    RCLCPP_DEBUG(logger_, "Token::Acquire(): Acquired token, returning true.");

                    return true;

                }

            } else {

                if (acquired_token_future.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {

                    if (acquired_token_future.get()) {

                        if (!has_token()) {

                            throw std::runtime_error("Slave acquired token but does not have it.");

                        }

                        if (!resource_local_lock_.try_lock()) {

                            throw std::runtime_error("Slave acquired token but cannot lock resource mutex.");

                        }

                        RCLCPP_DEBUG(logger_, "Token::Acquire(): Acquired token, returning true.");

                        return true;

                    }
                }
            }

            RCLCPP_DEBUG(logger_, "Token::Acquire(): Did not acquire token, erasing request and returning false.");

            lock.lock();

            auto it2 = std::find_if(
                slaves_request_queue_->begin(),
                slaves_request_queue_->end(),
                [&slave_request](const slave_request_t & slave_request_queue_item) {
                    return slave_request_queue_item.slave_name == slave_request.slave_name;
                }
            );

            if (it2 != slaves_request_queue_->end()) {

                slaves_request_queue_->erase(it2);

            }

            return false;

        }

        /**
         * @brief Releases the token, to be called by a slave.
         */
        void Release() {

            RCLCPP_DEBUG(logger_, "Token::Release(): Release token");

            if (is_master()) {

                throw std::runtime_error("Master cannot release token.");

            }

            if (!has_token()) {

                throw std::runtime_error("Slave does not have token.");

            }

            resource_local_lock_.unlock();

            (*token_holder_) = "master";

            master_acquired_token_callback_();
            
        }

        /**
         * @brief Gives the token to a slave, to be called by the master.
         * 
         * @param slave_name Slave name
         */
        void Give(const std::string & slave_name) {

            RCLCPP_DEBUG(logger_, "Token::Give(): Give token to slave %s", slave_name.c_str());

            answerSlaveAcquisitionRequest(
                slave_name, 
                true
            );

        }

        /**
         * @brief Denies the token to a slave, to be called by the master.
         * 
         * @param slave_name Slave name
         */
        void Deny(const std::string & slave_name) {

            answerSlaveAcquisitionRequest(
                slave_name, 
                false
            );

        }

        /**
         * @brief Denies all pending token requests, to be called by the master.
         */
        void DenyAll() {

            if (!is_master()) {

                throw std::runtime_error("Only master can give token.");

            }

            std::unique_lock<std::mutex> lock(*slaves_request_queue_mutex_);

            for (auto & slave_request_queue_item : *slaves_request_queue_) {

                slave_request_queue_item.acquired_token_promise->set_value(false);

            }

            slaves_request_queue_->clear();

        }

        /**
         * @brief Get the resource.
         * 
         * @return Resource
         */
        T & resource() const {

            if (!has_token()) {

                throw std::runtime_error("Thread does not have token.");

            }

            if (!resource_local_lock_.owns_lock()) {

                throw std::runtime_error("Thread does not own resource local lock.");

            }

            return *resource_;

        }

        /**
         * @brief Get all slave names.
         * 
         * @return Slave names
         */
        std::vector<std::string> slaves() const {

            std::unique_lock<std::mutex> lock(slaves_mutex_);

            return *slaves_;

        }

        /**
         * @brief Get the token holder name.
         * 
         * @return Token holder name
         */
        std::string token_holder() const {

            return token_holder_->Load();

        }

        /**
         * @brief Whether the token is held by the calling thread.
         */
        bool has_token() const {

            return token_holder_->Load() == name_;

        }

        /**
         * @brief Whether the token is held by a thread.
         * 
         * @param name Name
         */
        bool has_token(const std::string & name) const {

            return token_holder_->Load() == name;

        }

        /**
         * @brief Whether the token is held by the master.
         */
        bool master_has_token() const {

            return token_holder_->Load() == "master";

        }

        /**
         * @brief Whether a slave has requested the token.
         * 
         * @param slave_name Slave name
         */
        bool has_requested_token(const std::string & slave_name) const {

            std::unique_lock<std::mutex> lock(*slaves_request_queue_mutex_);

            auto it = std::find_if(
                slaves_request_queue_->begin(),
                slaves_request_queue_->end(),
                [this, &slave_name](const slave_request_t & slave_request_queue_item) {
                    return slave_request_queue_item.slave_name == slave_name;
                }
            );

            return it != slaves_request_queue_->end();

        }

        /**
         * @brief Slaves that have requested the token.
         */
        std::vector<std::string> requested_token_slaves() const {

            std::unique_lock<std::mutex> lock(slaves_request_queue_mutex_);

            std::vector<std::string> requested_token_slaves;

            for (auto & slave_request_queue_item : *slaves_request_queue_) {

                requested_token_slaves.push_back(slave_request_queue_item.slave_name);

            }

            return requested_token_slaves;

        }

        /**
         * @brief Assignment operator.
         */
        Token<T> & operator=(const T & rhs) {

            if (!has_token()) {

                throw std::runtime_error("Thread does not have token.");

            }

            if (!resource_local_lock_.owns_lock()) {

                throw std::runtime_error("Thread does not own resource local lock.");

            }

            *resource_ = rhs;

            return *this;

        }

        /**
         * @brief Shared pointer type.
         */
        typedef std::shared_ptr<Token<T>> SharedPtr;

    private:
        /**
         * @brief The logger.
         */
        rclcpp::Logger logger_;

        /**
         * @brief Token name counter map.
         */
        std::shared_ptr<std::map<std::string, unsigned int>> token_name_counter_;

        /**
         * @brief Answers a slave acquisition request
         * 
         * @param slave_name Slave name
         * @param give Whether to give the token to the slave
         */
        void answerSlaveAcquisitionRequest(
            const std::string & slave_name,
            bool give
        ) {

            if (!is_master()) {

                throw std::runtime_error("Only master can give token.");

            }

            if (slave_name == "master") {

                throw std::runtime_error("Slave name cannot be \"master\".");

            }

            std::unique_lock<std::mutex> slaves_lock(*slaves_mutex_);

            if (std::find(slaves_->begin(), slaves_->end(), slave_name) == slaves_->end()) {

                std::string msg = "Slave name does not exist: " + slave_name + ".\n";
                msg += "Available slaves: ";
                for (auto & slave : *slaves_) {
                    msg += "\n" + slave + ", ";
                }

                throw std::runtime_error(msg);

            }

            slaves_lock.unlock();

            if (give && !has_token()) {

                throw std::runtime_error("Master does not have token to give.");

            }

            std::unique_lock<std::mutex> slaves_request_queue_lock(*slaves_request_queue_mutex_);

            auto it = std::find_if(
                slaves_request_queue_->begin(),
                slaves_request_queue_->end(),
                [this, &slave_name](const slave_request_t & slave_request_queue_item) {
                    return slave_request_queue_item.slave_name == slave_name;
                }
            );

            if (it == slaves_request_queue_->end()) {

                throw std::runtime_error("Slave has not requested token.");

            }

            auto acquired_token_promise = it->acquired_token_promise;

            slaves_request_queue_->erase(it);

            slaves_request_queue_lock.unlock();

            (*token_holder_) = slave_name;

            if (give) {

                resource_local_lock_.unlock();

            }

            acquired_token_promise->set_value(give);

        }

        /**
         * @brief Resource mutex
         */
        std::shared_ptr<std::mutex> resource_mutex_;

        /**
         * @brief Resource local lock.
         */
        std::unique_lock<std::mutex> resource_local_lock_;

        /**
         * @brief Resource
         */
        std::shared_ptr<T> resource_;

        /**
         * @brief Is master
         */
        bool is_master() const {

            return name_ == "master";

        }

        /**
         * @brief Handle name
         */
        std::string name_;

        /**
         * @brief Slaves mutex
         */
        std::shared_ptr<std::mutex> slaves_mutex_;

        /**
         * @brief Slaves vector
         */
        std::shared_ptr<std::vector<std::string>> slaves_;

        /**
         * @brief Name of thread that has the token
         */
        iii_drone::utils::Atomic<std::string>::SharedPtr token_holder_;

        /**
         * @brief Slave request struct
         */
        struct slave_request_t {

            /**
             * @brief Slave name
             */
            std::string slave_name;

            /**
             * @brief Acquired token promise
             */
            std::shared_ptr<std::promise<bool>> acquired_token_promise;
        
        };

        /**
         * @brief Slaves request queue mutex
         */
        std::shared_ptr<std::mutex> slaves_request_queue_mutex_;

        /**
         * @brief Slaves request queue
         */
        std::shared_ptr<std::vector<slave_request_t>> slaves_request_queue_;

        /**
         * @brief Callback executed when master reacquires token from a slave
         */
        std::function<void()> master_acquired_token_callback_;

        /**
         * @brief Master acquired token internal callback.
         */
        void masterAcquiredTokenInternalCallback() {

            if (!resource_local_lock_.try_lock()) {

                throw std::runtime_error("Master acquired token but cannot lock resource mutex.");

            }

            if (master_acquired_token_callback_) {

                master_acquired_token_callback_();

            }
        }

    };

} // namespace utils
} // namespace iii_drone