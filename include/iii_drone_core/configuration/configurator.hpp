#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/configuration/parameter.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

/*****************************************************************************/
// Std:

#include <string>
#include <vector>
#include <mutex>
#include <memory>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_ros2 {

namespace configuration {

/*
* @brief Class for configuring the parameters of the node, including parameter event callbacks, etc.
*/
class Configurator {
public:
    /*
    * @brief Constructor
    *
    * @param node Reference to the handling node
    * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
    */
    Configurator(
        rclcpp::Node *node,
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
    );

    /*
    * @brief Constructor
    *
    * @param node Reference to the handling node
    * @param qos Quality of service for the parameter event subscription
    * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
    */
    Configurator(
        rclcpp::Node *node,
        const rclcpp::QoS &qos,
        std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
    );

    /*
    * @brief Destructor
    */
    ~Configurator();

    /**
     * @brief Starts the callbacks, called after initialization of the Constructor object to avoid deadlock.
     * 
     * @return void
     */
    void Start();

    /*
    * @brief Adds a parameter to the node.
    *
    * @param name Name of the parameter
    * @param default_value Default value of the parameter
    * @param is_constant Whether the parameter can be changed at runtime
    * @param callback function handle for validation of parameter change value with the following signature:
    *  bool callback(const rclcpp::ParameterValue & value), default is nullptr
    * 
    * @tparam T Type of the parameter
    * 
    * @return void
    */
    template<typename T>
    void DeclareParameter(
        const std::string & name, 
        const T & default_value,
        bool is_constant,
        std::function<bool(const rclcpp::ParameterValue &)> validation_callback = nullptr
    );

    /*
    * @brief Get a parameter.
    *
    * @param name Name of the parameter
    * 
    * @return The parameter
    */
    rclcpp::Parameter GetParameter(const std::string & name) const;

protected:
    /*
    * @brief Reference to the handling node.
    */
    rclcpp::Node *node_;

private:
    /*
    * @brief The parameters of the node.
    */
    std::vector<iii_ros2::configuration::Parameter> parameters_;

    /**
     * @brief QoS settings for the parameter event subscription.
     */
    std::unique_ptr<rclcpp::QoS> qos_ = nullptr;

    /*
    * @brief Mutex for the parameters vector.
    */
    std::mutex parameters_mutex_;

    /*
    * @brief Callback handle for the set parameters event.
    */
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_parameter_event_callback_handler_;

    /*
    * @brief Set parameters event callback function.
    *
    * @param parameters Vector of parameters to be set
    * 
    * @return Set parameters result
    */
    rcl_interfaces::msg::SetParametersResult setParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

    /*
    * @brief Asynchronous parameters client.
    */
    rclcpp::AsyncParametersClient::SharedPtr parameter_client_;

    /*
    * @brief Parameter event callback function.
    *   
    * @param parameter_evemt Parameter event
    * 
    * @return void
    */
    void parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event);

    /*
    * @brief Callback function called after successful parameter change.
    */
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

};

} // namespace configuration
} // namespace iii_ros2