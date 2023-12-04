#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/declare_parameter.hpp>

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
#include <shared_mutex>
#include <memory>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace configuration {

/**
 * @brief Class for handling parameters of a node. The class works in conjunction with the ConfigurationServer node.
 * The class will declare parameters with the ConfigurationServer node and will subscribe to the parameter event topic
 * in order to update the local copy of the parameters.
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

    /*
    * @brief Adds a parameter to the node.
    *
    * @param name Name of the parameter
    * 
    * @tparam T Type of the parameter
    * 
    * @return void
    */
    template<typename T>
    void DeclareParameter(const std::string & name);

    /**
     * @brief Get a parameter.
     *
     * @param name Name of the parameter
     * 
     * @return The parameter
     */
    rclcpp::Parameter GetParameter(const std::string & name) const;

    /**
     * @brief Gets multiple parameters.
     * 
     * @param names Names of the parameters
     * 
     * @return std::vector<rclcpp::Parameter> Vector of parameters
     */
    std::vector<rclcpp::Parameter> GetParameters(const std::vector<std::string> & names) const;

    /**
     * @brief Synchronizes parameters with the parameter server. Should never be necessary.
     * 
     * @param names Names of the parameters, default is empty (all parameters)
     * 
     * @return void
     */
    void SyncParameters(const std::vector<std::string> & names = {});

    /**
     * @brief Static function for getting string representation of a parameter type from template type.
     * 
     * @tparam T Type of the parameter
     * 
     * @return String representation of the parameter type
     */
    template<typename T>
    static std::string GetParameterTypeString();

    /**
     * @brief Prints all parameters to the console.
     * 
     * @return void
     */
    void PrintParameters() const;


protected:
    /*
    * @brief Reference to the handling node.
    */
    rclcpp::Node *node_;

private:
    /*
    * @brief The parameters of the node.
    */
    std::vector<rclcpp::Parameter> parameters_;

    /**
     * @brief QoS settings for the parameter event subscription.
     */
    std::unique_ptr<rclcpp::QoS> qos_ = nullptr;

    /*
    * @brief Mutex for access to the parameters vector.
    */
    mutable std::shared_mutex parameters_mutex_;

    /**
     * @brief DeclareParameter service client.
     */
    rclcpp::Client<iii_drone_interfaces::srv::DeclareParameter>::SharedPtr declare_parameter_client_;

    /**
     * @brief GetParameters service client.
     */
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;

    /**
     * @brief /parameter_events subscriber.
     */
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_events_subscriber_;

    /*
    * @brief Parameter event callback function.
    *   
    * @param parameter_evemt Parameter event
    * 
    * @return void
    */
    void parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event);

    /*
    * @brief Callback function reference called after successful parameter change.
    */
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback_;

    /**
     * @brief Sends a DeclareParameter request to the ConfigurationServer node,
     *       which will declare the parameter in the parameter server.
     * 
     * @param name Name of the parameter
     * @param type Type string representation of the parameter
     * 
     * @return bool True if the parameter was declared successfully, false otherwise
     */
    bool sendDeclareParameterRequest(
        const std::string & name,
        const std::string & type,
        std::string & message
    );

    /**
     * @brief Sends a GetParameters request to the ConfigurationServer node,
     *       which will return the parameter.
     * 
     * @param name Name of the parameter
     * @param parameter Parameter out reference
     * 
     * @return bool True if the parameter could be fetched.
     */
    bool sendGetParameterRequest(
        const std::string & name,
        rclcpp::Parameter & parameter
    );

    /**
     * @brief Sends a GetParameters request to the ConfigurationServer node,
     *       which will return the parameters.
     * 
     * @param names Names of the parameters
     * @param parameters Parameters out reference
     * 
     * @return bool True if the parameters could be fetched.
     */
    bool sendGetParametersRequest(
        const std::vector<std::string> & names,
        std::vector<rclcpp::Parameter> & parameters
    );


};

} // namespace configuration
} // namespace iii_ros2