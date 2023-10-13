#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_ros2 {

namespace configuration {

/*
* @brief Parameter class, adding constant parameter functionality to the ROS2 parameter class.
* 
* @tparam T Type of the parameter value
*/
class Parameter : public rclcpp::Parameter {
    public:
        // /*
        // * @brief Constructor from rclcpp:Parameter
        // *
        // * @param name Name of the parameter
        // * @param parameter rclcpp::Parameter object containing value
        // * @param is_constant Whether the parameter can be changed at runtime
        // * @param validation_callback Callback function for validating the parameter value upon change
        // */
        // Parameter(
        //     const std::string & name,
        //     const rclcpp::Parameter & parameter,
        //     bool is_constant,
        //     std::function<bool(const rclcpp::ParameterValue &)> validation_callback = nullptr
        // );

        /*
        * @brief Constructor
        *
        * @param name Name of the parameter
        * @param value Value of the parameter
        * @param is_constant Whether the parameter can be changed at runtime
        * @param validation_callback Callback function for validating the parameter value upon change
        */
        Parameter(
            const std::string & name,
            const rclcpp::ParameterValue & value,
            bool is_constant,
            std::function<bool(const rclcpp::ParameterValue &)> validation_callback = nullptr
        );

        // /*
        // * @brief Constructor from rcl_interfaces::msg::Parameter
        // *
        // * @param name Name of the parameter
        // * @param parameter rcl_interfaces::msg::Parameter object containing value
        // * @param is_constant Whether the parameter can be changed at runtime
        // * @param validation_callback Callback function for validating the parameter value upon change
        // */
        // Parameter(
        //     const std::string & name,
        //     const rcl_interfaces::msg::Parameter & parameter,
        //     bool is_constant,
        //     std::function<bool(const rclcpp::ParameterValue &)> validation_callback = nullptr
        // );

        /*
        * @brief Update value of parameter from other rclcpp::Parameter.
        * 
        * @param other rclcpp::Parameter to update from
        * 
        * @return Updated parameter
        */
        Parameter UpdateValue(const rclcpp::Parameter & other);

        /*
        * @brief Update value of parameter from other rcl_interfaces::msg::Parameter.
        * 
        * @param other rcl_interfaces::msg::Parameter to update from
        * 
        * @return Updated parameter
        */
        Parameter UpdateValue(const rcl_interfaces::msg::Parameter & other);

        /*
        * @brief Checks whether new value is valid by calling the validation callback if it is defined.
        *
        * @param new_value New value to be validated
        * 
        * @return True if the new value is valid or the validation callback is not defined, false otherwise
        */
        bool ValidateValue(const rclcpp::ParameterValue & new_value) const;

        /*
        * @brief Whether the parameter can be changed at runtime
        */
        bool is_constant() const;

        /*
        * @brief == operator overload, compares only the name of the parameter.
        * 
        * @param other Parameter to compare to
        */
        bool operator==(const Parameter & other) const;

        /*
        * @brief == operator overload, compares only the name of the parameter.
        * 
        * @param other rclcpp::Parameter to compare to
        */
        bool operator==(const rclcpp::Parameter & other) const;

        /*
        * @brief == operator overload, compares only the name of the parameter.
        * 
        * @param other rcl_interfaces::msg::Parameter to compare to
        */
        bool operator==(const rcl_interfaces::msg::Parameter & other) const;

        /*
        * @brief != operator overload, compares only the name of the parameter.
        */
        bool operator!=(const Parameter & other) const;

        /*
        * @brief != operator overload, compares only the name of the parameter.
        */
        bool operator!=(const rclcpp::Parameter & other) const;

        /*
        * @brief != operator overload, compares only the name of the parameter.
        */
        bool operator!=(const rcl_interfaces::msg::Parameter & other) const;

        /*
        * @brief rclcpp::Parameter typecast operator overload.
        */
        operator rclcpp::Parameter() const;

    private:
        /*
        * @brief Whether the parameter can be changed at runtime
        */
        bool is_constant_;

        /*
        * @brief Callback function for validating the parameter value upon change
        */
        std::function<bool(const rclcpp::ParameterValue &)> validation_callback_;

};

} // namespace configuration
} // namespace iii_ros2
