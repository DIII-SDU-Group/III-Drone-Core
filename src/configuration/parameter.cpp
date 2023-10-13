#include "iii_drone_core/configuration/parameter.hpp"

using namespace iii_ros2::configuration;

Parameter::Parameter(
    const std::string & name,
    const rclcpp::ParameterValue & value,
    bool is_constant,
    std::function<bool(const rclcpp::ParameterValue &)> validation_callback
) : rclcpp::Parameter(name, value),
    is_constant_(is_constant),
    validation_callback_(validation_callback) { 

    if (validation_callback_ != nullptr) {

        if (!validation_callback_(rclcpp::ParameterValue(value))) {

            throw std::invalid_argument("Parameter::Parameter: Parameter " + name + " value does not pass validation.");

        }
    }
}

bool Parameter::operator==(const Parameter & other) const {

    return this->get_name() == other.get_name();

}

bool Parameter::operator==(const rclcpp::Parameter & other) const {

    return this->get_name() == other.get_name();

}

bool Parameter::operator==(const rcl_interfaces::msg::Parameter & other) const {

    return this->get_name() == other.name;

}

bool Parameter::operator!=(const Parameter & other) const {

    return !(*this == other);

}

bool Parameter::operator!=(const rclcpp::Parameter & other) const {

    return !(*this == other);

}

bool Parameter::operator!=(const rcl_interfaces::msg::Parameter & other) const {

    return !(*this == other);

}

Parameter Parameter::UpdateValue(const rclcpp::Parameter & other) {

    if (this->get_name() == other.get_name()) {

        throw std::invalid_argument("Parameter::UpdateValue: Parameter names do not match.");

    }

    return Parameter(
        this->get_name(), 
        other.get_parameter_value(),
        this->is_constant(),
        this->validation_callback_
    );

    // rclcpp::ParameterType t = this->get_type();

    // switch (t) {

    //     case rclcpp::ParameterType::PARAMETER_BOOL:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_INTEGER:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_DOUBLE:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_STRING:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_NOT_SET:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     default:
    //         return Parameter(
    //             this->get_name(), 
    //             other.get_parameter_value(),
    //             this->is_constant(),
    //             this->validation_callback_
    //         );

    // }
}

Parameter::operator rclcpp::Parameter() const {

    rclcpp::ParameterType t = this->get_type();

    switch (t) {

        case rclcpp::ParameterType::PARAMETER_BOOL:
            return rclcpp::Parameter(this->get_name(), this->as_bool());
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return rclcpp::Parameter(this->get_name(), this->as_int());
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return rclcpp::Parameter(this->get_name(), this->as_double());
        case rclcpp::ParameterType::PARAMETER_STRING:
            return rclcpp::Parameter(this->get_name(), this->as_string());
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
            return rclcpp::Parameter(this->get_name(), this->as_byte_array());
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return rclcpp::Parameter(this->get_name(), this->as_bool_array());
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return rclcpp::Parameter(this->get_name(), this->as_integer_array());
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return rclcpp::Parameter(this->get_name(), this->as_double_array());
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return rclcpp::Parameter(this->get_name(), this->as_string_array());
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
            return rclcpp::Parameter(this->get_name(), this->as_string());
        default:
            return rclcpp::Parameter(this->get_name(), this->as_string());

    }
}

Parameter Parameter::UpdateValue(const rcl_interfaces::msg::Parameter & other) {

    if (this->get_name() == other.name) {

        throw std::invalid_argument("Parameter::UpdateValue: Parameter names do not match.");

    }

    return Parameter(
        this->get_name(), 
        rclcpp::ParameterValue(other.value),
        this->is_constant(),
        this->validation_callback_
    );

    // rclcpp::ParameterType t = this->get_type();

    // switch (t) {

    //     case rclcpp::ParameterType::PARAMETER_BOOL:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.bool_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_INTEGER:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.integer_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_DOUBLE:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.double_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_STRING:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.string_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.byte_array_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.bool_array_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.integer_array_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.double_array_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.string_array_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     case rclcpp::ParameterType::PARAMETER_NOT_SET:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.string_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );
    //     default:
    //         return Parameter(
    //             this->get_name(), 
    //             other.value.string_value,
    //             this->is_constant(),
    //             this->validation_callback_
    //         );

    // }
}

bool Parameter::ValidateValue(const rclcpp::ParameterValue & value) const {

    if (validation_callback_ == nullptr) {

        return true;

    }

    return validation_callback_(value);

}

bool Parameter::is_constant() const {

    return is_constant_;

}

// template Parameter::Parameter<bool>(
//     const std::string &,
//     const bool &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<int>(
//     const std::string &,
//     const int &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<double>(
//     const std::string &,
//     const double &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<std::string>(
//     const std::string &,
//     const std::string &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<std::vector<bool>>(
//     const std::string &,
//     const std::vector<bool> &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<std::vector<char>>(
//     const std::string &,
//     const std::vector<char> &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<std::vector<int>>(
//     const std::string &,
//     const std::vector<int> &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<std::vector<double>>(
//     const std::string &,
//     const std::vector<double> &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );

// template Parameter::Parameter<std::vector<std::string>>(
//     const std::string &,
//     const std::vector<std::string> &,
//     bool,
//     std::function<bool(const rclcpp::ParameterValue &)>
// );