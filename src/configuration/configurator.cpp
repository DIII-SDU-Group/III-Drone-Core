#include "iii_drone_core/configuration/configurator.hpp"

using namespace iii_ros2::configuration;

Configurator::Configurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

}

Configurator::Configurator(
    rclcpp::Node *node,
    const rclcpp::QoS &qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

    qos_ = std::make_unique<rclcpp::QoS>(qos);

}

Configurator::~Configurator() {

    set_parameter_event_callback_handler_.reset();
    parameter_client_.reset();

}

void Configurator::Start() {

    set_parameter_event_callback_handler_ = node_->add_on_set_parameters_callback(
        std::bind(
            &Configurator::setParametersCallback, 
            this, 
            std::placeholders::_1
        )
    );

    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_);

    if (qos_ == nullptr) {

        parameter_client_->on_parameter_event(
            std::bind(
                &Configurator::parameterEventCallback, 
                this, 
                std::placeholders::_1
            )
        );

    } else {

        parameter_client_->on_parameter_event(
            std::bind(
                &Configurator::parameterEventCallback, 
                this, 
                std::placeholders::_1
            ),
            (const rclcpp::QoS &)(*qos_)
        );

    }
}

template <typename T>
void Configurator::DeclareParameter(
    const std::string & name,
    const T & default_value,
    bool is_constant,
    std::function<bool(const rclcpp::ParameterValue &)> validation_callback
) {

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::DeclareParameter(): Declaring parameter %s", name.c_str());

    std::lock_guard<std::mutex> lock(parameters_mutex_);

    // Add parameter to the node:
    node_->declare_parameter<T>(
        name,
        default_value
    );

    // Get parameter value:
    rclcpp::ParameterValue value = node_->get_parameter(name).get_parameter_value();

    // Create parameter:
    Parameter parameter(
        name,
        value,
        is_constant,
        validation_callback
    );

    // Add parameter to the list of parameters:
    parameters_.push_back(parameter);

}

rclcpp::Parameter Configurator::GetParameter(const std::string & name) const {

    // std::lock_guard<std::mutex> lock(parameters_mutex_);

    // Search for the parameter:
    for (auto & p : parameters_) {

        if (p.get_name() == name) {

            return ((rclcpp::Parameter)p);

        }
    }

    throw std::runtime_error("Parameter " + name + " does not exist.");

}

rcl_interfaces::msg::SetParametersResult Configurator::setParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters
) {

    std::lock_guard<std::mutex> lock(parameters_mutex_);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Check if the parameters are valid:
    for (auto & p : parameters) {

        // Search for the parameter:
        for (auto & parameter : parameters_) {

            if (parameter == p) {

                // Check if the parameter is constant:
                if (parameter.is_constant()) {

                    result.successful = false;
                    result.reason = "Parameter " + parameter.get_name() + " can not be changed at runtime.";

                    RCLCPP_WARN(
                        node_->get_logger(),
                        result.reason.c_str()
                    );

                    return result;

                }

                // Check if the parameter is valid:
                if (!parameter.ValidateValue(p.get_parameter_value())) {

                    result.successful = false;
                    result.reason = "Parameter " + parameter.get_name() + " value is not valid.";

                    RCLCPP_WARN(
                        node_->get_logger(),
                        result.reason.c_str()
                    );

                    return result;

                }

                break;

            }
        }
    }

    return result;

}

void Configurator::parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event) {

    std::lock_guard<std::mutex> lock(parameters_mutex_);

    // Search for the parameter:
    for (auto & p : parameter_event.changed_parameters) {

        // Check if the parameter is in the list of parameters:
        for (int i = 0; i < parameters_.size(); i++) {

            Parameter & parameter = parameters_[i];

            if (parameter == p) {

                if (parameter.is_constant()) {

                    std::string error = "Parameter " + parameter.get_name() + " can not be changed at runtime but was changed anyways.";

                    RCLCPP_FATAL(
                        node_->get_logger(),
                        error.c_str()
                    );

                    throw std::runtime_error(error);

                }

                // Check if the parameter is valid:
                if (!parameter.ValidateValue(rclcpp::ParameterValue(p.value))) {

                    std::string error = "Parameter " + parameter.get_name() + " value is not valid but was set anyways.";    

                    RCLCPP_FATAL(
                        node_->get_logger(),
                        error.c_str()
                    );

                    throw std::runtime_error(error);

                }

                parameters_[i] = parameter.UpdateValue(p);

                if (after_parameter_change_callback_ != nullptr) {

                    after_parameter_change_callback_(parameters_[i]);

                }

                break;

            }
        }
    }
}

template void Configurator::DeclareParameter<bool>(
    const std::string &, 
    const bool &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<int>(
    const std::string &, 
    const int &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<double>(
    const std::string &, 
    const double &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<std::string>(
    const std::string &, 
    const std::string &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<std::vector<bool>>(
    const std::string &, 
    const std::vector<bool> &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<std::vector<uint8_t>>(
    const std::string &, 
    const std::vector<uint8_t> &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<std::vector<int>>(
    const std::string &, 
    const std::vector<int> &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<std::vector<double>>(
    const std::string &, 
    const std::vector<double> &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);

template void Configurator::DeclareParameter<std::vector<std::string>>(
    const std::string &, 
    const std::vector<std::string> &, 
    bool, 
    std::function<bool(const rclcpp::ParameterValue &)>
);