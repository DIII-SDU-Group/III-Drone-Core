/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/configuration/configurator.hpp"

using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

Configurator::Configurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

    declare_parameter_client_ = node_->create_client<iii_drone_interfaces::srv::DeclareParameter>("/configuration/configuration_server/declare_parameter");
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/configuration/configuration_server/configuration_server/get_parameters");

    parameter_events_subscriber_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        10,
        std::bind(
            &Configurator::parameterEventCallback, 
            this, 
            std::placeholders::_1
        )
    );

}

Configurator::Configurator(
    rclcpp::Node *node,
    const rclcpp::QoS &qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : after_parameter_change_callback_(after_parameter_change_callback) {

    RCLCPP_DEBUG(node->get_logger(), "Configurator::Configurator(): Initializing configurator");

    node_ = node;

    qos_ = std::make_unique<rclcpp::QoS>(qos);

    declare_parameter_client_ = node_->create_client<iii_drone_interfaces::srv::DeclareParameter>("/configuration/configuration_server/declare_parameter");
    get_parameters_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>("/configuration/configuration_server/configuration_server/get_parameters");

    parameter_events_subscriber_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        *qos_,
        std::bind(
            &Configurator::parameterEventCallback, 
            this, 
            std::placeholders::_1
        )
    );


}

Configurator::~Configurator() { }

template <typename T>
void Configurator::DeclareParameter(const std::string & name) {

    RCLCPP_DEBUG(node_->get_logger(), "Configurator::DeclareParameter(): Declaring parameter %s", name.c_str());

    // std::unique_lock<std::shared_mutex> lock(parameters_mutex_);

    // Check if parameter is already declared:
    for (auto & p : parameters_) {

        if (p.get_name() == name) {

            RCLCPP_WARN(
                node_->get_logger(),
                "Configurator::DeclareParameter(): Parameter %s already declared.",
                name.c_str()
            );

            return;

        }
    }

    // Send DeclareParameter request:
    std::string message;

    if (!sendDeclareParameterRequest(
            name,
            GetParameterTypeString<T>(),
            message
        )
    ) {

        std::string fatal_message = "Configurator::DeclareParameter(): Failed to declare parameter " + name + " with error message " + message + ".";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Get parameter value:
    rclcpp::Parameter parameter;

    if (!sendGetParameterRequest(
            name,
            parameter
        )
    ) {

        std::string fatal_message = "Configurator::DeclareParameter(): Failed to get parameter " + name + ".";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }


    // Add parameter to the list of parameters:
    parameters_.push_back(parameter);


    // lock.unlock();

}

rclcpp::Parameter Configurator::GetParameter(const std::string & name) const {

    std::vector<rclcpp::Parameter> parameters = GetParameters({name});

    return parameters[0];

}

std::vector<rclcpp::Parameter> Configurator::GetParameters(const std::vector<std::string> & names) const {

    // std::shared_lock<std::shared_mutex> lock(parameters_mutex_);

    std::vector<rclcpp::Parameter> parameters;  

    // Search for the parameter:
    for (auto & name : names) {

        // Check if the parameter is in the list of parameters:
        for (auto & p : parameters_) {

            if (p.get_name() == name) {

                parameters.push_back(p);

                break;

            }
        }
    }

    if (parameters.size() != names.size()) {

        std::string fatal_message = "Configurator::GetParameters(): Some parameters could not be found.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    return parameters;

}

void Configurator::SyncParameters(const std::vector<std::string> & names) {

    // std::unique_lock<std::shared_mutex> lock(parameters_mutex_);

    std::vector<std::string> names_to_sync;

    if (names.size() == 0) {

        // Get all parameters:
        for (auto & p : parameters_) {

            names_to_sync.push_back(p.get_name());

        }

    } else {

        names_to_sync = names;

    }

    std::vector<rclcpp::Parameter> parameters;

    if (!sendGetParametersRequest(
            names_to_sync,
            parameters
        )
    ) {

        std::string fatal_message = "Configurator::SyncParameters(): Failed to get parameters.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    // Check if the parameters are correct:
    for (int i = 0; i < parameters.size(); i++) {

        if (parameters[i].get_type() != parameters_[i].get_type() || parameters[i].get_name() != parameters_[i].get_name()) {

            std::string fatal_message = "Configurator::SyncParameters(): Parameter " + parameters[i].get_name() + " has type " + std::to_string(parameters[i].get_type()) + " but expected type " + std::to_string(parameters_[i].get_type()) + ".";

            RCLCPP_FATAL(
                node_->get_logger(),
                fatal_message.c_str()
            );

            throw std::runtime_error(fatal_message);

        }

    }

    // Update parameters:
    parameters_ = parameters;

}

template <typename T>
std::string Configurator::GetParameterTypeString() {

    if (std::is_same<T, bool>::value) {

        return "bool";

    } else if (std::is_same<T, int>::value) {

        return "int";

    } else if (std::is_same<T, float>::value) {

        return "float";

    } else if (std::is_same<T, double>::value) {

        return "float";

    } else if (std::is_same<T, std::string>::value) {

        return "string";

    } else if (std::is_same<T, std::vector<bool>>::value) {

        return "bool_array";

    } else if (std::is_same<T, std::vector<int>>::value) {

        return "int_array";

    } else if (std::is_same<T, std::vector<float>>::value) {

        return "float_array";

    } else if (std::is_same<T, std::vector<double>>::value) {

        return "float_array";

    } else if (std::is_same<T, std::vector<std::string>>::value) {

        return "string_array";

    } else {

        std::string fatal_message = "Configurator::GetParameterTypeString(): Parameter type not supported.";

        throw std::runtime_error(fatal_message);

    }

}

void Configurator::PrintParameters() const {

    // std::shared_lock<std::shared_mutex> lock(parameters_mutex_);

    RCLCPP_INFO(
        node_->get_logger(),
        "Configurator::PrintParameters(): Printing parameters:"
    );

    for (auto & p : parameters_) {

        RCLCPP_INFO(
            node_->get_logger(),
            "Configurator::PrintParameters(): %s: %s",
            p.get_name().c_str(),
            p.value_to_string().c_str()
        );

    }

}

void Configurator::parameterEventCallback(rcl_interfaces::msg::ParameterEvent parameter_event) {

    // std::unique_lock<std::shared_mutex> lock(parameters_mutex_);

    // Search for the parameter:
    for (auto & p : parameter_event.changed_parameters) {

        // Check if the parameter is in the list of parameters:
        for (int i = 0; i < parameters_.size(); i++) {

            rclcpp::Parameter & parameter = parameters_[i];

            if (parameter.get_name() == p.name) {

                if (p.value.type != parameter.get_type()) {

                    std::string fatal_message = "Configurator::parameterEventCallback(): Parameter " + p.name + " has type " + std::to_string(p.value.type) + " but expected type " + std::to_string(parameter.get_type()) + ".";

                    RCLCPP_FATAL(
                        node_->get_logger(),
                        fatal_message.c_str()
                    );

                    throw std::runtime_error(fatal_message);

                }

                parameters_[i] = rclcpp::Parameter::from_parameter_msg(p);

                if (after_parameter_change_callback_ != nullptr) {

                    after_parameter_change_callback_(parameters_[i]);

                }

                break;

            }
        }
    }
}

bool Configurator::sendDeclareParameterRequest(
    const std::string & name,
    const std::string & type,
    std::string & message
) {

    // Call DeclareParameter service:
    auto request = std::make_shared<iii_drone_interfaces::srv::DeclareParameter::Request>();

    request->name = name;
    request->type = type;

    // Wait for service:
    if (!declare_parameter_client_->wait_for_service(std::chrono::seconds(5))) {

        std::string fatal_message = "Configurator::DeclareParameter(): Service DeclareParameter not available after 5 seconds.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    bool finished = false;

    auto callback = [&finished](rclcpp::Client<iii_drone_interfaces::srv::DeclareParameter>::SharedFuture result_future) {
        finished = true;
    };

    // Call service:
    auto result = declare_parameter_client_->async_send_request(request, callback);

    // Wait for result:
    while (!finished) {
        rclcpp::spin_some(node_->get_node_base_interface());
    }

    // // Wait for result:
    // if (rclcpp::spin_until_future_complete(
    //         node_->get_node_base_interface(), 
    //         result,
    //         std::chrono::seconds(1)
    //     ) != rclcpp::FutureReturnCode::SUCCESS
    // ) {

    //     std::string fatal_message = "Configurator::DeclareParameter(): Service DeclareParameter timed out.";

    //     RCLCPP_FATAL(
    //         node_->get_logger(),
    //         fatal_message.c_str()
    //     );

    //     throw std::runtime_error(fatal_message);

    // }

    message = result.get()->message;

    return result.get()->succeeded;

}

bool Configurator::sendGetParameterRequest(
    const std::string & name,
    rclcpp::Parameter & parameter
) {

    std::vector<std::string> names;
    names.push_back(name);

    std::vector<rclcpp::Parameter> parameters;

    if (!sendGetParametersRequest(
            names,
            parameters
        )
    ) {

        return false;

    }

    parameter = parameters[0];

    return true;

}

bool Configurator::sendGetParametersRequest(
    const std::vector<std::string> & names,
    std::vector<rclcpp::Parameter> & parameters
) {

    // Call GetParameters service:
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    request->names = names;

    // Wait for service:
    if (!get_parameters_client_->wait_for_service(std::chrono::seconds(5))) {

        std::string fatal_message = "Configurator::GetParameter(): Service GetParameter not available after 5 seconds.";

        RCLCPP_FATAL(
            node_->get_logger(),
            fatal_message.c_str()
        );

        throw std::runtime_error(fatal_message);

    }

    bool finished = false;

    auto callback = [&finished](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture result_future) {
        finished = true;
    };

    // Call service:
    auto result = get_parameters_client_->async_send_request(request, callback);

    // Wait for result:
    while (!finished) {
        rclcpp::spin_some(node_->get_node_base_interface());
    }

    // // Wait for result:
    // if (rclcpp::spin_until_future_complete(
    //         node_->get_node_base_interface(), 
    //         result,
    //         std::chrono::seconds(1)
    //     ) != rclcpp::FutureReturnCode::SUCCESS
    // ) {

    //     RCLCPP_FATAL(
    //         node_->get_logger(),
    //         "Configurator::sendGetParametersRequest(): Service GetParameters timed out."
    //     );

    //     throw std::runtime_error("Failed to call service GetParameters.");

    // }

    // Check if the service call was successful:
    if (result.get()->values.size() != names.size()) {

        RCLCPP_FATAL(
            node_->get_logger(),
            "Configurator::sendGetParametersRequest(): Service GetParameters failed, some parameters could not be found."
        );

        throw std::runtime_error("Failed to call service GetParameters failed, some parameters could not be found.");

    }

    parameters.resize(names.size());

    for (int i = 0; i < names.size(); i++) {

        parameters[i] = rclcpp::Parameter(
            names[i], 
            result.get()->values[i]
        );

    }

    return true;

}

// Declare template specializations:
template void Configurator::DeclareParameter<bool>(const std::string & name);
template void Configurator::DeclareParameter<int>(const std::string & name);
template void Configurator::DeclareParameter<double>(const std::string & name);
template void Configurator::DeclareParameter<float>(const std::string & name);
template void Configurator::DeclareParameter<std::string>(const std::string & name);
template void Configurator::DeclareParameter<std::vector<bool>>(const std::string & name);
template void Configurator::DeclareParameter<std::vector<int>>(const std::string & name);
template void Configurator::DeclareParameter<std::vector<double>>(const std::string & name);
template void Configurator::DeclareParameter<std::vector<float>>(const std::string & name);
template void Configurator::DeclareParameter<std::vector<std::string>>(const std::string & name);

// Same for GetParameterTypeString:
template std::string Configurator::GetParameterTypeString<bool>();
template std::string Configurator::GetParameterTypeString<int>();
template std::string Configurator::GetParameterTypeString<double>();
template std::string Configurator::GetParameterTypeString<float>();
template std::string Configurator::GetParameterTypeString<std::string>();
template std::string Configurator::GetParameterTypeString<std::vector<bool>>();
template std::string Configurator::GetParameterTypeString<std::vector<int>>();
template std::string Configurator::GetParameterTypeString<std::vector<double>>();
template std::string Configurator::GetParameterTypeString<std::vector<float>>();
template std::string Configurator::GetParameterTypeString<std::vector<std::string>>();