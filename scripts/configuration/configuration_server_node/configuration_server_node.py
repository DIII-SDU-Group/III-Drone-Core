#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent, SetParametersResult
from rclpy.parameter import Parameter, ParameterValue

import os
import yaml

from iii_drone_interfaces.srv import DeclareParameter

from iii_drone_core.configuration.parameter_handler import ParameterHandler

###############################################################################
# Class
###############################################################################

class ConfigurationServer(Node):
    """
    Configuration server node, handles parameter declaration and parameter events, while loading configuration from file.
    """
    def __init__(
        self,
        node_name="configuration_server",
        namespace="/configuration/configuration_server"
    ):
        """
        Constructor, initializes node and loads configuration from file.
        
        Parameters:
            node_name (str): Node name.
            namespace (str): Node namespace.
        """

        super().__init__(
            node_name=node_name, 
            namespace=namespace
        )
        
        self.get_logger().info("ConfigurationServer.__init__(): Initializing node " + node_name + " in namespace " + namespace + ".")

        # Get user:
        self.user = os.environ["USER"]

        self.declare_parameter("parameters_file_path", "/home/" + self.user + "/.config/iii_drone/parameters.yaml")
        self.params_file_path = str(self.get_parameter("parameters_file_path").value)

        self.parameter_handler = ParameterHandler(self.params_file_path)

        # Declared params empty dict:
        self.declared_params = {}
        self.parameters_initialized = {}

        # Initialize service:
        self.declare_parameter_service = self.create_service(
            DeclareParameter,
            "declare_parameter",
            self.declare_parameter_callback
        )

        # Service callback that gets called before a parameter is set:
        self.set_parameter_event_callback_handler = self.add_on_set_parameters_callback(self.set_parameter_event_callback)

        #/parameter_events topic subscriber:
        self.parameter_events_subscription = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_events_callback,
            10
        )

        self.param_success = True

    def set_parameter_event_callback(
        self,
        parameters: "list[Parameter]"
    ) -> SetParametersResult:
        """
        Callback for parameter events, gets called before a parameter is set. 
        Rejects if the parameter is constant, if the type or value do not match the loaded parameters,
        if the parameter is not listed in the loaded parameters, or if the validation function returns false.

        Parameters:
            parameters (list[Parameter]): List of parameters to be set.

        Returns:
            SetParametersResult: Result of the callback.
        """

        result = SetParametersResult()

        for parameter in parameters:
            if parameter.name in self.declared_params:
                try:
                    self.parameter_handler.can_set_param(
                        parameter.name,
                        parameter.value,
                        self.parameters_initialized[parameter.name]
                    )
                    
                except KeyError as e:
                    self.get_logger().fatal("ConfigurationServer.set_parameter_event_callback(): Parameter not listed in parameters file: " + str(e))
                    result.successful = False

                    return result

                except AttributeError as e:
                    self.get_logger().error("ConfigurationServer.set_parameter_event_callback(): Parameter " + parameter.name + " is constant, rejecting.")
                    result.successful = False

                    return result

                except TypeError as e:
                    self.get_logger().error("ConfigurationServer.set_parameter_event_callback(): Parameter " + parameter.name + " has different type, rejecting.")
                    result.successful = False

                    return result
                
                except ValueError as e:
                    self.get_logger().error("ConfigurationServer.set_parameter_event_callback(): Parameter " + parameter.name + " failed validation, rejecting.")
                    result.successful = False

                    return result

        self.get_logger().info("ConfigurationServer.set_parameter_event_callback(): Request to set parameters accepted.")

        result.successful = True

        return result
    
    def parameter_events_callback(
        self,
        parameter_event: ParameterEvent
    ) -> None:
        """
        Callback for parameter events, gets called after a parameter is set.
        Updates the declared parameters dict with the new parameter values
        and updates the parameter_handler object.
        
        Parameters:
            parameter_event (ParameterEvent): Parameter event.

        Raises:
            RuntimeError: If parameters are deleted, since this is not supported.
            RuntimeError: If the param can not be set for whatever reason, since this should not happen as it is verified in the pre-set parameter callback.
        """

        if parameter_event.node != self.get_fully_qualified_name():
            return

        if len(parameter_event.deleted_parameters) > 0:
            self.get_logger().fatal("ConfigurationServer.parameter_events_callback(): Deleted parameters not supported.")
            raise RuntimeError("Deleted parameters not supported.")
        
        for parameter in parameter_event.changed_parameters + parameter_event.new_parameters:
            try:
                param_value = self._get_parameter_value_from_msg(
                    parameter.name, 
                    parameter.value
                )

                self.parameter_handler.set_param(
                    parameter.name, 
                    param_value,
                    self.parameters_initialized[parameter.name]
                )

                self.declared_params[parameter.name] = param_value
                self.parameters_initialized[parameter.name] = True

                self.get_logger().info("ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " set to " + str(param_value) + ".")

            except KeyError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " not listed in parameters file, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e

            except AttributeError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " is constant, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e
            
            except TypeError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " has different type, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e
            
            except ValueError as e:
                error_str = "ConfigurationServer.parameter_events_callback(): Parameter " + parameter.name + " failed validation, but was changed anyways."

                self.get_logger().fatal(error_str)
                raise RuntimeError(error_str) from e


    def _get_parameter_value_from_msg(
        self,
        parameter_name: str,
        parameter_value: ParameterValue
    ) -> str|int|float|bool|list[str|int|float|bool]:
        """
        Gets the parameter value from the ParameterValue object 
        based on the type.

        Parameters:
            parameter_name (str): Parameter name.
            parameter_value (ParameterValue): Parameter value.

        Returns:
            str|int|float|bool|list[str|int|float|bool]: Parameter value.

        Raises:
            KeyError: If the parameter name is not listed in the loaded parameters.
            TypeError: If the parameter type is not recognized.
        """

        param_dict = self.parameter_handler.get_param(parameter_name)

        if param_dict["type"] == "string":
            return parameter_value.string_value
        
        elif param_dict["type"] == "int":
            return parameter_value.integer_value
        
        elif param_dict["type"] == "float":
            return parameter_value.double_value
        
        elif param_dict["type"] == "bool":
            return parameter_value.bool_value
        
        elif param_dict["type"] == "string_array":
            return parameter_value.string_array_value
        
        elif param_dict["type"] == "integer_array":
            return parameter_value.integer_array_value
        
        elif param_dict["type"] == "float_array":
            return parameter_value.double_array_value
        
        elif param_dict["type"] == "bool_array":
            return parameter_value.bool_array_value
        
        else:
            raise TypeError("Type " + str(param_dict["type"]) + " not recognized.")

    def declare_parameter_callback(
        self,
        request: DeclareParameter.Request,
        response: DeclareParameter.Response
    ) -> DeclareParameter.Response:
        """
        Callback for declare_parameter service. Checks that the the parameter has been loaded,
        and that the type matches the loaded type.

        Parameters:
            request (DeclareParameter.Request): Service request.
            response (DeclareParameter.Response): Service response.

        Returns:
            DeclareParameter.Response: Service response.
        """

        try:
            param_dict = self.parameter_handler.get_param(request.name)
        
        except KeyError as e:
            response.succeeded = False
            response.message = "Parameter not listed in parameters file: " + str(e)

            return response

        already_declared = False

        if request.name in self.declared_params:
            already_declared = True

        if request.type != param_dict["type"]:
            response.succeeded = False
            response.message = "Type " + str(request.type) + " does not match type " + str(param_dict["type"]) + " in loaded parameters for parameter " + request.name + "."

            return response

        self.get_logger().info("ConfigurationServer.declare_parameter_callback(): Declaring parameter " + request.name + " with value " + str(param_dict["value"]) + " of type " + str(param_dict["type"]) + ".")
        
        if already_declared:
            response.succeeded = True
            response.message = "Parameter already declared."

            return response
        
        value = param_dict["value"]

        self.declare_parameter(
            request.name, 
            value
        )

        self.declared_params[request.name] = value
        self.parameters_initialized[request.name] = False

        response.succeeded = True
        response.message = "Parameter declared successfully."

        return response

###############################################################################
# Main
###############################################################################

def main():
    rclpy.init()

    print("Starting ConfigurationServer node...")
    node = ConfigurationServer()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == "__main__":
    main()