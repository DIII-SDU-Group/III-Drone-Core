#!/usr/bin/python3

###############################################################################
# Imports
###############################################################################

import rclpy
from rclpy.node import Node

import os
import yaml

from iii_drone_interfaces.srv import DeclareParameter

###############################################################################
# Class
###############################################################################

class ConfigurationServer(Node):
    def __init__(
        self,
        node_name="configuration_server",
        namespace="/configuration/configuration_server"
    ):
        super().__init__(
            node_name=node_name, 
            namespace=namespace
        )

        # Get user:
        self.user = os.environ["USER"]

        self.declare_parameter("parameters_file_path", "/home/" + self.user + "/.config/iii_drone/parameters.yaml")
        self.params_file_path = str(self.get_parameter("parameters_file_path").value)

        self.params: "dict[str, Any]" = self.load_parameter_file(self.params_file_path)
        self.params_info: "dict[str, dict[str,bool|type]]" = {}

        # Initialize service:
        self.declare_parameter_service = self.create_service(
            DeclareParameter,
            "declare_parameter",
            self.declare_parameter_callback
        )

        self.set_parameter_event_callback_handler = self.add_on_set_parameters_callback(self.set_parameter_event_callback)

        # AsyncParametersClient:
        self.async_parameters_client = rclpy.parameter_service.ParameterService(self)

    def load_parameter_file(
        self,
        params_file_path: str
    ) -> "dict[str, Any]":
        # Load parameters from file, parse yaml, declare them. 
        # The parameters are divided into namespaces represented by the keys of the yaml file.
        # The namespaces can be nested.
        # The deepest namespace is the parameter name.
        # If the parameters has no namespace, don't use the first "/".
        # Declare the parameters using self.declare_parameter("/namespace/namespace/.../parameter_name", value)
        # Infer the type of the parameter from the value.

        with open(params_file_path, "r") as f:
            params = yaml.safe_load(f)

        params_dict = self.load_params(
            params,
            {}
        )

        return params_dict

    def load_params(
        self,
        params: dict,
        params_dict: dict,
        namespace=""
    ) -> "dict[str, Any]":
        for key, value in params.items():
            if isinstance(value, dict):
                params_dict = self.load_params(
                    value,
                    params_dict,
                    namespace + "/" + key
                )
            else:
                params_dict[namespace + "/" + key] = value

        return params_dict

    def declare_parameter_callback(
        self,
        request: DeclareParameter.Request,
        response: DeclareParameter.Response
    ):
        if request.name not in self.params:
            response.success = False
            response.message = "Parameter not listed in parameters file."
            return response

        already_declared = False

        if request.name in self.params_info:
            already_declared = True

        type_ = None

        if request.type == request.PARAMETER_TYPE_BOOL:
            type_ = bool

        elif request.type == request.PARAMETER_TYPE_INT:
            type_ = int

        elif request.type == request.PARAMETER_TYPE_FLOAT:
            type_ = float

        elif request.type == request.PARAMETER_TYPE_STRING:
            type_ = str

        elif request.type == request.PARAMETER_TYPE_BOOL_ARRAY:
            type_ = list[bool]

        elif request.type == request.PARAMETER_TYPE_INT_ARRAY:
            type_ = list[int]

        elif request.type == request.PARAMETER_TYPE_FLOAT_ARRAY:
            type_ = list[float]

        elif request.type == request.PARAMETER_TYPE_STRING_ARRAY:
            type_ = list[str]

        else:
            response.success = False
            response.message = "Invalid type " + str(request.type) + "."

            return response
        
        if already_declared:
            if self.params_info[request.name]["is_constant"] != request.is_constant:
                response.success = False
                response.message = "Parameter already declared with different constant flag."

                return response

            if self.params_info[request.name]["type"] != type_:
                response.success = False
                response.message = "Parameter already declared with different type."

                return response
            
            response.success = True
            response.message = "Parameter already declared."

            return response
        
        value = None
        
        try:
            # Cast the value to the type of the parameter.
            value = type_(self.params[request.name])
        except ValueError:
            response.success = False
            response.message = "Invalid value " + str(self.params[request.name]) + " for type " + str(request.type) + "."

            return response
        
        self.declare_parameter(request.name, value)

        self.params_info[request.name] = {
            "type": type_,
            "is_constant": request.is_constant
        }

        response.success = True
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