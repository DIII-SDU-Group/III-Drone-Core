#!/usr/bin/python3

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32

from iii_drone_interfaces.msg import ChargerOperatingMode, ChargerStatus, GripperStatus
from iii_drone_interfaces.srv import GripperCommand

from iii_drone_configuration.configurator import Configurator

from threading import Lock, Thread
from time import sleep

import serial
import os
import sys

SIMULATION = os.environ.get('SIMULATION', 'false').lower() == 'true'

if SIMULATION:
    import debugpy

#import pigpio

class ChargerGripperNode(Node):
    def __init__(
            self,
            node_name="charger_gripper",
            node_namespace="/payload/charger_gripper"
        ):
        super().__init__(node_name, namespace=node_namespace)

        self.get_logger().info("Initializing Charger Gripper Node...")

        self.simulation_ = True if os.getenv("SIMULATION", "false").lower() == "true" else False 

        self.get_logger().info("Simulation: {}".format(self.simulation_))

        log_level = os.environ.get('CHARGER_GRIPPER_LOG_LEVEL')
        
        if log_level is not None:
            log_level = log_level.upper()
            
            if log_level == 'DEBUG':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            elif log_level == 'INFO':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
            elif log_level == 'WARN':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
            elif log_level == 'ERROR':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
            elif log_level == 'FATAL':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.gripper_status_pub_ = self.create_publisher(GripperStatus, "gripper_status", pub_qos)
        self.battery_voltage_pub_ = self.create_publisher(Float32, "battery_voltage", pub_qos)
        self.charging_power_pub_ = self.create_publisher(Float32, "charging_power", pub_qos)
        self.charger_operating_mode_pub_ = self.create_publisher(ChargerOperatingMode, "charger_operating_mode", pub_qos)
        self.charger_status_pub_ = self.create_publisher(ChargerStatus, "charger_status", pub_qos)

        self.get_logger().info("Charger Gripper Node initialized.")

    def on_configure(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ChargerGripperNode.on_configure()")
        
        ret = super().on_configure(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("ChargerGripperNode.on_configure(): Base class configuration failed.")
            return ret
        
        self.configurator = Configurator(
            node=self,
        )
        
        self.get_logger().info("Configuration loaded.")

        # #self.declare_parameter("gripper_command_only", False)
        self.gripper_command_only_ = self.configurator.get_parameter("gripper_command_only").value | self.simulation_

        # #self.declare_parameter("gripper_command_interface", "serial")
        self.gripper_command_interface_ = self.configurator.get_parameter("gripper_command_interface").value

        # #self.declare_parameter("gripper_open_command", 0x00)
        self.gripper_open_command_ = self.configurator.get_parameter("gripper_open_command").value

        # #self.declare_parameter("gripper_close_command", 0x01)
        self.gripper_close_command_ = self.configurator.get_parameter("gripper_close_command").value

        # #self.declare_parameter("gripper_command_complete_poll_interval_ms", 100)
        self.gripper_command_complete_poll_interval_ms_ = self.configurator.get_parameter("gripper_command_complete_poll_interval_ms").value

        # #self.declare_parameter("gripper_command_complete_timeout_ms", 1000)
        self.gripper_command_complete_timeout_ms_ = self.configurator.get_parameter("gripper_command_complete_timeout_ms").value

        if (self.configurator.get_parameter("gripper_command_interface").value != "serial" and \
                self.configurator.get_parameter("gripper_command_interface").value != "gpio"):
            self.get_logger().fatal("Invalid gripper command interface: {}".format(self.configurator.get_parameter("gripper_command_interface").value))
            exit(-1)

        if self.configurator.get_parameter("gripper_command_interface") == "serial" or not self.configurator.get_parameter("gripper_command_only").value:
        #     #self.declare_parameter("charger_gripper_serial_port", "/dev/ttyUSB0")
            self.charger_gripper_serial_port_ = self.configurator.get_parameter("charger_gripper_serial_port").value

        #     #self.declare_parameter("charger_gripper_serial_baudrate", 115200)
            self.charger_gripper_serial_baudrate_ = self.configurator.get_parameter("charger_gripper_serial_baudrate").value

        #     #self.declare_parameter("charger_gripper_serial_timeout", 0.1)
            self.charger_gripper_serial_timeout_ = self.configurator.get_parameter("charger_gripper_serial_timeout").value

        #     #self.declare_parameter("gripper_command_serial_period_ms", 500)
            self.gripper_command_serial_period_ms_ = self.configurator.get_parameter("gripper_command_serial_period_ms").value

            self.ser_lock_ = Lock()

        # #self.declare_parameter("status_timer_period_ms", 10)
        self.status_timer_period_ms_ = self.configurator.get_parameter("status_timer_period_ms").value

        if not self.gripper_command_only_ and not self.simulation_:
            #self.declare_parameter("status_message_first_byte", 0xFA)
            self.status_message_first_byte_ = self.configurator.get_parameter("status_message_first_byte").value

            #self.declare_parameter("status_message_last_byte", 0xFC)
            self.status_message_last_byte_ = self.configurator.get_parameter("status_message_last_byte").value

            #self.declare_parameter("status_message_length", 9)
            self.status_message_length_ = self.configurator.get_parameter("status_message_length").value

            #self.declare_parameter("status_message_battery_voltage_start_index", 1)
            self.status_message_battery_voltage_start_index_ = self.configurator.get_parameter("status_message_battery_voltage_start_index").value

            #self.declare_parameter("status_message_battery_voltage_length", 2)
            self.status_message_battery_voltage_length_ = self.configurator.get_parameter("status_message_battery_voltage_length").value

            #self.declare_parameter("status_message_charging_power_start_index", 3)
            self.status_message_charging_power_start_index_ = self.configurator.get_parameter("status_message_charging_power_start_index").value

            #self.declare_parameter("status_message_charging_power_length", 2)
            self.status_message_charging_power_length_ = self.configurator.get_parameter("status_message_charging_power_length").value

            #self.declare_parameter("status_message_charger_status_index", 5)
            self.status_message_charger_status_index_ = self.configurator.get_parameter("status_message_charger_status_index").value

            #self.declare_parameter("status_message_charger_operating_mode_index", 6)
            self.status_message_charger_operating_mode_index_ = self.configurator.get_parameter("status_message_charger_operating_mode_index").value

            #self.declare_parameter("status_message_gripper_status_index", 7)
            self.status_message_gripper_status_index_ = self.configurator.get_parameter("status_message_gripper_status_index").value

            #self.declare_parameter("battery_voltage_avg_filter_size", 10)
            self.battery_voltage_avg_filter_size_ = self.configurator.get_parameter("battery_voltage_avg_filter_size").value
            self.battery_voltage_buffer_ = []

            #self.declare_parameter("charging_power_avg_filter_size", 10)
            self.charging_power_avg_filter_size_ = self.configurator.get_parameter("charging_power_avg_filter_size").value
            self.charging_power_buffer_ = []

            self.received_data_ = bytearray()

        self.gripper_status_msg_ = GripperStatus()

        self.last_gripper_command_ = "open"

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ChargerGripperNode.on_cleanup()")
        
        ret = super().on_cleanup(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("ChargerGripperNode.on_cleanup(): Base class cleanup failed.")
            return ret

        del self.configurator

        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ChargerGripperNode.on_activate()")
        
        try:
            ret = super().on_activate(state)
            
            if ret != TransitionCallbackReturn.SUCCESS:
                self.get_logger().error("ChargerGripperNode.on_activate(): Base class activation failed.")
                return ret

            if self.configurator.get_parameter("gripper_command_interface") == "serial" or not self.configurator.get_parameter("gripper_command_only").value:
                if not self.simulation_:
                    self.get_logger().debug("ChargerGripperNode.on_activate(): Opening serial port.")

                    self.ser_ = serial.Serial(
                        port=self.configurator.get_parameter("charger_gripper_serial_port").value,
                        baudrate=self.configurator.get_parameter("charger_gripper_serial_baudrate").value,
                        timeout=self.configurator.get_parameter("charger_gripper_serial_timeout").value
                    )

                self.get_logger().debug("ChargerGripperNode.on_activate(): Starting serial gripper command timer.")
                
                self.serial_gripper_cmd_timer_ = self.create_timer(
                    timer_period_sec=self.configurator.get_parameter("gripper_command_serial_period_ms").value / 1000.0,
                    callback=self.send_serial_gripper_cmd_callback
                )


            if self.configurator.get_parameter("gripper_command_interface").value == "gpio" and not self.simulation_:
                # #self.declare_parameter("charger_gripper_rpi_gpio_pin", 18)
                self.charger_gripper_rpi_gpio_pin_ = self.configurator.get_parameter("charger_gripper_rpi_gpio_pin").value

                self.pi_gpio_ = pigpio.pi()

                self.pi_gpio_.set_mode(self.charger_gripper_rpi_gpio_pin_, pigpio.OUTPUT)
                self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, 0 if self.gripper_open_command_ == 0x00 else 1)


            self.status_timer_ = self.create_timer(
                self.status_timer_period_ms_ / 1000,
                self.status_timer_callback
            )

            self.gripper_command_srv_ = self.create_service(
                GripperCommand,
                "gripper_command",
                self.gripper_command_srv_callback
            )

        except Exception as e:
            self.get_logger().error("ChargerGripperNode.on_activate(): Error occurred.")
            self.get_logger().error("Error: " + str(e.with_traceback(None)))
            self.get_logger().error("Error type: " + str(type(e)))
            raise e

        self.get_logger().info("ChargerGripperNode.on_activate(): Charger gripper node activated.")
        return TransitionCallbackReturn.SUCCESS
        
    def on_deactivate(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ChargerGripperNode.on_deactivate()")
        
        ret = super().on_deactivate(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("ChargerGripperNode.on_deactivate(): Base class deactivation failed.")
            return ret

        if self.configurator.get_parameter("gripper_command_interface").value == "serial" or not self.configurator.get_parameter("gripper_command_only").value:
            if not self.simulation_:
                self.ser_.close()
                del self.ser_

            self.serial_gripper_cmd_timer_.cancel()
            self.serial_gripper_cmd_timer_.destroy()

        if self.configurator.get_parameter("gripper_command_interface").value == "gpio" and not self.simulation_:
            # self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, 0 if self.gripper_open_command_ == 0x00 else 1)
            self.pi_gpio_.stop()
            del self.pi_gpio_

        self.status_timer_.cancel()
        self.status_timer_.destroy()

        self.gripper_command_srv_.destroy()

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("ChargerGripperNode.on_shutdown()")
        
        ret = super().on_shutdown(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("ChargerGripperNode.on_shutdown(): Base class shutdown failed.")
            return ret
        
        if hasattr(self, "configurator"):
            del self.configurator
        
        def shutdown():
            sleep(1)
            
            rclpy.shutdown()
        
        shutdown_thread = Thread(target=shutdown)
        shutdown_thread.start()

        return TransitionCallbackReturn.SUCCESS
    
    def on_error(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        # self.get_logger().fatal("ChargerGripperNode.on_error(): Error occurred.")
        
        # raise RuntimeError("ChargerGripperNode.on_error(): Error occurred.")
        
        ret = super().on_error(state)
        self.get_logger().error("ChargerGripperNode.on_error(): Error occurred:")
        return ret

    def status_timer_callback(self):
        #if (self.ser_.in_waiting > 0):
        #    print(self.ser_.read(1))

        #return

        if not self.gripper_command_only_ and not self.simulation_:
            # self.ser_lock_.acquire()

            if (self.ser_.in_waiting > 0):
                if (len(self.received_data_) >= self.status_message_length_):
                    self.get_logger().error("Received data is too long. Discarding.")
                    self.received_data_.clear()

                if (len(self.received_data_) == 0):
                    if (self.ser_.read(1)[0] == self.status_message_first_byte_):
                        self.received_data_.append(self.status_message_first_byte_)

                    # self.ser_lock_.release()
                    return

                #data = self.ser_.read(1)
                #self.received_data_.append(data)
                self.received_data_.append(self.ser_.read(1)[0])

                if (len(self.received_data_) == self.status_message_length_):
                    if (self.received_data_[self.status_message_length_ - 1] == self.status_message_last_byte_):
                        self.parse_and_publish_data()
                    else:
                        self.get_logger().warn("Received wrong status message last byte. Discarding.")

                    self.received_data_.clear()

            # self.ser_lock_.release()

        else:
            gripper_status_msg = GripperStatus()
            gripper_status_msg.gripper_status = GripperStatus.GRIPPER_STATUS_CLOSED if self.last_gripper_command_ == "close" else GripperStatus.GRIPPER_STATUS_OPEN

            self.gripper_status_msg_ = gripper_status_msg
            self.gripper_status_pub_.publish(gripper_status_msg)

    def send_serial_gripper_cmd_callback(self):
        gripper_cmd = None

        if self.last_gripper_command_ == "open":
            gripper_cmd = self.gripper_open_command_

            self.get_logger().info("Opening gripper using serial.")

        elif self.last_gripper_command_ == "close":
            gripper_cmd = self.gripper_close_command_

            self.get_logger().info("Closing gripper using serial.")

        else:
            self.get_logger().error("Invalid gripper command.")
            return

        # self.ser_lock_.acquire()

        self.ser_.write(bytes([gripper_cmd]))

        # self.ser_lock_.release()

    def parse_and_publish_data(self):
        battery_voltage = int(self.received_data_[self.status_message_battery_voltage_start_index_]) * 256 + int(self.received_data_[self.status_message_battery_voltage_start_index_ + self.status_message_battery_voltage_length_ - 1])
        self.battery_voltage_buffer_.append(battery_voltage)
        if (len(self.battery_voltage_buffer_) > self.battery_voltage_avg_filter_size_):
            self.battery_voltage_buffer_.pop(0)
        battery_voltage = sum(self.battery_voltage_buffer_) / len(self.battery_voltage_buffer_)
        
        charging_power = int(self.received_data_[self.status_message_charging_power_start_index_]) * 256 + int(self.received_data_[self.status_message_charging_power_start_index_ + self.status_message_charging_power_length_ - 1])
        self.charging_power_buffer_.append(charging_power)
        if (len(self.charging_power_buffer_) > self.charging_power_avg_filter_size_):
            self.charging_power_buffer_.pop(0)
        charging_power = sum(self.charging_power_buffer_) / len(self.charging_power_buffer_)

        charger_status = int(self.received_data_[self.status_message_charger_status_index_])
        charger_operating_mode = int(self.received_data_[self.status_message_charger_operating_mode_index_])
        gripper_status = int(self.received_data_[self.status_message_gripper_status_index_])

        battery_voltage_msg = Float32()
        battery_voltage_msg.data = battery_voltage / 100.0

        charging_power_msg = Float32()
        charging_power_msg.data = charging_power / 10.0

        charger_status_msg = ChargerStatus()
        if (charger_status == ChargerStatus.CHARGER_STATUS_DISABLED):
            charger_status_msg.charger_status = ChargerStatus.CHARGER_STATUS_DISABLED

        elif (charger_status == ChargerStatus.CHARGER_STATUS_CHARGING):
            charger_status_msg.charger_status = ChargerStatus.CHARGER_STATUS_CHARGING

        elif (charger_status == ChargerStatus.CHARGER_STATUS_FULLY_CHARGED):
            charger_status_msg.charger_status = ChargerStatus.CHARGER_STATUS_FULLY_CHARGED

        else:
            self.get_logger().warn("Received unknown charger status. Publishing anyways.")
            charger_status_msg.charger_status = charger_status

        charger_operating_mode_msg = ChargerOperatingMode()
        if (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_OPEN):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_OPEN

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_1):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_1

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_2):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_2

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_3):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_3

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_4):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_4

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_5):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_5

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_6):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_6

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_7):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_7

        elif (charger_operating_mode == ChargerOperatingMode.OPERATING_MODE_8):
            charger_operating_mode_msg.operating_mode = ChargerOperatingMode.OPERATING_MODE_8

        else:
            self.get_logger().warn("Received unknown charger operating mode. Publishing anyways.")
            print("Op mode:", charger_operating_mode)
            charger_operating_mode_msg.operating_mode = charger_operating_mode

        gripper_status_msg = GripperStatus()
        if (gripper_status == GripperStatus.GRIPPER_STATUS_OPEN):
            gripper_status_msg.gripper_status = GripperStatus.GRIPPER_STATUS_OPEN

        elif (gripper_status == GripperStatus.GRIPPER_STATUS_CLOSED):
            gripper_status_msg.gripper_status = GripperStatus.GRIPPER_STATUS_CLOSED

        else:
            self.get_logger().warn("Received unknown gripper status. Publishing anyways.")
            gripper_status_msg.gripper_status = gripper_status

        self.gripper_status_msg_ = gripper_status_msg

        self.battery_voltage_pub_.publish(battery_voltage_msg)
        self.charging_power_pub_.publish(charging_power_msg)
        self.charger_status_pub_.publish(charger_status_msg)
        self.charger_operating_mode_pub_.publish(charger_operating_mode_msg)
        self.gripper_status_pub_.publish(gripper_status_msg)

    def gripper_command_srv_callback(self, request: GripperCommand.Request, response: GripperCommand.Response):
        self.get_logger().info("Received gripper command: " + str(request.gripper_command))

        gripper_command = request.gripper_command
        previous_gripper_command = self.last_gripper_command_

        if (gripper_command == GripperCommand.Request.GRIPPER_COMMAND_OPEN):
            self.open_gripper()
            response.gripper_command_response = GripperCommand.Response.GRIPPER_COMMAND_RESPONSE_SUCCESS
            self.last_gripper_command_ = "open"

        elif (gripper_command == GripperCommand.Request.GRIPPER_COMMAND_CLOSE):
            self.close_gripper()
            response.gripper_command_response = GripperCommand.Response.GRIPPER_COMMAND_RESPONSE_SUCCESS
            self.last_gripper_command_ = "close"

        else:
            response.gripper_command_response = GripperCommand.Response.GRIPPER_COMMAND_RESPONSE_INVALID_COMMAND
            return response

        if not self.gripper_command_only_ and not self.simulation_:
            start_time = self.get_clock().now()

            while (self.get_clock().now() - start_time).nanoseconds / 1e6 < self.gripper_command_complete_timeout_ms_:
                if (self.last_gripper_command_ == "open" and self.gripper_status_msg_.gripper_status == GripperStatus.GRIPPER_STATUS_OPEN) or (self.last_gripper_command_ == "close" and self.gripper_status_msg_.gripper_status == GripperStatus.GRIPPER_STATUS_CLOSED):
                    response.gripper_command_response = GripperCommand.Response.GRIPPER_COMMAND_RESPONSE_SUCCESS
                    return response

            # Timeout
            response.gripper_command_response = GripperCommand.Response.GRIPPER_COMMAND_RESPONSE_TIMEOUT
            self.last_gripper_command_ = previous_gripper_command
            if (previous_gripper_command == "open"):
                self.open_gripper()

            elif (previous_gripper_command == "close"):
                self.close_gripper()

            else:
                self.get_logger().error("Invalid previous gripper command: {}".format(previous_gripper_command))

        return response


    def open_gripper(self):
        if self.simulation_:
            self.get_logger().info("Opening gripper in simulation.")
            return
        
        if self.gripper_command_interface_ == "gpio":
            self.get_logger().info("Opening gripper using GPIO pin {}.".format(self.charger_gripper_rpi_gpio_pin_))
            gpio_output = 0 if self.gripper_open_command_ == 0x00 else 1
            self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, gpio_output)

        elif self.gripper_command_interface_ == "serial":
            # self.ser_lock_.acquire()

            self.get_logger().info("Opening gripper using serial.")
            self.ser_.write(bytes([self.gripper_open_command_]))

        else:
            self.get_logger().error("Unknown gripper command interface. Not opening gripper.")

    def close_gripper(self):
        if self.simulation_:
            self.get_logger().info("Closing gripper in simulation.")
            return
        
        if self.gripper_command_interface_ == "gpio":
            self.get_logger().info("Closing gripper using GPIO pin {}.".format(self.charger_gripper_rpi_gpio_pin_))
            gpio_output = 0 if self.gripper_close_command_ == 0x00 else 1
            self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, gpio_output)

        elif self.gripper_command_interface_ == "serial":
            self.get_logger().info("Closing gripper using serial.")
            self.ser_.write(bytes([self.gripper_close_command_]))

        else:
            self.get_logger().error("Unknown gripper command interface. Not closing gripper.")

def main():
    if SIMULATION:
        DEBUG_PORT = int(os.environ.get('CHARGER_GRIPPER_DEBUG_PORT', 0))
        
        if DEBUG_PORT > 0:
            debugpy.listen(
                (
                    'localhost',
                    DEBUG_PORT
                )
            )
            
            print("Listening for debugger on port " + str(DEBUG_PORT))

    rclpy.init(args=sys.argv)

    node = ChargerGripperNode()

    node.get_logger().info("Charger gripper node started.")
    
    executor = rclpy.executors.MultiThreadedExecutor()
    
    executor.add_node(node)

    try:
        executor.spin()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except KeyboardInterrupt:
        del node.configurator

if __name__ == '__main__':
    main()

