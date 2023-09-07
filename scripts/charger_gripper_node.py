#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32

from iii_interfaces.msg import ChargerOperatingMode, ChargerStatus, GripperStatus
from iii_interfaces.srv import GripperCommand

from threading import Lock

import serial

#import pigpio

class ChargerGripperNode(Node):
    def __init__(
            self,
            node_name="charger_gripper",
            node_namespace="charger_gripper"
        ):
        super().__init__(node_name, namespace=node_namespace)

        self.get_logger().info("Initializing Charger Gripper Node...")

        self.declare_parameter("simulation", False)
        self.simulation_ = self.get_parameter("simulation").value

        if self.simulation_:
            raise NotImplementedError("Simulation mode is not implemented yet.")

        self.declare_parameter("gripper_command_only", False)
        self.gripper_command_only_ = self.get_parameter("gripper_command_only").value | self.simulation_

        self.declare_parameter("gripper_command_interface", "serial")
        self.gripper_command_interface_ = self.get_parameter("gripper_command_interface").value

        self.declare_parameter("gripper_open_command", 0x00)
        self.gripper_open_command_ = self.get_parameter("gripper_open_command").value

        self.declare_parameter("gripper_close_command", 0x01)
        self.gripper_close_command_ = self.get_parameter("gripper_close_command").value

        if (self.gripper_command_interface_ != "serial" and self.gripper_command_interface_ != "gpio"):
            self.get_logger().fatal("Invalid gripper command interface: {}".format(self.gripper_command_interface_))
            exit(-1)

        if self.gripper_command_interface_ == "serial" or not self.gripper_command_only_:
            self.declare_parameter("charger_gripper_serial_port", "/dev/ttyUSB0")
            self.charger_gripper_serial_port_ = self.get_parameter("charger_gripper_serial_port").value

            self.declare_parameter("charger_gripper_serial_baudrate", 115200)
            self.charger_gripper_serial_baudrate_ = self.get_parameter("charger_gripper_serial_baudrate").value

            self.declare_parameter("charger_gripper_serial_timeout", 0.1)
            self.charger_gripper_serial_timeout_ = self.get_parameter("charger_gripper_serial_timeout").value

            self.declare_parameter("gripper_command_serial_period_ms", 500)
            self.gripper_command_serial_period_ms_ = self.get_parameter("gripper_command_serial_period_ms").value

            self.ser_lock_ = Lock()

            self.ser_ = serial.Serial(
                port=self.charger_gripper_serial_port_,
                baudrate=self.charger_gripper_serial_baudrate_,
                timeout=self.charger_gripper_serial_timeout_
            )

            self.serial_gripper_cmd_timer_ = self.create_timer(
                timer_period_sec=self.gripper_command_serial_period_ms_ / 1000.0,
                callback=self.send_serial_gripper_cmd_callback
            )

        if self.gripper_command_interface_ == "gpio":
            self.declare_parameter("charger_gripper_rpi_gpio_pin", 18)
            self.charger_gripper_rpi_gpio_pin_ = self.get_parameter("charger_gripper_rpi_gpio_pin").value

            self.pi_gpio_ = pigpio.pi()

            self.pi_gpio_.set_mode(self.charger_gripper_rpi_gpio_pin_, pigpio.OUTPUT)
            self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, 0 if self.gripper_open_command_ == 0x00 else 1)

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.declare_parameter("status_timer_period_ms", 10)
        self.status_timer_period_ms_ = self.get_parameter("status_timer_period_ms").value

        if not self.gripper_command_only_:
            self.declare_parameter("status_message_first_byte", 0xFA)
            self.status_message_first_byte_ = self.get_parameter("status_message_first_byte").value

            self.declare_parameter("status_message_last_byte", 0xFC)
            self.status_message_last_byte_ = self.get_parameter("status_message_last_byte").value

            self.declare_parameter("status_message_length", 9)
            self.status_message_length_ = self.get_parameter("status_message_length").value

            self.declare_parameter("status_message_battery_voltage_start_index", 1)
            self.status_message_battery_voltage_start_index_ = self.get_parameter("status_message_battery_voltage_start_index").value

            self.declare_parameter("status_message_battery_voltage_length", 2)
            self.status_message_battery_voltage_length_ = self.get_parameter("status_message_battery_voltage_length").value

            self.declare_parameter("status_message_charging_power_start_index", 3)
            self.status_message_charging_power_start_index_ = self.get_parameter("status_message_charging_power_start_index").value

            self.declare_parameter("status_message_charging_power_length", 2)
            self.status_message_charging_power_length_ = self.get_parameter("status_message_charging_power_length").value

            self.declare_parameter("status_message_charger_status_index", 5)
            self.status_message_charger_status_index_ = self.get_parameter("status_message_charger_status_index").value

            self.declare_parameter("status_message_charger_operating_mode_index", 6)
            self.status_message_charger_operating_mode_index_ = self.get_parameter("status_message_charger_operating_mode_index").value

            self.declare_parameter("status_message_gripper_status_index", 7)
            self.status_message_gripper_status_index_ = self.get_parameter("status_message_gripper_status_index").value

            self.battery_voltage_pub_ = self.create_publisher(Float32, "battery_voltage", pub_qos)
            self.charging_power_pub_ = self.create_publisher(Float32, "charging_power", pub_qos)

            self.charger_operating_mode_pub_ = self.create_publisher(ChargerOperatingMode, "charger_operating_mode", pub_qos)
            self.charger_status_pub_ = self.create_publisher(ChargerStatus, "charger_status", pub_qos)

            self.received_data_ = bytearray()

        self.gripper_status_pub_ = self.create_publisher(GripperStatus, "gripper_status", pub_qos)

        self.status_timer_ = self.create_timer(
            self.status_timer_period_ms_ / 1000,
            self.status_timer_callback
        )

        self.last_gripper_command_ = "open"

        self.gripper_command_srv_ = self.create_service(
            GripperCommand,
            "gripper_command",
            self.gripper_command_srv_callback
        )

    def status_timer_callback(self):
        #if (self.ser_.in_waiting > 0):
        #    print(self.ser_.read(1))

        #return

        if not self.gripper_command_only_:
            self.ser_lock_.acquire()

            if (self.ser_.in_waiting > 0):
                if (len(self.received_data_) >= self.status_message_length_):
                    self.get_logger().error("Received data is too long. Discarding.")
                    self.received_data_.clear()

                if (len(self.received_data_) == 0):
                    if (self.ser_.read(1)[0] == self.status_message_first_byte_):
                        self.received_data_.append(self.status_message_first_byte_)

                    self.ser_lock_.release()
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

            self.ser_lock_.release()

        else:
            gripper_status_msg = GripperStatus()
            gripper_status_msg.gripper_status = GripperStatus.GRIPPER_STATUS_CLOSED if self.last_gripper_command_ == "close" else GripperStatus.GRIPPER_STATUS_OPEN

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

        self.ser_lock_.acquire()

        self.ser_.write(bytes([gripper_cmd]))

        self.ser_lock_.release()

    def parse_and_publish_data(self):
        battery_voltage = int(self.received_data_[self.status_message_battery_voltage_start_index_]) * 256 + int(self.received_data_[self.status_message_battery_voltage_start_index_ + self.status_message_battery_voltage_length_ - 1])
        charging_power = int(self.received_data_[self.status_message_charging_power_start_index_]) * 256 + int(self.received_data_[self.status_message_charging_power_start_index_ + self.status_message_charging_power_length_ - 1])
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

        self.battery_voltage_pub_.publish(battery_voltage_msg)
        self.charging_power_pub_.publish(charging_power_msg)
        self.charger_status_pub_.publish(charger_status_msg)
        self.charger_operating_mode_pub_.publish(charger_operating_mode_msg)
        self.gripper_status_pub_.publish(gripper_status_msg)

    def gripper_command_srv_callback(self, request: GripperCommand.Request, response: GripperCommand.Response):
        self.get_logger().info("Received gripper command: " + str(request.gripper_command))

        gripper_command = request.gripper_command

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

    def open_gripper(self):
        if self.gripper_command_interface_ == "gpio":
            self.get_logger().info("Opening gripper using GPIO pin {}.".format(self.charger_gripper_rpi_gpio_pin_))
            gpio_output = 0 if self.gripper_open_command_ == 0x00 else 1
            self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, gpio_output)

        elif self.gripper_command_interface_ == "serial":
            self.ser_lock_.acquire()

            self.get_logger().info("Opening gripper using serial.")
            self.ser_.write(bytes([self.gripper_open_command_]))

        else:
            self.get_logger().error("Unknown gripper command interface. Not opening gripper.")

    def close_gripper(self):
        if self.gripper_command_interface_ == "gpio":
            self.get_logger().info("Closing gripper using GPIO pin {}.".format(self.charger_gripper_rpi_gpio_pin_))
            gpio_output = 0 if self.gripper_close_command_ == 0x00 else 1
            self.pi_gpio_.write(self.charger_gripper_rpi_gpio_pin_, gpio_output)

        elif self.gripper_command_interface_ == "serial":
            self.get_logger().info("Closing gripper using serial.")
            self.ser_.write(bytes([self.gripper_close_command_]))

        else:
            self.get_logger().error("Unknown gripper command interface. Not closing gripper.")

def main(args=None):
    rclpy.init(args=args)

    node = ChargerGripperNode()

    node.get_logger().info("Charger gripper node started.")

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

