#include <iii_drone_interfaces/msg/target.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("serialization_test");

    rclcpp::Serialization<iii_drone_interfaces::msg::Target> target_serializer;

    iii_drone_interfaces::msg::Target target;
    target.reference_frame_id = "test";
    target.target_id = 123;
    target.target_transform.translation.x = 1.0;
    target.target_transform.translation.y = 2.0;
    target.target_transform.translation.z = 3.0;
    target.target_transform.rotation.w = 1.0;
    target.target_transform.rotation.x = 2.0;
    target.target_transform.rotation.y = 3.0;
    target.target_transform.rotation.z = 4.0;
    target.target_type = target.TARGET_TYPE_CABLE;

    std::cout << "Target: " << std::endl;
    std::cout << "  reference_frame_id: " << target.reference_frame_id << std::endl;
    std::cout << "  target_id: " << target.target_id << std::endl;
    std::cout << "  target_transform: " << std::endl;
    std::cout << "    translation: " << std::endl;
    std::cout << "      x: " << target.target_transform.translation.x << std::endl;
    std::cout << "      y: " << target.target_transform.translation.y << std::endl;
    std::cout << "      z: " << target.target_transform.translation.z << std::endl;
    std::cout << "    rotation: " << std::endl;
    std::cout << "      w: " << target.target_transform.rotation.w << std::endl;
    std::cout << "      x: " << target.target_transform.rotation.x << std::endl;
    std::cout << "      y: " << target.target_transform.rotation.y << std::endl;
    std::cout << "      z: " << target.target_transform.rotation.z << std::endl;
    std::cout << "  target_type: " << target.target_type << std::endl;

    std::cout << std::endl;
    std::cout << "Serializing target..." << std::endl;

    rclcpp::SerializedMessage serialized_target;
    // serialized_target.reserve(8u + static_cast<size_t>(sizeof(target)));

    target_serializer.serialize_message(&target, &serialized_target);

    std::cout << "Finished serializing target." << std::endl;

    std::cout << "Serialized target: " << std::endl;
    std::cout << "  size: " << serialized_target.size() << std::endl;
    std::cout << "  capacity: " << serialized_target.capacity() << std::endl;
    
    std::cout << std::endl;

    auto s = serialized_target.get_rcl_serialized_message();
    uint8_t *data = s.buffer;
    
    // Convert to char *
    char *str = new char[s.buffer_length];
    for (size_t i = 0; i < s.buffer_length; i++) {
        str[i] = data[i];
    }

    std::cout << "Serialized target data: " << std::endl;
    std::cout << str << std::endl;

    std::cout << std::endl;

    // Convert back to uint8_t *
    uint8_t *data2 = new uint8_t[s.buffer_length];
    for (size_t i = 0; i < s.buffer_length; i++) {
        data2[i] = str[i];
    }

    std::cout << "Attempt deserialization..." << std::endl;

    rclcpp::SerializedMessage serialized_target_2 = rclcpp::Seri
    serialized_target_2.

    iii_drone_interfaces::msg::Target deserialized_target;

    std::cout << "Deserializing target..." << std::endl;

    target_serializer.deserialize_message(&serialized_target_2, &deserialized_target);

    std::cout << "Finished deserializing target." << std::endl;

    std::cout << "Deserialized target: " << std::endl;
    std::cout << "  reference_frame_id: " << deserialized_target.reference_frame_id << std::endl;
    std::cout << "  target_id: " << deserialized_target.target_id << std::endl;
    std::cout << "  target_transform: " << std::endl;
    std::cout << "    translation: " << std::endl;
    std::cout << "      x: " << deserialized_target.target_transform.translation.x << std::endl;
    std::cout << "      y: " << deserialized_target.target_transform.translation.y << std::endl;
    std::cout << "      z: " << deserialized_target.target_transform.translation.z << std::endl;
    std::cout << "    rotation: " << std::endl;
    std::cout << "      w: " << deserialized_target.target_transform.rotation.w << std::endl;
    std::cout << "      x: " << deserialized_target.target_transform.rotation.x << std::endl;
    std::cout << "      y: " << deserialized_target.target_transform.rotation.y << std::endl;
    std::cout << "      z: " << deserialized_target.target_transform.rotation.z << std::endl;
    std::cout << "  target_type: " << deserialized_target.target_type << std::endl;

    std::cout << std::endl;

    rclcpp::shutdown();

    return 0;
}