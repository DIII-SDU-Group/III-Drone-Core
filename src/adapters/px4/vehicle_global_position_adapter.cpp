/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/px4/vehicle_global_position_adapter.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::adapters::px4;

VehicleGlobalPositionAdapter::VehicleGlobalPositionAdapter() {
    stamp_ = rclcpp::Time(0);
    latitude_ = 0.0;
    longitude_ = 0.0;
    altitude_ = 0.0;
    altitude_ellipsoid_ = 0.0;
    delta_altitude_ = 0.0;
    delta_terrain_ = 0.0;
    lat_lon_reset_counter_ = 0;
    alt_reset_counter_ = 0;
    terrain_reset_counter_ = 0;
    eph_ = 0.0;
    epv_ = 0.0;
    terrain_alt_ = 0.0;
}

VehicleGlobalPositionAdapter::VehicleGlobalPositionAdapter(const px4_msgs::msg::VehicleGlobalPosition & vehicle_global_position_msg) {
    UpdateFromMsg(vehicle_global_position_msg);
}

void VehicleGlobalPositionAdapter::UpdateFromMsg(const px4_msgs::msg::VehicleGlobalPosition & vehicle_global_position_msg) {
    stamp_ = rclcpp::Time(vehicle_global_position_msg.timestamp * 1000);
    latitude_ = vehicle_global_position_msg.lat;
    longitude_ = vehicle_global_position_msg.lon;
    altitude_ = vehicle_global_position_msg.alt;
    altitude_ellipsoid_ = vehicle_global_position_msg.alt_ellipsoid;
    delta_altitude_ = vehicle_global_position_msg.delta_alt;
    delta_terrain_ = vehicle_global_position_msg.delta_alt;
    lat_lon_reset_counter_ = vehicle_global_position_msg.lat_lon_reset_counter;
    alt_reset_counter_ = vehicle_global_position_msg.alt_reset_counter;
    terrain_reset_counter_ = vehicle_global_position_msg.terrain_reset_counter;
    eph_ = vehicle_global_position_msg.eph;
    epv_ = vehicle_global_position_msg.epv;
    terrain_alt_ = vehicle_global_position_msg.terrain_alt;
}

const rclcpp::Time & VehicleGlobalPositionAdapter::stamp() const {
    return stamp_;
}

double VehicleGlobalPositionAdapter::latitude() const {
    return latitude_;
}

double VehicleGlobalPositionAdapter::longitude() const {
    return longitude_;
}

float VehicleGlobalPositionAdapter::altitude() const {
    return altitude_;
}

float VehicleGlobalPositionAdapter::altitude_ellipsoid() const {
    return altitude_ellipsoid_;
}

float VehicleGlobalPositionAdapter::delta_altitude() const {
    return delta_altitude_;
}

float VehicleGlobalPositionAdapter::delta_terrain() const {
    return delta_terrain_;
}

uint8_t VehicleGlobalPositionAdapter::lat_lon_reset_counter() const {
    return lat_lon_reset_counter_;
}

uint8_t VehicleGlobalPositionAdapter::alt_reset_counter() const {
    return alt_reset_counter_;
}

uint8_t VehicleGlobalPositionAdapter::terrain_reset_counter() const {
    return terrain_reset_counter_;
}

float VehicleGlobalPositionAdapter::eph() const {
    return eph_;
}

float VehicleGlobalPositionAdapter::epv() const {
    return epv_;
}

float VehicleGlobalPositionAdapter::terrain_alt() const {
    return terrain_alt_;
}

bool VehicleGlobalPositionAdapter::terrain_alt_valid() const {
    return terrain_alt_valid_;
}

bool VehicleGlobalPositionAdapter::dead_reckoning() const {
    return dead_reckoning_;
}