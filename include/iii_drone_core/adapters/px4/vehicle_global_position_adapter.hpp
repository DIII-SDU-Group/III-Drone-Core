#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_global_position.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {
namespace px4 {

    /**
     * @brief PX4 global position adapter class
     */
    class VehicleGlobalPositionAdapter {
    public:
        /**
         * @brief Default constructor
        */
        VehicleGlobalPositionAdapter();

        /* * 
         * @brief Constructor
         *
         * @param vehicle_global_position_msg PX4 global position message
         */
        VehicleGlobalPositionAdapter(const px4_msgs::msg::VehicleGlobalPosition & vehicle_global_position_msg);

        /**
         * @brief Updates the object from a new PX4 global position message
         *
         * @param vehicle_global_position_msg PX4 global position message   
         * 
         * @return void
         */
        void UpdateFromMsg(const px4_msgs::msg::VehicleGlobalPosition & vehicle_global_position_msg);

        /**
         * @brief Timestamp getter
         *
         * @return ROS2 timestamp
         */
        const rclcpp::Time & stamp() const;

        /**
         * @brief Latitude getter
         * 
         * @return Latitude
         */
        double latitude() const;

        /**
         * @brief Longitude getter
         * 
         * @return Longitude
         */
        double longitude() const;

        /**
         * @brief Altitude getter
         * 
         * @return Altitude
         */
        float altitude() const;

        /**
         * @brief Altitude ellipsoid getter
         * 
         * @return Altitude ellipsoid
         */
        float altitude_ellipsoid() const;

        /**
         * @brief Delta altitude getter
         * 
         * @return Delta altitude
         */
        float delta_altitude() const;

        /**
         * @brief Delta terrain getter
         * 
         * @return Delta terrain
         */
        float delta_terrain() const;

        /**
         * @brief Lat lon reset counter getter
         * 
         * @return Lat lon reset counter
         */
        uint8_t lat_lon_reset_counter() const;

        /**
         * @brief Altitude reset counter getter
         * 
         * @return Altitude reset counter
         */
        uint8_t alt_reset_counter() const;

        /**
         * @brief Terrain reset counter getter
         * 
         * @return Terrain reset counter
         */
        uint8_t terrain_reset_counter() const;

        /**
         * @brief Standard deviation of horizontal position error getter
         * 
         * @return Standard deviation of horizontal position error
         */
        float eph() const;

        /**
         * @brief Standard deviation of vertical position error getter
         * 
         * @return Standard deviation of vertical position error
         */
        float epv() const;

        /**
         * @brief Terrain altitude getter
         * 
         * @return Terrain altitude
         */
        float terrain_alt() const;

        /**
         * @brief Terrain altitude valid getter
         * 
         * @return Terrain altitude valid
         */
        bool terrain_alt_valid() const;

        /**
         * @brief Dead reckoning getter
         * 
         * @return Dead reckoning
         */
        bool dead_reckoning() const;

    private:
        /**
         * @brief ROS2 timestamp
         */
        rclcpp::Time stamp_;

        /**
         * @brief Latitude
         */
        double latitude_;

        /**
         * @brief Longitude
         */
        double longitude_;

        /**
         * @brief Altitude
         */
        float altitude_;

        /**
         * @brief Altitude ellipsoid
         */
        float altitude_ellipsoid_;

        /**
         * @brief Delta altitude
         */
        float delta_altitude_;

        /**
         * @brief Delta terrain
         */
        float delta_terrain_;

        /**
         * @brief Lat lon reset counter
         */
        uint8_t lat_lon_reset_counter_;

        /**
         * @brief Altitude reset counter
         */
        uint8_t alt_reset_counter_;

        /**
         * @brief Terrain reset counter
         */
        uint8_t terrain_reset_counter_;

        /**
         * @brief Standard deviation of horizontal position error
         */
        float eph_;

        /**
         * @brief Standard deviation of vertical position error
         */
        float epv_;

        /**
         * @brief Terrain altitude
         */
        float terrain_alt_;

        /**
         * @brief Terrain altitude valid
         */
        bool terrain_alt_valid_;

        /**
         * @brief Dead reckoning
         */
        bool dead_reckoning_;

    };
}}}