#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/adapters/single_line_adapter.hpp>
#include <iii_drone_core/adapters/projection_plane_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    /**
     * @brief Powerline message adapter class
     */
    class PowerlineAdapter {
    public:
        /**
         * @brief Constructor.
         */
        PowerlineAdapter();

        /**
         * @brief Constructor from message.
         * 
         * @param msg Message containing the powerline data.
         */
        PowerlineAdapter(const iii_drone_interfaces::msg::Powerline & msg);

        /**
         * @brief Constructor from members.
         * 
         * @param stamp The timestamp.
         * @param single_line_adapters The line adapters.
         * @param projection_plane The projection plane.
         */
        PowerlineAdapter(
            const rclcpp::Time & stamp,
            const std::vector<SingleLineAdapter> & single_line_adapters,
            const iii_drone::types::plane_t & projection_plane
        );

        /**
         * @brief Updates the powerline adapter object from a message.
         * 
         * @param msg Powerline msg.
        */
        void UpdateFromMsg(const iii_drone_interfaces::msg::Powerline & msg);

        /**
         * @brief Converts the powerline adapter object to a message.
         */
        const iii_drone_interfaces::msg::Powerline ToMsg() const;

        /**
         * @brief Returns the line adapters that are currently in the sensor FOV.
         * 
         * @return The lines that are currently in the sensor FOV.
        */
        const std::vector<SingleLineAdapter> GetVisibleLineAdapters() const;

        /**
         * @brief Stamp getter.
         */
        const rclcpp::Time & stamp() const;

        /**
         * @brief Line adapters getter.
         */
        const std::vector<SingleLineAdapter> & single_line_adapters() const;

        /**
         * @brief Projection plane getter.
         */
        const iii_drone::types::plane_t & projection_plane() const;

    private:
        /**
         * @brief Time stamp for latest change.
        */
        rclcpp::Time stamp_;

        /**
         * @brief The line adapters.
        */
        std::vector<SingleLineAdapter> single_line_adapters_;

        /**
         * @brief The projection plane.
        */
        iii_drone::types::plane_t projection_plane_;
        
    };

} // namespace adapters
} // namespace iii_drone