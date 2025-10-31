#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/projection_plane.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace adapters {

    /**
     * @brief ProjectionPlane message adapter class
     */
    class ProjectionPlaneAdapter {

    public:
        /**
         * @brief Default constructor.
         */
        ProjectionPlaneAdapter();

        /**
         * @brief Constructor from msg.
         * 
         * @param projection_plane_msg ProjectionPlane message.
         */
        ProjectionPlaneAdapter(const iii_drone_interfaces::msg::ProjectionPlane & projection_plane_msg);

        /**
         * @brief Constructor from members.
         * 
         * @param normal The normal vector.
         * @param origin The origin point.
         */
        ProjectionPlaneAdapter(
            const iii_drone::types::vector_t & normal,
            const iii_drone::types::point_t & origin
        );

        /**
         * @brief Constructor from projection plane.
         * 
         * @param projection_plane The projection plane.
         */
        ProjectionPlaneAdapter(const iii_drone::types::plane_t & projection_plane);

        /**
         * @brief Updates the projection plane adapter object from a message.
         * 
         * @param projection_plane_msg ProjectionPlane message.
         */
        void UpdateFromMsg(const iii_drone_interfaces::msg::ProjectionPlane & projection_plane_msg);

        /**
         * @brief Converts the projection plane adapter object to a message.
         */
        const iii_drone_interfaces::msg::ProjectionPlane ToMsg() const;

        /**
         * @brief Projection plane getter.
         */
        const iii_drone::types::plane_t projection_plane() const;

    private:
        /**
         * @brief Projection plane.
         */
        iii_drone::types::plane_t projection_plane_;

    };

} // namespace adapters
} // namespace iii_drone