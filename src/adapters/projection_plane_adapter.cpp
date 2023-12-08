/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/projection_plane_adapter.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::adapters;
using namespace iii_drone::types;

ProjectionPlaneAdapter::ProjectionPlaneAdapter() {

    projection_plane_.normal = vector_t(1.0, 0.0, 0.0);
    projection_plane_.p = point_t(0.0, 0.0, 0.0);

}

ProjectionPlaneAdapter::ProjectionPlaneAdapter(const iii_drone_interfaces::msg::ProjectionPlane & projection_plane_msg) {

    UpdateFromMsg(projection_plane_msg);

}

ProjectionPlaneAdapter::ProjectionPlaneAdapter(
    const iii_drone::types::vector_t & normal,
    const iii_drone::types::point_t & point
) {

    projection_plane_.normal = normal;
    projection_plane_.p = point;

}

ProjectionPlaneAdapter::ProjectionPlaneAdapter(const iii_drone::types::plane_t & projection_plane) {

    projection_plane_ = projection_plane;

}

void ProjectionPlaneAdapter::UpdateFromMsg(const iii_drone_interfaces::msg::ProjectionPlane & projection_plane_msg) {

    projection_plane_.normal = vector_t(
        projection_plane_msg.normal.x,
        projection_plane_msg.normal.y,
        projection_plane_msg.normal.z
    );

    projection_plane_.p = point_t(
        projection_plane_msg.point.x,
        projection_plane_msg.point.y,
        projection_plane_msg.point.z
    );

}

const iii_drone_interfaces::msg::ProjectionPlane ProjectionPlaneAdapter::ToMsg() const {

    iii_drone_interfaces::msg::ProjectionPlane msg{};
    msg.normal.x = projection_plane_.normal.x();
    msg.normal.y = projection_plane_.normal.y();
    msg.normal.z = projection_plane_.normal.z();
    msg.point.x = projection_plane_.p.x();
    msg.point.y = projection_plane_.p.y();
    msg.point.z = projection_plane_.p.z();

    return msg;

}

const iii_drone::types::plane_t ProjectionPlaneAdapter::projection_plane() const {
    return projection_plane_;
}