/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/powerline_adapter.hpp>

using namespace iii_drone::adapters;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineAdapter::PowerlineAdapter() {

    stamp_ = rclcpp::Clock().now();

    projection_plane_ = createPlane(
        iii_drone::types::point_t::Zero(),
        iii_drone::types::vector_t::UnitX()
    );

}

PowerlineAdapter::PowerlineAdapter(const iii_drone_interfaces::msg::Powerline & msg) {

    UpdateFromMsg(msg);

}

PowerlineAdapter::PowerlineAdapter(
    const rclcpp::Time & stamp,
    const std::vector<SingleLineAdapter> & single_line_adapters,
    const iii_drone::types::plane_t & projection_plane
) {

    stamp_ = stamp;

    for (unsigned int i = 0; i < single_line_adapters.size(); i++) {

        single_line_adapters_.push_back(single_line_adapters[i]);

    }

    projection_plane_ = projection_plane;

}

void PowerlineAdapter::UpdateFromMsg(const iii_drone_interfaces::msg::Powerline & msg) {

    rclcpp::Time stamp;
    bool first_stamp = true;

    for (unsigned int i = 0; i < msg.lines.size(); i++) {

        SingleLineAdapter line_adapter(msg.lines[i]);

        single_line_adapters_.push_back(line_adapter);

        if (first_stamp) {

            stamp = line_adapter.stamp();
            first_stamp = false;

        } else {

            if (line_adapter.stamp() < stamp) {

                stamp = line_adapter.stamp();

            }
        }
    }

    stamp_ = stamp;

    projection_plane_ = ProjectionPlaneAdapter(msg.projection_plane).projection_plane();

}

const iii_drone_interfaces::msg::Powerline PowerlineAdapter::ToMsg() const {

    iii_drone_interfaces::msg::Powerline msg{};

    msg.stamp = stamp_;

    for (unsigned int i = 0; i < single_line_adapters_.size(); i++) {

        msg.lines.push_back(single_line_adapters_[i].ToMsg());

    }

    msg.projection_plane = ProjectionPlaneAdapter(projection_plane_).ToMsg();

    return msg;

}

const std::vector<SingleLineAdapter> PowerlineAdapter::GetVisibleLineAdapters() const {

    std::vector<SingleLineAdapter> visible_line_adapters;

    for (unsigned int i = 0; i < single_line_adapters_.size(); i++) {

        if (single_line_adapters_[i].in_fov()) {

            visible_line_adapters.push_back(single_line_adapters_[i]);

        }
    }

    return visible_line_adapters;

}

const SingleLineAdapter PowerlineAdapter::GetClosestLine(const iii_drone::types::point_t & point) const {

    if (single_line_adapters_.size() == 0) {

        throw std::runtime_error("No lines are contained in the powerline.");

    }

    SingleLineAdapter closest_line = single_line_adapters_[0];
    double min_distance = (point - closest_line.position()).norm();

    for (unsigned int i = 1; i < single_line_adapters_.size(); i++) {

        double distance = (point - single_line_adapters_[i].position()).norm();

        if (distance < min_distance) {

            closest_line = single_line_adapters_[i];
            min_distance = distance;

        }
    }

    return closest_line;

}

const SingleLineAdapter PowerlineAdapter::GetLine(int id) const {

    for (unsigned int i = 0; i < single_line_adapters_.size(); i++) {

        if (single_line_adapters_[i].id() == id) {

            return single_line_adapters_[i];

        }
    }

    throw std::runtime_error("Line not found.");

}

bool PowerlineAdapter::HasLine(int id) const {

    for (unsigned int i = 0; i < single_line_adapters_.size(); i++) {

        if (single_line_adapters_[i].id() == id) {

            return true;

        }
    }

    return false;

}

const rclcpp::Time & PowerlineAdapter::stamp() const {
    return stamp_;
}

const std::vector<SingleLineAdapter> & PowerlineAdapter::single_line_adapters() const {
    return single_line_adapters_;
}

const iii_drone::types::plane_t & PowerlineAdapter::projection_plane() const {
    return projection_plane_;
}