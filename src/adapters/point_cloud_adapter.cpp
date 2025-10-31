/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/adapters/point_cloud_adapter.hpp>

/*****************************************************************************/
// Implementation
/*****************************************************************************/

using namespace iii_drone::adapters;
using namespace iii_drone::types;

PointCloudAdapter::PointCloudAdapter() {

    stamp_ = rclcpp::Clock().now();

    frame_id_ = "";

    points_ = std::vector<iii_drone::types::point_t>();

}

PointCloudAdapter::PointCloudAdapter(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    const std::vector<iii_drone::types::point_t> & points
) {

    stamp_ = stamp;

    frame_id_ = frame_id;

    points_ = points;

}

PointCloudAdapter::PointCloudAdapter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    stamp_ = msg->header.stamp;

    frame_id_ = msg->header.frame_id;

    points_ = readPoints(*msg);

}

const sensor_msgs::msg::PointCloud2 PointCloudAdapter::ToMsg() const {

    sensor_msgs::msg::PointCloud2 msg{};
    msg.header.stamp = stamp_;
    msg.header.frame_id = frame_id_;

    msg.height = 1;
    msg.width = points_.size();
    msg.point_step = 3*sizeof(float);
    msg.is_bigendian = false;
    msg.is_dense = false;

    msg.fields.resize(3);

    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = sizeof(float);
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 2*sizeof(float);
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.data.resize(points_.size() * msg.point_step);

    for (size_t i = 0; i < points_.size(); i++) {
        memcpy(
            &msg.data[i * msg.point_step + msg.fields[0].offset], 
            &points_[i](0), 
            sizeof(float)
        );
        memcpy(
            &msg.data[i * msg.point_step + msg.fields[1].offset], 
            &points_[i](1), 
            sizeof(float)
        );
        memcpy(
            &msg.data[i * msg.point_step + msg.fields[2].offset], 
            &points_[i](2), 
            sizeof(float)
        );
    }

    msg.row_step = msg.data.size();

    return msg;

}

const rclcpp::Time & PointCloudAdapter::stamp() const {

    return stamp_;

}

const std::string & PointCloudAdapter::frame_id() const {

    return frame_id_;

}

const std::vector<iii_drone::types::point_t> & PointCloudAdapter::points() const {

    return points_;

}

const std::vector<iii_drone::types::point_t> PointCloudAdapter::readPoints(const sensor_msgs::msg::PointCloud2 msg) const {

    std::vector<iii_drone::types::point_t> points;

    unsigned int pcl_size = msg.width;
    const uint8_t *ptr = msg.data.data();
    uint32_t point_step = msg.point_step;

    for (size_t i = 0; i < pcl_size; i++) {

        point_t point(
            *(float *)(ptr + msg.fields[0].offset),
            *(float *)(ptr + msg.fields[1].offset),
            *(float *)(ptr + msg.fields[2].offset)
        );

        ptr += point_step;

        points.push_back(point);

    }

    return points;

}