#ifndef LOCALIZATION__LIDAR__UTILS_HPP_
#define LOCALIZATION__LIDAR__UTILS_HPP_

#include "types.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <deque>

namespace g_edf_loc
{
namespace utils
{

void convertAndAdapt(const sensor_msgs::msg::PointCloud2::SharedPtr& in_msg, pcl::PointCloud<PointXYZT>& out_cloud);

Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped);

Closest_Filter_Result findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp, double imu_period);

}  // namespace utils
}  // namespace g_edf_loc

#endif  // LOCALIZATION__LIDAR__UTILS_HPP_
