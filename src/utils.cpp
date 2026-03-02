#include "LiDAR/utils.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>

namespace g_edf_loc
{
namespace utils
{

void convertAndAdapt(const sensor_msgs::msg::PointCloud2::SharedPtr& in_msg, pcl::PointCloud<PointXYZT>& out_cloud){
    pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    pcl::fromROSMsg(*in_msg, out_cloud);
    pcl::console::setVerbosityLevel(old_level);
}

Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped){
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    Eigen::Quaternionf q(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z
    );
    Eigen::Matrix3f rotation = q.toRotationMatrix();
    Eigen::Vector3f translation(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    );

    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = translation;

    return transform;
}

Closest_Filter_Result findClosestFilterData(const std::deque<Filter_Data>& Filter_queue, double target_timestamp, double imu_period) {
    
    Closest_Filter_Result result;
    result.found = false;

    if (Filter_queue.empty()) {
        return result;
    }

    auto it = std::lower_bound(Filter_queue.begin(), Filter_queue.end(), target_timestamp,
        [](const Filter_Data& data, double value) {
            return data.timestamp < value;
        });

    const Filter_Data* closest = nullptr;

    if (it == Filter_queue.begin()) {
        closest = &(*it);
    } 
    else if (it == Filter_queue.end()) {
        closest = &Filter_queue.back();
    } 
    else {
        auto prev_it = std::prev(it);
        if (std::abs(it->timestamp - target_timestamp) < std::abs(prev_it->timestamp - target_timestamp)) {
            closest = &(*it);
        } else {
            closest = &(*prev_it);
        }
    }

    if (closest && std::abs(target_timestamp - closest->timestamp) < 3 * imu_period) {
        result.Filter_data = *closest;
        result.found = true;
    }

    return result;
}

}  // namespace utils
}  // namespace g_edf_loc
