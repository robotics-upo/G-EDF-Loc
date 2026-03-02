#ifndef LOCALIZATION__LIDAR__SCAN_PROCESSING_HPP_
#define LOCALIZATION__LIDAR__SCAN_PROCESSING_HPP_

#include "types.hpp"
#include "utils.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <deque>
#include <mutex>
#include <vector>
#include <string>
#include <omp.h> // Importante
#include <algorithm>
#include <cmath>
namespace g_edf_loc
{

class ScanProcessing
{
public:
    ScanProcessing();
    ~ScanProcessing();

    void init(std::string lidar_type, double min_range, double max_range, double leaf_size);

    void convertAndAdapt(const sensor_msgs::msg::PointCloud2::SharedPtr& in_msg, pcl::PointCloud<PointXYZT>& out_cloud);

    bool unwrap(pcl::PointCloud<PointXYZT> &in, 
                std::vector<pcl::PointXYZ> &out, 
                double scan_header_time, 
                double lidar_period, 
                const std::string& timestamp_mode,
                const std::deque<Filter_Data>& filter_queue, 
                std::mutex& queue_mutex, 
                double imu_period);
    
    void downsample(const pcl::PointCloud<PointXYZT>& in, pcl::PointCloud<PointXYZT>& out);
private:
    std::string m_lidar_type;
    double m_min_range;
    double m_max_range;
    double m_leaf_size;
};

}  // namespace g_edf_loc

#endif  // LOCALIZATION__LIDAR__SCAN_PROCESSING_HPP_
