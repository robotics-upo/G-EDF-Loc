#ifndef LOCALIZATION__LIDAR__LIDAR_LOCALIZATION_HPP_
#define LOCALIZATION__LIDAR__LIDAR_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gaussian_map/gaussian_map.hpp"

#include <queue>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

#include "filter/eskf.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include "LiDAR/types.hpp"
#include "LiDAR/utils.hpp"
#include "LiDAR/utils.hpp"
#include "LiDAR/scan_processing.hpp"
#include "solver/solver_analitic.hpp"

namespace g_edf_loc
{

class LiDARLocalization : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LiDARLocalization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LiDARLocalization();

private:
  void loadParameters();


  // Example parameters
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_topic_;
  std::string imu_topic_;
  double imu_period_;
  double lidar_period_;
  std::string map_path_;
  std::string timestamp_mode_;
  double calibration_time_;
  
  // Initial Pose
  double m_init_x, m_init_y, m_init_z;
  double m_init_roll, m_init_pitch, m_init_yaw;
  bool initialized_;


  // Map
  gaussian_map::GaussianMap map_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool m_tfImuCache;
  geometry_msgs::msg::TransformStamped m_staticTfImu;
  geometry_msgs::msg::TransformStamped m_staticTfPointCloud;
  bool m_tfPointCloudCache;

  // ESKF
  ESKF eskf;
  double m_x, m_y, m_z;
  Eigen::Quaterniond m_q;
  double m_vx, m_vy, m_vz;
  double m_x_last, m_y_last, m_z_last;
  double m_last_scan_time = 0.0;
  bool use_fixed_imu_dt_;
  
  // Scan Processing
  ScanProcessing scan_processor_;
  std::shared_ptr<DLL6DSolver> solver_;
  
  // IMU Parameters
  double gyr_dev_, gyr_rw_dev_, acc_dev_, acc_rw_dev_;
  
  std::deque<Filter_Data> Filter_filtered_queue_;
  std::mutex Filter_queue_mutex_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscriber_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;


  // Callbacks (Wrappers for queueing)
  void imuCallbackWrapper(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pointcloudCallbackWrapper(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


  // Processing methods
  void processQueues();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  

  


  // Queue management
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::thread processing_thread_;
  bool stop_processing_;
  int message_processed_;
  int message_count_;

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pcl_queue_;

  std::ofstream time_log_file_;
};

}  // namespace g_edf_loc

#endif  // LOCALIZATION__LIDAR__LIDAR_LOCALIZATION_HPP_
