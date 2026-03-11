#include "LiDAR/LiDAR_localization_no_imu.hpp"
#include <pcl/common/transforms.h>

namespace g_edf_loc
{

LiDARLocalizationNoIMU::LiDARLocalizationNoIMU(const rclcpp::NodeOptions & options)
: Node("lidar_localization_no_imu", options)
{
  loadParameters();
  RCLCPP_INFO(this->get_logger(), BOLD_GRN "LiDAR Localization (No IMU) Node Initialized" RST);
  
  m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
  
  time_log_file_.open("execution_times.txt", std::ios::out);
  if (time_log_file_.is_open()) {
    time_log_file_ << "Deskew,Downsample,Optimization,Total,OriginalPoints,DownsampledPoints,Iterations" << std::endl;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open execution_times.txt");
  }
  predict_file_.open("predict_data.csv", std::ios::out);
  if (predict_file_.is_open()) {
     predict_file_ << "time,field.header.seq,field.header.stamp,"
                  << "field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,"
                  << "field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,"
                  << "field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,"
                  << "field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z,"
                  << "gbx,gby,gbz,abx,aby,abz\n";
  } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open predict_data.csv");
  }

}

LiDARLocalizationNoIMU::~LiDARLocalizationNoIMU()
{
  if (time_log_file_.is_open()) {
    time_log_file_.close();
  }
  if (predict_file_.is_open()) {
    predict_file_.close();
  }
}

void LiDARLocalizationNoIMU::loadParameters()
{
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
  this->declare_parameter<double>("lidar_frequency", 10.0);
  this->declare_parameter<std::string>("map_path", "");

  this->declare_parameter<double>("m_init_x", 0.0);
  this->declare_parameter<double>("m_init_y", 0.0);
  this->declare_parameter<double>("m_init_z", 0.0);
  this->declare_parameter<double>("m_init_roll", 0.0);
  this->declare_parameter<double>("m_init_pitch", 0.0);
  this->declare_parameter<double>("m_init_yaw", 0.0);
  
  std::string lidar_type;
  double leaf_size, min_range, max_range;

  this->declare_parameter<std::string>("lidar_type", "ouster");
  this->declare_parameter<double>("leaf_size", 0.1);
  this->declare_parameter<double>("min_range", 1.0);
  this->declare_parameter<double>("max_range", 100.0);
  
  this->declare_parameter<int>("solver_max_iter", 75);
  this->declare_parameter<int>("solver_num_threads", 4);
  this->declare_parameter<double>("robust_kernel_scale", 1.0);

  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("map_path", map_path_);

  this->get_parameter("m_init_x", m_init_x);
  this->get_parameter("m_init_y", m_init_y);
  this->get_parameter("m_init_z", m_init_z);
  this->get_parameter("m_init_roll", m_init_roll);
  this->get_parameter("m_init_pitch", m_init_pitch);
  this->get_parameter("m_init_yaw", m_init_yaw);
  this->get_parameter("lidar_type", lidar_type);
  this->get_parameter("leaf_size", leaf_size);
  this->get_parameter("min_range", min_range);
  this->get_parameter("max_range", max_range);
  
  m_x = m_init_x;
  m_y = m_init_y;
  m_z = m_init_z;
  
  tf2::Quaternion q_init;
  q_init.setRPY(m_init_roll, m_init_pitch, m_init_yaw);
  m_q = Eigen::Quaterniond(q_init.w(), q_init.x(), q_init.y(), q_init.z());

  m_x_last = m_init_x;
  m_y_last = m_init_y;
  m_z_last = m_init_z;

  m_vx = m_vy = m_vz = 0.0;

  scan_processor_.init(lidar_type, min_range, max_range, leaf_size);

  int solver_max_iter, solver_num_threads;
  double robust_kernel_scale;
  this->get_parameter("solver_max_iter", solver_max_iter);
  this->get_parameter("solver_num_threads", solver_num_threads);
  this->get_parameter("robust_kernel_scale", robust_kernel_scale);

  solver_ = std::make_shared<DLL6DSolver>(&map_);
  solver_->setMaxIterations(solver_max_iter);
  solver_->setMaxThreads(solver_num_threads);
  solver_->setRobustKernelScale(robust_kernel_scale);
  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("localization/pose", 1);
  deskewed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("localization/deskewed_cloud", 1);
  
  double lidar_freq;
  this->get_parameter("lidar_frequency", lidar_freq);
  lidar_period_ = 1.0 / lidar_freq;

  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  lidar_topic: %s", lidar_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  lidar_period: %.4f (freq: %.1f Hz)", lidar_period_, lidar_freq);
  RCLCPP_INFO(this->get_logger(), "  kernel_scale: %.4f", robust_kernel_scale);


  if (map_path_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Map path is empty. No map loaded.");
  } else {
    std::ifstream f(map_path_.c_str());
    if (f.good()) {
      RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path_.c_str());
      if (map_.loadMapFromBin(map_path_)) {
        RCLCPP_INFO(this->get_logger(), "Map loaded successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map from: %s", map_path_.c_str());
      }
    } else {
       RCLCPP_ERROR(this->get_logger(), "Map file does not exist: %s", map_path_.c_str());
    }
  }

  auto sensor_qos = rclcpp::QoS(rclcpp::KeepAll()).reliable().durability_volatile();

  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, sensor_qos, std::bind(&LiDARLocalizationNoIMU::pointcloudCallback, this, std::placeholders::_1));
}

void LiDARLocalizationNoIMU::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    double scan_time = rclcpp::Time(msg->header.stamp).seconds();
    
    // 1. TF & Preprocessing
    if (!m_tfPointCloudCache) {	
        try {
            m_staticTfPointCloud = m_tfBuffer->lookupTransform(
                base_frame_, msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(2.0)); 
            m_tfPointCloudCache = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    pcl::PointCloud<PointXYZT> pcl_cloud;
    scan_processor_.convertAndAdapt(msg, pcl_cloud);

    Eigen::Matrix4f transform_matrix = utils::getTransformMatrix(m_staticTfPointCloud);
    pcl::PointCloud<PointXYZT> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform_matrix);
    RCLCPP_INFO(this->get_logger(), "DEBUG: Starting Downsample (Points in: %lu)", transformed_cloud.size());

    auto start_ds = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<PointXYZT> cloud_downsampled;
    scan_processor_.downsample(transformed_cloud, cloud_downsampled);
    auto end_ds = std::chrono::high_resolution_clock::now();
    
    std::vector<pcl::PointXYZ> c;
    c.reserve(cloud_downsampled.size());
    for (const auto& pt : cloud_downsampled.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            c.emplace_back(pt.x, pt.y, pt.z);
        }
    }
    RCLCPP_INFO(this->get_logger(), "DEBUG: Downsample Done. Points out: %lu", cloud_downsampled.size());

    int original_points = transformed_cloud.size();
    int downsampled_points = c.size();
    
    // 2. Initialization
    if (!initialized_)
    {
        m_x_last = m_x; m_y_last = m_y; m_z_last = m_z;
        m_vx = 0.0; m_vy = 0.0; m_vz = 0.0;
        
        initialized_ = true;
        m_last_scan_time = scan_time;
        RCLCPP_INFO(this->get_logger(), BOLD_YEL "First frame initialized." RST);
        return;
    }

    double dt_step = scan_time - m_last_scan_time;
    if (dt_step < 0.001) dt_step = 0.001; 

    // 3. Prediction (Static Guess)
    double x_sol = m_x;
    double y_sol = m_y;
    double z_sol = m_z;
    Eigen::Quaterniond q_sol = m_q; 

    // 4. Optimization
    auto start_opt = std::chrono::high_resolution_clock::now();
    
    bool converged = solver_->solve(c, x_sol, y_sol, z_sol, q_sol);
    
    auto end_opt = std::chrono::high_resolution_clock::now();

    // 5. State Update
    if (converged) {
        m_x = x_sol; 
        m_y = y_sol; 
        m_z = z_sol;
        m_q = q_sol;
        
        m_vx = (m_x - m_x_last) / dt_step;
        m_vy = (m_y - m_y_last) / dt_step;
        m_vz = (m_z - m_z_last) / dt_step;
        RCLCPP_WARN(this->get_logger(), "Solver Converged.");

    } else {
        RCLCPP_WARN(this->get_logger(), "Solver failed! Staying still.");
        m_vx = 0.0; m_vy = 0.0; m_vz = 0.0;
    }
    
    m_last_scan_time = scan_time;
    m_x_last = m_x; m_y_last = m_y; m_z_last = m_z;
    
    // 6. Logs & Data
    int iterations = solver_->getFinalNumIterations();
    double time_ds_ms = std::chrono::duration<double, std::milli>(end_ds - start_ds).count();
    double time_opt_ms = std::chrono::duration<double, std::milli>(end_opt - start_opt).count();
    double total_ms = time_ds_ms + time_opt_ms;

    tf2::Quaternion q_tf(m_q.x(), m_q.y(), m_q.z(), m_q.w());
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), 
        "Time: %.1fms (DS: %.1f, Opt: %.1f) | Pose: (%.2f, %.2f, %.2f) | RPY: (%.3f, %.3f, %.3f) | Vel: (%.2f, %.2f)",
        total_ms, time_ds_ms, time_opt_ms,
        m_x, m_y, m_z,
        roll, pitch, yaw,
        m_vx, m_vy);

    if (time_log_file_.is_open()) {
      time_log_file_ << "0," 
                     << time_ds_ms << "," 
                     << time_opt_ms << "," 
                     << total_ms << "," 
                     << original_points << "," << downsampled_points << "," << iterations << std::endl;
    }

    if (predict_file_.is_open()) {
        predict_file_ << std::fixed << std::setprecision(0)
                << scan_time * 1e9 << "," << 0 << "," << scan_time * 1e9 << ","  
                << std::fixed << std::setprecision(9)
                << m_x << "," << m_y << "," << m_z << ","
                << m_q.x() << "," << m_q.y() << "," << m_q.z() << "," << m_q.w() << "," 
                << m_vx << "," << m_vy << "," << m_vz << "," 
                << 0.0 << "," << 0.0 << "," << 0.0 << "," 
                << 0.0 << "," << 0.0 << "," << 0.0 << "\n"; 
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = m_x;
    odom_msg.pose.pose.position.y = m_y;
    odom_msg.pose.pose.position.z = m_z;
    odom_msg.pose.pose.orientation.x = m_q.x();
    odom_msg.pose.pose.orientation.y = m_q.y();
    odom_msg.pose.pose.orientation.z = m_q.z();
    odom_msg.pose.pose.orientation.w = m_q.w();
    odom_msg.twist.twist.linear.x = m_vx;
    odom_msg.twist.twist.linear.y = m_vy;
    odom_msg.twist.twist.linear.z = m_vz;
    pose_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = m_x;
    tf_msg.transform.translation.y = m_y;
    tf_msg.transform.translation.z = m_z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
    
    if (deskewed_pub_->get_subscription_count() > 0) {
        pcl::PointCloud<pcl::PointXYZ> pcl_out;
        pcl_out.width = c.size();
        pcl_out.height = 1;
        pcl_out.points.resize(c.size());
        for(size_t i=0; i<c.size(); ++i) pcl_out.points[i] = c[i];
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(pcl_out, ros_cloud);
        ros_cloud.header.stamp = msg->header.stamp;
        ros_cloud.header.frame_id = base_frame_;
        deskewed_pub_->publish(ros_cloud);
    }
}

}  // namespace g_edf_loc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(g_edf_loc::LiDARLocalizationNoIMU)
