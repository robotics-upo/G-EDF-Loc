#include "LiDAR/scan_processing.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <iostream>
#include <map>
#include <tuple>

namespace g_edf_loc
{

ScanProcessing::ScanProcessing() 
    : m_lidar_type("ouster"), m_min_range(1.0), m_max_range(100.0), m_leaf_size(0.1)
{
}

ScanProcessing::~ScanProcessing()
{
}

void ScanProcessing::init(std::string lidar_type, double min_range, double max_range, double leaf_size)
{
    m_lidar_type = lidar_type;
    m_min_range = min_range;
    m_max_range = max_range;
    m_leaf_size = leaf_size;
}

void ScanProcessing::convertAndAdapt(const sensor_msgs::msg::PointCloud2::SharedPtr& in_msg, pcl::PointCloud<PointXYZT>& out_cloud)
{
    pcl::console::VERBOSITY_LEVEL old_level = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    pcl::fromROSMsg(*in_msg, out_cloud);
    pcl::console::setVerbosityLevel(old_level);
}
bool ScanProcessing::unwrap(pcl::PointCloud<PointXYZT> &in, 
                            std::vector<pcl::PointXYZ> &out, 
                            double scan_header_time, 
                            double lidar_period_param, 
                            const std::string& timestamp_mode, // "TIME_FROM_PTP_1588", "TIME_FROM_ROS_TIME", etc.
                            const std::deque<Filter_Data>& filter_queue, 
                            std::mutex& queue_mutex, 
                            double /*imu_period*/)
{
    out.clear(); 
    out.reserve(in.points.size());

    // =========================================================
    // 1. REAL DURATION ANALYSIS (Hz Robustness)
    // =========================================================
    bool has_time_channel = false;
    double max_rel_time = 0.0;

    for (const auto& pt : in.points) {
        if (!std::isfinite(pt.x)) continue;
        
        double t_val = 0.0;
        bool valid_pt_time = false;

        // OUSTER Case (t = nanoseconds relative to start)
        if (m_lidar_type == "ouster" && pt.t > 0) {
            t_val = static_cast<double>(pt.t) * 1e-9;
            valid_pt_time = true;
        } 
        // HESAI Case (timestamp = absolute seconds)
        else if (m_lidar_type == "hesai") {
            t_val = pt.timestamp - scan_header_time;
            valid_pt_time = true;
        }

        if (valid_pt_time) {
            has_time_channel = true;
            if (t_val > max_rel_time) max_rel_time = t_val;
        }
    }

    // Effective scan duration
    double actual_duration = 0.0;
    if (has_time_channel && max_rel_time > 0.001) {
        actual_duration = max_rel_time; // Real measurement (Robust to 7.5Hz)
        
        // Specific correction for HESAI if absolute times overflow
        if (m_lidar_type == "hesai" && std::abs(actual_duration) > 1.0) {
            actual_duration = lidar_period_param; 
        }
    } else {
        actual_duration = lidar_period_param; // Fallback
    }

    if (actual_duration > 0.3) actual_duration = lidar_period_param;

    // =========================================================
    // 2. DEFINE REAL SCAN START (Ouster Mapping)
    // =========================================================
    
    double scan_start_time_abs = scan_header_time;

    // OUSTER DRIVER NAMING LOGIC:
    // If mode is "TIME_FROM_ROS_TIME", stamp is when msg arrives -> END of scan.
    if (timestamp_mode == "TIME_FROM_ROS_TIME") {
        scan_start_time_abs = scan_header_time - actual_duration;
    } 
    // If mode is "TIME_FROM_PTP_1588", "TIME_FROM_INTERNAL_OSC", etc.
    // Stamp is hardware -> START of scan (Default Ouster behavior).
    else {
        // We assume Start of Scan by default for PTP/GPS/Internal
        scan_start_time_abs = scan_header_time;
    }

    // Target is always the physical end of the sweep
    double target_deskew_time = scan_start_time_abs + actual_duration;

    // =========================================================
    // 3. GET IMU SNAPSHOT
    // =========================================================
    std::vector<Filter_Data> imu_snapshot;
    imu_snapshot.reserve(200);
    
    double search_start = scan_start_time_abs - 0.05;
    double search_end   = target_deskew_time + 0.02; 

    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        
        if (filter_queue.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("ScanDebug"), "UNWRAP FAILED: Filter queue is EMPTY!");
            return false;
        }

        double last_imu_time = filter_queue.back().timestamp;
        
        // 2. Detailed Sync Error Log
        // We use a temporary "ScanDebug" logger to show up in ROS console
        if (last_imu_time < target_deskew_time - 0.005) {
            auto logger = rclcpp::get_logger("ScanDebug");
            double missing_time = target_deskew_time - last_imu_time;

            RCLCPP_ERROR(logger, "------------------------------------------------");
            RCLCPP_ERROR(logger, "[UNWRAP ERROR] Sync Failed (Missing Future IMU)");
            RCLCPP_ERROR(logger, "  -> Scan Header Time:    %.6f", scan_header_time);
            RCLCPP_ERROR(logger, "  -> Actual Duration:     %.6f", actual_duration);
            RCLCPP_ERROR(logger, "  -> Target Deskew Time:  %.6f (Header + Duration)", target_deskew_time);
            RCLCPP_ERROR(logger, "  -> Last IMU Available:  %.6f", last_imu_time);
            RCLCPP_ERROR(logger, "  -> MISSING DATA:        %.6f sec", missing_time);
            RCLCPP_ERROR(logger, "  -> Solution: Increase margin in processQueues (+0.02)");
            RCLCPP_ERROR(logger, "------------------------------------------------");
            return false; 
        }

        auto it_start = std::lower_bound(filter_queue.begin(), filter_queue.end(), search_start, 
            [](const Filter_Data& a, double t){ return a.timestamp < t; });
        
        if (it_start != filter_queue.begin()) it_start--; 

        for (auto it = it_start; it != filter_queue.end(); ++it) {
            if (it->timestamp > search_end) break;
            imu_snapshot.push_back(*it);
        }
    }

    if (imu_snapshot.size() < 2) return false;

    // =========================================================
    // 4. CALCULATE BASE POSE (T_base) at Target Time
    // =========================================================
    auto it_target = std::lower_bound(imu_snapshot.begin(), imu_snapshot.end(), target_deskew_time, 
         [](const Filter_Data& a, double t){ return a.timestamp < t; });
    
    tf2::Transform T_base;

    if (it_target != imu_snapshot.begin() && it_target != imu_snapshot.end()) {
        const auto& prev = *std::prev(it_target);
        const auto& next = *it_target;
        
        double dt = next.timestamp - prev.timestamp;
        double ratio = (dt > 1e-9) ? (target_deskew_time - prev.timestamp) / dt : 0.0;
        
        tf2::Vector3 p = tf2::Vector3(prev.x, prev.y, prev.z).lerp(tf2::Vector3(next.x, next.y, next.z), ratio);
        tf2::Quaternion q = tf2::Quaternion(prev.qx, prev.qy, prev.qz, prev.qw).slerp(tf2::Quaternion(next.qx, next.qy, next.qz, next.qw), ratio);
        T_base.setOrigin(p); T_base.setRotation(q);
    } else {
        auto& ref = (it_target == imu_snapshot.end()) ? imu_snapshot.back() : imu_snapshot.front();
        T_base.setOrigin({ref.x, ref.y, ref.z});
        T_base.setRotation({ref.qx, ref.qy, ref.qz, ref.qw});
    }
    tf2::Transform T_base_inv = T_base.inverse();

    // =========================================================
    // 5. CORRECTION LOOP
    // =========================================================
    auto it_imu = imu_snapshot.begin();
    int num_points = in.points.size();

    for (int i = 0; i < num_points; ++i)
    {
        const auto& pt = in.points[i];
        if (!std::isfinite(pt.x)) continue;

        // --- A. Point Absolute Time ---
        double point_timestamp_abs = 0.0;

        if (has_time_channel) {
            if (m_lidar_type == "ouster") {
                // Relative to calculated START
                point_timestamp_abs = scan_start_time_abs + static_cast<double>(pt.t) * 1e-9;
            } 
            else if (m_lidar_type == "hesai") {
                // Hesai is absolute, ignore relative start/end calculation
                point_timestamp_abs = pt.timestamp;
            }
        } else {
            // Linear Interpolation (Fallback)
            double ratio = static_cast<double>(i) / static_cast<double>(num_points);
            point_timestamp_abs = scan_start_time_abs + (ratio * actual_duration);
        }

        // --- B. Find IMU ---
        while (std::next(it_imu) != imu_snapshot.end() && std::next(it_imu)->timestamp < point_timestamp_abs) {
            it_imu++;
        }
        
        auto next_it = std::next(it_imu);
        tf2::Transform T_i;

        // --- C. Interpolate Pose (T_i) ---
        if (next_it == imu_snapshot.end()) {
            T_i.setOrigin({it_imu->x, it_imu->y, it_imu->z});
            T_i.setRotation({it_imu->qx, it_imu->qy, it_imu->qz, it_imu->qw});
        } 
        else {
            const auto& prev = *it_imu;
            const auto& next = *next_it;
            double dt = next.timestamp - prev.timestamp;
            double ratio = (dt > 1e-9) ? (point_timestamp_abs - prev.timestamp) / dt : 0.0;
            
            tf2::Vector3 p_i = tf2::Vector3(prev.x, prev.y, prev.z).lerp(tf2::Vector3(next.x, next.y, next.z), ratio);
            tf2::Quaternion q_i = tf2::Quaternion(prev.qx, prev.qy, prev.qz, prev.qw).slerp(tf2::Quaternion(next.qx, next.qy, next.qz, next.qw), ratio);
            T_i.setOrigin(p_i); T_i.setRotation(q_i);
        }

        // --- D. Project ---
        tf2::Vector3 pt_vec(pt.x, pt.y, pt.z);
        tf2::Vector3 pt_corrected = (T_base_inv * T_i) * pt_vec;

        out.emplace_back(pt_corrected.x(), pt_corrected.y(), pt_corrected.z());
    }

    return true;
}
void ScanProcessing::downsample(const pcl::PointCloud<PointXYZT>& in, pcl::PointCloud<PointXYZT>& out)
{
    // 1. Precalcular distancias al cuadrado para optimizar el rendimiento (evitar std::sqrt)
    double min_range_sq = m_min_range * m_min_range;
    double max_range_sq = m_max_range * m_max_range;

    // 2. Caso donde NO hay downsample (leaf_size muy pequeño) pero SÍ queremos aplicar el filtro de rango
    if (m_leaf_size <= 0.001) {
        out.points.clear();
        out.points.reserve(in.points.size());
        
        for (const auto& p : in.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            
            // Filtro de distancia
            double distance_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            if (distance_sq >= min_range_sq && distance_sq <= max_range_sq) {
                out.points.push_back(p);
            }
        }
        
        out.width = out.points.size();
        out.height = 1;
        out.is_dense = true;
        return;
    }

    // 3. Caso normal: Voxel Grid + Filtro de rango
    std::map<std::tuple<int, int, int>, PointXYZT> grid;
    double inv_leaf_size = 1.0 / m_leaf_size;

    for (const auto& p : in.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        
        // Aplicar el filtro de rango ANTES de hacer los cálculos del Voxel
        double distance_sq = p.x * p.x + p.y * p.y + p.z * p.z;
        if (distance_sq < min_range_sq || distance_sq > max_range_sq) continue;
        
        int ix = static_cast<int>(std::floor(p.x * inv_leaf_size));
        int iy = static_cast<int>(std::floor(p.y * inv_leaf_size));
        int iz = static_cast<int>(std::floor(p.z * inv_leaf_size));

        auto idx = std::make_tuple(ix, iy, iz);
        
        // Guardar solo si el voxel está vacío (Estrategia del primer punto)
        if (grid.find(idx) == grid.end()) {
            grid[idx] = p;
        }
    }

    // 4. Reconstruir la nube de salida
    out.points.clear();
    out.points.reserve(grid.size());
    for (const auto& kv : grid) {
        out.points.push_back(kv.second);
    }
    out.width = out.points.size();
    out.height = 1;
    out.is_dense = true;
}
}  // namespace g_edf_loc
