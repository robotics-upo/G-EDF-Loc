#include "LiDAR/LiDAR_localization.hpp"
#include <pcl/common/transforms.h>

namespace g_edf_loc
{

LiDARLocalization::LiDARLocalization(const rclcpp::NodeOptions & options)
: Node("lidar_localization", options)
{
  loadParameters();

  
  RCLCPP_INFO(this->get_logger(), BOLD_GRN "LiDAR Localization Node Initialized successfully!" RST);
  RCLCPP_INFO(this->get_logger(), BOLD_CYN "Parameters: Odom Frame: %s, Base Frame: %s" RST, odom_frame_.c_str(), base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), BOLD_CYN "Initial Pose: x=%.2f, y=%.2f, z=%.2f" RST, m_init_x, m_init_y, m_init_z);
  RCLCPP_INFO(this->get_logger(), BOLD_CYN "Calibration Time: %.2f s" RST, calibration_time_);

  // Initialize threading
  stop_processing_ = false;
  message_processed_ = 0;
  message_count_ = 0;
  processing_thread_ = std::thread(&LiDARLocalization::processQueues, this);

  RCLCPP_INFO(this->get_logger(), "LiDAR Localization Node initialized");
  
  // Init buffers
  m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
  
  m_tfImuCache = false;
  m_tfPointCloudCache = false;
  initialized_ = false;
  
  // Initialize IMU Filter
  eskf.setup(imu_period_, calibration_time_,
          gyr_dev_, // gyr_dev
          gyr_rw_dev_, // gyr_rw_dev
          acc_dev_, // acc_dev
          acc_rw_dev_,  // acc_rw_dev
          m_init_x, m_init_y, m_init_z,
          m_init_roll, m_init_pitch, m_init_yaw
  );
  


  time_log_file_.open("execution_times.txt", std::ios::out);
  if (time_log_file_.is_open()) {
    time_log_file_ << "Deskew,Downsample,Optimization,Total,OriginalPoints,DownsampledPoints,Iterations" << std::endl;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open execution_times.txt");
  }

}

LiDARLocalization::~LiDARLocalization()
{
  stop_processing_ = true;
  queue_condition_.notify_all();
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  if (time_log_file_.is_open()) {
    time_log_file_.close();
  }
}

void LiDARLocalization::loadParameters()
{
  // Declare parameters with default values
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
  this->declare_parameter<std::string>("imu_topic", "imu/data");
  this->declare_parameter<double>("imu_frequency", 100.0);
  this->declare_parameter<double>("lidar_frequency", 10.0);
  this->declare_parameter<std::string>("map_path", "");
  this->declare_parameter<std::string>("timestamp_mode", "START_OF_SCAN");
  this->declare_parameter<double>("calibration_time", 1.0);
  this->declare_parameter<double>("gyr_dev", 1.0);
  this->declare_parameter<double>("gyr_rw_dev", 1.0);
  this->declare_parameter<double>("acc_dev", 1.0);
  this->declare_parameter<double>("acc_rw_dev", 1.0);
  this->declare_parameter<bool>("use_fixed_imu_dt", true);
  
  // Initial Pose
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
  
  // Solver Parameters (Matched to DRIL)
  this->declare_parameter<int>("solver_max_iter", 75);
  this->declare_parameter<int>("solver_num_threads", 10);
  this->declare_parameter<double>("robust_kernel_scale", 1.0);

  // Get parameters
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("map_path", map_path_);
  this->get_parameter("timestamp_mode", timestamp_mode_);
  this->get_parameter("calibration_time", calibration_time_);
  this->get_parameter("gyr_dev", gyr_dev_);
  this->get_parameter("gyr_rw_dev", gyr_rw_dev_);
  this->get_parameter("acc_dev", acc_dev_);
  this->get_parameter("acc_rw_dev", acc_rw_dev_);
  
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
  this->get_parameter("use_fixed_imu_dt", use_fixed_imu_dt_);
  
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
  
  double imu_freq, lidar_freq;
  this->get_parameter("imu_frequency", imu_freq);
  this->get_parameter("lidar_frequency", lidar_freq);

  // Store as period (1/f)
  imu_period_ = 1.0 / imu_freq;
  lidar_period_ = 1.0 / lidar_freq;


  RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
  RCLCPP_INFO(this->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  lidar_topic: %s", lidar_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  imu_topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  imu_period: %.4f (freq: %.1f Hz)", imu_period_, imu_freq);
  RCLCPP_INFO(this->get_logger(), "  lidar_period: %.4f (freq: %.1f Hz)", lidar_period_, lidar_freq);
  RCLCPP_INFO(this->get_logger(), "  timestamp_mode: %s", timestamp_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "  kernel_scale: %.4f", robust_kernel_scale);


  // Load Map
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

  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, sensor_qos, std::bind(&LiDARLocalization::imuCallbackWrapper, this, std::placeholders::_1));

  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, sensor_qos, std::bind(&LiDARLocalization::pointcloudCallbackWrapper, this, std::placeholders::_1));
}

void LiDARLocalization::imuCallbackWrapper(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  imu_queue_.push(msg);
  queue_condition_.notify_one();
}

void LiDARLocalization::pointcloudCallbackWrapper(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  pcl_queue_.push(msg);
  message_count_++;
  queue_condition_.notify_one();
}

// Función auxiliar para leer el tiempo final sin procesar toda la nube
double get_scan_end_time(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, double theoretical_period)
{
    // 1. Buscar campo de tiempo
    std::string time_field_name = "";
    int time_offset = -1;
    int time_datatype = -1;

    for (const auto& field : msg->fields) {
        if (field.name == "t" || field.name == "timestamp" || field.name == "time" || field.name == "rel_time") {
            time_field_name = field.name;
            time_offset = field.offset;
            time_datatype = field.datatype;
            // Prioridad a campos conocidos
            if (field.name == "timestamp" || field.name == "t") break; 
        }
    }

    double header_time = rclcpp::Time(msg->header.stamp).seconds();

    // --- FALLBACK: Si no hay campo de tiempo, usamos el periodo teórico ---
    if (time_offset == -1) {
        return header_time + theoretical_period;
    }

    // 2. Calcular posición del último punto en memoria
    // data_index = (ultimo_pixel * paso_punto) + offset_tiempo
    size_t num_points = msg->width * msg->height;
    if (num_points == 0) return header_time;

    size_t last_point_idx = num_points - 1;
    size_t data_index = (last_point_idx * msg->point_step) + time_offset;

    // Safety check
    if (data_index + 8 > msg->data.size()) return header_time + theoretical_period;

    // 3. Leer y convertir
    double extracted_time = 0.0;
    bool is_absolute = false;

    // Ouster (uint32 nanosegundos relativos)
    if (time_field_name == "t" && time_datatype == sensor_msgs::msg::PointField::UINT32) {
        uint32_t t_raw = *reinterpret_cast<const uint32_t*>(&msg->data[data_index]);
        extracted_time = static_cast<double>(t_raw) * 1e-9;
    } 
    // Hesai (double segundos absolutos)
    else if (time_field_name == "timestamp" && time_datatype == sensor_msgs::msg::PointField::FLOAT64) {
        double t_raw = *reinterpret_cast<const double*>(&msg->data[data_index]);
        extracted_time = t_raw;
        is_absolute = true;
    }
    // Genérico (float segundos relativos)
    else if (time_datatype == sensor_msgs::msg::PointField::FLOAT32) {
        float t_raw = *reinterpret_cast<const float*>(&msg->data[data_index]);
        extracted_time = static_cast<double>(t_raw);
    }

    if (is_absolute) return extracted_time;
    else return header_time + extracted_time;
}

void LiDARLocalization::processQueues() {
    
    // --- CÁLCULO DINÁMICO DEL MARGEN ---
    // Regla: Necesitamos al menos 1 mensaje de IMU "del futuro" para interpolar.
    // Usamos 1.5 veces el periodo para asegurar que cubrimos el hueco incluso con un poco de lag.
    // Añadimos un suelo de 0.01s (10ms) por si la frecuencia configurada es altísima (ej. 1000Hz) 
    // pero el sistema tiene latencia.
    double dynamic_margin = imu_period_ * 1.5; 
    if (dynamic_margin < 0.01) dynamic_margin = 0.01; 

    // Mensaje de debug para confirmar que se ha adaptado bien
    RCLCPP_INFO(this->get_logger(), "Sync Margin set to: %.4f s (based on IMU freq)", dynamic_margin);

    while (!stop_processing_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // 1. ESPERA (WAIT) - Usa el margen dinámico
        queue_condition_.wait(lock, [this, dynamic_margin]{ 
            if (stop_processing_) return true;
            if (imu_queue_.empty() || pcl_queue_.empty()) return false;
            
            double scan_end_time = get_scan_end_time(pcl_queue_.front(), lidar_period_);
            double last_imu_time = rclcpp::Time(imu_queue_.back()->header.stamp).seconds();
            
            // Esperamos hasta tener datos que cubran "Scan End + Margen"
            return last_imu_time > (scan_end_time + dynamic_margin); 
        });

        if (stop_processing_) break;

        auto pcl_msg = pcl_queue_.front();
        double scan_end_real = get_scan_end_time(pcl_msg, lidar_period_);

        // 2. CONSUMO (POP) - Usa el mismo margen dinámico
        // Esto garantiza que la IMU necesaria para interpolar (la que está justo después del scan)
        // entre en el bloque de procesamiento y no se quede en la cola.
        double integration_limit = scan_end_real + dynamic_margin;

        while (!imu_queue_.empty()) {
            auto imu_msg = imu_queue_.front();
            double imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();

            if (imu_time <= integration_limit) {
                imu_queue_.pop();
                lock.unlock();
                imuCallback(imu_msg); 
                lock.lock();
            } else {
                // Esta IMU es demasiado futura, stop.
                break; 
            }
        }

        pcl_queue_.pop();
        lock.unlock(); 
        
        pointcloudCallback(pcl_msg); 
        
        message_processed_++;
    }
}
void LiDARLocalization::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Pre-cache transform for point-cloud to base frame
    if(!m_tfImuCache)
    {	
        try
        {
            // --- FIX: Limpiar la barra '/' del frame_id ---
            std::string source_frame = msg->header.frame_id;
            if (!source_frame.empty() && source_frame[0] == '/') source_frame = source_frame.substr(1);
            
            // Usamos 'source_frame' limpio en lugar de msg->header.frame_id
            m_staticTfImu = m_tfBuffer->lookupTransform(base_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(2.0));
            m_tfImuCache = true;
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    double current_stamp = rclcpp::Time(msg->header.stamp).seconds();
    static double prev_stamp = -1.0;

    if (prev_stamp < 0.0) {
        prev_stamp = current_stamp;
        return;
    }

    double dt_real = current_stamp - prev_stamp;

    if (dt_real <= 0.0) {
        return; 
    }

    if (dt_real > 1.0) { 
        RCLCPP_WARN(this->get_logger(), "Large IMU gap (%.6f s). Resetting integration.", dt_real);
        prev_stamp = current_stamp;
        return; 
    }

    double dt;
    
    if (use_fixed_imu_dt_) {
        dt = imu_period_; 

    } else {
        dt = dt_real;  
    }

    prev_stamp = current_stamp;

    // Adapt IMU velocity and aceleration to base system reference
    tf2::Quaternion q_static(
        m_staticTfImu.transform.rotation.x,
        m_staticTfImu.transform.rotation.y,
        m_staticTfImu.transform.rotation.z,
        m_staticTfImu.transform.rotation.w
    );
    
    // Rotate angular velocities
    tf2::Vector3 v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    tf2::Matrix3x3 rot_matrix(q_static);
    tf2::Vector3 v_base = rot_matrix * v;

    // Rotate and compensate for accelerations (lever arm effect)
    tf2::Vector3 a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    static tf2::Vector3 v_base_prev = v_base;
    tf2::Vector3 a_base = rot_matrix * a;
    
    // Lever arm compensation
    tf2::Vector3 lever_arm(
        m_staticTfImu.transform.translation.x,
        m_staticTfImu.transform.translation.y,
        m_staticTfImu.transform.translation.z
    );

    a_base += ((v_base - v_base_prev) / dt).cross(lever_arm) 
            + v_base.cross(v_base.cross(lever_arm));
    
    v_base_prev = v_base;

    // Modified msg for imuFilter
    sensor_msgs::msg::Imu modified_msg = *msg;
    modified_msg.angular_velocity.x = v_base.x();
    modified_msg.angular_velocity.y = v_base.y();
    modified_msg.angular_velocity.z = v_base.z();
    modified_msg.linear_acceleration.x = a_base.x();
    modified_msg.linear_acceleration.y = a_base.y();
    modified_msg.linear_acceleration.z = a_base.z();

    // Prediction Step
    if (!eskf.isInit()) {
        static int counter = 0;
        if (counter++ % 100 == 0) RCLCPP_INFO(this->get_logger(), BOLD_YEL "Waiting for IMU calibration/initialization..." RST);
        eskf.initialize(modified_msg); 
    } else {
        eskf.predict(
            v_base.x(), v_base.y(), v_base.z(),
            a_base.x(), a_base.y(), a_base.z(),
            current_stamp, dt);
        eskf.getposition(m_x, m_y, m_z);
        double qx, qy, qz, qw;
        eskf.getQuat(qx, qy, qz, qw);
        m_q = Eigen::Quaterniond(qw, qx, qy, qz);
    }

    // EKF Data Storage for LiDAR Deskewing
    Filter_Data data;
    data.timestamp = current_stamp;
    eskf.getposition(data.x, data.y, data.z);
    double qx, qy, qz, qw;
    eskf.getQuat(qx, qy, qz, qw);
    data.qx = qx; data.qy = qy; data.qz = qz; data.qw = qw;

    {
        std::lock_guard<std::mutex> lock(Filter_queue_mutex_);
        Filter_filtered_queue_.emplace_back(data);
        while(!Filter_filtered_queue_.empty() && (current_stamp - Filter_filtered_queue_.front().timestamp > 5.0)){
            Filter_filtered_queue_.pop_front();
        }
    }
}

void LiDARLocalization::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    double current_stamp_pcl = rclcpp::Time(msg->header.stamp).seconds();
    
    RCLCPP_INFO(this->get_logger(), "DEBUG: Starting Pointcloud Callback");

    auto start_total = std::chrono::high_resolution_clock::now();

    // Pre-cache transform for point-clouds to base frame and transform the pc 
    if (!m_tfPointCloudCache)
    {	
        try
        {
            // --- FIX: Limpiar la barra '/' del frame_id ---
            std::string source_frame = msg->header.frame_id;
            if (!source_frame.empty() && source_frame[0] == '/') source_frame = source_frame.substr(1);
            
            // AHORA SÍ: Usamos 'source_frame' (la variable limpia)
            m_staticTfPointCloud = m_tfBuffer->lookupTransform(
                base_frame_, 
                source_frame, // <--- CAMBIO AQUÍ
                tf2::TimePointZero, 
                tf2::durationFromSec(2.0)); 
            
            m_tfPointCloudCache = true;
            return;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }

    // Wait for filter initialization
    if (!eskf.isInit()) {
        return;
    }

    // Cloud Preproccessing
    pcl::PointCloud<PointXYZT> pcl_cloud;
    scan_processor_.convertAndAdapt(msg, pcl_cloud);

    Eigen::Matrix4f transform_matrix = utils::getTransformMatrix(m_staticTfPointCloud);
    pcl::PointCloud<PointXYZT> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform_matrix);
    
    // --- NUEVO ORDEN: 1. DOWNSAMPLE ---
    double scan_time = rclcpp::Time(msg->header.stamp).seconds();
    
    RCLCPP_INFO(this->get_logger(), "DEBUG: Starting Downsample (Points in: %lu)", transformed_cloud.size());
    auto start_ds = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<PointXYZT> cloud_downsampled;
    // Usamos la nueva sobrecarga que mantiene PointXYZT (y por tanto el tiempo)
    scan_processor_.downsample(transformed_cloud, cloud_downsampled);

    auto end_ds = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "DEBUG: Downsample Done. Points out: %lu", cloud_downsampled.size());

    // --- NUEVO ORDEN: 2. DESKEW (UNWRAP) ---
    RCLCPP_INFO(this->get_logger(), "DEBUG: Starting Deskew");
    
    std::vector<pcl::PointXYZ> c; // Vector final para el solver (ya sin tiempo)
    auto start_deskew = std::chrono::high_resolution_clock::now();

    // Pasamos la nube PEQUEÑA ('cloud_downsampled') al unwrap
    // Esto debería tardar <1ms con OpenMP
    bool unwrap_success = scan_processor_.unwrap(cloud_downsampled, c, scan_time, lidar_period_, timestamp_mode_, Filter_filtered_queue_, Filter_queue_mutex_, imu_period_);
    
    auto end_deskew = std::chrono::high_resolution_clock::now();
    
    // Fallback si falla el unwrap (usamos la nube downsampleada sin corregir)
    if (!unwrap_success) 
    {
        RCLCPP_WARN(this->get_logger(), "Unwrap failed (IMU missing/sync error). Using RAW downsampled cloud.");

        c.clear();
        c.reserve(cloud_downsampled.size());
        for (const auto& pt : cloud_downsampled.points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                c.emplace_back(pt.x, pt.y, pt.z);
            }
        }
    }

    // Actualizamos las variables para el log posterior
    int original_points = transformed_cloud.size();
    int downsampled_points = c.size();
    eskf.getposition(m_x, m_y, m_z);
    double qx, qy, qz, qw;
    eskf.getQuat(qx, qy, qz, qw);
    m_q = Eigen::Quaterniond(qw, qx, qy, qz);
    
    // First Frame Initialization logic (similar to DLIO)
    // Avoid calculating velocity on first frame against far-away init guess
    if (!initialized_)
    {
        // Set last position to current estimate (so next frame dx is small)
        m_x_last = m_x;
        m_y_last = m_y;
        m_z_last = m_z;
        
        initialized_ = true;
        
        // Skip optimization and velocity update for this first frame
        RCLCPP_INFO(this->get_logger(), BOLD_YEL "First frame initialized. Skipping solver to stable velocity." RST);
        return;
    }

    double x_sol = m_x, y_sol = m_y, z_sol = m_z;
    // double roll_sol, pitch_sol, yaw_sol; // Removed as we use m_q directly

    
    // 2. Optimization
    RCLCPP_INFO(this->get_logger(), "DEBUG: Starting Optimization");
    auto start_opt = std::chrono::high_resolution_clock::now();
    
    // Save previous state (optional depending on logic, but adjustYaw is removed)
    // double yaw_prev = yaw_sol; 
    
    RCLCPP_INFO(this->get_logger(), "DEBUG: Solver Initial Guess: (%.2f, %.2f, %.2f) Quat: (%.2f, %.2f, %.2f, %.2f)", 
                x_sol, y_sol, z_sol, m_q.w(), m_q.x(), m_q.y(), m_q.z());
    
    // Check map bounds for initial guess
    // gaussian_map::GaussianMap& grid = solver_->getGrid(); // Can't access grid easily, assume ok
    
    // Pass m_q directly. Solver updates x_sol, y_sol, z_sol and m_q
    solver_->solve(c, x_sol, y_sol, z_sol, m_q);
    auto end_opt = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "DEBUG: Optimization Done");

    // Adjust Yaw - REMOVED per user request and new logic
    // yaw_sol = adjustYaw(yaw_prev, yaw_sol);
    
    // Update State (Important: Update member vars to reflect solver output before EKF uses them?)
    // DRIL updates m_x, m_y, m_z here.
    m_x = x_sol; m_y = y_sol; m_z = z_sol;
    
    // m_q is already updated by solver ref


    // Velocity calculation using actual time difference (dt_scan)
    // This handles cases where the solver lags and dt > lidar_period_
    double dt_scan = lidar_period_; // Default fallback
    if (m_last_scan_time > 0.0) {
        dt_scan = scan_time - m_last_scan_time;
    }
    
    // Safety check for very small dt
    if (dt_scan < 0.001) dt_scan = 0.001;

    double vx_sol = (x_sol - m_x_last) / dt_scan;
    double vy_sol = (y_sol - m_y_last) / dt_scan;
    double vz_sol = (z_sol - m_z_last) / dt_scan;
    
    m_last_scan_time = scan_time;
    
    double v_mod = sqrt(vx_sol*vx_sol + vy_sol*vy_sol + vz_sol*vz_sol);
    RCLCPP_INFO(this->get_logger(), "DEBUG VEL: v_mod=%.2f (dt_scan=%.4f)", v_mod, dt_scan);

    // 4. Update Filter with optimized values
    RCLCPP_INFO(this->get_logger(), "DEBUG: Starting EKF Update");
    // Full update requires tuned solver.
    // D-LIO uses 0.001*0.001 (1e-6) for velocity.
    eskf.update_pose(Eigen::Vector3d(x_sol, y_sol, z_sol), 
                              m_q,
                              0.01 * 0.01,
                              0.025 * 0.025,
                              scan_time); // Variance closer to D-LIO/DRIL logic
    
    RCLCPP_INFO(this->get_logger(), "DEBUG: EKF Update Done");
    
    // Sync member variables with EKF state (DRIL does this)
    eskf.getposition(m_x, m_y, m_z);
    double qx_up, qy_up, qz_up, qw_up;
    eskf.getQuat(qx_up, qy_up, qz_up, qw_up);
    m_q = Eigen::Quaterniond(qw_up, qx_up, qy_up, qz_up);
    
    // Update last state for next velocity calculation
    m_x_last = m_x;
    m_y_last = m_y;
    m_z_last = m_z;
    
    auto end_total = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "Time: Deskew=%.1fms, DS=%.1fms, Opt=%.1fms, Total=%.1fms",
        std::chrono::duration<double, std::milli>(end_deskew - start_deskew).count(),
        std::chrono::duration<double, std::milli>(end_ds - start_ds).count(),
        std::chrono::duration<double, std::milli>(end_opt - start_opt).count(),
        std::chrono::duration<double, std::milli>(end_total - start_total).count());

    int iterations = solver_->getFinalNumIterations();

    if (time_log_file_.is_open()) {
      time_log_file_ << std::chrono::duration<double, std::milli>(end_deskew - start_deskew).count() << ","
                     << std::chrono::duration<double, std::milli>(end_ds - start_ds).count() << ","
                     << std::chrono::duration<double, std::milli>(end_opt - start_opt).count() << ","
                     << std::chrono::duration<double, std::milli>(end_total - start_total).count() << ","
                     << original_points << "," << downsampled_points << "," << iterations << std::endl;
    }

    
    RCLCPP_INFO(this->get_logger(), BOLD_GRN "Updated State -> Pose: (%.2f, %.2f, %.2f) | Vel: (%.2f, %.2f, %.2f)" RST, 
        m_x, m_y, m_z, vx_sol, vy_sol, vz_sol);


    // Publish Pose (Odometry)
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
    
    pose_pub_->publish(odom_msg);

    // Broadcast TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = m_x;
    tf_msg.transform.translation.y = m_y;
    tf_msg.transform.translation.z = m_z;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
    
    // Publish Deskewed Cloud
    if (deskewed_pub_->get_subscription_count() > 0) {
        pcl::PointCloud<pcl::PointXYZ> pcl_out;
        pcl_out.width = c.size();
        pcl_out.height = 1;
        pcl_out.points.resize(c.size());
        for(size_t i=0; i<c.size(); ++i) {
            pcl_out.points[i] = c[i];
        }
        
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(pcl_out, ros_cloud);
        ros_cloud.header.stamp = msg->header.stamp;
        ros_cloud.header.frame_id = base_frame_;
        deskewed_pub_->publish(ros_cloud);
    }

    

}




// function adjustYaw removed
}  // namespace g_edf_loc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(g_edf_loc::LiDARLocalization)
