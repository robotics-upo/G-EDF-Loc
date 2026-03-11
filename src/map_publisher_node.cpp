#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "gaussian_map/gaussian_map.hpp"

using namespace std::chrono_literals;

class MapPublisherNode : public rclcpp::Node
{
public:
    MapPublisherNode() : Node("map_publisher_node")
    {
        // Parameters
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("resolution", 0.5);
        this->declare_parameter("iso_threshold", 0.001);
        this->declare_parameter("output_filename", "");

        map_path_ = this->get_parameter("map_path").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        iso_threshold_ = this->get_parameter("iso_threshold").as_double();
        output_filename_ = this->get_parameter("output_filename").as_string();

        // Initialize Map
        if (map_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'map_path' is empty!");
        } else {
             if (!map_.loadMapFromBin(map_path_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map from: %s", map_path_.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Map loaded successfully.");
            }
        }

        // Publisher
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visualization_map", qos);

        // Service
        generate_service_ = this->create_service<std_srvs::srv::Trigger>(
            "generate_map",
            std::bind(&MapPublisherNode::generateMapService, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Map Publisher Node Ready. Call service /generate_map to visualize.");
    }

private:

    void generateMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        // Define Bounds
        
        double min_x = -40.0; double max_x = 40.0;
        double min_y = -40.0; double max_y = 40.0;
        double min_z = -50.0;  double max_z = 500.0;



        RCLCPP_INFO(this->get_logger(), "Generando mapa completo en rango de búsqueda: X[%.0f, %.0f] Y[%.0f, %.0f]", min_x, max_x, min_y, max_y);
        RCLCPP_INFO(this->get_logger(), "Resolución: %.3f m | Threshold: %.4f", resolution_, iso_threshold_);

        // Resolution Protection
        if (resolution_ < 0.1) {
            RCLCPP_WARN(this->get_logger(), "Resolution < 0.1 detected. Adjusting to 0.2m.");
            resolution_ = 0.2;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Memory Estimation
        cloud->points.reserve(200000); 

        // Generation Loop
        int checked_voxels = 0;
        int valid_voxels = 0;

        for (double x = min_x; x <= max_x; x += resolution_) {
            for (double y = min_y; y <= max_y; y += resolution_) {
                for (double z = min_z; z <= max_z; z += resolution_) {
                    
                    // Check valid voxel
                    if (!map_.isIntoGrid(x, y, z)) continue;

                    double val = map_.getDistanceAt(x, y, z);
                    
                    // Isosurface Filter
                    if (val > iso_threshold_) {
                        pcl::PointXYZI p;
                        p.x = x; 
                        p.y = y; 
                        p.z = z;
                        p.intensity = val;
                        cloud->points.push_back(p);
                        valid_voxels++;
                    }
                }
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->header.frame_id = map_frame_;

        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty cloud generated.");
            response->success = false;
            response->message = "Generated cloud empty.";
            return;
        }

        // Publish
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        map_pub_->publish(msg);
        
        std::string result_msg = "Mapa publicado. Puntos totales: " + std::to_string(cloud->points.size());
        RCLCPP_INFO(this->get_logger(), "%s", result_msg.c_str());

        // Save to File
        if (!output_filename_.empty()) {
            if (pcl::io::savePCDFileBinaryCompressed(output_filename_, *cloud) == 0) {
                 result_msg += " Guardado en " + output_filename_;
                 RCLCPP_INFO(this->get_logger(), "PCD saved: %s", output_filename_.c_str());
            } else {
                 result_msg += " ERROR al guardar fichero.";
                 RCLCPP_ERROR(this->get_logger(), "Failed to save PCD: %s", output_filename_.c_str());
            }
        }

        response->success = true;
        response->message = result_msg;
    }

    gaussian_map::GaussianMap map_;
    std::string map_path_;
    std::string map_frame_;
    double resolution_;
    double iso_threshold_;
    std::string output_filename_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}