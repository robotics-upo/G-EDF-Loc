#ifndef LOCALIZATION__LIDAR__TYPES_HPP_
#define LOCALIZATION__LIDAR__TYPES_HPP_

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>


// ANSI Color Codes
#define RST  "\x1B[0m"
#define RED  "\x1B[31m"
#define GRN  "\x1B[32m"
#define YEL  "\x1B[33m"
#define BLU  "\x1B[34m"
#define MAG  "\x1B[35m"
#define CYN  "\x1B[36m"
#define WHT  "\x1B[37m"

#define BOLD "\x1B[1m"
#define UNDL "\x1B[4m"

#define BOLD_RED  "\x1B[1;31m"
#define BOLD_GRN  "\x1B[1;32m"
#define BOLD_YEL  "\x1B[1;33m"
#define BOLD_BLU  "\x1B[1;34m"
#define BOLD_MAG  "\x1B[1;35m"
#define BOLD_CYN  "\x1B[1;36m"
#define BOLD_WHT  "\x1B[1;37m"

namespace g_edf_loc
{

struct Filter_Data {
    double timestamp;
    double x, y, z;
    double qx, qy, qz, qw;
};

struct Closest_Filter_Result {
    Filter_Data Filter_data;
    bool found;
};

struct PointXYZT
{
    PCL_ADD_POINT4D;                  
    double timestamp;                
    uint32_t t;                       
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace g_edf_loc

POINT_CLOUD_REGISTER_POINT_STRUCT(g_edf_loc::PointXYZT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (double, timestamp, timestamp)
    (uint32_t, t, t)
)

#endif  // LOCALIZATION__LIDAR__TYPES_HPP_
