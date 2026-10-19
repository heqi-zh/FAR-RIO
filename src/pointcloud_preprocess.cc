#include "pointcloud_preprocess.h"

#include <cmath>
#include <pcl_conversions/pcl_conversions.h>

namespace far_rio {

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    Snail_radar(msg);
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    NTU_radar(msg);
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Snail_radar(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    
    // 解析 PointCloud2 数据
    pcl::PointCloud<PointType> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for(int i = 0; i < plsize; i++)
    {
        const auto& point = pl_orig.points[i];
        
        // 检查点是否有效
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) 
            continue;

        // int print_count = std::min(10, plsize);
        // for(int i = 0; i < plsize; i++)
        // {
        //     const auto& point = pl_orig.points[i];
            
        //     // 检查点是否有效
        //     if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        //         if (i < print_count) {
        //             ROS_WARN_STREAM("Point " << i << " has invalid coordinates: " 
        //                 << point.x << ", " << point.y << ", " << point.z);
        //         }
        //         continue;
        //     }

        //     // 打印前几个点的详细信息
        //     if (i < print_count) {
        //         std::cout<<"=== Point " << i << " ==="<<std::endl;
        //         ROS_INFO_STREAM("Basic coordinates - x: " << point.x << ", y: " << point.y << ", z: " << point.z);
        //         ROS_INFO_STREAM("Intensity: " << point.intensity);
        //         ROS_INFO_STREAM("Doppler: " << point.doppler);
        //         ROS_INFO_STREAM("Range_std: " << point.range_std);
        //         ROS_INFO_STREAM("Azimuth_std: " << point.azimuth_std);
        //         ROS_INFO_STREAM("Elevation_std: " << point.elevation_std);
        //         ROS_INFO_STREAM("Doppler_std: " << point.doppler_std);
                
        //         // 计算并显示计算值
        //         float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        //         float azimuth = std::atan2(point.y, point.x);
        //         float elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
                
        //         ROS_INFO_STREAM("Calculated range: " << range);
        //         ROS_INFO_STREAM("Calculated azimuth: " << azimuth << " radians, " << azimuth * 180.0/M_PI << " degrees");
        //         ROS_INFO_STREAM("Calculated elevation: " << elevation << " radians, " << elevation * 180.0/M_PI << " degrees");
        //     }
        // }

        PointType added_pt;
        
        // 直接赋值基本字段
        added_pt.x = point.x;
        added_pt.y = point.y;
        added_pt.z = point.z;
        
        // 赋值其他字段 - 根据您提供的字段定义
        added_pt.intensity = point.intensity;
        float doppler_value = 0.0f;

        // 优先级：优先使用 Doppler（大写），如果为0或无效则使用 doppler（小写）
        if (std::isfinite(point.Doppler) && point.Doppler != 0.0f) {
            doppler_value = point.Doppler;  // 使用大写的Doppler
            // ROS_DEBUG("Using Doppler (uppercase) value: %f", doppler_value);
        } else if (std::isfinite(point.doppler) && point.doppler != 0.0f) {
            doppler_value = point.doppler;  // 使用小写的doppler
            // ROS_DEBUG("Using doppler (lowercase) value: %f", doppler_value);
        } else {
            // 如果两个字段都无效，使用默认值
            doppler_value = 0.0f;
            // ROS_WARN("Both Doppler fields are invalid, using default value 0.0");
        }

        added_pt.doppler = -doppler_value;

        added_pt.doppler = -doppler_value;
        added_pt.range_std = (point.range_std > 0) ? point.range_std : 0.86;
        added_pt.azimuth_std = (point.azimuth_std > 0) ? point.azimuth_std : 0.44 * (M_PI / 180.0);
        added_pt.elevation_std = (point.elevation_std > 0) ? point.elevation_std : 0.175 * (M_PI / 180.0);
        added_pt.doppler_std = (point.doppler_std > 0) ? point.doppler_std : 0.09;
        
        // 设置默认值（如果需要）
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0;
        
        // 计算 range 和 azimuth, elevation（如果需要）
        // 注意：根据您的字段定义，range 字段不存在，可能需要计算
        added_pt.range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        
        // 计算方位角和高低角
        added_pt.azimuth = std::atan2(point.y, point.x);
        added_pt.elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
        
        cloud_out_.points.push_back(added_pt);
    }
}

void PointCloudPreprocess::NTU_radar(const sensor_msgs::PointCloud::ConstPtr &msg) {
      // 1) 找到 range 和 doppler 在 channels[] 中的索引
    // int idx_range   = -1;
    // int idx_doppler = -1;
    // for (size_t i = 0; i < msg->channels.size(); ++i) {
    //     const auto &ch = msg->channels[i];
    //     if (ch.name == "Range")   idx_range   = i;
    //     if (ch.name == "Doppler") idx_doppler = i;
    // }
    // ROS_INFO("--> idx_range = %d, idx_doppler = %d", idx_range, idx_doppler);

    cloud_out_.clear();
    int plsize = msg->points.size();
    cloud_out_.reserve(plsize);

    for(int i = 0; i < msg->points.size(); i++)
    {
        // cout << i << ":    " <<msg->points[i].x<<endl;
        if (msg->points[i].x == NAN || msg->points[i].y == NAN || msg->points[i].z == NAN) continue;
        if (msg->points[i].x == INFINITY || msg->points[i].y == INFINITY || msg->points[i].z == INFINITY) continue;
        PointType added_pt;
        added_pt.x = msg->points[i].x;
        added_pt.y = msg->points[i].y;
        added_pt.z = msg->points[i].z;
        added_pt.intensity = 0;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.range= msg->channels[1].values[i];
        added_pt.range_std=0.86;
        added_pt.doppler= -msg->channels[0].values[i];
        added_pt.doppler_std=0.09;
        added_pt.azimuth=-msg->channels[3].values[i]* (M_PI / 180.0);
        added_pt.azimuth_std=0.7*(M_PI/180);
        added_pt.elevation=-msg->channels[4].values[i]* (M_PI / 180.0);
        added_pt.elevation_std=0.175 * (M_PI / 180.0);
        cloud_out_.points.push_back(added_pt);
        added_pt.curvature = 0; 
    }
}

}  // namespace far_rio
