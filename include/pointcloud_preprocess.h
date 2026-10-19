#ifndef FAR_RIO_POINTCLOUD_PROCESSING_H
#define FAR_RIO_POINTCLOUD_PROCESSING_H

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "common_lib.h"

namespace far_rio {

// Preserve the numeric values used by the existing radar configurations.
enum class RadarType { Snail = 4, NTU = 5 };

/// Convert radar messages to the common point format.
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Process(const sensor_msgs::PointCloud::ConstPtr &msg, PointCloudType::Ptr &pcl_out);

    RadarType GetRadarType() const { return radar_type_; }
    void SetRadarType(RadarType type) { radar_type_ = type; }

   private:
    void Snail_radar(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void NTU_radar(const sensor_msgs::PointCloud::ConstPtr &msg);

    PointCloudType cloud_out_;
    RadarType radar_type_ = RadarType::Snail;
};
}  // namespace far_rio

#endif
