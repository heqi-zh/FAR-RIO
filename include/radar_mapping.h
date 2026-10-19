#ifndef FAR_RIO_RADAR_MAPPING_H
#define FAR_RIO_RADAR_MAPPING_H

#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <condition_variable>
#include <thread>

#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"

namespace far_rio {

class RadarMapping {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

    RadarMapping();
    ~RadarMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "radar mapping deconstruct";
    }

    /// init with ros
    bool InitROS(ros::NodeHandle &nh);

    /// init without ros
    bool InitOffline(const std::string &config_yaml);

    void Run();

    // callbacks of radar and imu
    void StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void StandardPCLCallBack(const sensor_msgs::PointCloud::ConstPtr &msg);
    void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

    // sync radar with imu
    bool SyncPackages();

    /// interface of mtk, customized obseravtion model
    void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    ////////////////////////////// debug save / show ////////////////////////////////////////////////////////////////
    void PublishPath(const ros::Publisher pub_path);
    void PublishOdometry(const ros::Publisher &pub_odom_aft_mapped);
    void PublishFrameWorld();
    void PublishFrameBody(const ros::Publisher &pub_radar_cloud_body);
    void PublishFrameEffectWorld(const ros::Publisher &pub_radar_cloud_effect_world);
    void Savetrajectory(const std::string &traj_file);

    void Finish();

   private:
    template <typename T>
    void SetPosestamp(T &out);

    void PointBodyToWorld(PointType const *pi, PointType *const po);
    void PointBodyToWorld(const common::V3F &pi, PointType *const po);
    void PointBodyRadarToIMU(PointType const *const pi, PointType *const po);

    void MapIncremental();

    void SubAndPubToROS(ros::NodeHandle &nh);

    bool LoadParams(ros::NodeHandle &nh);
    bool LoadParamsFromYAML(const std::string &yaml);

    void PrintState(const state_ikfom &s);

   private:
    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    float det_range_ = 300.0f;
    double cube_len_ = 0;
    double filter_size_map_min_ = 0;
    bool localmap_initialized_ = false;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // radar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // radar-imu rotation
    std::string map_file_path_;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    std::vector<PointVector> nearest_points_;         // nearest points of current scan
    common::VV5F corr_pts_;                           // inlier pts
    common::VV4F corr_norm_;                          // inlier plane norms
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan
    std::vector<float> residuals_;                    // point-to-plane residuals
    std::vector<float> range_;                    // point-to-plane residuals
    std::vector<bool> point_selected_surf_;           // selected points
    common::VV4F plane_coef_;                         // plane coeffs

    /// ros pub and sub stuffs
    ros::Subscriber sub_pcl_;
    ros::Subscriber sub_imu_;
    ros::Publisher pub_radar_cloud_world_;
    ros::Publisher pub_radar_cloud_body_;
    ros::Publisher pub_radar_cloud_effect_world_;
    ros::Publisher pub_odom_aft_mapped_;
    ros::Publisher pub_path_;
    std::string tf_imu_frame_;
    std::string tf_world_frame_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<PointCloudType::Ptr> radar_buffer_;
    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    nav_msgs::Odometry odom_aft_mapped_;

    /// options
    double last_timestamp_radar_ = 0;
    double radar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_radar_time_ = 0.0;
    bool radar_pushed_ = false;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    int pcd_index_ = 0;
    double radar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_num_ = 0, frame_num_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    common::MeasureGroup measures_;                    // sync IMU and radar scan
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;  // esekf
    state_ikfom state_point_;                          // ekf current state
    Eigen::Isometry3d last_keyframe;

    vect3 pos_radar_;                                  // radar position after eskf update
    common::V3D euler_cur_ = common::V3D::Zero();      // rotation in euler angles
    bool extrinsic_est_en_ = true;

    /////////////////////////  debug show / save /////////////////////////////////////////////////////////
    bool run_in_offline_ = false;
    bool path_pub_en_ = true;
    bool scan_pub_en_ = false;
    bool dense_pub_en_ = false;
    bool scan_body_pub_en_ = false;
    bool scan_effect_pub_en_ = false;
    bool pcd_save_en_ = false;
    bool runtime_pos_log_ = true;
    int pcd_save_interval_ = -1;
    bool path_save_en_ = false;
    std::string dataset_;

    PointCloudType::Ptr pcl_wait_save_{new PointCloudType()};  // debug save
    nav_msgs::Path path_;
    geometry_msgs::PoseStamped msg_body_pose_;

    //keyframe
    bool using_doppler;
    bool keyframe=0;
    int keyframe_=0;

    double gyr_cov_;

    //bool keyframe_enable = true;
    float keyframe_distance;
    float keyframe_rotation;
    float keyframe_roll;
    float keyframe_pitch;
    float keyframe_yaw;
    bool keyframe_all;
    bool keyframe_none;
    bool all_scan_matching;
};

}  // namespace far_rio

#endif  // FAR_RIO_RADAR_MAPPING_H
