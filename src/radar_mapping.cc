#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>
#include <pcl/filters/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>
#include "radar_mapping.h"
#include "utils.h"

namespace far_rio {

bool RadarMapping::InitROS(ros::NodeHandle &nh) {
    if (!LoadParams(nh)) {
        return false;
    }
    SubAndPubToROS(nh);

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    return true;
}

bool RadarMapping::InitOffline(const std::string &config_yaml) {
    LOG(INFO) << "init radar mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

bool RadarMapping::LoadParams(ros::NodeHandle &nh) {
    // get params from param server
    int radar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D radar_T_wrt_IMU;
    common::M3D radar_R_wrt_IMU;

    nh.param<bool>("path_save_en", path_save_en_, true);
    nh.param<bool>("publish/path_publish_en", path_pub_en_, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en_, false);
    nh.param<std::string>("publish/tf_imu_frame", tf_imu_frame_, "body");
    nh.param<std::string>("publish/tf_world_frame", tf_world_frame_, "camera_init");

    nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh.param<std::string>("map_file_path", map_file_path_, "");
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    nh.param<double>("cube_side_length", cube_len_, 200);
    nh.param<float>("mapping/det_range", det_range_, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<int>("preprocess/radar_type", radar_type, 4);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log_, true);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval_, -1);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());

    nh.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

    nh.param<float>("keyframe/keyframe_distance", keyframe_distance, 0.5);
    nh.param<float>("keyframe/keyframe_rotation", keyframe_rotation, 0.25);
    nh.param<float>("keyframe/keyframe_roll", keyframe_roll, 0.2);
    nh.param<float>("keyframe/keyframe_pitch", keyframe_pitch, 0.1);
    nh.param<float>("keyframe/keyframe_yaw", keyframe_yaw, 0.2);
    nh.param<bool>("keyframe/keyframe_all", keyframe_all, false);
    nh.param<bool>("keyframe/keyframe_none", keyframe_none, false);
    nh.param<bool>("keyframe/all_scan_matching", all_scan_matching, false);


    LOG(INFO) << "radar type " << radar_type;
    if (radar_type == static_cast<int>(RadarType::Snail)) {
        preprocess_->SetRadarType(RadarType::Snail);
        LOG(INFO) << "Using Snail Radar";
    } else if (radar_type == static_cast<int>(RadarType::NTU)) {
        preprocess_->SetRadarType(RadarType::NTU);
        LOG(INFO) << "Using NTU Radar";
    } else {
        LOG(ERROR) << "Unsupported radar type: " << radar_type << "; use 4 (Snail) or 5 (NTU)";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = tf_world_frame_;

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    radar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    radar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(radar_T_wrt_IMU, radar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

bool RadarMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int radar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D radar_T_wrt_IMU;
    common::M3D radar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        tf_imu_frame_ = yaml["publish"]["tf_imu_frame"].as<std::string>("body");
        tf_world_frame_ = yaml["publish"]["tf_world_frame"].as<std::string>("camera_init");
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        radar_type = yaml["preprocess"]["radar_type"].as<int>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "radar type " << radar_type;
    if (radar_type == static_cast<int>(RadarType::Snail)) {
        preprocess_->SetRadarType(RadarType::Snail);
        LOG(INFO) << "Using Snail Radar";
    } else if (radar_type == static_cast<int>(RadarType::NTU)) {
        preprocess_->SetRadarType(RadarType::NTU);
        LOG(INFO) << "Using NTU Radar";
    } else {
        LOG(ERROR) << "Unsupported radar type: " << radar_type << "; use 4 (Snail) or 5 (NTU)";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    radar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    radar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(radar_T_wrt_IMU, radar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    gyr_cov_=gyr_cov;
    run_in_offline_ = true;
    return true;
}

void RadarMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    // ROS subscribe initialization
    std::string radar_topic, imu_topic;
    nh.param<std::string>("common/radar_topic", radar_topic, "/radar/points");
    nh.param<std::string>("common/imu_topic", imu_topic, "/imu/data");

    if (preprocess_->GetRadarType() == RadarType::NTU) {
        sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud>(
            radar_topic, 200000, [this](const sensor_msgs::PointCloud::ConstPtr &msg) { StandardPCLCallBack(msg); });
    }else {
        sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>(
            radar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });
    }

    sub_imu_ = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
                                              [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    // ROS publisher init
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = tf_world_frame_;

    pub_radar_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pub_radar_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pub_radar_cloud_effect_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 100000);
    pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 100000);
}

RadarMapping::RadarMapping() {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void RadarMapping::Run() {
    if (!SyncPackages()) {
        return;
    }
    Timer::Evaluate(
    [&, this]() {
    using_doppler=1;
    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        p_imu_->if_filter=5;
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        state_point_ = kf_.get_x();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(&scan_undistort_->points[i], &scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);
        first_radar_time_ = measures_.radar_bag_time_;
        flg_first_scan_ = false;
        keyframe_=3;
        
        last_keyframe.translation()=state_point_.pos;
        last_keyframe.linear()=state_point_.rot.toRotationMatrix();
        return;
    }
    //kf_.update_iterated_dyn_share();
    common::V3D gyr = p_imu_->get_gyro();
    if(gyr.norm()<0.3){
        if(!all_scan_matching){
        kf_.update_iterated_dyn_share();
        }
    }
    else{
        keyframe_=3;
    }
    //if keyframe
    // 判断是否为关键帧，直接在 if 语句里计算
    if ((last_keyframe.inverse() * kf_.get_x().pos).norm() > keyframe_distance
    || Eigen::AngleAxisd(last_keyframe.linear().inverse() * kf_.get_x().rot.toRotationMatrix()).angle() > keyframe_rotation
        || std::abs(gyr[0])> keyframe_roll
    || std::abs(gyr[1])>keyframe_pitch
    || std::abs(gyr[2])>keyframe_yaw
    ) {
        keyframe=true;
        std::cout << "This is a keyframe!" << std::endl;
    } else {
        //std::cout << "This is not a keyframe."
    }

    flg_EKF_inited_ = (measures_.radar_bag_time_ - first_radar_time_) >= options::INIT_TIME;
    
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if(cur_pts<100){
        p_imu_->if_filter=5;
        keyframe=0;
        if (cur_pts < 5) {
            LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
            keyframe=1;  
            return;
        }
    }
    nearest_points_.resize(cur_pts);
    if(keyframe_all || all_scan_matching){
        keyframe=1;
    }
    //keyframe=1;
    if(keyframe || (keyframe_--)>0){
        using_doppler=0;
        scan_down_world_->resize(cur_pts);
        //nearest_points_.resize(cur_pts);
        residuals_.resize(cur_pts, 0);
        range_.resize(cur_pts, 0);
        point_selected_surf_.resize(cur_pts, true);
        plane_coef_.resize(cur_pts, common::V4F::Zero());
        // ICP and iterated Kalman filter update
        Timer::Evaluate(
            [&, this]() {
                // iterated state estimation
                double solve_H_time = 0;
                // update the observation model, will call nn and point-to-plane residual computation
                kf_.update_iterated_dyn_share_modified(options::RADAR_POINT_COV, solve_H_time);

                last_keyframe.translation() = kf_.get_x().pos;
                last_keyframe.linear() = kf_.get_x().rot.toRotationMatrix();
                
            },
            "IEKF Solve and Update");
        //keyframe_=2;
    }
    else{}

    state_point_ = kf_.get_x();
    euler_cur_ = SO3ToEuler(state_point_.rot);
    pos_radar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_R_I;
    //keyframe=true;
    // update local map
    if(keyframe){
        Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping"); 
        //keyframe_--;
        keyframe=false;
    }

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;

    LOG(INFO) << "ba" << kf_.get_x().ba << " bg" << kf_.get_x().bg;
    // publish or save map pcd
    if (run_in_offline_) {
        if (pcd_save_en_) {
            PublishFrameWorld();
        }
        if (path_save_en_) {
            PublishPath(pub_path_);
        }
    } else {
        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        if (scan_pub_en_ || pcd_save_en_) {
            PublishFrameWorld();
        }
        if (scan_pub_en_ && scan_body_pub_en_) {
            PublishFrameBody(pub_radar_cloud_body_);
        }
        if (scan_pub_en_ && scan_effect_pub_en_) {
            PublishFrameEffectWorld(pub_radar_cloud_effect_world_);
        }
    }

    // Debug variables
    frame_num_++;
    },
        "All in one");
}

void RadarMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_radar_) {
                LOG(ERROR) << "radar loop back, clear buffer";
                radar_buffer_.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            radar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_radar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void RadarMapping::StandardPCLCallBack(const sensor_msgs::PointCloud::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_radar_) {
                LOG(ERROR) << "radar loop back, clear buffer";
                radar_buffer_.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            radar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_radar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void RadarMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count_++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if(preprocess_->GetRadarType() == RadarType::NTU){
        msg->angular_velocity.y    = -msg->angular_velocity.y;
        msg->angular_velocity.z    = -msg->angular_velocity.z;
        msg->linear_acceleration.y = -msg->linear_acceleration.y;
        msg->linear_acceleration.z = -msg->linear_acceleration.z;
    }
    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

bool RadarMapping::SyncPackages() {
    if (radar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a radar scan ***/
    if (!radar_pushed_) {
        measures_.radar_ = radar_buffer_.front();
        measures_.radar_bag_time_ = time_buffer_.front();

        if (measures_.radar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            radar_end_time_ = measures_.radar_bag_time_ + radar_mean_scantime_;
        } else if (measures_.radar_->points.back().curvature / double(1000) < 0.5 * radar_mean_scantime_) {
            radar_end_time_ = measures_.radar_bag_time_ + radar_mean_scantime_;
        } else {
            scan_num_++;
            radar_end_time_ = measures_.radar_bag_time_ + measures_.radar_->points.back().curvature / double(1000);
            radar_mean_scantime_ +=
                (measures_.radar_->points.back().curvature / double(1000) - radar_mean_scantime_) / scan_num_;
        }

        measures_.radar_end_time_ = radar_end_time_;
        radar_pushed_ = true;
    }

    if (last_timestamp_imu_ < radar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < radar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > radar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    radar_buffer_.pop_front();
    time_buffer_.pop_front();
    radar_pushed_ = false;
    return true;
}

void RadarMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_R_I.coeffs().transpose() << ", t: " << s.offset_T_R_I.transpose();
}

void RadarMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Radar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void RadarMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {

    if(using_doppler){
        //using doppler
        Timer::Evaluate(
            [&, this]() {
                int cnt_pts = scan_undistort_->size();
                std::vector<size_t> index(cnt_pts);
                for (size_t i = 0; i < index.size(); ++i) {
                    index[i] = i;
                }
                auto R_wl = (s.rot * s.offset_R_R_I).cast<double>();
                //auto t_wl = (s.rot * s.offset_T_R_I + s.pos).cast<double>();
                Eigen::MatrixXd A(index.size(), 3);  // 方程矩阵
                Eigen::VectorXd b(index.size());     // 多普勒数据向量

                for (size_t i = 0; i < index.size(); ++i) {
                    // 获取点数据
                    PointType &point_body = scan_undistort_->points[i];
                    PointType &point_world = scan_undistort_->points[i];
                    
                    // azimuth, elevation，doppler)
                    float azimuth = point_body.azimuth;
                    float elevation = point_body.elevation;
                    float doppler = point_body.doppler;
                    // 计算方向向量
                    float cos_azimuth = cos(azimuth);
                    float sin_azimuth = sin(azimuth);
                    float cos_elevation = cos(elevation);
                    float sin_elevation = sin(elevation);
                    
                    // 构建 A 矩阵 (包含方向信息)
                    A(i, 0) = cos_azimuth * cos_elevation;  // v_x 的方向分量
                    A(i, 1) = sin_azimuth * cos_elevation;  // v_y 的方向分量
                    A(i, 2) = sin_elevation;                // v_z 的方向分量
                    
                    // 构建 b 向量 (多普勒频移)
                    b(i) = doppler;
                }
                //common::V3D w = p_imu_->get_gyro();

                // 使用最小二乘法计算速度 v = (v_x, v_y, v_z)
                Eigen::Vector3d velocity_radar = (A.transpose() * A).ldlt().solve(A.transpose() * b);

                //init
                ekfom_data.z = Eigen::MatrixXd::Zero(3, 1);
                ekfom_data.h_x = Eigen::MatrixXd::Zero(3, 23);
                ekfom_data.h.resize(3);
                ekfom_data.R = Eigen::MatrixXd::Zero(3, 3);
                ekfom_data.h_v = Eigen::MatrixXd::Identity(3, 3);
                
                //res
                //Eigen::Vector3d velocity_world = R_wl * velocity_radar;
                common::M3D angv_crossmat;
                common::V3D gyr = p_imu_->get_gyro();
                common::V3D gyr_vec(gyr[0] - s.bg(0), gyr[1] - s.bg(1), gyr[2] - s.bg(2));
                angv_crossmat << SKEW_SYM_MATRIX(gyr_vec);

                common::V3D res =  velocity_radar - s.offset_R_R_I.toRotationMatrix().transpose() * (s.rot.toRotationMatrix().transpose() * s.vel + angv_crossmat * s.offset_T_R_I);
                // 打印雷达速度
                std::cout << "velocity_radar: " << velocity_radar.transpose() << std::endl;
                // 打印根据旋转矩阵和角速度计算的速度
                common::V3D transformed_velocity = s.offset_R_R_I.toRotationMatrix().transpose() * 
                                                    (s.rot.toRotationMatrix().transpose() * s.vel + angv_crossmat * s.offset_T_R_I);
                std::cout << "Transformed velocity: " << transformed_velocity.transpose() << std::endl;

                ekfom_data.h(0) = res.x();
                ekfom_data.h(1) = res.y();
                ekfom_data.h(2) = res.z();

                //jacobian
                double lambda = 1;
                Eigen::Matrix3d rot_crossmat;
                common::V3D tmp_vel = s.rot.toRotationMatrix().transpose() * s.vel;
                rot_crossmat << SKEW_SYM_MATRIX(tmp_vel); // 当前状态imu系下 点坐标反对称矩阵
                ekfom_data.h_x.block<3, 3>(0,3) = -s.offset_R_R_I.toRotationMatrix().transpose() * rot_crossmat; // diff w.r.t. rot
                // common::V3D tmp_vel = s.vel;
                // rot_crossmat << SKEW_SYM_MATRIX(tmp_vel); // 当前状态imu系下 点坐标反对称矩阵
                // ekfom_data.h_x.block<3, 3>(0,3) = -s.offset_R_R_I.toRotationMatrix().transpose() * s.rot.toRotationMatrix().transpose() * rot_crossmat; // diff w.r.t. rot
                ekfom_data.h_x.block<3, 3>(0,12) = -s.offset_R_R_I.toRotationMatrix().transpose() * s.rot.toRotationMatrix().transpose(); // diff w.r.t. vel
                common::M3D bg_crossmat;
                bg_crossmat << SKEW_SYM_MATRIX(s.offset_T_R_I);
                ekfom_data.h_x.block<3, 3>(0,15) = -s.offset_R_R_I.toRotationMatrix().transpose() * bg_crossmat; // diff w.r.t. bg

                Eigen::Matrix3d tmp_mat = s.offset_R_R_I.toRotationMatrix().transpose() * bg_crossmat;
                Eigen::Matrix3d cov_mat = 0.001 * Eigen::Matrix3d::Identity();
                //cov_mat(0, 0) = wheel_cov;
                // if (gyr_vec.norm() > 0.3){
                //     cov_mat(1, 1) = wheel_velocity * gyr_vec.norm();
                // }else{
                //     cov_mat(1, 1) = nhc_y_cov;
                // }
                // cov_mat(2, 2) = nhc_z_cov;
                cov_mat = cov_mat + tmp_mat * tmp_mat.transpose() * gyr_cov_;
                ekfom_data.R = cov_mat;
                //std::cout<< velocity_world << s.vel <<std::endl;
            },
            "    ObsModel(doppler)");
        //using_doppler=0;
        return;
    }
    
    int cnt_pts = scan_down_body_->size();
    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    
    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_R_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_R_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];
                float range=scan_down_body_->points[i].range;
                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        range_[i]=range;
                        residuals_[i] = ((500.0f-range)/250.0f)*pd2;
                        //residuals_[i] = 1.0f/(0.5f+sqrt(range))*pd2;
                    }
                }
            });
        },
        "    ObsModel (Radar Match)");
    
    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_].head<4>() = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];   // 第 4 个元素
            corr_pts_[effect_feat_num_][4] = range_[i];       // 第 5 个元素
            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);
    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            //std::cout<<"??"<<std::endl;
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_R_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_R_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }
                // 2) 只给这一行雅可比乘上 wi
                ekfom_data.h_x.block<1,12>(i,0) *= (500.0f-corr_pts_[i][4])/250.0f;
                //ekfom_data.h_x.block<1,12>(i,0) *= 1.0f/(0.5f+sqrt(corr_pts_[i][4]));
                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

void RadarMapping::PublishPath(const ros::Publisher pub_path) {
    SetPosestamp(msg_body_pose_);
    msg_body_pose_.header.stamp = ros::Time().fromSec(radar_end_time_);
    msg_body_pose_.header.frame_id = tf_world_frame_;

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path.publish(path_);
    }
}

void RadarMapping::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    odom_aft_mapped_.header.frame_id = tf_world_frame_;
    odom_aft_mapped_.child_frame_id = tf_imu_frame_;
    odom_aft_mapped_.header.stamp = ros::Time().fromSec(radar_end_time_);  // ros::Time().fromSec(radar_end_time_);
    SetPosestamp(odom_aft_mapped_.pose);
    pub_odom_aft_mapped.publish(odom_aft_mapped_);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_aft_mapped_.pose.pose.position.x, odom_aft_mapped_.pose.pose.position.y,
                                    odom_aft_mapped_.pose.pose.position.z));
    q.setW(odom_aft_mapped_.pose.pose.orientation.w);
    q.setX(odom_aft_mapped_.pose.pose.orientation.x);
    q.setY(odom_aft_mapped_.pose.pose.orientation.y);
    q.setZ(odom_aft_mapped_.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odom_aft_mapped_.header.stamp, tf_world_frame_, tf_imu_frame_));
}

void RadarMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr radarCloudWorld;
    if (dense_pub_en_) {
        PointCloudType::Ptr radarCloudFullRes(scan_undistort_);
        int size = radarCloudFullRes->points.size();
        radarCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&radarCloudFullRes->points[i], &radarCloudWorld->points[i]);
        }
    } else {
        radarCloudWorld = scan_down_world_;
    }

    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::PointCloud2 radarCloudmsg;
        pcl::toROSMsg(*radarCloudWorld, radarCloudmsg);
        radarCloudmsg.header.stamp = ros::Time().fromSec(radar_end_time_);
        radarCloudmsg.header.frame_id = tf_world_frame_;
        pub_radar_cloud_world_.publish(radarCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en_) {
        *pcl_wait_save_ += *radarCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(options::OUTPUT_DIR + "/PCD/scans_") + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to " << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

void RadarMapping::PublishFrameBody(const ros::Publisher &pub_radar_cloud_body) {
    int size = scan_undistort_->points.size();
    PointCloudType::Ptr radar_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyRadarToIMU(&scan_undistort_->points[i], &radar_cloud_imu_body->points[i]);
    }

    sensor_msgs::PointCloud2 radarCloudmsg;
    pcl::toROSMsg(*radar_cloud_imu_body, radarCloudmsg);
    radarCloudmsg.header.stamp = ros::Time().fromSec(radar_end_time_);
    radarCloudmsg.header.frame_id = "body";
    pub_radar_cloud_body.publish(radarCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void RadarMapping::PublishFrameEffectWorld(const ros::Publisher &pub_radar_cloud_effect_world) {
    int size = corr_pts_.size();
    PointCloudType::Ptr radar_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToWorld(corr_pts_[i].head<3>(), &radar_cloud->points[i]);
    }
    sensor_msgs::PointCloud2 radarCloudmsg;
    pcl::toROSMsg(*radar_cloud, radarCloudmsg);
    radarCloudmsg.header.stamp = ros::Time().fromSec(radar_end_time_);
    radarCloudmsg.header.frame_id = tf_world_frame_;
    pub_radar_cloud_effect_world.publish(radarCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void RadarMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp tx ty tz qx qy qz qw" << std::endl;
    for (const auto &p : path_.poses) {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template <typename T>
void RadarMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);
    out.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.orientation.w = state_point_.rot.coeffs()[3];
}

void RadarMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_R_I * p_body + state_point_.offset_T_R_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RadarMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_R_I * p_body + state_point_.offset_T_R_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void RadarMapping::PointBodyRadarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_radar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_R_I * p_body_radar + state_point_.offset_T_R_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void RadarMapping::Finish() {
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("scans.pcd");
        std::string all_points_dir(std::string(options::OUTPUT_DIR + "/PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        LOG(INFO) << "current scan saved to " << all_points_dir;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    LOG(INFO) << "finish done";
}
}  // namespace far_rio
