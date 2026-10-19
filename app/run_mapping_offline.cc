//
// Created by xiang on 2021/10/9.
//
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <csignal>

#include "radar_mapping.h"
#include "runtime_config.h"
#include "utils.h"

void SigHandle(int sig) {
    far_rio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "far_rio_offline");
    ros::NodeHandle private_nh("~");
    std::string config_file;
    if (!private_nh.getParam("config_file", config_file) || config_file.empty()) {
        LOG(ERROR) << "Set config_file in launch/offline.launch.";
        return -1;
    }
    far_rio::RuntimeConfig runtime;
    if (!far_rio::LoadRuntimeConfig(config_file, runtime)) {
        return -1;
    }
    if (runtime.bag_file.empty()) {
        LOG(ERROR) << "Set input.bag_file in " << config_file;
        return -1;
    }

    auto radar_mapping = std::make_shared<far_rio::RadarMapping>();
    if (!radar_mapping->InitOffline(config_file)) {
        LOG(ERROR) << "radar mapping init failed.";
        return -1;
    }
    signal(SIGINT, SigHandle);

    rosbag::Bag bag;
    try {
        bag.open(runtime.bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException &error) {
        LOG(ERROR) << "Failed to open " << runtime.bag_file << ": " << error.what();
        return -1;
    }
    const std::vector<std::string> topics{runtime.radar_topic, runtime.imu_topic};
    for (const rosbag::MessageInstance &message : rosbag::View(bag, rosbag::TopicQuery(topics))) {
        if (!ros::ok() || far_rio::options::FLAG_EXIT) {
            break;
        }
        auto point_cloud_msg = message.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg) {
            far_rio::Timer::Evaluate(
                [&radar_mapping, &point_cloud_msg]() {
                    radar_mapping->StandardPCLCallBack(point_cloud_msg);
                    radar_mapping->Run();
                },
                "Radar Mapping Single Run");
            continue;
        }
        auto ntu_cloud_msg = message.instantiate<sensor_msgs::PointCloud>();
        if (ntu_cloud_msg) {
            far_rio::Timer::Evaluate(
                [&radar_mapping, &ntu_cloud_msg]() {
                    radar_mapping->StandardPCLCallBack(ntu_cloud_msg);
                    radar_mapping->Run();
                },
                "Radar Mapping Single Run");
            continue;
        }
        auto imu_msg = message.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
            radar_mapping->IMUCallBack(imu_msg);
        }
    }

    LOG(INFO) << "finishing mapping";
    radar_mapping->Finish();
    const double mean_time = far_rio::Timer::GetMeanTime("Radar Mapping Single Run");
    if (mean_time > 0.0) {
        LOG(INFO) << "FAR-RIO average FPS: " << 1000.0 / mean_time;
    }
    radar_mapping->Savetrajectory(runtime.TrajectoryFile());
    far_rio::Timer::PrintAll();
    far_rio::Timer::DumpIntoFile(runtime.TimeLogFile());
    return 0;
}
