//
// Created by xiang on 2021/10/8.
//
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

    ros::init(argc, argv, "far_rio");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string config_file;
    if (!private_nh.getParam("config_file", config_file) || config_file.empty()) {
        LOG(ERROR) << "Set config_file in the launch file, for example launch/Snail.launch.";
        return -1;
    }
    far_rio::RuntimeConfig runtime;
    if (!far_rio::LoadRuntimeConfig(config_file, runtime)) {
        return -1;
    }

    auto radar_mapping = std::make_shared<far_rio::RadarMapping>();
    if (!radar_mapping->InitROS(nh)) {
        LOG(ERROR) << "FAR-RIO initialization failed.";
        return -1;
    }

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    while (ros::ok() && !far_rio::options::FLAG_EXIT) {
        ros::spinOnce();
        radar_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    radar_mapping->Finish();
    radar_mapping->Savetrajectory(runtime.TrajectoryFile());
    far_rio::Timer::PrintAll();
    far_rio::Timer::DumpIntoFile(runtime.TimeLogFile());
    return 0;
}
