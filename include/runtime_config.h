#ifndef FAR_RIO_RUNTIME_CONFIG_H
#define FAR_RIO_RUNTIME_CONFIG_H

#include <string>

namespace far_rio {

struct RuntimeConfig {
    std::string bag_file;
    std::string radar_topic;
    std::string imu_topic;
    std::string output_directory;

    std::string TrajectoryFile() const { return output_directory + "/trajectory.txt"; }
    std::string TimeLogFile() const { return output_directory + "/time.csv"; }
};

/// Resolve paths relative to the YAML file and prepare output directories.
bool LoadRuntimeConfig(const std::string &config_file, RuntimeConfig &config);

}  // namespace far_rio

#endif
