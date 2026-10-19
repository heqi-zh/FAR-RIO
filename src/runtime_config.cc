#include "runtime_config.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "options.h"

namespace far_rio {

bool LoadRuntimeConfig(const std::string &config_file, RuntimeConfig &config) {
    namespace fs = std::filesystem;
    try {
        const fs::path config_directory = fs::absolute(config_file).parent_path();
        const auto yaml = YAML::LoadFile(config_file);
        const auto resolve = [&config_directory](const std::string &value) {
            const fs::path path(value);
            return (path.is_absolute() ? path : config_directory / path).lexically_normal().string();
        };

        const auto input = yaml["input"];
        const std::string bag_file = input ? input["bag_file"].as<std::string>("") : "";
        config.bag_file = bag_file.empty() ? "" : resolve(bag_file);
        config.radar_topic = yaml["common"]["radar_topic"].as<std::string>();
        config.imu_topic = yaml["common"]["imu_topic"].as<std::string>();

        const auto output = yaml["output"];
        const std::string directory = output ? output["directory"].as<std::string>("../output") : "../output";
        if (directory.empty()) {
            LOG(ERROR) << "Set output.directory in " << config_file;
            return false;
        }
        config.output_directory = resolve(directory);
        fs::create_directories(fs::path(config.output_directory) / "Log");
        fs::create_directories(fs::path(config.output_directory) / "PCD");
        options::OUTPUT_DIR = config.output_directory;
        return true;
    } catch (const std::exception &error) {
        LOG(ERROR) << "Failed to load runtime configuration from " << config_file << ": " << error.what();
        return false;
    }
}

}  // namespace far_rio
