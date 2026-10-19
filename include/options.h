//
// Created by xiang on 2021/10/8.
//

#ifndef FAR_RIO_OPTIONS_H
#define FAR_RIO_OPTIONS_H

#include <string>

namespace far_rio::options {

/// fixed params
constexpr double INIT_TIME = 0.1;
constexpr double RADAR_POINT_COV = 0.00001;
constexpr int PUBFRAME_PERIOD = 20;
constexpr int NUM_MATCH_POINTS = 5;      // required matched points in current
constexpr int MIN_NUM_MATCH_POINTS = 3;  // minimum matched points in current

/// configurable params
extern int NUM_MAX_ITERATIONS;      // max iterations of ekf
extern float ESTI_PLANE_THRESHOLD;  // plane threshold
extern bool FLAG_EXIT;              // flag for exitting
extern std::string OUTPUT_DIR;      // resolved from output.directory in the YAML configuration

}  // namespace far_rio::options

#endif  // FAR_RIO_OPTIONS_H
