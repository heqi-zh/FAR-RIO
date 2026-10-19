add_library(${PROJECT_NAME}
        radar_mapping.cc
        runtime_config.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_definitions(-DPCL_NO_PRECOMPILE)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
