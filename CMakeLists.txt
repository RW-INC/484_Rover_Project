cmake_minimum_required(VERSION 3.16)

project(offline_cluster_csv)

find_package(PCL CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_executable(offline_cluster_csv offline_cluster_csv.cpp)

target_link_libraries(offline_cluster_csv PRIVATE
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)