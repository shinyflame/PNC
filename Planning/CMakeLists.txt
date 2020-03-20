

cmake_minimum_required (VERSION 2.8)

project(plan2.0)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -O0 -Wno-unused-variable -pthread")


#file(GLOB PLAN_SRC "/home/sxl/Planning/common/*.cpp")
aux_source_directory(./common COMMON_SOURCE)
aux_source_directory(./common/path COMMON_PATH_SOURCE)
aux_source_directory(./common/speed SPEED_SOURCE)
aux_source_directory(./common/trajectory TRAJECTORY_SOURCE)
aux_source_directory(./common/util COMMON_UTIL_SOURCE)
aux_source_directory(./common/vehicle_state VEHICLE_STATE_SOURCE)

aux_source_directory(./config CONFIG_SOURCE)
aux_source_directory(./constraint_checker CONSTRAINT_CHECKER_SOURCE)
aux_source_directory(./json JSON_SOURCE)

aux_source_directory(./lattice/behavior LATTICE_BEHAVIOR_SOURCE)
aux_source_directory(./lattice/trajectory1d LATTICE_TRAJECTORY1D_SOURCE)
aux_source_directory(./lattice/trajectory_generation TRAJECTORY_GENERATION_SOURCE)

aux_source_directory(./map MAP_SOURCE)

aux_source_directory(./math MATH_SOURCE)
aux_source_directory(./math/curve1d MATH_CURVE1D_SOURCE)

aux_source_directory(./planner PLANNER_SOURCE)
aux_source_directory(./planner/lattice PLANNER_LATTICE_SOURCE)

aux_source_directory(./reference_line REFERENCE_LINE_SOURCE)
aux_source_directory(./toolkits/deciders DECIDERS_SOURCE)


ADD_LIBRARY(PlanPublish SHARED ${COMMON_SOURCE}
			${COMMONPATH_SOURCE}
                        ${SPEED_SOURCE}
			${TRAJECTORY_SOURCE}
			${COMMON_UTIL_SOURCE}
                        ${VEHICLE_STATE_SOURCE}
			${CONFIG_SOURCE}
			${CONSTRAINT_CHECKER_SOURCE}
                        ${JSON_SOURCE}
			${LATTICE_BEHAVIOR_SOURCE}
			${LATTICE_TRAJECTORY1D_SOURCE}
                        ${TRAJECTORY_GENERATION_SOURCE}
			${MAP_SOURCE}
			${HDMAP_SOURCE}
                        ${PNC_MAP_SOURCE}
			${MATH_SOURCE}
			${MATH_CURVE1D_SOURCE}
                        ${PLANNER_SOURCE}
			${PLANNER_LATTICE_SOURCE}
                        ${REFERENCE_LINE_SOURCE}
			${DECIDERS_SOURCE}
 			PlanPublish.cpp planning.cpp std_planning.cpp planning_base.cpp)







