

cmake_minimum_required (VERSION 2.8)

project(hdmap2.0)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -O0 -Wno-unused-variable -pthread")





aux_source_directory(./hdmap HDMAP_SOURCE)
aux_source_directory(./pnc_map PNC_MAP_SOURCE)
aux_source_directory(./config CONFIG_SOURCE)
aux_source_directory(./json JSON_SOURCE)
aux_source_directory(./common COMMMON_SOURCE)



ADD_LIBRARY(map SHARED 
			${HDMAP_SOURCE}
                        ${PNC_MAP_SOURCE}
			${CONFIG_SOURCE}
                        ${JSON_SOURCE}
			${COMMMON_SOURCE}
 			hdmap.cpp map_interface.cpp)







