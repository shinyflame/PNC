#ifndef WRITE_READ_FILE_H
#define WRITE_READ_FILE_H
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include <iostream>
#include "../common/struct.h"

namespace planning {


int ReadAndWrite(char *LocationFilePath,
                 char *PeridictionFilePath,
                 char *ChassisFilePath,
                 char *TrafficDetectionFilePath,
                 char *PbTrajectoryFilePath,
                 char *OutFusionFilePath  
                 );

int ReadChassisData( FILE* ChResult,
                     canbus::Chassis &chassis_,int &read_times_);

int ReadLocationData( FILE* LoResult,
                      localization::LocalizationEstimate &localization_,
                      int &read_times_
                     );

int ReadTrafficData( FILE* TrResult,
                     perception::TrafficLightDetection  &traffic_detection_,
                     int &read_times_
                    );
int ReadPeridictionData( FILE*  PeResult,
                         prediction::PredictionObstacles &obstacles_ ,
                         int &read_times_
                       );
int ReadPbTrajectoryData( FILE* PbResult,
                          PbTrajectory  &pb_trajectory,
                          int &read_times_ );

///**************************************************************************//
int MakeData(char *ChassisFilePath,
             char *LocationFilePath,
             char *TrafficFilePath,
             char *PeridictionFilePath,
             char *PbTrajectoryFilePath );

int MakeChassisData(FILE* ChResult,
                    uint64_t Timestamp_ms,
                    int FrameNum
                    );

int MakeLocationData(FILE* LoResult,
                     uint64_t Timestamp_ms,
                     int FrameNum
                     );

int MakeTrafficData(FILE* TrResult,
                    uint64_t Timestamp_ms,
                    int32_t  TrafficFram,
                    int FrameNum
                    );
int MakePeridictionData(FILE*  PeResult,
                        uint64_t Timestamp_ms,
                        int32_t  ObstacleNum,
                        int32_t  PolygonNum,
                        int32_t  TrajectoryNum,
                        int32_t  TrajectoryPointNum,
                        int32_t  FrameNum
                        );
int MakeTrajectoryData(FILE* PbResult,
                       PbHeader Timestamp
                       );
///*************************************************************************//
int WriteChassisData(FILE* ChResult,
                    canbus::Chassis chassis_
                    );

int WriteLocationData(FILE* LoResult,
                     localization::LocalizationEstimate localization_
                     );

int WriteTrafficData(FILE* TrResult,
                    perception::TrafficLightDetection  traffic_detection_
                    );
int WritePeridictionData(FILE*  PeResult,
                        prediction::PredictionObstacles obstacles_
                        );
int WriteTrajectoryData(FILE* PbResult,PbTrajectory  pb_trajectory);
///**************************************************************************//
#endif // WRITE_READ_FILE_H

} //namespace planning
