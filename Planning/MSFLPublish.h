#ifndef MSFLPUBLISH_H
#define MSFLPUBLISH_H

namespace PublishData {

//==== MSFL: output ====//

struct MSFLoutput
{
    char    head;  // 帧头
    unsigned long long timestamp;
    int gpsWeek;
    double gpsSec;
    double  lat;  // degree
    double  lon;  // degree
    double  height;    // m
    double  velEast;   //m/s
    double  velNorth;   // m/s
    double  velUp;   // m/s
    double  pitch; // degree, 向上为正,   [-90,  90]
    double  roll;  // degree, 右倾为正,    [-180, 180
    double  yaw;   // degree, 北偏东为正,   [0,  360]

    double sigmaLat; // unit:m
    double sigmaLon;// unit:m
    double sigmaHeight;// unit:m
    double  sigmaVelEast;   //m/s
    double  sigmaVelNorth;   // m/s
    double  sigmaVelUp;   // m/s
    double  sigmaPitch; // degree
    double  sigmaRoll;  // degree
    double  sigmaYaw;   // degree

    double  accX; // m/s^2
    double  accY; // m/s^2
    double  accZ;  // m/s^2
    double  gyroX; // degree/s
    double  gyroY;  // degree/s
    double  gyroZ;  // degree/s

    char  navState;  // 组合导航状态; 0: 初始化; bit0 = 1: INS;  bit1 = 1 : GPS; bit2 = 1 : MC; bit3 = 1 : ODE; bit4 = 1 : Lidar; bit5 = 1 : Camera;
    char  gpsState;    //GPS状态 	0: SINGLE_POINT；1： DGPS；2： SBAS；3：PPP； 4： RTK_COM；5: RTK_FIX; 6: OTHER;

    bool  newNavOutFlag;

    char  hardwareFaultInfo; // bit0= 1 : config parameter invalid; bit1 = 1, imu; bit2:gps
    char  inputParamFaultInfo;
    char  alignFaultInfo;
    char  sensorOutLostInfo;
    char  alignState; // alignState
    char  fault2; // tbd  to be defination


};

typedef vector<MSFLoutput> MSFLoutputS;

struct StrFaultInfo
{
    char  hardwareFaultInfo; // bit0= 1 : gyro fault; bit1= 1 : acc fault; bit2 = 1 : gnss recv fault;  bit4 = 1 : power too low; bit5 = 1 : power too high; bit6 = 1 : temperature out of operation range
    char  inputParamFaultInfo; //bit0 = 1: msfl config param invalid;    bit1 = 1: gyro invalid;   bit2 = 1: acc invalid;
    char  alignFaultInfo;
    char  sensorOutLostInfo;
};


struct MSFLtoLidarOutput
{
    char    head;  // 帧头
    unsigned long long timestamp;
    int gpsWeek;
    double gpsSec;
    double  lat;  // degree
    double  lon;  // degree
    double  height;    // m
    double  velEast;   //m/s
    double  velNorth;   // m/s
    double  velUp;   // m/s
    double  pitch; // degree, 向上为正,   [-90,  90]
    double  roll;  // degree, 右倾为正,    [-180, 180
    double  yaw;   // degree, 北偏东为正,   [0,  360]
    double  accX; // m/s^2
    double  accY; // m/s^2
    double  accZ;  // m/s^2
    double  gyroX; // degree/s
    double  gyroY;  // degree/s
    double  gyroZ;  // degree/s

    char  navState;  // 组合导航状态
    char  gpsState;    //GPS状态

    char solnState;
    char solnSVs;//卫星个数

};


}

#endif // MSFLPUBLISH_H

