
#pragma once

#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/basic_binary_iprimitive.hpp>
#include <string> 
#include <sstream>

#include <fstream>
#include <utility>


#include "../HdMapPublish.h"
#include "../MSFLPublish.h"


namespace boost {
namespace serialization {
/*---------------------------------------------------------INPUT----------------------------------*/
/*-------------------MSFLoutput--------------------*/
template<class Archive>
void serialize(Archive & ar, PublishData::MSFLoutput& msfl_output, const unsigned int version)
{
    ar & msfl_output.head;
    ar & msfl_output.timestamp;
    ar & msfl_output.gpsWeek;
    ar & msfl_output.gpsSec;
    ar & msfl_output.lat;
    ar & msfl_output.lon;
    ar & msfl_output.height;
    ar & msfl_output.velEast;
    ar & msfl_output.velNorth;
    ar & msfl_output.velUp;
    ar & msfl_output.pitch;
    ar & msfl_output.roll;
    ar & msfl_output.yaw;

    ar & msfl_output.sigmaLat;
    ar & msfl_output.sigmaLon;
    ar & msfl_output.sigmaHeight;
    ar & msfl_output.sigmaVelEast;
    ar & msfl_output.sigmaVelNorth;
    ar & msfl_output.sigmaVelUp;
    ar & msfl_output.sigmaPitch;
    ar & msfl_output.sigmaRoll;
    ar & msfl_output.sigmaYaw;

    ar & msfl_output.accX;
    ar & msfl_output.accY;
    ar & msfl_output.accZ;
    ar & msfl_output.gyroX;
    ar & msfl_output.gyroY;
    ar & msfl_output.gyroZ;

    ar & msfl_output.navState;
    ar & msfl_output.gpsState;

    ar & msfl_output.newNavOutFlag;

    ar & msfl_output.hardwareFaultInfo;
    ar & msfl_output.inputParamFaultInfo;
    ar & msfl_output.alignFaultInfo;
    ar & msfl_output.sensorOutLostInfo;
    ar & msfl_output.alignState;
    ar & msfl_output.fault2;
}


} // namespace serialization
} // namespace boost









