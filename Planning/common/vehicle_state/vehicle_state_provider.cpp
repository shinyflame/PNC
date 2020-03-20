
#include <cmath>
#include <Eigen/Core>

#include "../../math/euler_angles_zxy.h"
#include "../../math/quaternion.h"
#include "../../common/vehicle_state/vehicle_state_provider.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {



Status VehicleStateProvider::Update(const localization::LocalizationEstimate &localization,
                                    const canbus::Chassis &chassis)
{

  original_localization_ = localization;
  if (!ConstructExceptLinearVelocity(localization)) {
    cout<<"Fail to update because ConstructExceptLinearVelocity error."
        <<"localization error"<<endl;
    return Status::LOCALIZATION_ERROR ;
  }

  if (!std::isnan(localization.pose_30.back().timestamp_ms))
   {
      vehicle_state_.timestamp_ms = localization.pose_30.back().timestamp_ms;
   }
  else if (!std::isnan(localization.pose_30.back().timestamp_ms ))
   {
      cout << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead. error in vehicle_state_provider.cpp line 57"<<endl;
      vehicle_state_.timestamp_ms = chassis.timestamp_ms;
   }

  if (chassis.has_data) {
    vehicle_state_.linear_velocity = chassis.speed_mps;
  }

  if (chassis.gear_location != GEAR_NONE) {
    vehicle_state_.gear = chassis.gear_location;
  } else {
    vehicle_state_.gear = GEAR_NONE;
  }
  vehicle_state_.driving_mode = chassis.driving_mode;

  return Status::OK;

}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
            const localization::LocalizationEstimate &localization) {
  if (!localization.pose_30.back().has_data) {
    cout << "Invalid localization input."<<endl;
    return false;
  }


  vehicle_state_.x = localization.pose_30.back().position.x;
  vehicle_state_.y = localization.pose_30.back().position.y;
  vehicle_state_.z = localization.pose_30.back().position.z;
  vehicle_state_.heading = localization.pose_30.back().euler_angles.z;
  vehicle_state_.angular_velocity = localization.pose_30.back().angular_velocity_vrf.z;
  if(localization.pose_30.back().linear_velocity.y < g_config_param.max_stop_speed)
    vehicle_state_.linear_acceleration = 0.0;
  else
    vehicle_state_.linear_acceleration = localization.pose_30.back().linear_acceleration_vrf.y;

  position_state_.point_llh.lat        = localization.pose_30.back().position.x;
  position_state_.point_llh.lon        = localization.pose_30.back().position.y;

  position_state_.velocity_xyz     = localization.pose_30.back().linear_velocity;
  position_state_.acceleration_xyz = localization.pose_30.back().linear_acceleration_vrf;
  position_state_.angular_velocity = localization.pose_30.back().angular_velocity_vrf;
  position_state_.euler_angles     = localization.pose_30.back().euler_angles;

  if (abs(vehicle_state_.linear_velocity) < 0.2) {
    vehicle_state_.kappa = 0.0;
  } else {
    vehicle_state_.kappa =
        vehicle_state_.angular_velocity /vehicle_state_.linear_velocity;
  }

  vehicle_state_.roll  =localization.pose_30.back().euler_angles.x;
  vehicle_state_.pitch =localization.pose_30.back().euler_angles.y;
  vehicle_state_.yaw   =localization.pose_30.back().euler_angles.z;

  return true;
}

double VehicleStateProvider::x() const { return vehicle_state_.x; }

double VehicleStateProvider::y() const { return vehicle_state_.y; }

double VehicleStateProvider::z() const { return vehicle_state_.z; }

double VehicleStateProvider::roll() const { return vehicle_state_.roll; }

double VehicleStateProvider::pitch() const { return vehicle_state_.pitch; }

double VehicleStateProvider::yaw() const { return vehicle_state_.yaw; }

double VehicleStateProvider::heading() const {
  return vehicle_state_.heading;
}

double VehicleStateProvider::kappa() const { return vehicle_state_.kappa; }

double VehicleStateProvider::linear_velocity() const {
  return vehicle_state_.linear_velocity;
}

double VehicleStateProvider::angular_velocity() const {
  return vehicle_state_.angular_velocity;
}

double VehicleStateProvider::linear_acceleration() const {
  return vehicle_state_.linear_acceleration;
}

double VehicleStateProvider::gear() const { return vehicle_state_.gear; }

uint64_t VehicleStateProvider::timestamp() const {
  return vehicle_state_.timestamp_ms;
}

//const Pose &VehicleStateProvider::pose() const {
//  return vehicle_state_.pose;
//}

const Pose &VehicleStateProvider::original_pose() const {
  return original_localization_.pose_30.back();
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  vehicle_state_.linear_velocity = linear_velocity;
}

const VehicleState &VehicleStateProvider::vehicle_state() const {
  return vehicle_state_;
}

const VehiclePositonState &VehicleStateProvider::position_state() const{
  return position_state_;
}

Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity;
  if (vehicle_state_.gear == GEAR_REVERSE) {
    v = -vehicle_state_.linear_velocity;
  }
  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity *
                      (1.0 - std::cos(vehicle_state_.angular_velocity * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity * t) * v /
                      vehicle_state_.angular_velocity;
  }

  // If we have rotation information, take it into consideration.
#if 0
  if ( &vehicle_state_.pose.orientation ) { // get ptr and is not null
    const auto &orientation = vehicle_state_.pose.orientation;
    Eigen::Quaternion<double> quaternion(orientation.qw, orientation.qx,
                                         orientation.qy, orientation.qz);
    Eigen::Vector3d pos_vec(vehicle_state_.x, vehicle_state_.y,
                            vehicle_state_.z);
    auto future_pos_3d = quaternion.toRotationMatrix() * vec_distance + pos_vec;
    return Vec2d(future_pos_3d[0], future_pos_3d[1]);
    }else{
      cout<<"ERROR &vehicle_state_.pose.orientation == null"<<endl;
    }
#endif
  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return Vec2d(vec_distance[0] + vehicle_state_.x,
               vec_distance[1] + vehicle_state_.y);
}


#if 0
Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v(0.0, rear_to_com_distance, 0.0);
  Eigen::Vector3d pos_vec(vehicle_state_.x, vehicle_state_.y,
                          vehicle_state_.z);
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  // If we have rotation information, take it into consideration.
  if ( &vehicle_state_.pose.orientation ) { // get ptr and is not null
    const auto &orientation = vehicle_state_.pose.orientation;
    Eigen::Quaternion<double> quaternion(orientation.qw, orientation.qx,
                                         orientation.qy, orientation.qz);
    // Update the COM position with rotation
    com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
  }else{
      cout<<"ERROR &vehicle_state_.pose.orientation == null"<<endl;
    }

  return Vec2d(com_pos_3d[0], com_pos_3d[1]);
}
#endif

} //end namespace planning
