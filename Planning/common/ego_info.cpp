
/**
 * @file speed_limit.cc
 **/

#include "../common/ego_info.h"

//#include "modules/common/configs/vehicle_config_helper.h"
//#include "modules/common/log.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {


EgoInfo::EgoInfo() {
  VehicleParam ego_vehicle_config_ = g_vehicle_config;
      //common::VehicleConfigHelper::GetConfig();
}

bool EgoInfo::Update(const TrajectoryPoint& start_point,
                     const VehicleState& vehicle_state,
                     const std::vector<const Obstacle*>& obstacles) {
  set_start_point(start_point);
  set_vehicle_state(vehicle_state);
  CalculateFrontObstacleClearDistance(obstacles);
  return true;
}

void EgoInfo::Clear() {
  start_point_={0};
  VehicleState vehicle_state;
  vehicle_state_= vehicle_state;
  front_clear_distance_ = std::numeric_limits<double>::max();
}

void EgoInfo::CalculateFrontObstacleClearDistance(
    const std::vector<const Obstacle*>& obstacles) {
  Vec2d position(vehicle_state_.x, vehicle_state_.y);

  const auto& param = ego_vehicle_config_;
  Vec2d vec_to_center(
      (param.front_edge_to_center - param.back_edge_to_center) / 2.0,
      (param.left_edge_to_center - param.right_edge_to_center) / 2.0);

  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading + M_PI/2.0));

  const double buffer = 0.1;  // in meters
  Box2d ego_box(center, vehicle_state_.heading + M_PI/2.0, param.length + buffer,
                param.width + buffer);
  const double adc_half_diagnal = ego_box.diagonal() / 2.0;

  Vec2d unit_vec_heading = Vec2d::CreateUnitVec2d(vehicle_state_.heading+ M_PI/2.0);

  // Due to the error of ego heading, only short range distance is meaningful
  const double kDistanceThreshold = 50.0;
  const double impact_region_length =
      param.length + buffer + kDistanceThreshold;
  Box2d ego_front_region(center + unit_vec_heading * kDistanceThreshold / 2.0,
                         vehicle_state_.heading + M_PI/2.0, impact_region_length,
                         param.width + buffer);

  for (const auto& obstacle : obstacles) {
    if (obstacle->IsVirtual() ||
        !ego_front_region.HasOverlap(obstacle->PerceptionBoundingBox())) {
      continue;
    }

    double dist = ego_box.center().DistanceTo(
                      obstacle->PerceptionBoundingBox().center()) -
                  adc_half_diagnal;

    if (front_clear_distance_ < 0.0 || dist < front_clear_distance_) {
      front_clear_distance_ = dist;
    }
  }
}

}  // namespace planning

