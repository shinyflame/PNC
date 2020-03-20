
/**
 * @file vehicle_state.h
 *
 * @brief Declaration of the class VehicleStateProvider.
 */
#ifndef VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_
#define VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_

#include <memory>
#include <string>

#include "../../common/macro.h"
#include "../../math/box2d.h"
#include "../../math/vec2d.h"
#include "../../common/struct.h"

using common::Status;
using namespace math;
namespace planning {



class VehicleStateProvider {
 public:

   VehicleStateProvider() = default;
   virtual ~VehicleStateProvider() = default;

  /**
   * @brief Constructor by information of localization and chassis.
   * @param localization Localization information of the vehicle.
   * @param chassis Chassis information of the vehicle.
   */
   Status Update(const localization::LocalizationEstimate& localization,
                 const canbus::Chassis& chassis);

  /**
   * @brief Update VehicleStateProvider instance by protobuf files.
   * @param localization_file the localization protobuf file.
   * @param chassis_file The chassis protobuf file
   */
  void Update(const std::string& localization_file,
              const std::string& chassis_file);

  uint64_t timestamp() const;

 // const Pose& pose() const;
  const Pose& original_pose() const;

  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  double kappa() const;

  /**
   * @brief Get the vehicle roll angle.
   * @return The euler roll angle.
   */
  double roll() const;

  /**
   * @brief Get the vehicle pitch angle.
   * @return The euler pitch angle.
   */
  double pitch() const;

  /**
   * @brief Get the vehicle yaw angle.
   *  As of now, use the heading instead of yaw angle.
   *  Heading angle with East as zero, yaw angle has North as zero
   * @return The euler yaw angle.
   */
  double yaw() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

  /**
   * @brief Get the vehicle's gear position.
   * @return The vehicle's gear position.
   */
  double gear() const;

  /**
   * @brief Set the vehicle's linear velocity.
   * @param linear_velocity The value to set the vehicle's linear velocity.
   */
  void set_linear_velocity(const double linear_velocity);

  /**
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  Vec2d EstimateFuturePosition(const double t) const;

  /**
   * @brief Compute the position of center of mass(COM) of the vehicle,
   *        given the distance from rear wheels to the center of mass.
   * @param rear_to_com_distance Distance from rear wheels to
   *        the vehicle's center of mass.
   * @return The position of the vehicle's center of mass.
   */
  //Vec2d ComputeCOMPosition(const double rear_to_com_distance) const;

  const VehicleState& vehicle_state() const;
  const VehiclePositonState  &position_state() const;

 private:
  bool ConstructExceptLinearVelocity(
      const localization::LocalizationEstimate& localization);

  VehicleState vehicle_state_;
  VehiclePositonState position_state_;
  localization::LocalizationEstimate original_localization_;


  //DECLARE_SINGLETON(VehicleStateProvider);
};


} //end namespace planning

#endif  // MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_
