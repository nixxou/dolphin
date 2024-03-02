// Copyright 2019 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/WiimoteEmu/Dynamics.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "Common/MathUtil.h"
#include "Core/Config/SYSCONFSettings.h"
#include "Core/HW/WiimoteEmu/WiimoteEmu.h"
#include "InputCommon/ControllerEmu/ControlGroup/Buttons.h"
#include "InputCommon/ControllerEmu/ControlGroup/Cursor.h"
#include "InputCommon/ControllerEmu/ControlGroup/Force.h"
#include "InputCommon/ControllerEmu/ControlGroup/IMUAccelerometer.h"
#include "InputCommon/ControllerEmu/ControlGroup/IMUCursor.h"
#include "InputCommon/ControllerEmu/ControlGroup/IMUGyroscope.h"
#include "InputCommon/ControllerEmu/ControlGroup/Tilt.h"

namespace
{
// Given a velocity, acceleration, and maximum jerk value,
// calculate change in position after a stop in the shortest possible time.
// Used to smoothly adjust acceleration and come to complete stops at precise positions.
// Based on equations for motion with constant jerk.
// s = s0 + v0 t + a0 t^2 / 2 + j t^3 / 6
double CalculateStopDistance(double velocity, double acceleration, double max_jerk)
{
  // Math below expects velocity to be non-negative.
  const auto velocity_flip = (velocity < 0 ? -1 : 1);

  const auto v_0 = velocity * velocity_flip;
  const auto a_0 = acceleration * velocity_flip;
  const auto j = max_jerk;

  // Time to reach zero acceleration.
  const auto t_0 = a_0 / j;

  // Distance to reach zero acceleration.
  const auto d_0 = std::pow(a_0, 3) / (3 * j * j) + (a_0 * v_0) / j;

  // Velocity at zero acceleration.
  const auto v_1 = v_0 + a_0 * std::abs(t_0) - std::copysign(j * t_0 * t_0 / 2, t_0);

  // Distance to complete stop.
  const auto d_1 = std::copysign(std::pow(std::abs(v_1), 3.0 / 2), v_1) / std::sqrt(j);

  return (d_0 + d_1) * velocity_flip;
}

double CalculateStopDistance(double velocity, double max_accel)
{
  return velocity * velocity / (2 * std::copysign(max_accel, velocity));
}

}  // namespace

namespace WiimoteEmu
{
Common::Quaternion ComplementaryFilter(const Common::Quaternion& gyroscope,
                                       const Common::Vec3& accelerometer, float accel_weight,
                                       const Common::Vec3& accelerometer_normal)
{
  const auto gyro_vec = gyroscope * accelerometer_normal;
  const auto normalized_accel = accelerometer.Normalized();

  const auto cos_angle = normalized_accel.Dot(gyro_vec);

  // If gyro to accel angle difference is betwen 0 and 180 degrees we make an adjustment.
  const auto abs_cos_angle = std::abs(cos_angle);
  if (abs_cos_angle > 0 && abs_cos_angle < 1)
  {
    const auto axis = gyro_vec.Cross(normalized_accel).Normalized();
    return Common::Quaternion::Rotate(std::acos(cos_angle) * accel_weight, axis) * gyroscope;
  }
  else
  {
    return gyroscope;
  }
}

void EmulateShake(PositionalState* state, ControllerEmu::Shake* const shake_group,
                  float time_elapsed)
{
  auto target_position = shake_group->GetState() * float(shake_group->GetIntensity() / 2);
  for (std::size_t i = 0; i != target_position.data.size(); ++i)
  {
    if (state->velocity.data[i] * std::copysign(1.f, target_position.data[i]) < 0 ||
        state->position.data[i] / target_position.data[i] > 0.5)
    {
      target_position.data[i] *= -1;
    }
  }

  // Time from "top" to "bottom" of one shake.
  const auto travel_time = 1 / shake_group->GetFrequency() / 2;

  Common::Vec3 jerk;
  for (std::size_t i = 0; i != target_position.data.size(); ++i)
  {
    const auto half_distance =
        std::max(std::abs(target_position.data[i]), std::abs(state->position.data[i]));

    jerk.data[i] = half_distance / std::pow(travel_time / 2, 3);
  }

  ApproachPositionWithJerk(state, target_position, jerk, time_elapsed);
}

void EmulateTilt(RotationalState* state, ControllerEmu::Tilt* const tilt_group, float time_elapsed)
{
  const auto target = tilt_group->GetState();

  // 180 degrees is currently the max tilt value.
  const ControlState roll = target.x * MathUtil::PI;
  const ControlState pitch = target.y * MathUtil::PI;

  const auto target_angle = Common::Vec3(pitch, -roll, 0);

  // For each axis, wrap around current angle if target is farther than 180 degrees.
  for (std::size_t i = 0; i != target_angle.data.size(); ++i)
  {
    auto& angle = state->angle.data[i];
    if (std::abs(angle - target_angle.data[i]) > float(MathUtil::PI))
      angle -= std::copysign(MathUtil::TAU, angle);
  }

  const auto max_accel = std::pow(tilt_group->GetMaxRotationalVelocity(), 2) / MathUtil::TAU;

  ApproachAngleWithAccel(state, target_angle, max_accel, time_elapsed);
}

void EmulateSwing(MotionState* state, ControllerEmu::Force* swing_group, float time_elapsed)
{
  const auto input_state = swing_group->GetState();
  const float max_distance = swing_group->GetMaxDistance();
  const float max_angle = swing_group->GetTwistAngle();

  // Note: Y/Z swapped because X/Y axis to the swing_group is X/Z to the wiimote.
  // X is negated because Wiimote X+ is to the left.
  const auto target_position = Common::Vec3{-input_state.x, -input_state.z, input_state.y};

  // Jerk is scaled based on input distance from center.
  // X and Z scale is connected for sane movement about the circle.
  const auto xz_target_dist = Common::Vec2{target_position.x, target_position.z}.Length();
  const auto y_target_dist = std::abs(target_position.y);
  const auto target_dist = Common::Vec3{xz_target_dist, y_target_dist, xz_target_dist};
  const auto speed = MathUtil::Lerp(Common::Vec3{1, 1, 1} * float(swing_group->GetReturnSpeed()),
                                    Common::Vec3{1, 1, 1} * float(swing_group->GetSpeed()),
                                    target_dist / max_distance);

  // Convert our m/s "speed" to the jerk required to reach this speed when traveling 1 meter.
  const auto max_jerk = speed * speed * speed * 4;

  // Rotational acceleration to approximately match the completion time of our swing.
  const auto max_accel = max_angle * speed.x * speed.x;

  // Apply rotation based on amount of swing.
  const auto target_angle =
      Common::Vec3{-target_position.z, 0, target_position.x} / max_distance * max_angle;

  // Angular acceleration * 2 seems to reduce "spurious stabs" in ZSS.
  // TODO: Fix properly.
  ApproachAngleWithAccel(state, target_angle, max_accel * 2, time_elapsed);

  // Clamp X and Z rotation.
  for (const int c : {0, 2})
  {
    if (std::abs(state->angle.data[c] / max_angle) > 1 &&
        MathUtil::Sign(state->angular_velocity.data[c]) == MathUtil::Sign(state->angle.data[c]))
    {
      state->angular_velocity.data[c] = 0;
    }
  }

  // Adjust target position backwards based on swing progress and max angle
  // to simulate a swing with an outstretched arm.
  const auto backwards_angle = std::max(std::abs(state->angle.x), std::abs(state->angle.z));
  const auto backwards_movement = (1 - std::cos(backwards_angle)) * max_distance;

  // TODO: Backswing jerk should be based on x/z speed.

  ApproachPositionWithJerk(state, target_position + Common::Vec3{0, backwards_movement, 0},
                           max_jerk, time_elapsed);

  // Clamp Left/Right/Up/Down movement within the configured circle.
  const auto xz_progress =
      Common::Vec2{state->position.x, state->position.z}.Length() / max_distance;
  if (xz_progress > 1)
  {
    state->position.x /= xz_progress;
    state->position.z /= xz_progress;

    state->acceleration.x = state->acceleration.z = 0;
    state->velocity.x = state->velocity.z = 0;
  }

  // Clamp Forward/Backward movement within the configured distance.
  // We allow additional backwards movement for the back swing.
  const auto y_progress = state->position.y / max_distance;
  const auto max_y_progress = 2 - std::cos(max_angle);
  if (y_progress > max_y_progress || y_progress < -1)
  {
    state->position.y =
        std::clamp(state->position.y, -1.f * max_distance, max_y_progress * max_distance);
    state->velocity.y = 0;
    state->acceleration.y = 0;
  }
}

WiimoteCommon::AccelData ConvertAccelData(const Common::Vec3& accel, u16 zero_g, u16 one_g)
{
  const auto scaled_accel = accel * (one_g - zero_g) / float(GRAVITY_ACCELERATION);

  // 10-bit integers.
  constexpr long MAX_VALUE = (1 << 10) - 1;

  return WiimoteCommon::AccelData(
      {u16(std::clamp(std::lround(scaled_accel.x + zero_g), 0l, MAX_VALUE)),
       u16(std::clamp(std::lround(scaled_accel.y + zero_g), 0l, MAX_VALUE)),
       u16(std::clamp(std::lround(scaled_accel.z + zero_g), 0l, MAX_VALUE))});
}

void EmulatePoint(MotionState* state, ControllerEmu::Cursor* ir_group,
                  const ControllerEmu::InputOverrideFunction& override_func, float time_elapsed,
                  std::string lastActiveGame, int lastRatio, bool fastPointer)
{

  const bool correctaim = ir_group->m_autocorrectaim_setting.GetValue();
  auto cursor = ir_group->GetState(true, override_func);

  float vertical_offset = ir_group->GetVerticalOffset(-1);
  float yaw = ir_group->GetTotalYaw(-1);
  float pitch = ir_group->GetTotalPitch(-1);

  bool aim_corrected = false;
  if (correctaim)
  {
    if (lastRatio == 0)
    {
      if (lastActiveGame == "S3AE5G")  // Attack of the Movies 3D
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(25);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }

      if (lastActiveGame == "RCSE20" || lastActiveGame == "RCSP7J")  // Chicken Shoot
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(-0.2);
        yaw = ir_group->GetTotalYaw(61.5);
        pitch = ir_group->GetTotalPitch(32.0);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y > 0)
          cursor.y = cursor.y - (0.09 * abs(xori) * abs(yori));
        if (cursor.y < 0)
          cursor.y = cursor.y + (0.09 * abs(xori) * abs(yori));
      }

      if (lastActiveGame == "RMRE5Z" || lastActiveGame == "RMRPNK" ||
          lastActiveGame == "RMRXNK")  // Cocoto Magic Circu
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.3);
        yaw = ir_group->GetTotalYaw(25.2);
        pitch = ir_group->GetTotalPitch(18.7);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.x > 0)
          cursor.x += 0.01 * abs(xori);
        if (cursor.x < 0)
          cursor.x -= 0.01 * abs(xori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SC2E8P")  // Conduit 2
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26.2);
        pitch = ir_group->GetTotalPitch(20.5);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }

        if (xori < 0)
        {
          if (cursor.y > 0)
            cursor.y = cursor.y - ((0.15 * abs(xori) * abs(yori)));
          if (cursor.y < 0)
            cursor.y = cursor.y + ((0.15 * abs(xori) * abs(yori)));
        }
        if (xori > 0)
        {
          if (cursor.y > 0)
            cursor.y = cursor.y - ((0.15 * abs(xori) * abs(yori)));
          if (cursor.y < 0)
            cursor.y = cursor.y + ((0.15 * abs(xori) * abs(yori)));
        }
      }
      if (lastActiveGame == "RZJD69" || lastActiveGame == "RZJE69" || lastActiveGame == "RZJJ13" ||
          lastActiveGame == "RZJP69")  // DeadSpace
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(23.9);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SUNEYG")  // DD Legends
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.0);
        pitch = ir_group->GetTotalPitch(14.5);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SJUE20")  // Dino Strike
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.3);
        pitch = ir_group->GetTotalPitch(18.9);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.08 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "W6BE01")  // Eco Shooter
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(0);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(20.5);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WFAEJS")  // Fast Draw Showdown
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(18.8);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RGSE8P" || lastActiveGame == "RGSJ8P" ||
          lastActiveGame == "RGSP8P")  // Ghost Squad
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.6);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SQDE8P" ||
          lastActiveGame == "SQDP8P")  // Gunblade NY & LA Machineguns: Arcade Hits Pack
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.3);
        pitch = ir_group->GetTotalPitch(18.5);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SW7EVN")  // Gunslingers
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.5);
        pitch = ir_group->GetTotalPitch(15.8);
        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WHYETY")  // Heavy Fire - Black Arms
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(29.5);
        pitch = ir_group->GetTotalPitch(22);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WHFETY")  // Heavy Fire - Special Operations
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(29.5);
        pitch = ir_group->GetTotalPitch(22);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.06 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.06 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SH4EFP")  // Heavy Fire - Afghanistan (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "R8XE52")  // Jurassic - The Hunted (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.8);
        pitch = ir_group->GetTotalPitch(19);
        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RZPE01")  // Link's Crossbow Training
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(18);
        pitch = ir_group->GetTotalPitch(10);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RQ5E5G" || lastActiveGame == "RQ5P5G" ||
          lastActiveGame == "RQ5X5G")  // Mad Dog McCree - Gunslinger Pack
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RQ7E20")  // Martian Panic
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25);
        pitch = ir_group->GetTotalPitch(19);
        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RL6E69")  // Nerf N Strike
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(21.5);
        yaw = ir_group->GetTotalYaw(16);
        pitch = ir_group->GetTotalPitch(12.5);
        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SKXE20" || lastActiveGame == "SKXPFH")  // Pirate Blast
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(18.5);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y > 0)
          cursor.y += 0.01 * abs(yori);
        if (cursor.y < 0)
          cursor.y -= 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "STDEFP")  // Reload
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26);
        pitch = ir_group->GetTotalPitch(18.8);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SBHEFP")  // Remington Great American Bird Hunt (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.3);
        pitch = ir_group->GetTotalPitch(18.8);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SS7EFP")  // Remington Super Slam Hunting - Africa (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y < 0)
          cursor.y += 0.04 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SRKEFP")  // Remington Super Slam Hunting - Alaska (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.3);
        pitch = ir_group->GetTotalPitch(18.5);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y > 0)
          cursor.y += 0.05 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SBSEFP")  // Remington Super Slam Hunting - North America (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.3);
        pitch = ir_group->GetTotalPitch(18.5);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SBDE08" || lastActiveGame == "SBDJ08" || lastActiveGame == "SBDK08" ||
          lastActiveGame == "SBDP08")  // RESIDENT EVIL THE DARKSIDE CHRONICLES
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(0);
        yaw = ir_group->GetTotalYaw(21);
        pitch = ir_group->GetTotalPitch(15.8);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.x > 0)
          cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
        if (cursor.x < 0)
          cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
      }
      if (lastActiveGame == "RBUE08")  // Resident Evil - The Umbrella Chronicles (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(17.3);
        pitch = ir_group->GetTotalPitch(11.7);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.x < 0)
          cursor.x += 0.02 * abs(xori);
        if (cursor.x > 0)
          cursor.x -= 0.02 * abs(xori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "R2VE01" || lastActiveGame == "R2VP01" ||
          lastActiveGame == "R2VJ01")  // Sin & Punishment - Star Successor (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25);
        pitch = ir_group->GetTotalPitch(19.2);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.06 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.06 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SSNEYG")  // Sniper Elite
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(24);
        pitch = ir_group->GetTotalPitch(13.5);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.03 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RGDEA4")  // Target: Terror
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26.7);
        pitch = ir_group->GetTotalPitch(19.7);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.x < 0)
          cursor.x += 0.02 * abs(xori);
        if (cursor.x > 0)
          cursor.x -= 0.02 * abs(xori);
        if (cursor.y < 0)
          cursor.y += 0.02 * abs(yori);
        if (cursor.y > 0)
          cursor.y -= 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RCJE8P" || lastActiveGame == "RCJP8P")  // The Conduit
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.5);
        pitch = ir_group->GetTotalPitch(18.7);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y < 0)
          cursor.y -= 0.005 * abs(yori);
        if (cursor.y > 0)
          cursor.y += 0.005 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RHDE8P" || lastActiveGame == "RHDJ8P" ||
          lastActiveGame == "RHDP8P")  // The House of the Dead 2 & 3 Return
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26.7);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y < 0)
          cursor.y -= 0.005 * abs(yori);
        if (cursor.y > 0)
          cursor.y += 0.005 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RHOE8P" || lastActiveGame == "RHOJ8P" ||
          lastActiveGame == "RHOP8P")  // House Of The Dead: OVERKILL
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.x < 0)
          cursor.x += 0.01 * abs(xori);
        if (cursor.x > 0)
          cursor.x -= 0.01 * abs(xori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "ST9E52")  // Top Shot Arcade (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(24.7);
        pitch = ir_group->GetTotalPitch(19.0);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "R8XZ52")  // Top Shot Dino
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SW9EVN")  // Wicked Monster Blast
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(24.3);
        pitch = ir_group->GetTotalPitch(23.7);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WB4EGL")  // Wild West Guns
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.7);
        pitch = ir_group->GetTotalPitch(19.0);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SSRE20" || lastActiveGame == "SSRPXT")  // Wild West Shootout
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26.3);
        pitch = ir_group->GetTotalPitch(19.0);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WZPERZ")  // Zombie Panic
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.7);
        pitch = ir_group->GetTotalPitch(24.3);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SBQE4Z")  //Big Buck Hunter Pro
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(7.0);
        yaw = ir_group->GetTotalYaw(25.5);
        pitch = ir_group->GetTotalPitch(18.9);

        double xori = cursor.x;
        double yori = cursor.y;
        /*
        if (cursor.y < 0)
          cursor.y -= 0.005 * abs(yori);
        if (cursor.y > 0)
          cursor.y += 0.005 * abs(yori);
        */

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RRBE41" || lastActiveGame == "RRBJ41" || lastActiveGame == "RRBP41")  // Rayman Raving Rabbids
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(-0.7);
        yaw = ir_group->GetTotalYaw(35.0);
        pitch = ir_group->GetTotalPitch(25.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y -= 0.04 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }

        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }



          if (cursor.y > 0)
            cursor.y = cursor.y + (0.05 * abs(yori) * abs(xori));
          if (cursor.y < 0)
            cursor.y = cursor.y - (0.05 * abs(yori) * abs(xori));
        
      }
      if (lastActiveGame == "RY2P41")  //Rayman - Raving Rabbids 2 (Europe) (En,Fr,De,Es,It,Nl) (Rev 2)
      {
          aim_corrected = true;
          vertical_offset = ir_group->GetVerticalOffset(15.0);
          yaw = ir_group->GetTotalYaw(26.0);
          pitch = ir_group->GetTotalPitch(19.0);

          double xori = cursor.x;
          double yori = cursor.y;
          
          if (yori < 0)
          {
            if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
            if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
          }
      }

      if (lastActiveGame == "RY3E41" || lastActiveGame == "RY3J41" || lastActiveGame == "RY3K41" || lastActiveGame == "RY3P41")  // Rayman - TV PARTY
      {
          aim_corrected = true;
          vertical_offset = ir_group->GetVerticalOffset(15.0);
          yaw = ir_group->GetTotalYaw(26.0);
          pitch = ir_group->GetTotalPitch(19.0);

          double xori = cursor.x;
          double yori = cursor.y;
          
          if (yori < 0)
          {
            if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
            if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
          }
      }

      if (lastActiveGame == "RM2E69")  // Medal of Honor HERO 2 (usa)
      {
          aim_corrected = true;
          vertical_offset = ir_group->GetVerticalOffset(0.0);
          yaw = ir_group->GetTotalYaw(47.2);
          pitch = ir_group->GetTotalPitch(26.5);

          double xori = cursor.x;
          double yori = cursor.y;

          
          if (cursor.x > 0)
          cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
          cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
          
      }


    }
    if (lastRatio == 1)
    {

      if (lastActiveGame == "S3AE5G")  // Attack of the Movies 3D
      {
          aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19);
        pitch = ir_group->GetTotalPitch(19.8);

        double xori = cursor.x;
        double yori = cursor.y;

        
        if (cursor.y < 0)
          cursor.y += 0.02 * abs(yori);

        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.01 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.01 * abs(yori) * abs(xori));
        }

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }

      if (lastActiveGame == "RCSE20" || lastActiveGame == "RCSP7J")  // Chicken Shoot
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(-0.2);
        yaw = ir_group->GetTotalYaw(61.5);
        pitch = ir_group->GetTotalPitch(32.0);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y > 0)
          cursor.y = cursor.y - (0.09 * abs(xori) * abs(yori));
        if (cursor.y < 0)
          cursor.y = cursor.y + (0.09 * abs(xori) * abs(yori));
      }

      if (lastActiveGame == "RMRE5Z" || lastActiveGame == "RMRPNK" ||
          lastActiveGame == "RMRXNK")  // Cocoto Magic Circu
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.3);
        yaw = ir_group->GetTotalYaw(25.2);
        pitch = ir_group->GetTotalPitch(18.7);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.x > 0)
          cursor.x += 0.01 * abs(xori);
        if (cursor.x < 0)
          cursor.x -= 0.01 * abs(xori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SC2E8P")  // Conduit 2
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19.2);
        pitch = ir_group->GetTotalPitch(21);
        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y += 0.01 * abs(yori);


        if (xori > 0)
          cursor.x *= (1 + (0.05 * (1 - abs(xori))));

        if (xori < 0)
          cursor.x *= (1 + (0.03 * (1 - abs(xori))));

        if (xori < 0 && yori > 0) //coté haut gauche
        {
          cursor.x = cursor.x + (0.025 * abs(yori) * abs(xori));
          cursor.y = cursor.y - ((0.1 * abs(xori) * abs(yori)));
        }
        if (xori > 0 && yori > 0)  // coté haut droit
        {
          cursor.x = cursor.x - (0.025 * abs(yori) * abs(xori));
          cursor.y = cursor.y - ((0.1 * abs(xori) * abs(yori)));
        }
        if (xori < 0 && yori < 0)  // coté haut gauche
        {
          cursor.x = cursor.x + (0.065 * abs(yori) * abs(xori));
          cursor.y = cursor.y + ((0.1 * abs(xori) * abs(yori)));
        }
        if (xori > 0 && yori < 0)  // coté haut droit
        {
          cursor.x = cursor.x - (0.065 * abs(yori) * abs(xori));
          cursor.y = cursor.y + ((0.1 * abs(xori) * abs(yori)));
        }

      }
      if (lastActiveGame == "RZJD69" || lastActiveGame == "RZJE69" || lastActiveGame == "RZJJ13" ||
          lastActiveGame == "RZJP69")  // DeadSpace
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(18.3);
        pitch = ir_group->GetTotalPitch(19.5);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= (1 + (0.02 * (1 - abs(xori))));
        

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.055 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.055 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SUNEYG")  // DD Legends
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(14.0);
        pitch = ir_group->GetTotalPitch(11.0);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        cursor.y *= (1 + (0.02 * (1 - abs(yori))));

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }

        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SJUE20")  // Dino Strike
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.0);
        yaw = ir_group->GetTotalYaw(14.3);
        pitch = ir_group->GetTotalPitch(19.5);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        if (cursor.y < 0) cursor.y *= (1 + (0.02 * (1 - abs(yori))));

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.06 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.06 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "W6BE01")  // Eco Shooter
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(0);
        yaw = ir_group->GetTotalYaw(20.5);
        pitch = ir_group->GetTotalPitch(21.2);

        double xori = cursor.x;
        double yori = cursor.y;
        
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.03 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.03 * abs(yori) * abs(xori));
        }
        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.03 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.03 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "WFAEJS")  // Fast Draw Showdown
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(18.8);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "RGSE8P" || lastActiveGame == "RGSJ8P" ||
          lastActiveGame == "RGSP8P")  // Ghost Squad
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.6);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SQDE8P" ||
          lastActiveGame == "SQDP8P")  // Gunblade NY & LA Machineguns: Arcade Hits Pack
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.2);
        yaw = ir_group->GetTotalYaw(15.3);
        pitch = ir_group->GetTotalPitch(18.5);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));

        if (cursor.y < 0)
          cursor.y *= (1 + (0.02 * (1 - abs(yori))));
        if (cursor.y > 0)
          cursor.y *= (1 + (0.03 * (abs(yori))));

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.06 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.06 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SW7EVN")  // Gunslingers
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(14.25);
        pitch = ir_group->GetTotalPitch(12.0);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));


        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.03 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.03 * abs(yori) * abs(xori));
        }
        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x + (0.01 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x - (0.01 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WHYETY")  // Heavy Fire - Black Arms
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.0);
        yaw = ir_group->GetTotalYaw(16.5);
        pitch = ir_group->GetTotalPitch(22.8);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));

        if (cursor.y < 0)
          cursor.y += 0.008 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.07 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.07 * abs(yori) * abs(xori));
        }

      }
      if (lastActiveGame == "WHFETY")  // Heavy Fire - Special Operations
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(16.5);
        pitch = ir_group->GetTotalPitch(23);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        if (cursor.y < 0)
          cursor.y += 0.015 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.078 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.073 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "SH4EFP")  // Heavy Fire - Afghanistan (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.0);
        yaw = ir_group->GetTotalYaw(14.0);
        pitch = ir_group->GetTotalPitch(19.2);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        if (cursor.y > 0)
          cursor.y += 0.012 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }

      }
      if (lastActiveGame == "R8XE52")  // Jurassic - The Hunted (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(14.5);
        pitch = ir_group->GetTotalPitch(19);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RZPE01")  // Link's Crossbow Training
      {
        aim_corrected = true;
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        xori = cursor.x;

        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(10.15);
        pitch = ir_group->GetTotalPitch(8);

        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.y = cursor.y - (0.025 * abs(xori) * abs(yori));
          if (cursor.x < 0)
            cursor.y = cursor.y - (0.025 * abs(xori) * abs(yori));
        }

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "RQ5E5G" || lastActiveGame == "RQ5P5G" ||
          lastActiveGame == "RQ5X5G")  // Mad Dog McCree - Gunslinger Pack
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RQ7E20")  // Martian Panic
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(14.3);
        pitch = ir_group->GetTotalPitch(19.5);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        xori = cursor.x;
        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.01 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.01 * abs(yori) * abs(xori));
        }

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RL6E69")  // Nerf N Strike
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(21.5);
        yaw = ir_group->GetTotalYaw(9.3);
        pitch = ir_group->GetTotalPitch(13.0);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        xori = cursor.x;
 
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "SKXE20" || lastActiveGame == "SKXPFH")  // Pirate Blast
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(27.5);
        pitch = ir_group->GetTotalPitch(18.5);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y > 0)
          cursor.y += 0.01 * abs(yori);
        if (cursor.y < 0)
          cursor.y -= 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "STDEFP")  // Reload
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15.0);
        yaw = ir_group->GetTotalYaw(14.5);
        pitch = ir_group->GetTotalPitch(18.8);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.03 * (1 - abs(xori))));
        xori = cursor.x;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "SBHEFP")  // Remington Great American Bird Hunt (USA)
      {
        aim_corrected = true;
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= 1.33;
        cursor.x *= (1 + (0.033 * (1 - abs(xori))));
        cursor.x += 0.018;
        xori = cursor.x;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.030 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.030 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SS7EFP")  // Remington Super Slam Hunting - Africa (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(25.5);
        pitch = ir_group->GetTotalPitch(18.7);
        
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x += 0.24;
       
        if (cursor.y>0) cursor.y += 0.04 * abs(yori);
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SRKEFP")  // Remington Super Slam Hunting - Alaska (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(25.5);
        pitch = ir_group->GetTotalPitch(19.5);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x += 0.25;

        if (cursor.y < 0)
          cursor.y += 0.045 * abs(yori);

        if (cursor.x > 0)
          cursor.x -= 0.016 * abs(xori);
        if (cursor.x < 0)
          cursor.x -= 0.016 * abs(xori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "SBSEFP")  // Remington Super Slam Hunting - North America (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(18.7);
        pitch = ir_group->GetTotalPitch(18.5);
        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x += 0.02;
        
        if (cursor.y > 0)
          cursor.y += 0.02 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "SBDE08" || lastActiveGame == "SBDJ08" || lastActiveGame == "SBDK08" ||
          lastActiveGame == "SBDP08")  // RESIDENT EVIL THE DARKSIDE CHRONICLES
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(0);
        yaw = ir_group->GetTotalYaw(21);
        pitch = ir_group->GetTotalPitch(15.8);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.x > 0)
          cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
        if (cursor.x < 0)
          cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
      }
      if (lastActiveGame == "RBUE08")  // Resident Evil - The Umbrella Chronicles (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(17.3);
        pitch = ir_group->GetTotalPitch(11.7);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.x < 0)
          cursor.x += 0.028 * abs(xori);
        if (cursor.x > 0)
          cursor.x -= 0.028 * abs(xori);

        if (yori > 0)
        {
          cursor.y -= 0.025 * abs(xori);
        }

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "R2VE01" || lastActiveGame == "R2VP01" ||
          lastActiveGame == "R2VJ01")  // Sin & Punishment - Star Successor (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(18.7);
        pitch = ir_group->GetTotalPitch(19.2);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.06 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.06 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SSNEYG")  // Sniper Elite
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19);
        pitch = ir_group->GetTotalPitch(14);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.045 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.045 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RGDEA4")  // Target: Terror
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26.7);
        pitch = ir_group->GetTotalPitch(19.7);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.x < 0)
          cursor.x += 0.02 * abs(xori);
        if (cursor.x > 0)
          cursor.x -= 0.02 * abs(xori);
        if (cursor.y < 0)
          cursor.y += 0.02 * abs(yori);
        if (cursor.y > 0)
          cursor.y -= 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RCJE8P" || lastActiveGame == "RCJP8P")  // The Conduit
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(18.8);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.x < 0)
          cursor.x += 0.006 * abs(xori);
        if (cursor.y > 0)
          cursor.y += 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
        
      }//a cootinuer
      if (lastActiveGame == "RHDE8P" || lastActiveGame == "RHDJ8P" ||
          lastActiveGame == "RHDP8P")  // The House of the Dead 2 & 3 Return
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(14.9);
        yaw = ir_group->GetTotalYaw(26.7);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;
        if (cursor.y < 0)
          cursor.y -= 0.005 * abs(yori);
        if (cursor.y > 0)
          cursor.y += 0.005 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "RHOE8P" || lastActiveGame == "RHOJ8P" ||
          lastActiveGame == "RHOP8P")  // House Of The Dead: OVERKILL
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19.35);
        pitch = ir_group->GetTotalPitch(19);

        double xori = cursor.x;
        double yori = cursor.y;
        cursor.x *= (1 + (0.01 * (1 - abs(xori))));

        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.01 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.01 * abs(yori) * abs(xori));
        }
        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "ST9E52")  // Top Shot Arcade (USA)
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(24.9);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "R8XZ52")  // Top Shot Dino
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19.5);
        pitch = ir_group->GetTotalPitch(19.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y += 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
        
      }
      if (lastActiveGame == "SW9EVN")  // Wicked Monster Blast
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(18.3);
        pitch = ir_group->GetTotalPitch(18.7);
        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y += 0.025 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WB4EGL")  // Wild West Guns
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(18.95);
        pitch = ir_group->GetTotalPitch(19.5);
        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y += 0.01 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SSRE20" || lastActiveGame == "SSRPXT")  // Wild West Shootout
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19.25);
        pitch = ir_group->GetTotalPitch(19);
        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "WZPERZ")  // Zombie Panic
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(15);
        yaw = ir_group->GetTotalYaw(19);
        pitch = ir_group->GetTotalPitch(19.8);
        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y += 0.015 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.05 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.05 * abs(yori) * abs(xori));
        }
      }
      if (lastActiveGame == "SBQE4Z")  // Big Buck Hunter Pro
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(7.0);
        yaw = ir_group->GetTotalYaw(25.5);
        pitch = ir_group->GetTotalPitch(18.9);

        double xori = cursor.x;
        double yori = cursor.y;

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }
      }

      if (lastActiveGame == "RRBE41" || lastActiveGame == "RRBJ41" || lastActiveGame == "RRBP41")  // Rayman Raving Rabbids
      {
        aim_corrected = true;
        vertical_offset = ir_group->GetVerticalOffset(-0.7);
        yaw = ir_group->GetTotalYaw(35.0);
        pitch = ir_group->GetTotalPitch(25.0);

        double xori = cursor.x;
        double yori = cursor.y;

        if (cursor.y < 0)
          cursor.y -= 0.04 * abs(yori);

        if (yori < 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
        }

        if (yori > 0)
        {
          if (cursor.x > 0)
            cursor.x = cursor.x - (0.02 * abs(yori) * abs(xori));
          if (cursor.x < 0)
            cursor.x = cursor.x + (0.02 * abs(yori) * abs(xori));
        }



          if (cursor.y > 0)
            cursor.y = cursor.y + (0.05 * abs(yori) * abs(xori));
          if (cursor.y < 0)
            cursor.y = cursor.y - (0.05 * abs(yori) * abs(xori));
        
      }

      if (lastActiveGame == "RY2P41")  //Rayman - Raving Rabbids 2 (Europe) (En,Fr,De,Es,It,Nl) (Rev 2)
      {
          aim_corrected = true;
          vertical_offset = ir_group->GetVerticalOffset(15.0);
          yaw = ir_group->GetTotalYaw(26.0);
          pitch = ir_group->GetTotalPitch(19.0);

          double xori = cursor.x;
          double yori = cursor.y;
          
          if (yori < 0)
          {
            if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
            if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
          }
      }

      if (lastActiveGame == "RY3E41" || lastActiveGame == "RY3J41" || lastActiveGame == "RY3K41" || lastActiveGame == "RY3P41")  // Rayman - TV PARTY
      {
          aim_corrected = true;
          vertical_offset = ir_group->GetVerticalOffset(15.0);
          yaw = ir_group->GetTotalYaw(26.0);
          pitch = ir_group->GetTotalPitch(19.0);

          double xori = cursor.x;
          double yori = cursor.y;
          
          if (yori < 0)
          {
            if (cursor.x > 0)
            cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
            if (cursor.x < 0)
            cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
          }
      }

      if (lastActiveGame == "RM2E69")  //Medal of Honor HERO 2 (usa)
      {
          aim_corrected = true;
          vertical_offset = ir_group->GetVerticalOffset(0.0);
          yaw = ir_group->GetTotalYaw(36.0);
          pitch = ir_group->GetTotalPitch(26.5);

          
          double xori = cursor.x;
          double yori = cursor.y;
          
          if (cursor.x > 0)
          cursor.x = cursor.x - (0.04 * abs(yori) * abs(xori));
          if (cursor.x < 0)
          cursor.x = cursor.x + (0.04 * abs(yori) * abs(xori));
          
          
      }



    }
  }

  if (!cursor.IsVisible())
  {
    // Move the wiimote a kilometer forward so the sensor bar is always behind it.
    *state = {};
    state->position = {0, -1000, 0};
    return;
  }

  // Nintendo recommends a distance of 1-3 meters.
  constexpr float NEUTRAL_DISTANCE = 2.f;

  // When the sensor bar position is on bottom, apply the "offset" setting negatively.
  // This is kinda odd but it does seem to maintain consistent cursor behavior.
  const bool sensor_bar_on_top = Config::Get(Config::SYSCONF_SENSOR_BAR_POSITION) != 0;

  const float height = vertical_offset * (sensor_bar_on_top ? 1 : -1);

  const float yaw_scale = yaw / 2;
  const float pitch_scale = pitch / 2;

  // Just jump to the target position.
  state->position = {0, NEUTRAL_DISTANCE, -height};
  state->velocity = {};
  state->acceleration = {};

  const auto target_angle = Common::Vec3(pitch_scale * -cursor.y, 0, yaw_scale * -cursor.x);

  // If cursor was hidden, jump to the target angle immediately.
  if (state->position.y < 0)
  {
    state->angle = target_angle;
    state->angular_velocity = {};

    return;
  }

  // Higher values will be more responsive but increase rate of M+ "desync".
  // I'd rather not expose this value in the UI if not needed.
  // At this value, sync is very good and responsiveness still appears instant.
  if (fastPointer && aim_corrected)
  {
    constexpr auto MAX_ACCEL = float(MathUtil::TAU * 50);
    ApproachAngleWithAccel(state, target_angle, MAX_ACCEL, time_elapsed);
  }
  else
  {
    constexpr auto MAX_ACCEL = float(MathUtil::TAU * 8);
    ApproachAngleWithAccel(state, target_angle, MAX_ACCEL, time_elapsed);
  }

}

void ApproachAngleWithAccel(RotationalState* state, const Common::Vec3& angle_target,
                            float max_accel, float time_elapsed)
{
  const auto stop_distance =
      Common::Vec3(CalculateStopDistance(state->angular_velocity.x, max_accel),
                   CalculateStopDistance(state->angular_velocity.y, max_accel),
                   CalculateStopDistance(state->angular_velocity.z, max_accel));

  const auto offset = angle_target - state->angle;
  const auto stop_offset = offset - stop_distance;
  const auto accel = MathUtil::Sign(stop_offset) * max_accel;

  state->angular_velocity += accel * time_elapsed;

  const auto change_in_angle =
      state->angular_velocity * time_elapsed + accel * time_elapsed * time_elapsed / 2;

  for (std::size_t i = 0; i != offset.data.size(); ++i)
  {
    // If new angle will overshoot stop right on target.
    if (std::abs(offset.data[i]) < 0.0001 || (change_in_angle.data[i] / offset.data[i] > 1.0))
    {
      state->angular_velocity.data[i] =
          (angle_target.data[i] - state->angle.data[i]) / time_elapsed;
      state->angle.data[i] = angle_target.data[i];
    }
    else
    {
      state->angle.data[i] += change_in_angle.data[i];
    }
  }
}

void EmulateIMUCursor(IMUCursorState* state, ControllerEmu::IMUCursor* imu_ir_group,
                      ControllerEmu::IMUAccelerometer* imu_accelerometer_group,
                      ControllerEmu::IMUGyroscope* imu_gyroscope_group, float time_elapsed)
{
  const auto ang_vel = imu_gyroscope_group->GetState();

  // Reset if pointing is disabled or we have no gyro data.
  if (!imu_ir_group->enabled || !ang_vel.has_value())
  {
    *state = {};
    return;
  }

  // Apply rotation from gyro data.
  const auto gyro_rotation = GetRotationFromGyroscope(*ang_vel * -1 * time_elapsed);
  state->rotation = gyro_rotation * state->rotation;

  // If we have some non-zero accel data use it to adjust gyro drift.
  const auto accel_weight = imu_ir_group->GetAccelWeight();
  auto const accel = imu_accelerometer_group->GetState().value_or(Common::Vec3{});
  if (accel.LengthSquared())
    state->rotation = ComplementaryFilter(state->rotation, accel, accel_weight);

  // Clamp yaw within configured bounds.
  const auto yaw = GetYaw(state->rotation);
  const auto max_yaw = float(imu_ir_group->GetTotalYaw() / 2);
  auto target_yaw = std::clamp(yaw, -max_yaw, max_yaw);

  // Handle the "Recenter" button being pressed.
  if (imu_ir_group->controls[0]->GetState<bool>())
  {
    state->recentered_pitch = GetPitch(state->rotation);
    target_yaw = 0;
  }

  // Adjust yaw as needed.
  if (yaw != target_yaw)
    state->rotation *= Common::Quaternion::RotateZ(target_yaw - yaw);

  // Normalize for floating point inaccuracies.
  state->rotation = state->rotation.Normalized();
}

void ApproachPositionWithJerk(PositionalState* state, const Common::Vec3& position_target,
                              const Common::Vec3& max_jerk, float time_elapsed)
{
  const auto stop_distance =
      Common::Vec3(CalculateStopDistance(state->velocity.x, state->acceleration.x, max_jerk.x),
                   CalculateStopDistance(state->velocity.y, state->acceleration.y, max_jerk.y),
                   CalculateStopDistance(state->velocity.z, state->acceleration.z, max_jerk.z));

  const auto offset = position_target - state->position;
  const auto stop_offset = offset - stop_distance;
  const auto jerk = MathUtil::Sign(stop_offset) * max_jerk;

  state->acceleration += jerk * time_elapsed;

  state->velocity += state->acceleration * time_elapsed + jerk * time_elapsed * time_elapsed / 2;

  const auto change_in_position = state->velocity * time_elapsed +
                                  state->acceleration * time_elapsed * time_elapsed / 2 +
                                  jerk * time_elapsed * time_elapsed * time_elapsed / 6;

  for (std::size_t i = 0; i != offset.data.size(); ++i)
  {
    // If new velocity will overshoot assume we would have stopped right on target.
    // TODO: Improve check to see if less jerk would have caused undershoot.
    if ((change_in_position.data[i] / offset.data[i]) > 1.0)
    {
      state->acceleration.data[i] = 0;
      state->velocity.data[i] = 0;
      state->position.data[i] = position_target.data[i];
    }
    else
    {
      state->position.data[i] += change_in_position.data[i];
    }
  }
}

Common::Quaternion GetRotationFromAcceleration(const Common::Vec3& accel)
{
  const auto normalized_accel = accel.Normalized();

  const auto angle = std::acos(normalized_accel.Dot({0, 0, 1}));
  const auto axis = normalized_accel.Cross({0, 0, 1});

  // Check that axis is non-zero to handle perfect up/down orientations.
  return Common::Quaternion::Rotate(angle, axis.LengthSquared() ? axis.Normalized() :
                                                                  Common::Vec3{0, 1, 0});
}

Common::Quaternion GetRotationFromGyroscope(const Common::Vec3& gyro)
{
  const auto length = gyro.Length();
  return (length != 0) ? Common::Quaternion::Rotate(length, gyro / length) :
                         Common::Quaternion::Identity();
}

Common::Matrix33 GetRotationalMatrix(const Common::Vec3& angle)
{
  return Common::Matrix33::RotateZ(angle.z) * Common::Matrix33::RotateY(angle.y) *
         Common::Matrix33::RotateX(angle.x);
}

float GetPitch(const Common::Quaternion& world_rotation)
{
  const auto vec = world_rotation * Common::Vec3{0, 0, 1};
  return std::atan2(vec.y, Common::Vec2(vec.x, vec.z).Length());
}

float GetRoll(const Common::Quaternion& world_rotation)
{
  const auto vec = world_rotation * Common::Vec3{0, 0, 1};
  return std::atan2(vec.x, vec.z);
}

float GetYaw(const Common::Quaternion& world_rotation)
{
  const auto vec = world_rotation.Inverted() * Common::Vec3{0, 1, 0};
  return std::atan2(vec.x, vec.y);
}

}  // namespace WiimoteEmu
