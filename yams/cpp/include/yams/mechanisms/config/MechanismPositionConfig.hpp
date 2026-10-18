// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <cmath>
#include <functional>
#include <optional>
#include <utility>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>

namespace yams::mechanisms::config {

/**
 * Where a mechanism is on the robot, for each mechanism's GetPose3d(): where it is mounted, how it is
 * turned there, the plane it moves in, and optionally the mechanism it is mounted on, such as an arm on
 * an elevator's carriage.
 *
 * @code{.cpp}
 * MechanismPositionConfig position;
 * position.WithRelativePosition(wpi::math::Translation3d{0.2_m, wpi::units::meter_t{0}, 0.3_m})  // 20 cm forward, 30 cm up
 *     .WithMovementPlane(MechanismPositionConfig::Plane::XZ);
 * @endcode
 */
class MechanismPositionConfig {
 public:
  /** The planes that the mechanism could move in. */
  enum class Plane {
    /** X-Z plane: forward and up. The default. */
    XZ,
    /** Y-Z plane: left and up. */
    YZ,
    /** X-Y plane: forward and left, such as a turret. */
    XY
  };

  MechanismPositionConfig() = default;

  /**
   * Set where the mechanism is mounted, relative to the robot, or to its parent.
   *
   * @param robotToMechanism Translation from the robot, or from its parent, to the mechanism.
   * @return *this for chaining.
   */
  MechanismPositionConfig& WithRelativePosition(const wpi::math::Translation3d& robotToMechanism) {
    m_robotToMechanism = robotToMechanism;
    return *this;
  }

  /**
   * Set where the mechanism is mounted, relative to the robot, or to its parent, and how it is turned
   * there. The mechanism's pose is worked out from here: an arm's or pivot's axis, an elevator's
   * carriage at zero height, or a flywheel's axle.
   *
   * @param robotToMechanism Transform from the robot, or from its parent, to the mechanism.
   * @return *this for chaining.
   */
  MechanismPositionConfig& WithRelativePosition(const wpi::math::Transform3d& robotToMechanism) {
    m_robotToMechanism = robotToMechanism.Translation();
    m_mountRotation = robotToMechanism.Rotation();
    return *this;
  }

  /**
   * Set the plane that the mechanism moves in.
   *
   * @param plane The plane. Default is the X-Z plane.
   * @return *this for chaining.
   */
  MechanismPositionConfig& WithMovementPlane(Plane plane) {
    m_plane = plane;
    return *this;
  }

  /**
   * Mount the mechanism on another, such as an arm on an elevator's carriage, so it moves with it. The
   * mechanism's relative position is then from the parent's pose, rather than from the robot.
   *
   * @param parentPose The pose the mechanism is mounted on, such as the elevator's GetPose3d().
   * @return *this for chaining.
   */
  MechanismPositionConfig& WithParent(std::function<wpi::math::Pose3d()> parentPose) {
    m_parent = std::move(parentPose);
    return *this;
  }

  /** Get the translation from the robot, or from the parent, to the mechanism, if set. */
  std::optional<wpi::math::Translation3d> GetRelativePosition() const { return m_robotToMechanism; }

  /** Get the plane that the mechanism moves in. */
  Plane GetMovementPlane() const { return m_plane; }

  /**
   * Where the mechanism is mounted: its parent's pose, or the robot's origin, moved and turned by its
   * relative position.
   */
  wpi::math::Pose3d GetMountPose() const {
    wpi::math::Pose3d base = m_parent ? m_parent() : wpi::math::Pose3d{};
    return base.TransformBy(wpi::math::Transform3d{m_robotToMechanism.value_or(wpi::math::Translation3d{}),
                                                   m_mountRotation});
  }

  /**
   * The pose of a part that turns about the axis normal to the movement plane, such as an arm or a
   * pivot, at an angle. At 0 it points along +X in the XZ and XY planes, and along +Y in the YZ plane,
   * and a positive angle turns it toward +Z in the XZ and YZ planes, and toward +Y in the XY plane.
   *
   * @param angle The angle it is turned to.
   * @return Its pose: where it is mounted, turned by the angle.
   */
  wpi::math::Pose3d GetRotatedPose(wpi::units::radian_t angle) const {
    return GetMountPose().TransformBy(
        wpi::math::Transform3d{wpi::math::Translation3d{}, RotationInPlane(angle)});
  }

  /**
   * The pose of a part that slides along the movement plane, such as an elevator's carriage.
   *
   * @param distance How far it has slid from where it is mounted.
   * @param angle The direction it slides in, in the movement plane: 0 along +X (+Y in the YZ plane),
   *              90 degrees along +Z (+Y in the XY plane).
   * @return Its pose: where it is mounted, moved by the distance, and not turned.
   */
  wpi::math::Pose3d GetTranslatedPose(wpi::units::meter_t distance, wpi::units::radian_t angle) const {
    const wpi::units::meter_t along = distance * std::cos(angle.value());
    const wpi::units::meter_t across = distance * std::sin(angle.value());
    wpi::math::Translation3d slide;
    switch (m_plane) {
      case Plane::XZ:
        slide = wpi::math::Translation3d{along, wpi::units::meter_t{0}, across};
        break;
      case Plane::YZ:
        slide = wpi::math::Translation3d{wpi::units::meter_t{0}, along, across};
        break;
      case Plane::XY:
        slide = wpi::math::Translation3d{along, across, wpi::units::meter_t{0}};
        break;
    }
    return GetMountPose().TransformBy(wpi::math::Transform3d{slide, wpi::math::Rotation3d{}});
  }

  /**
   * The rotation of a part turned by an angle about the axis normal to the movement plane.
   *
   * @param angle The angle.
   * @return The rotation.
   */
  wpi::math::Rotation3d RotationInPlane(wpi::units::radian_t angle) const {
    switch (m_plane) {
      case Plane::YZ:
        // +Y turns toward +Z: a positive roll about +X.
        return wpi::math::Rotation3d{angle, wpi::units::radian_t{0}, wpi::units::radian_t{0}};
      case Plane::XY:
        // +X turns toward +Y: a positive yaw about +Z.
        return wpi::math::Rotation3d{wpi::units::radian_t{0}, wpi::units::radian_t{0}, angle};
      case Plane::XZ:
      default:
        // +X turns toward +Z: a negative pitch about +Y.
        return wpi::math::Rotation3d{wpi::units::radian_t{0}, -angle, wpi::units::radian_t{0}};
    }
  }

 private:
  std::optional<wpi::math::Translation3d> m_robotToMechanism;
  wpi::math::Rotation3d m_mountRotation;
  Plane m_plane{Plane::XZ};
  std::function<wpi::math::Pose3d()> m_parent;
};

}  // namespace yams::mechanisms::config
