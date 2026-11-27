---
description: >-
  The tilt_scaled_odometry example robot in the YAMS repository: how
  TiltOdometryScaler supports cos(tilt) projection, linear trust ramps, and
  vision tilt gating in Java (Commands v2 and v3) and C++.
icon: code
---

# The tilt-scaled odometry example

The [`tilt_scaled_odometry` example](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands2/tilt_scaled_odometry) in the YAMS repository layers a standalone `TiltOdometryScaler` and `SwerveDrivePoseEstimator` on top of the standard YAMS swerve drive. Every loop, it computes the robot's 3D tilt angle from `drive.getGyroRotation3d()`, scales each module's wheel-distance delta using either `COSINE_PROJECTION` or `LINEAR_TRUST_RAMP`, and gates incoming vision measurements whenever tilt exceeds `10°`. It ships in Java with Commands v2 and [Commands v3](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands3/tilt_scaled_odometry), with the C++ equivalent below.

[Tilt-scaled odometry](./) compares `COSINE_PROJECTION` and `LINEAR_TRUST_RAMP`. This page connects the math to each file and shows how to run and tune it.

**It builds on:** the [Swerve Drive](../../tutorials/swerve-drive/) tutorial.

{% hint style="info" %}
The example uses only YAMS's public `SwerveDrive` getters (`getGyroRotation3d()`, `getModulePositions()`, `getRotation()`), requiring zero changes to core YAMS. The C++ port of YAMS exposes the same getters (`GetGyroRotation3d()`, `GetModulePositions()`, `GetRotation()`).
{% endhint %}

## Run it first

1. In the YAMS repository, generate the project from `examples`: `make tilt_scaled_odometry` for Commands v2, or `make commands3-tilt_scaled_odometry` for Commands v3.
2. Open `examples/generated/tilt_scaled_odometry` (or `commands3-tilt_scaled_odometry`) in VS Code and run **WPILib: Simulate Robot Code**.
3. Connect a controller, enable teleop (in Commands v3, pick **Tilt Scaled Odometry Teleop**), and drive with the sticks.
4. Open AdvantageScope and graph:
   * `NT:/TiltOdometry/TiltDeg` — current 3D chassis tilt angle in degrees.
   * `NT:/TiltOdometry/Scale` — active wheel-delta scale factor in `[0.0, 1.0]`.
   * `NT:/TiltOdometry/VisionAllowed` — `false` whenever tilt exceeds `10°`.
   * `NT:/Mechanisms/swerve/field` — compare the robot pose with `Tilt-Scaled Pose`.

## Where each idea is in the code

| Idea | Read about it in | In the example |
| --- | --- | --- |
| Exact 3D tilt angle from roll & pitch | [Choosing modes](./#choosing-cosine_projection-vs-linear_trust_ramp) | `TiltOdometryScaler.computeTiltAngle(Rotation3d)` |
| `COSINE_PROJECTION` vs. `LINEAR_TRUST_RAMP` | [Choosing modes](./#choosing-cosine_projection-vs-linear_trust_ramp) | `TiltOdometryScaler.computeScale(Rotation3d)` (`switch` over `ScalingMode`) |
| Scaling incremental wheel-distance deltas | [How the sources' ideas apply to YAMS](./#how-the-sources-ideas-apply-to-yams) | `TiltOdometryScaler.scalePositions(Rotation3d, SwerveModulePosition[])` |
| Vision tilt gate | [Tuning the YAMS side](./#tuning-the-yams-side) | `TiltOdometryScaler.shouldAcceptVision(Rotation3d)` + `addVisionMeasurement(...)` in `SwerveSubsystem` |

## Scaling wheel deltas and gating vision

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="TiltOdometryScaler.java (core scaling & vision gate)" %}
```java
public static Angle computeTiltAngle(Rotation3d attitude) {
  double cosTilt = Math.cos(attitude.getX()) * Math.cos(attitude.getY());
  return Radians.of(Math.acos(Math.max(-1.0, Math.min(1.0, cosTilt))));
}

public double computeScale(Rotation3d attitude) {
  double tiltRad = computeTiltAngle(attitude).in(Radians);
  return switch (mode) {
    case COSINE_PROJECTION -> Math.max(0.0, Math.cos(tiltRad));
    case LINEAR_TRUST_RAMP -> {
      if (tiltRad <= fullTrustTiltRad) {
        yield 1.0;
      }
      if (tiltRad >= zeroTrustTiltRad) {
        yield 0.0;
      }
      yield 1.0 - (tiltRad - fullTrustTiltRad) / (zeroTrustTiltRad - fullTrustTiltRad);
    }
  };
}

public boolean shouldAcceptVision(Rotation3d attitude) {
  return computeTiltAngle(attitude).in(Radians) <= maxVisionTiltRad + 1e-9;
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="SwerveMechanism.java (periodic update & vision gate)" %}
```java
// Inside SwerveMechanism.periodic():
Rotation3d attitude = drive.getGyroRotation3d();
SwerveModulePosition[] scaled = tiltScaler.scalePositions(attitude, drive.getModulePositions());
tiltPoseEstimator.update(drive.getRotation(), scaled);

// Before fusing any YALL / PhotonVision / Limelight pose:
public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
  if (tiltScaler.shouldAcceptVision(drive.getGyroRotation3d())) {
    tiltPoseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="TiltOdometryScaler.hpp (C++ WPILib 2027)" %}
```cpp
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <units/angle.h>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>

enum class TiltScalingMode { CosineProjection, LinearTrustRamp };

class TiltOdometryScaler {
 public:
  TiltOdometryScaler(TiltScalingMode mode,
                     units::radian_t fullTrustTilt,
                     units::radian_t zeroTrustTilt,
                     units::radian_t maxVisionTilt)
      : m_mode(mode),
        m_fullTrustRad(fullTrustTilt.value()),
        m_zeroTrustRad(zeroTrustTilt.value()),
        m_maxVisionRad(maxVisionTilt.value()) {}

  static units::radian_t ComputeTiltAngle(const wpi::math::Rotation3d& attitude) {
    const double cosTilt = std::cos(attitude.X().value()) * std::cos(attitude.Y().value());
    return units::radian_t{std::acos(std::clamp(cosTilt, -1.0, 1.0))};
  }

  double ComputeScale(const wpi::math::Rotation3d& attitude) const {
    const double tiltRad = ComputeTiltAngle(attitude).value();
    switch (m_mode) {
      case TiltScalingMode::CosineProjection:
        return std::max(0.0, std::cos(tiltRad));
      case TiltScalingMode::LinearTrustRamp:
        if (tiltRad <= m_fullTrustRad) return 1.0;
        if (tiltRad >= m_zeroTrustRad) return 0.0;
        return 1.0 - (tiltRad - m_fullTrustRad) / (m_zeroTrustRad - m_fullTrustRad);
    }
    return 1.0;
  }

  bool ShouldAcceptVision(const wpi::math::Rotation3d& attitude) const {
    return ComputeTiltAngle(attitude).value() <= m_maxVisionRad + 1e-9;
  }

 private:
  TiltScalingMode m_mode;
  double m_fullTrustRad;
  double m_zeroTrustRad;
  double m_maxVisionRad;
};
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Make it yours

1. **Pick your mode:** start with `ScalingMode.COSINE_PROJECTION` in `Constants.TiltConstants` for ramps; switch to `ScalingMode.LINEAR_TRUST_RAMP` if your drivetrain frequently unweights over abrupt bumps.
2. **Verify gyro mounting orientation:** if your Pigeon2 or navX3 is mounted sideways, configure its mount pose first (see [How do I use a gyro mounted on its side?](../../how-to-motor-controllers-and-sensors/how-do-i-use-a-gyro-mounted-on-its-side.md)) so flat carpet reads `0°` roll and `0°` pitch.
3. **Wire vision through `addVisionMeasurement(...)`:** route YALL / PhotonVision / Limelight estimates through `shouldAcceptVision(drive.getGyroRotation3d())` so tilted frames are dropped cleanly.
4. **Reset the scaler on pose resets:** always call `tiltScaler.reset(drive.getModulePositions())` inside `resetOdometry(Pose2d)`.

## If it does not work

* **`TiltDeg` is non-zero while sitting flat on carpet.** Check your IMU mount pose calibration — a `5°` static mounting offset will continuously bias `LINEAR_TRUST_RAMP`.
* **Pose lags behind real travel on a long ramp.** You are using `LINEAR_TRUST_RAMP` (`0.43` at `15°`); switch to `COSINE_PROJECTION` (`0.966` at `15°`).
