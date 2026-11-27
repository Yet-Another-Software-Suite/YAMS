---
description: >-
  The swerve_skid_detection example robot in the YAMS repository: how
  SkidDetector and a standalone SwerveDrivePoseEstimator work in Java
  (Commands v2 and v3) and C++.
icon: code
---

# The swerve skid detection example

The [`swerve_skid_detection` example](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands2/swerve_skid_detection) in the YAMS repository wraps the standard YAMS swerve drive with a standalone `SkidDetector` and a skid-filtered `SwerveDrivePoseEstimator`. During normal driving, all four modules integrate into the pose estimate. When one or more wheels slip (ratio above `1.35`), only the slipping wheels' distance deltas are frozen while the gripping wheels keep advancing the pose. It comes in Java with Commands v2 and [Commands v3](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands3/swerve_skid_detection), and the C++ equivalent is shown in the tabs below.

[Swerve skid detection](./) explains the math and why gyro `ω` is required. This page connects the math to each file and shows how to run and tune it.

**It builds on:** the [Swerve Drive](../../tutorials/swerve-drive/) tutorial, whose `SwerveDrive` setup and `SwerveInputStream` it reuses unchanged.

{% hint style="info" %}
The example runs purely on top of YAMS's public `SwerveDrive` getters (`getModuleStates()`, `getModulePositions()`, `getRotation()`), so no changes to core YAMS are required. The C++ port of YAMS exposes the same getters (`GetModuleStates()`, `GetModulePositions()`, `GetRotation()`), so the C++ tab below uses the exact same algorithm.
{% endhint %}

## Run it first

1. In the YAMS repository, generate the project from `examples`: `make swerve_skid_detection` for Commands v2, or `make commands3-swerve_skid_detection` for Commands v3.
2. Open `examples/generated/swerve_skid_detection` (or `commands3-swerve_skid_detection`) in VS Code and run **WPILib: Simulate Robot Code**.
3. Connect a controller, enable teleop (in Commands v3, pick **Swerve Skid Teleop**), and drive with the left and right sticks.
4. Open AdvantageScope and graph:
   * `NT:/SkidDetection/Ratio` — translational speed ratio across the four modules (`1.0` when gripping cleanly).
   * `NT:/SkidDetection/IsSkidding` — `true` whenever `Ratio > 1.35` and the slowest module is above `0.20 m/s`.
   * `NT:/SkidDetection/SlippingModuleCount` — how many modules are currently excluded from odometry.
   * `NT:/Mechanisms/swerve/field` — compare the robot pose with `Filtered Pose`.

## Where each idea is in the code

The file names below are the Commands v2 ones. In Commands v3, `SwerveSubsystem` is `mechanisms/SwerveMechanism`, and bindings live in `opmodes/teleop/SwerveSkidTeleop`.

| Idea | Read about it in | In the example |
| --- | --- | --- |
| Gyro-referenced translational velocity per module | [Why gyro ω matters](./#why-gyro-ω-and-a-slowest-module-deadband-matter) | `SkidDetector.detect(SwerveModuleVelocity[], AngularVelocity)` |
| Slowest-module low-speed deadband | [Why gyro ω matters](./#why-gyro-ω-and-a-slowest-module-deadband-matter) | `minSpeed < minTranslationalSpeedMps` guard in `SkidDetector.detect(...)` |
| Per-wheel slip mask (keep gripping wheels moving) | [Tuning the YAMS side](./#tuning-the-yams-side) | `medianSpeed` comparison and `slippingModules[]` in `SkidDetector.Result` |
| Standalone skid-filtered pose estimator | [How the sources' ideas apply to YAMS](./#how-the-sources-ideas-apply-to-yams) | `updateFilteredOdometry()` in `SwerveSubsystem` / `SwerveMechanism` |

## Detecting skids with gyro ω

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="SkidDetector.java (core detect method)" %}
```java
public Result detect(SwerveModuleVelocity[] moduleStates, AngularVelocity gyroYawRate) {
  double omegaRadPerSec = gyroYawRate.in(RadiansPerSecond);
  double[] transSpeeds = new double[moduleLocations.length];
  double minSpeed = Double.POSITIVE_INFINITY;
  double maxSpeed = 0.0;

  for (int i = 0; i < moduleLocations.length; i++) {
    double angleRad = moduleStates[i].angle.getRadians();
    double measVx = moduleStates[i].velocity * Math.cos(angleRad);
    double measVy = moduleStates[i].velocity * Math.sin(angleRad);
    Translation2d loc = moduleLocations[i];
    // Subtract rigid-body rotational velocity: omega x r = (-omega * y, +omega * x).
    double transVx = measVx - (-omegaRadPerSec * loc.getY());
    double transVy = measVy - (omegaRadPerSec * loc.getX());
    transSpeeds[i] = Math.hypot(transVx, transVy);
    minSpeed = Math.min(minSpeed, transSpeeds[i]);
    maxSpeed = Math.max(maxSpeed, transSpeeds[i]);
  }

  // Apply deadband to the SLOWEST module so low-speed encoder noise cannot spike max / min.
  if (minSpeed < minTranslationalSpeedMps) {
    return Result.noSkid(moduleLocations.length);
  }

  double ratio = maxSpeed / minSpeed;
  boolean skidding = ratio > skidRatioThreshold;
  boolean[] slipping = new boolean[moduleLocations.length];
  int slippingCount = 0;
  if (skidding) {
    double referenceSpeed = median(transSpeeds);
    for (int i = 0; i < transSpeeds.length; i++) {
      double devRatio =
          Math.max(transSpeeds[i], referenceSpeed) / Math.max(1e-6, Math.min(transSpeeds[i], referenceSpeed));
      if (devRatio > skidRatioThreshold) {
        slipping[i] = true;
        slippingCount++;
      }
    }
    if (slippingCount == 0) {
      // Split 2-vs-2 slip (e.g., both left wheels): flag modules above the slowest wheel.
      for (int i = 0; i < transSpeeds.length; i++) {
        if ((transSpeeds[i] / minSpeed) > skidRatioThreshold) {
          slipping[i] = true;
          slippingCount++;
        }
      }
    }
  }
  return new Result(ratio, skidding, slipping, slippingCount);
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="SkidDetector.java (identical pure math class)" %}
```java
// SkidDetector has zero Commands v2/v3 dependencies — Commands v3 uses the exact same class
// inside SwerveMechanism.periodic(), updating a standalone SwerveDrivePoseEstimator:
SkidDetector.Result skid = skidDetector.detect(drive.getModuleStates(), getGyroYawRate());
SwerveModulePosition[] filtered = filterSlippingPositions(drive.getModulePositions(), skid);
filteredPoseEstimator.update(drive.getRotation(), filtered);
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="SkidDetector.hpp (C++ WPILib 2027)" %}
```cpp
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>

struct SkidResult {
  double ratio{1.0};
  bool isSkidding{false};
  std::array<bool, 4> slippingModules{false, false, false, false};
  int slippingCount{0};
};

class SkidDetector {
 public:
  SkidDetector(std::array<wpi::math::Translation2d, 4> locations,
               double ratioThreshold = 1.35,
               units::meters_per_second_t minTransSpeed = 0.20_mps)
      : m_locations(locations),
        m_ratioThreshold(ratioThreshold),
        m_minTransSpeedMps(minTransSpeed.value()) {}

  SkidResult Detect(const std::array<wpi::math::SwerveModuleVelocity, 4>& states,
                    units::radians_per_second_t gyroYawRate) const {
    const double omega = gyroYawRate.value();
    std::array<double, 4> trans{};
    double minSpeed = 1e9;
    double maxSpeed = 0.0;

    for (size_t i = 0; i < 4; ++i) {
      const double ang = states[i].angle.Radians().value();
      const double vx = states[i].velocity.value() * std::cos(ang);
      const double vy = states[i].velocity.value() * std::sin(ang);
      const double tx = vx - (-omega * m_locations[i].Y().value());
      const double ty = vy - (omega * m_locations[i].X().value());
      trans[i] = std::hypot(tx, ty);
      minSpeed = std::min(minSpeed, trans[i]);
      maxSpeed = std::max(maxSpeed, trans[i]);
    }
    if (minSpeed < m_minTransSpeedMps) {
      return SkidResult{};
    }
    SkidResult out;
    out.ratio = maxSpeed / minSpeed;
    out.isSkidding = out.ratio > m_ratioThreshold;
    if (out.isSkidding) {
      auto sorted = trans;
      std::sort(sorted.begin(), sorted.end());
      const double median = 0.5 * (sorted[1] + sorted[2]);
      for (size_t i = 0; i < 4; ++i) {
        const double dev = std::max(trans[i], median) / std::max(1e-6, std::min(trans[i], median));
        if (dev > m_ratioThreshold) {
          out.slippingModules[i] = true;
          out.slippingCount++;
        }
      }
      if (out.slippingCount == 0) {
        for (size_t i = 0; i < 4; ++i) {
          if ((trans[i] / minSpeed) > m_ratioThreshold) {
            out.slippingModules[i] = true;
            out.slippingCount++;
          }
        }
      }
    }
    return out;
  }

 private:
  std::array<wpi::math::Translation2d, 4> m_locations;
  double m_ratioThreshold;
  double m_minTransSpeedMps;
};
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Make it yours

1. **Match your module locations:** update `MODULE_LOCATIONS` in `Constants.SkidConstants` to match the exact `Translation2d` coordinates you pass to each `SwerveModuleConfig.withLocation(...)`.
2. **Use your real gyro's yaw rate:** pass your Pigeon2 (`gyro.getAngularVelocityZWorld().getValue()`) or navX3 yaw rate in radians/second (CCW positive) — never `kinematics.toChassisVelocities(states).omega`.
3. **Log before filtering:** drive hard on carpet with `NT:/SkidDetection/Ratio` graphed in AdvantageScope. Set `SKID_RATIO_THRESHOLD` just above your normal hard-acceleration peak (typically `1.30`–`1.45`).
4. **Reset both estimators together:** whenever you reset odometry (e.g., at the start of auto), reset both YAMS's `drive.resetOdometry(pose)` and the standalone `filteredPoseEstimator.resetPosition(...)` plus `lastRawPositions`.

## If it does not work

* **`IsSkidding` turns true during pure rotation in place.** Your gyro yaw-rate sign is inverted relative to WPILib (`+Z` up, CCW positive). Negate the gyro rate passed to `SkidDetector.detect(...)`.
* **`IsSkidding` flickers true at crawl speed.** Raise `MIN_MODULE_TRANSLATIONAL_SPEED` (e.g., from `0.20 m/s` to `0.30 m/s`).
* **Filtered pose diverges from `drive.getPose()` during normal driving.** Verify `MODULE_LOCATIONS` order matches `drive.getModulePositions()` order (`frontLeft, frontRight, backLeft, backRight`).
