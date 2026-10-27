#!/usr/bin/env python3
# Copyright (c) 2026 Yet Another Software Suite
# SPDX-License-Identifier: LGPL-3.0-or-later
"""Pick the unit test groups CI runs for a change.

A pull request runs the test groups its changes can affect; a push to master runs every group. Each
changed file is classified by the first rule it matches:

- A test file runs the groups it belongs to: a Java test class by the --tests filters below, a C++
  test file by the Catch2 tags in it.
- A mechanism or swerve source file runs the mechanism or swerve groups.
- Any other file under yams/java, yams/cpp or vendordep runs every group of its language, or of
  both. This includes the motor controllers, which every test uses, test helpers and build files.
  A file nothing else matches can affect anything, so it runs everything.
- This workflow and this script run every group.
- Files elsewhere, such as the docs, the other workflows and the examples (which have their own
  workflow), run nothing.

Usage: select_test_groups.py [--all] < changed-files
Reads the changed files, one per line, unless --all. Prints `java=<matrix JSON>` and
`cpp=<matrix JSON>` lines for $GITHUB_OUTPUT, and why each file runs its groups to stderr.
"""

import fnmatch
import json
import re
import sys
from pathlib import Path

# Java unit test groups. Each runs in its own JVM, on its own runner.
JAVA_GROUPS = [
    # Each mechanism integration test runs in its own JVM: these create the largest number of
    # simulated motor controller devices per process.
    {"group": "arm", "test-filter": '--tests "yams.mechs.ArmTest"'},
    {"group": "elevator", "test-filter": '--tests "yams.mechs.ElevatorTest"'},
    {"group": "pivot", "test-filter": '--tests "yams.mechs.PivotTest"'},
    {"group": "shooter", "test-filter": '--tests "yams.mechs.ShooterTest"'},
    {"group": "mechanism-limits", "test-filter": '--tests "yams.mechs.MechanismLimitTest"'},
    # A double jointed arm on two SPARK MAXes moving around exclusion zones, compensating kG for where it is, and its
    # telemetry and live tuning.
    {"group": "double-jointed-arm", "test-filter": '--tests "yams.mechs.DoubleJointedArmTest"'},
    # Loosely coupled followers whose closed loop runs on the motor controller or on the SystemCore
    # (a Nova, a SPARK with an exponential profile and a SPARK with an LQR) following and stopping
    # with their leader. A TalonFX leads one, which Phoenix runs in real time.
    {"group": "loose-followers", "test-filter": '--tests "yams.mechs.LooselyCoupledFollowerTest"'},
    # A pivot driven into its hard stops, and a Talon set to a position, on every motor controller:
    # simulated Talons, which Phoenix runs in real time.
    {"group": "hard-stops", "test-filter": '--tests "yams.mechs.HardStopTest"'},
    # Four module swerve drives on a Pigeon2: TalonFX and SPARK MAX both ways round, and mixes of
    # SPARK MAX, SPARK Flex, TalonFXS and TalonFX, all with the same PID gains. Also SwerveInputStream
    # aiming and telemetry for Commands v2 and v3, on SPARK MAX swerve drives, which take a second.
    {"group": "swerve-drive",
     "test-filter": '--tests "yams.mechs.SwerveDriveTest"'
                    ' --tests "yams.commands2.swerve.SwerveInputStreamAimTest"'
                    ' --tests "yams.commands2.telemetry.SwerveInputStreamTelemetryTest"'
                    ' --tests "yams.commands3.swerve.SwerveInputStreamAimTest"'
                    ' --tests "yams.commands3.telemetry.SwerveInputStreamTelemetryTest"'},
    # Every motor controller, closed loop controller, and attached encoder crossing the continuous
    # wrapping point: many simulated Talons, which Phoenix runs in real time.
    {"group": "continuous-wrapping",
     "test-filter": '--tests "yams.core.motorcontrollers.ContinuousWrappingTest"'},
    # External encoders on each motor controller: CANcoders and CANdis on Talons, the SPARK absolute
    # encoder and Through Bore Encoders (wired and over CAN) on SPARKs, and starting away from zero
    # with and without an absolute encoder.
    {"group": "absolute-encoders",
     "test-filter": '--tests "yams.core.motorcontrollers.CANcoderTest"'
                    ' --tests "yams.core.motorcontrollers.CANdiTest"'
                    ' --tests "yams.core.motorcontrollers.SparkAbsoluteEncoderTest"'
                    ' --tests "yams.core.motorcontrollers.ThroughBoreEncoderTest"'
                    ' --tests "yams.core.motorcontrollers.StartingPositionTest"'},
    # Every absolute encoder under every closed loop controller: these create many simulated Talons,
    # which Phoenix runs in real time, so each runs in its own JVM.
    {"group": "discontinuity-points",
     "test-filter": '--tests "yams.core.motorcontrollers.DiscontinuityPointTest"'},
    {"group": "zero-offsets",
     "test-filter": '--tests "yams.core.motorcontrollers.ZeroOffsetTest"'
                    ' --tests "yams.core.motorcontrollers.ZeroOffsetDiscontinuityPointTest"'},
    # Motor controller unit tests that are not about an external encoder: the configuration each
    # vendor's device is given, feedforwards, exponential profiles and the simulation period. Add new
    # motor controller test classes here.
    {"group": "motor-controllers",
     "test-filter": '--tests "yams.core.motorcontrollers.ArmFeedforwardTest"'
                    ' --tests "yams.core.motorcontrollers.ExponentialProfileConfigTest"'
                    ' --tests "yams.core.motorcontrollers.ForceFeedforwardTest"'
                    ' --tests "yams.core.motorcontrollers.SimulationPeriodTest"'
                    ' --tests "yams.core.motorcontrollers.SparkDeviceConfigTest"'
                    ' --tests "yams.core.motorcontrollers.TalonDeviceConfigTest"'},
    # Live tuning: every motor controller applying the tuned values, the Live Tuning command a
    # mechanism registers for its motor controllers, and driving a leader and its loosely coupled
    # follower at the tuning voltage, on pivots led by a SPARK Flex, a TalonFX and a SPARK MAX.
    {"group": "live-tuning",
     "test-filter": '--tests "yams.core.motorcontrollers.LiveTuningTest"'
                    ' --tests "yams.core.telemetry.MechanismLiveTuningTest"'
                    ' --tests "yams.mechs.VoltageLiveTuningTest"'},
    # Every motor controller (SPARK MAX, SPARK Flex, TalonFX, TalonFXS and Nova) given the same gains
    # in YAMS units: the voltage of each feedback and feedforward gain, and the gains of the selected
    # closed loop slot. The Talons run in real time, as Phoenix simulates them, and one JVM with all
    # the closed loop gain tests' Talons falls behind, so the profile tests run on their own.
    {"group": "closed-loop-gains",
     "test-filter": '--tests "yams.core.motorcontrollers.FeedbackTest"'
                    ' --tests "yams.core.motorcontrollers.FeedforwardTest"'
                    ' --tests "yams.core.motorcontrollers.ClosedLoopSlotTest"'},
    # Every motor controller following a trapezoidal profile from each closed loop slot.
    {"group": "closed-loop-profiles",
     "test-filter": '--tests "yams.core.motorcontrollers.FeedforwardProfileTest"'},
    # The units of the gains each motor controller is given: per rotation for an arm, per meter for
    # an elevator, on the motor controller and on the SystemCore.
    {"group": "closed-loop-gain-units",
     "test-filter": '--tests "yams.core.motorcontrollers.ClosedLoopGainUnitsTest"'},
    # Everything else: swerve config tests, the battery, motor and double jointed arm simulation unit
    # tests, and the math tests. Gradle's --tests cannot exclude the groups above, so each class is
    # listed: add new test classes here unless they get their own group.
    {"group": "other",
     "test-filter": '--tests "yams.core.mechanisms.*"'
                    ' --tests "yams.core.motorcontrollers.simulation.*"'
                    ' --tests "yams.core.math.*"'
                    ' --tests "yams.ChineseRemainderTheoremTest"'},
]

# C++ unit test groups: the same groups as the Java unit tests, for the groups the C++ tests cover.
# A comma in a Catch2 filter means "or".
CPP_GROUPS = [
    {"group": "arm", "catch-filter": "[ArmTest]"},
    {"group": "elevator", "catch-filter": "[ElevatorTest]"},
    {"group": "pivot", "catch-filter": "[PivotTest]"},
    {"group": "shooter", "catch-filter": "[ShooterTest]"},
    # The four module swerve drive, and the SwerveInputStream telemetry that runs on it.
    {"group": "swerve-drive", "catch-filter": "[SwerveDriveTest],[SwerveInputStreamTelemetryTest]"},
    # Motor controller unit tests: the parity and regression tests of each vendor's wrapper. Add new
    # motor controller tags here and to the exclusions of the other group.
    {"group": "motor-controllers",
     "catch-filter": "[SMCParity],[SMCRegression],[SparkParity],[NovaParity],[TalonApi]"},
    # The Talon parity tests wait on simulated Talons and CANcoders, which Phoenix runs in real time,
    # so they can time out on a busy runner: they run on their own.
    {"group": "talon-parity", "catch-filter": "[TalonParity]"},
    # Everything else: the math tests. New tags run here unless they get their own group and are
    # excluded.
    {"group": "other",
     "catch-filter": "~[ArmTest]~[ElevatorTest]~[PivotTest]~[ShooterTest]~[SwerveDriveTest]"
                     "~[SwerveInputStreamTelemetryTest]~[SMCParity]~[SMCRegression]~[SparkParity]"
                     "~[NovaParity]~[TalonApi]~[TalonParity]"},
]

JAVA_MECHANISM_GROUPS = {"arm", "elevator", "pivot", "shooter", "mechanism-limits", "hard-stops",
                         "loose-followers", "live-tuning", "swerve-drive", "double-jointed-arm", "other"}
JAVA_SWERVE_GROUPS = {"swerve-drive", "other"}
CPP_MECHANISM_GROUPS = {"arm", "elevator", "pivot", "shooter", "swerve-drive", "other"}

JAVA_SOURCE = "yams/java/"
JAVA_TESTS = "vendordep/src/test/java/"
CPP_SOURCE = "yams/cpp/"
CPP_TESTS = "vendordep/src/test/native/cpp/"

# Changed files that run every group.
CI = [".github/workflows/ci.yml", ".github/scripts/select_test_groups.py"]

# Changed files that affect no unit test.
NO_TESTS = ["*.md", "docs/*", "examples/*", "vendordep/README.md"]

# Source that only the mechanism tests, or only the swerve tests, use.
JAVA_SWERVE = ["yams/java/yams/core/mechanisms/swerve/*", "yams/java/yams/commands2/swerve/*",
               "yams/java/yams/commands3/swerve/*"]
JAVA_MECHANISMS = ["yams/java/yams/core/mechanisms/*", "yams/java/yams/commands2/mechanisms/*",
                   "yams/java/yams/commands3/mechanisms/*"]
CPP_SWERVE = ["yams/cpp/*/mechanisms/swerve/*"]
CPP_MECHANISMS = ["yams/cpp/*/mechanisms/*"]


def names(groups):
    return {group["group"] for group in groups}


def matches(path, patterns):
    return any(fnmatch.fnmatch(path, pattern) for pattern in patterns)


def java_test_groups(path):
    """The Java groups whose --tests filters include the test class in path."""
    class_name = path[len(JAVA_TESTS):].removesuffix(".java").replace("/", ".")
    selected = set()
    for group in JAVA_GROUPS:
        for test in re.findall(r'--tests "([^"]+)"', group["test-filter"]):
            if fnmatch.fnmatch(class_name, test):
                selected.add(group["group"])
    return selected


def cpp_test_groups(path):
    """The C++ groups that run the Catch2 tags in path; tags no group names run in other."""
    try:
        tags = set(re.findall(r'"(\[[A-Za-z0-9]+\])"', Path(path).read_text()))
    except OSError:
        return set()
    selected = set()
    for tag in tags:
        in_group = {group["group"] for group in CPP_GROUPS
                    if tag in group["catch-filter"].split(",")}
        selected |= in_group or {"other"}
    return selected


def select(path):
    """The (Java, C++) groups a changed file affects."""
    if path in CI:
        return names(JAVA_GROUPS), names(CPP_GROUPS)
    if matches(path, NO_TESTS):
        return set(), set()
    if path.startswith(JAVA_TESTS) and path.endswith(".java"):
        # A file no group runs, such as a test helper, can affect any group.
        return java_test_groups(path) or names(JAVA_GROUPS), set()
    if path.startswith(CPP_TESTS) and path.endswith(".cpp"):
        # A file with no tags, such as the test runner, can affect any group.
        return set(), cpp_test_groups(path) or names(CPP_GROUPS)
    if matches(path, JAVA_SWERVE):
        return set(JAVA_SWERVE_GROUPS), set()
    if matches(path, JAVA_MECHANISMS):
        return set(JAVA_MECHANISM_GROUPS), set()
    if matches(path, CPP_SWERVE):
        return set(), {"swerve-drive", "other"}
    if matches(path, CPP_MECHANISMS):
        return set(), set(CPP_MECHANISM_GROUPS)
    if path.startswith(JAVA_SOURCE):
        return names(JAVA_GROUPS), set()
    if path.startswith(CPP_SOURCE) or path.startswith("vendordep/src/test/native/"):
        return set(), names(CPP_GROUPS)
    if path.startswith("vendordep/"):
        # Build files and vendordeps: everything.
        return names(JAVA_GROUPS), names(CPP_GROUPS)
    return set(), set()


def main(args):
    run_all = "--all" in args
    files = [] if run_all else [line.strip() for line in sys.stdin if line.strip()]
    java, cpp = (names(JAVA_GROUPS), names(CPP_GROUPS)) if run_all else (set(), set())
    for path in files:
        java_groups, cpp_groups = select(path)
        if java_groups or cpp_groups:
            print(f"{path}: Java {sorted(java_groups)}, C++ {sorted(cpp_groups)}", file=sys.stderr)
        java |= java_groups
        cpp |= cpp_groups
    print("java=" + json.dumps([group for group in JAVA_GROUPS if group["group"] in java]))
    print("cpp=" + json.dumps([group for group in CPP_GROUPS if group["group"] in cpp]))


if __name__ == "__main__":
    main(sys.argv[1:])
