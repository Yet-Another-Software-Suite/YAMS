// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;

/** CAN IDs and analog channels. Everything is on one CAN bus. */
public final class Ports {
    // Systemcore has no "rio" bus; its first onboard CAN port takes that role.
    public static final CANPort kCANPort = CANPort.CAN_S0;
    public static final CANBus kCTRECANBus = new CANBus(kCANPort);

    // Swerve SPARK MAX IDs (NEOs) and Thrifty absolute encoder analog channels.
    public static final int kFrontLeftDrive = 1;
    public static final int kFrontLeftSteer = 2;
    public static final int kFrontLeftEncoder = 0;

    public static final int kFrontRightDrive = 3;
    public static final int kFrontRightSteer = 4;
    public static final int kFrontRightEncoder = 1;

    public static final int kBackLeftDrive = 5;
    public static final int kBackLeftSteer = 6;
    public static final int kBackLeftEncoder = 2;

    public static final int kBackRightDrive = 7;
    public static final int kBackRightSteer = 8;
    public static final int kBackRightEncoder = 3;

    // Mechanisms
    public static final int kIntakeRoller = 20;  // SPARK Flex, NEO Vortex
    public static final int kHood = 31;          // SPARK MAX, NEO
    public static final int kAgitator = 32;      // SPARK MAX, NEO
    public static final int kIndexer = 34;       // SPARK MAX, NEO
    public static final int kKicker = 35;        // SPARK MAX, NEO
    public static final int kShooterLeader = 36;   // TalonFX, Kraken X60
    public static final int kShooterFollower = 37; // TalonFX, Kraken X60
    public static final int kIntakeArmLeader = 40;   // SPARK MAX, NEO
    public static final int kIntakeArmFollower = 41; // SPARK MAX, NEO

    private Ports() {
    }
}
