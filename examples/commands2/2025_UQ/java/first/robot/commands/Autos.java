// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot.commands;

import first.robot.Constants.ArmConstants;
import first.robot.subsystems.ArmSubsystem;
import first.robot.subsystems.DriveTrain;
import first.robot.subsystems.Intake;
import org.wpilib.command2.Command;

public final class Autos {
  /** Scores L1 coral and knocks off low algae from the left side. */
  public static Command autoSideLeft(DriveTrain drive, ArmSubsystem arm, Intake intake) {

    // Drive straight
    return drive.moveStraight(-0.25).withTimeout(2.75).

    // Turn to face reef wall
    andThen(drive.turn(-0.25).withTimeout(1.0)).

    // Move to reef wall
    andThen(drive.moveStraight(-0.20).withTimeout(1.0)).

    // Run intake while flush with reef wall, depositing L1 coral
    andThen(intake.moveIntake(-1.0).withTimeout(3.0)).

    // Move backwards with intake still running to dislodge algae
    andThen(intake.moveIntake(-1.0).withTimeout(3.0).alongWith(drive.moveStraight(0.1).withTimeout(3))).

    // Do all of the above while maintaining arm position at the 'remove low algae' position
    alongWith(arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow)).repeatedly();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
