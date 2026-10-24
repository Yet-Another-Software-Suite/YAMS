// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import first.robot.subsystems.LQRArmSubsystem;
import first.robot.subsystems.LQRFlyWheelSubsystem;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.framework.TimedRobot;

/**
 * An arm and a flywheel run by LQRs. If you change the name of this class or the package after creating this project,
 * you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public final LQRArmSubsystem arm = new LQRArmSubsystem();
  public final LQRFlyWheelSubsystem flyWheel = new LQRFlyWheelSubsystem();

  private final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  public Robot() {
    // A raises the arm level, B stows it.
    operatorController.a().onTrue(arm.setAngle(Degrees.of(0)));
    operatorController.b().onTrue(arm.setAngle(Degrees.of(-20)));
    // Hold the right bumper to spin the flywheel up.
    operatorController.rightBumper().whileTrue(flyWheel.spinUp(RPM.of(3000)));
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, and with it each subsystem's periodic(), which publishes the YAMS telemetry.
    CommandScheduler.getInstance().run();
  }
}
