package yams.math;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.MechanismGearing;

/**
 * Linear Quadratic Regulator (Regulator is a type of controller)
 */
public class LQRController
{

  /**
   * Get the {@link LinearSystem} for an elevator.
   *
   * @param motor      {@link DCMotor} of the elevator.
   * @param mass       {@link Mass} of the elevator carriage.
   * @param drumRadius {@link Distance} of the elevator drum radius.
   * @param gearing    {@link MechanismGearing} of the elevator from the drum to the rotor.
   * @return {@link LinearSystem}
   */
  public static LinearSystemLoop<N2, N1, N1> getElevatorSystem(DCMotor motor, Mass mass,
                                                               Distance drumRadius,
                                                               MechanismGearing gearing,
                                                               Distance positionTolerance,
                                                               LinearVelocity velocityTolerance, Voltage maxVoltage,
                                                               Time loopPeriod, Distance modelTrustPosition,
                                                               LinearVelocity modelTrustVelocity, double encoderTrust)
  {
    var elevatorPlant = LinearSystemId.createElevatorSystem(motor,
                                                            mass.in(Kilograms),
                                                            drumRadius.in(Meters),
                                                            gearing.getMechanismToRotorRatio());
    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
            VecBuilder.fill(positionTolerance.in(Meters), velocityTolerance.in(MetersPerSecond)), // qelms. Position
            // and velocity error tolerances, in meters and meters per second. Decrease this to more
            // heavily penalize state excursion, or make the controller behave more aggressively. In
            // this example we weight position much more highly than velocity, but this can be
            // tuned to balance the two.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    KalmanFilter<N2, N1, N1> observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
            VecBuilder.fill(modelTrustPosition.in(Meters), modelTrustVelocity.in(MetersPerSecond)), // How accurate we
            // think our model is, in meters and meters/second.
            VecBuilder.fill(encoderTrust), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);
    LinearSystemLoop<N2, N1, N1> loop =
        new LinearSystemLoop<>(
            (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
            controller,
            observer,
            maxVoltage.in(Volts),
            loopPeriod.in(Seconds));
    return loop;
  }

  /**
   * Get the {@link LinearSystem} for an arm.
   *
   * @param motor   {@link DCMotor} of the arm.
   * @param moi     {@link MomentOfInertia} of the arm. Imperial units are {@link yams.units.YUnits#PoundSquareFeet} or
   *                {@link yams.units.YUnits#PoundSquareInches}
   * @param gearing {@link MechanismGearing} of the arm from the rotor to the drum.
   *                {@code gearing.getMechanismToRotorRatio()}
   * @return {@link LinearSystem}
   */
  public static LinearSystemLoop getArmSystem(DCMotor motor, MomentOfInertia moi,
                                              MechanismGearing gearing, Distance positionTolerance,
                                              LinearVelocity velocityTolerance, Voltage maxVoltage,
                                              Time loopPeriod, double modelTrust, double encoderTrust)
  {
    var armPlant = LinearSystemId.createSingleJointedArmSystem(motor,
                                                               moi.in(KilogramSquareMeters),
                                                               gearing.getMechanismToRotorRatio());
    return null;
  }

  /**
   * Get the {@link LinearSystem} for an Flywheel.
   *
   * @param motor        {@link DCMotor} of the flywheel.
   * @param moi          {@link MomentOfInertia} of the flywheel. Imperial units are
   *                     {@link yams.units.YUnits#PoundSquareFeet} or {@link yams.units.YUnits#PoundSquareInches}
   * @param gearing      {@link MechanismGearing} of the flywheel from the rotor to the drum.
   *                     {@code gearing.getMechanismToRotorRatio()}
   * @param qelms        Velocity error tolerance. Decrease this to more heavily penalize state excursion, or make the
   *                     controller behave more starting point because that is the (approximate) maximum voltage of a
   *                     battery.
   * @param relms        Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or
   *                     make the controller less aggressive. 12 is a good starting point because that is the
   *                     (approximate) maximum voltage of a battery.
   * @param maxVoltage   Maximum voltage that can be applied.
   * @param loopPeriod   Nominal time between loops, normally 20ms.
   * @param modelTrust   Arbitrary scale of how accurate we think our model is (normally 3)
   * @param encoderTrust Arbitrary scale of how accurate we think our encoder is. (normally 0.01)
   * @return {@link LinearSystem}
   */
  public static LinearSystemLoop<N1, N1, N1> getFlywheelSystem(DCMotor motor, MomentOfInertia moi,
                                                               MechanismGearing gearing, AngularVelocity qelms,
                                                               Voltage relms,
                                                               Voltage maxVoltage,
                                                               Time loopPeriod, double modelTrust, double encoderTrust)
  {
    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    var flywheelPlant = LinearSystemId.createFlywheelSystem(motor,
                                                            moi.in(KilogramSquareMeters),
                                                            gearing.getMechanismToRotorRatio());
    LinearQuadraticRegulator<N1, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            flywheelPlant,
            VecBuilder.fill(qelms.in(RadiansPerSecond)),
            // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(relms.in(Volts)),
            // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            loopPeriod.in(Seconds)); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(modelTrust), // How accurate we think our model is
            VecBuilder.fill(encoderTrust), // How accurate we think our encoder
            // data is
            loopPeriod.in(Seconds));

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    LinearSystemLoop<N1, N1, N1> loop =
        new LinearSystemLoop<>(flywheelPlant, controller, m_observer, maxVoltage.in(Volts), loopPeriod.in(Seconds));
    return loop;
  }


}
