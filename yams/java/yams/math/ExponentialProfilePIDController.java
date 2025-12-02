package yams.math;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.Optional;
import yams.gearing.MechanismGearing;

/**
 * Exponential profile PID controller. Similar to {@link PIDController} or
 * {@link edu.wpi.first.math.controller.ProfiledPIDController}, but uses an {@link ExponentialProfile}
 */
public class ExponentialProfilePIDController
{

  /**
   * Iteration timer.
   */
  private final Timer              timer   = new Timer();
  /**
   * The wrapped PID controller.
   */
  private final PIDController      controller;
  /**
   * The wrapped profile.
   */
  private       ExponentialProfile profile = null;
  /**
   * The current state from {@link ExponentialProfile}
   */
  private       ExponentialProfile.State           currentState = new State();
  /**
   * The next state from {@link ExponentialProfile}
   */
  private       Optional<ExponentialProfile.State> nextState    = Optional.empty();
  /**
   * Loop time.
   */
  private       Time                               loopTime     = Milliseconds.of(20);
  /**
   * {@link ExponentialProfile.Constraints} for the {@link ExponentialProfile}.
   */
  private       Constraints                        constraints  = null;

  /**
   * Constructor.
   *
   * @param controller  The wrapped PID controller.
   * @param constraints The wrapped profile constraints.
   */
  public ExponentialProfilePIDController(PIDController controller, Constraints constraints)
  {
    this.controller = controller;
    this.constraints = constraints;
    this.profile = new ExponentialProfile(constraints);
  }

  /**
   * Constructor.
   *
   * @param kP          kP value for the {@link PIDController}
   * @param kI          kI value for the {@link PIDController}
   * @param kD          kD value for the {@link PIDController}
   * @param constraints {@link Constraints} for the {@link ExponentialProfile}
   */
  public ExponentialProfilePIDController(double kP, double kI, double kD, Constraints constraints)
  {
    this(new PIDController(kP, kI, kD), constraints);
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for an elevator.
   *
   * @param maxVolts   Maximum input voltage for profile generation.
   * @param motor      {@link DCMotor} of the elevator.
   * @param mass       {@link Mass} of the elevator carriage.
   * @param drumRadius {@link Distance} of the elevator drum radius.
   * @param gearing    {@link MechanismGearing} of the elevator from the drum to the rotor.
   * @return {@link ExponentialProfile.Constraints}
   */
  public static ExponentialProfile.Constraints createElevatorConstraints(Voltage maxVolts, DCMotor motor, Mass mass,
                                                                         Distance drumRadius,
                                                                         MechanismGearing gearing)
  {
    var sysid = LinearSystemId.createElevatorSystem(motor,
                                                    mass.in(Kilograms),
                                                    drumRadius.in(Meters),
                                                    gearing.getMechanismToRotorRatio());
    var circumference = (2.0 * Math.PI * drumRadius.in(Meters));

    var A  = sysid.getA(0, 0);
    var B  = sysid.getB(0, 0);
    var kV = MetersPerSecond.of(-A / B);
    var kA = MetersPerSecondPerSecond.of(1.0 / B);
    return ExponentialProfile.Constraints.fromCharacteristics(maxVolts.in(Volts),
                                                              kV.in(MetersPerSecond) / circumference,
                                                              kA.in(MetersPerSecondPerSecond) / circumference);
//    return ExponentialProfile.Constraints.fromStateSpace(maxVolts.in(Volts), A, B);
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for an arm.
   *
   * @param maxVolts Maximum input voltage for profile generation.
   * @param motor    {@link DCMotor} of the arm.
   * @param moi      {@link MomentOfInertia} of the arm.
   * @param gearing  {@link MechanismGearing} of the arm from the rotor to the drum.
   *                 {@code gearing.getMechanismToRotorRatio()}
   * @return {@link ExponentialProfile.Constraints}
   */
  public static ExponentialProfile.Constraints createArmConstraints(Voltage maxVolts, DCMotor motor, MomentOfInertia moi,
                                                                    MechanismGearing gearing)
  {
    var sysid = LinearSystemId.createSingleJointedArmSystem(motor,
                                                            moi.in(KilogramSquareMeters),
                                                            gearing.getMechanismToRotorRatio());
    var A  = sysid.getA(0, 0); // radians
    var B  = sysid.getB(0, 0); // radians
    var kV = RadiansPerSecond.of(-A / B);
    var kA = RadiansPerSecondPerSecond.of(1.0 / B);
//    return ExponentialProfile.Constraints.fromStateSpace(maxVolts.in(Volts), A, B);
    return ExponentialProfile.Constraints.fromCharacteristics(maxVolts.in(Volts),
                                                              kV.in(RotationsPerSecond),
                                                              kA.in(RotationsPerSecondPerSecond));
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for an arm.
   *
   * @param maxVolts Maximum input voltage for profile generation.
   * @param motor    {@link DCMotor} of the arm.
   * @param mass     {@link Mass} of the arm.
   * @param length   {@link Distance} of the arm length.
   * @param gearing  {@link MechanismGearing} of the arm from the rotor to the drum.
   *                 {@code gearing.getMechanismToRotorRatio()}
   * @return {@link ExponentialProfile.Constraints}
   */
  public static ExponentialProfile.Constraints createArmConstraints(Voltage maxVolts, DCMotor motor, Mass mass,
                                                                    Distance length, MechanismGearing gearing)
  {
    return createArmConstraints(maxVolts, motor,
            KilogramSquareMeters.of(SingleJointedArmSim.estimateMOI(length.in(Meters), mass.in(Kilograms))),
            gearing);
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for a flywheel.
   *
   * @param maxVolts Maximum input voltage for profile generation.
   * @param motor    {@link DCMotor} of the flywheel.
   * @param moi      {@link MomentOfInertia} of the flywheel.
   * @param gearing  {@link MechanismGearing} of the flywheel from the rotor to the drum.
   * @return {@link ExponentialProfile.Constraints}
   */
  public static ExponentialProfile.Constraints createFlywheelConstraints(Voltage maxVolts, DCMotor motor, MomentOfInertia moi,
                                                                         MechanismGearing gearing)
  {
    return createArmConstraints(maxVolts, motor, moi, gearing);
//    var sysid = LinearSystemId.createFlywheelSystem(motor,
//                                                    SingleJointedArmSim.estimateMOI(radius.in(Meters),
//                                                                                    mass.in(Kilograms)),
//                                                    gearing.getMechanismToRotorRatio());
//    var A = RadiansPerSecond.of(sysid.getA(0, 0));
//    var B = RadiansPerSecondPerSecond.of(sysid.getB(0, 0));
//    return ExponentialProfile.Constraints.fromStateSpace(maxVolts.in(Volts), A.in(RotationsPerSecond), B.in(RotationsPerSecondPerSecond));
////    return ExponentialProfile.Constraints.fromCharacteristics(maxVolts.in(Volts), -A/B, 1.0/B);
  }

  /**
   * Get the {@link ExponentialProfile.Constraints} for a flywheel.
   *
   * @param maxVolts Maximum input voltage for profile generation.
   * @param motor    {@link DCMotor} of the flywheel.
   * @param mass     {@link Mass} of the flywheel.
   * @param radius   {@link Distance} of the flywheel radius.
   * @param gearing  {@link MechanismGearing} of the flywheel from the rotor to the drum.
   * @return {@link ExponentialProfile.Constraints}
   */
  public static ExponentialProfile.Constraints createFlywheelConstraints(Voltage maxVolts, DCMotor motor, Mass mass,
                                                                         Distance radius, MechanismGearing gearing)
  {
    return createArmConstraints(maxVolts, motor, mass, radius, gearing);
//    var sysid = LinearSystemId.createFlywheelSystem(motor,
//                                                    SingleJointedArmSim.estimateMOI(radius.in(Meters),
//                                                                                    mass.in(Kilograms)),
//                                                    gearing.getMechanismToRotorRatio());
//    var A = RadiansPerSecond.of(sysid.getA(0, 0));
//    var B = RadiansPerSecondPerSecond.of(sysid.getB(0, 0));
//    return ExponentialProfile.Constraints.fromStateSpace(maxVolts.in(Volts), A.in(RotationsPerSecond), B.in(RotationsPerSecondPerSecond));
////    return ExponentialProfile.Constraints.fromCharacteristics(maxVolts.in(Volts), -A/B, 1.0/B);
  }

  /**
   * Create a generic constraints object.
   *
   * @param maxVolts        Maximum input voltage for profile generation.
   * @param maxVelocity     Maximum velocity.
   * @param maxAcceleration Maximum acceleration.
   * @return {@link ExponentialProfile.Constraints}
   */
  public static Constraints createConstraints(Voltage maxVolts, AngularVelocity maxVelocity,
                                              AngularAcceleration maxAcceleration)
  {
    var maxV = maxVolts.in(Volts);
    return ExponentialProfile.Constraints.fromStateSpace(maxVolts.in(Volts),
                                                         maxV / maxVelocity.in(RotationsPerSecond),
                                                         maxV / maxAcceleration.in(RotationsPerSecondPerSecond));
  }

  /**
   * Get the velocity gain as constant.
   *
   * @return kV with (-A/B)
   */
  public AngularVelocity getKv()
  {
    if (constraints == null)
    {
      throw new IllegalStateException("constraints must be set before getting Kv");
    }
    var A = constraints.A;
    var B = constraints.B;
    return RotationsPerSecond.of(-A / B);
  }

  /**
   * Get the acceleration gain kA
   *
   * @return kA interpreted as (1.0/B)
   */
  public AngularAcceleration getKa()
  {
    if (constraints == null)
    {
      throw new IllegalStateException("constraints must be set before getting Kv");
    }
    var A = constraints.A;
    var B = constraints.B;
    return RotationsPerSecondPerSecond.of(1.0 / B);
  }

  /**
   * Reset the controller, set the next setpoint to empty.
   *
   * @param measurement Measurement in Rotations, and Rotations per Second.
   */
  public void reset(State measurement)
  {
    controller.reset();
    currentState = measurement;
    nextState = Optional.empty();
  }

  /**
   * Reset the PID and profile with the given postion and velocity as the measured position and velocity.
   *
   * @param position Measured position
   * @param velocity Measured velocity.
   */
  public void reset(double position, double velocity)
  {
    reset(new ExponentialProfile.State(position, velocity));
  }

  /**
   * Get the constraints.
   *
   * @return {@link ExponentialProfile.Constraints}
   */
  public Optional<Constraints> getConstraints()
  {
    return Optional.ofNullable(constraints);
  }

  /**
   * Sets the constraints for the {@link ExponentialProfile}.
   *
   * @param constraints The constraints for the {@link ExponentialProfile}.
   */
  public void setConstraints(Constraints constraints)
  {
    this.constraints = constraints;
    profile = new ExponentialProfile(constraints);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param tolerance – Error which is tolerable.
   */
  public void setTolerance(double tolerance)
  {
    controller.setTolerance(tolerance);
  }

  /**
   * Returns true if the error is within the tolerance of the setpoint. The error tolerance defaults to 0.05, and the
   * error derivative tolerance defaults to ∞. This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds
   */
  public boolean atSetpoint()
  {
    return controller.atSetpoint();
  }

  /**
   * Get the setpoint.
   *
   * @return setpoint.
   */
  public double getSetpoint()
  {
    return controller.getSetpoint();
  }

  /**
   * Get the current state of the {@link ExponentialProfile}.
   *
   * @return {@link ExponentialProfile.State} given by the {@link ExponentialProfile}.
   */
  public State getCurrentState()
  {
    return currentState;
  }

  /**
   * Get the current angle.
   *
   * @return {@link Angle} from {@link ExponentialProfile}
   */
  public Angle getCurrentAngle()
  {
    return Rotations.of(currentState.position);
  }

  /**
   * Get the next angle from the {@link ExponentialProfile}
   *
   * @return {@link Angle} from {@link ExponentialProfile}
   */
  public Angle getNextAngle()
  {
    return Rotations.of(nextState.orElseThrow().position);
  }

  /**
   * Get the current velocity from {@link ExponentialProfile}.
   *
   * @return {@link AngularVelocity} from {@link ExponentialProfile}
   */
  public AngularVelocity getCurrentVelocitySetpoint()
  {
    return RotationsPerSecond.of(currentState.velocity);
  }

  /**
   * Get the next velocity from {@link ExponentialProfile}
   *
   * @return Next {@link AngularVelocity} from {@link ExponentialProfile}
   */
  public AngularVelocity getNextVelocitySetpoint()
  {
    return RotationsPerSecond.of(nextState.orElseThrow().velocity);
  }

  /**
   * Get the next state of the {@link ExponentialProfile}.
   *
   * @return {@link ExponentialProfile.State} given by the {@link ExponentialProfile}.
   */
  public Optional<State> getNextState()
  {
    return nextState;
  }

  /**
   * Calculate the feedback, assuming previous state velocity.
   *
   * @param measurementPosition Measurement position to set as the current state..
   * @param setpointVelocity    Setpoint velocity.
   * @param setpointPosition    Setpoint position.
   * @return
   */
  public double calculate(double measurementPosition, double setpointVelocity, double setpointPosition)
  {
    if (timer.isRunning())
    {
      loopTime = Seconds.of(timer.get());
    }
    timer.reset();
    timer.start();
    var feedback = controller.calculate(measurementPosition, currentState.position);
    nextState.ifPresent(state -> currentState = state);
    nextState = Optional.of(profile.calculate(loopTime.in(Seconds),
                                              currentState,
                                              new State(setpointPosition, setpointVelocity)));
    return feedback;
  }

  /**
   * Calculate the feedback, assuming no setpoint velocity.
   *
   * @param measurementPosition Measurement position to set as the current state.
   * @param setpointPosition    Setpoint position.
   * @return
   */
  public double calculate(double measurementPosition, double setpointPosition)
  {
    return calculate(measurementPosition, 0, setpointPosition);
  }

  /**
   * Returns the error tolerance of this controller. Defaults to 0.05.
   *
   * @return the error tolerance of the controlle
   */
  public double getPositionTolerance()
  {
    return controller.getErrorTolerance();
  }

  /**
   * Get the Proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP()
  {
    return controller.getP();
  }

  /**
   * Sets the Proportional coefficient of the PID controller gain.
   *
   * @param kP The proportional coefficient. Must be &gt;= 0.
   */
  public void setP(double kP)
  {
    controller.setP(kP);
  }

  /**
   * Get the Integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI()
  {
    return controller.getI();
  }

  /**
   * Sets the Integral coefficient of the PID controller gain.
   *
   * @param kI The integral coefficient. Must be >= 0.
   */
  public void setI(double kI)
  {
    controller.setI(kI);
  }

  /**
   * Get the Differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD()
  {
    return controller.getD();
  }

  /**
   * Sets the Differential coefficient of the PID controller gain.
   *
   * @param kD The differential coefficient. Must be >= 0.
   */
  public void setD(double kD)
  {
    controller.setD(kD);
  }

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput)
  {
    controller.enableContinuousInput(minimumInput, maximumInput);
  }
}
