package yams.math;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;
import java.util.OptionalDouble;
import yams.gearing.MechanismGearing;

/**
 * LQR Configuration for {@link LQRController}
 */
public class LQRConfig
{

  /**
   * Get the {@link LQRType} of the LQR.
   *
   * @return {@link LQRType} of the LQR.
   */
  public LQRType getType()
  {
    return m_type.orElseThrow();
  }

  /**
   * Get the loop time for the LQR.
   *
   * @return {@link Time} for the loop time.
   */
  public Time getPeriod()
  {
    return m_period;
  }

  /**
   * LQR Type
   */
  public enum LQRType
  {
    /**
     * Flywheel LQR
     */
    FLYWHEEL,
    /**
     * Arm LQR
     */
    ARM,
    /**
     * Elevator LQR
     */
    ELEVATOR
  }

  /**
   * LQR Type
   */
  private       Optional<LQRType>            m_type               = Optional.empty();
  /**
   * {@link DCMotor} for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}.
   */
  private final DCMotor                      m_motor;
  /**
   * {@link MechanismGearing} for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}.
   */
  private final MechanismGearing             m_gearing;
  /**
   * {@link MomentOfInertia} for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}.
   */
  private final MomentOfInertia              m_moi;
  /**
   * Loop time for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}. Could be faster if using
   * {@link edu.wpi.first.wpilibj.Notifier}s.
   */
  private       Time                         m_period             = Milliseconds.of(20);
  /**
   * Maximum voltage for the {@link edu.wpi.first.math.system.LinearSystemLoop}. Default is 12v.
   */
  private       Voltage                      m_maxVoltage         = Volts.of(12);
  /**
   * Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the controller
   * less aggressive. 12 is a good starting point because that is the (approximate) maximum voltage of a battery.
   */
  private       Voltage                      m_relms              = Volts.of(12);
  /**
   * Tolerance. Decrease this to more heavily penalize state excursion, or make the controller behave more
   * aggressively.
   */
  private       Optional<Vector<?>>          m_qelms              = Optional.empty();
  /**
   * Standard deviation of the model, size of the vector depends on the model type.
   */
  private       Optional<Vector<?>>          m_modelStdDevs       = Optional.empty();
  /**
   * Standard deviation of the encoder, size of the vector depends on the model type.
   */
  private       Optional<Vector<?>>          m_encoderStdDevs     = Optional.empty();
  /**
   * Elevator mass for {@link LinearSystemId#createElevatorSystem(DCMotor, double, double, double)}
   */
  private       Optional<Mass>               m_elevatorMass       = Optional.empty();
  /**
   * Elevator radius for {@link LinearSystemId#createElevatorSystem(DCMotor, double, double, double)}
   */
  private       Optional<Distance>           m_elevatorDrumRadius = Optional.empty();
  /**
   * Measurement delay.
   */
  private       Optional<Time>               m_measurementDelay   = Optional.empty();
  /**
   * Agressiveness.
   */
  private       OptionalDouble               m_aggressiveness     = OptionalDouble.empty();

  /**
   * Create a new LQR Configuration.
   *
   * @param motor   {@link DCMotor} for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}.
   * @param gearing {@link MechanismGearing} for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}.
   * @param moi     {@link MomentOfInertia} for the {@link edu.wpi.first.math.controller.LinearQuadraticRegulator}.
   */
  public LQRConfig(DCMotor motor, MechanismGearing gearing, MomentOfInertia moi)
  {
    m_motor = motor;
    m_gearing = gearing;
    m_moi = moi;
  }

  /**
   * Set the control effort for the LQR.
   *
   * @param relms Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the
   *              controller less aggressive. 12 is a good starting point because that is the (approximate) maximum
   *              voltage of a battery.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withRelms(Voltage relms)
  {
    m_relms = relms;
    return this;
  }


  /**
   * Set the control effort for the LQR.
   *
   * @param effort Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make
   *               the controller less aggressive. 12 is a good starting point because that is the (approximate) maximum
   *               voltage of a battery.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withControlEffort(Voltage effort)
  {
    return withRelms(effort);
  }

  /**
   * Set the maximum voltage for the {@link edu.wpi.first.math.system.LinearSystemLoop}.
   *
   * @param voltage Maximum voltage output.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withMaxVoltage(Voltage voltage)
  {
    m_maxVoltage = voltage;
    return this;
  }

  /**
   * Set the measurement delay of the LQR.
   *
   * @param delay Measurement delay.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withMeasurementDelay(Time delay)
  {
    m_measurementDelay = Optional.of(delay);
    return this;
  }

  /**
   * Agressiveness of the LQR, howfast it will attempt to achieve the desired state.
   *
   * @param agressiveness Usually 10, arbitrary scale.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withAgressiveness(double agressiveness)
  {
    m_aggressiveness = OptionalDouble.of(agressiveness);
    return this;
  }


  /**
   * Construct a Flywheel LQR Configuration.
   *
   * @param qelms        Velocity error tolerance. Decrease this to more heavily penalize state excursion, or make the
   *                     controller behave more starting point because that is the (approximate) maximum voltage of a
   *                     battery.
   * @param modelTrust   Standard deviation of the model, represented in {@link AngularVelocity}.
   * @param encoderTrust Standard deviation of the encoder, represented in {@link AngularVelocity}.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withFlyWheel(AngularVelocity qelms, AngularVelocity modelTrust, AngularVelocity encoderTrust)
  {
    m_type = Optional.of(LQRType.FLYWHEEL);
    m_qelms = Optional.of(VecBuilder.fill(qelms.in(RadiansPerSecond)));
    m_modelStdDevs = Optional.of(VecBuilder.fill(modelTrust.in(RadiansPerSecond)));
    m_encoderStdDevs = Optional.of(VecBuilder.fill(encoderTrust.in(RadiansPerSecond)));
    return this;
  }

  /**
   * Construct an Elevator LQR Configuration.
   *
   * @param qelmsPosition        Position error tolerance, in meters. Decrease this to more heavily penalize state
   *                             excursion, or make the controller behave more aggressively. This can be tuned to
   *                             balance the position and velocity errors.
   * @param qelmsVelocity        Velocity error tolerance, in meters per second. Decrease this to more heavily penalize
   *                             state excursion, or make the controller behave more aggressively. This can be tuned to
   *                             balance the position and velocity errors.
   * @param modelPositionTrust   Standard deviation of the model position, represented in {@link Distance}.
   * @param modelVelocityTrust   Standard deviation of the model velocity, represented in {@link LinearVelocity}.
   * @param encoderPositionTrust Standard deviation of the encoder position, represented in {@link Distance}.
   * @param mass                 Mass of the elevator, represented in {@link Mass}.
   * @param drumRadius           Radius of the elevator drum, represented in {@link Distance}.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withElevator(Distance qelmsPosition, LinearVelocity qelmsVelocity, Distance modelPositionTrust,
                                LinearVelocity modelVelocityTrust, Distance encoderPositionTrust, Mass mass,
                                Distance drumRadius)
  {
    m_type = Optional.of(LQRType.ELEVATOR);
    m_qelms = Optional.of(VecBuilder.fill(qelmsPosition.in(Meters), qelmsVelocity.in(MetersPerSecond)));
    m_modelStdDevs = Optional.of(VecBuilder.fill(modelPositionTrust.in(Meters),
                                                 modelVelocityTrust.in(MetersPerSecond)));
    m_encoderStdDevs = Optional.of(VecBuilder.fill(encoderPositionTrust.in(Meters)));
    m_elevatorMass = Optional.of(mass);
    m_elevatorDrumRadius = Optional.of(drumRadius);
    return this;
  }

  /**
   * Construct an Arm LQR Configuration.
   *
   * @param qelmsPosition        Position error tolerance, in rotations. Decrease this to more heavily penalize state
   *                             excursion, or make the controller behave more aggressively. This can be tuned to
   *                             balance the position and velocity errors.
   * @param qelmsVelocity        Velocity error tolerance, in rotations per second. Decrease this to more heavily
   *                             penalize state excursion, or make the controller behave more aggressively. This can be
   *                             tuned to balance the position and velocity errors.
   * @param modelPositionTrust   Standard deviation of the model position, represented in {@link Angle}.
   * @param modelVelocityTrust   Standard deviation of the model velocity, represented in {@link AngularVelocity}.
   * @param encoderPositionTrust Standard deviation of the encoder position, represented in {@link Angle}.
   * @return {@link LQRConfig} for chaining.
   */
  public LQRConfig withArm(Angle qelmsPosition, AngularVelocity qelmsVelocity, Angle modelPositionTrust,
                           AngularVelocity modelVelocityTrust, Angle encoderPositionTrust)
  {
    m_type = Optional.of(LQRType.ARM);
    m_qelms = Optional.of(VecBuilder.fill(qelmsPosition.in(Radians), qelmsVelocity.in(RadiansPerSecond)));
    m_modelStdDevs = Optional.of(VecBuilder.fill(modelPositionTrust.in(Radians),
                                                 modelVelocityTrust.in(RadiansPerSecond)));
    m_encoderStdDevs = Optional.of(VecBuilder.fill(encoderPositionTrust.in(Radians)));
    return this;
  }

  /**
   * Get the {@link LinearSystem} for the LQR. with {@link LinearSystemId}
   *
   * @return {@link LinearSystem} for the LQR.
   */
  public LinearSystem<?, ?, ?> getSystem()
  {
    switch (m_type.orElseThrow())
    {
      case FLYWHEEL ->
      {
        return LinearSystemId.createFlywheelSystem(m_motor,
                                                   m_moi.in(KilogramSquareMeters),
                                                   m_gearing.getMechanismToRotorRatio());
      }
      case ARM ->
      {
        return LinearSystemId.createSingleJointedArmSystem(m_motor,
                                                           m_moi.in(KilogramSquareMeters),
                                                           m_gearing.getMechanismToRotorRatio());
      }
      case ELEVATOR ->
      {
        return LinearSystemId.createElevatorSystem(m_motor,
                                                   m_elevatorMass.orElseThrow().in(Kilograms),
                                                   m_elevatorDrumRadius.orElseThrow().in(Meters),
                                                   m_gearing.getMechanismToRotorRatio());
      }
    }
    throw new IllegalStateException("Invalid LQR Type");
  }

  /**
   * Get the {@link KalmanFilter} for the LQR.
   *
   * @param plant {@link LinearSystem} for the LQR, fetched from {@link #getSystem()}.
   * @return {@link KalmanFilter} for the LQR.
   */
  @SuppressWarnings("unchecked")
  public KalmanFilter<?, ?, ?> getKalmanFilter(LinearSystem<?, ?, ?> plant)
  {
    switch (m_type.orElseThrow())
    {
      case FLYWHEEL -> /// 1 modeled state, velocity. Inputs are volts. Outputs are velocity.
      {
        return new KalmanFilter<N1, N1, N1>(Nat.N1(),
                                            Nat.N1(),
                                            (LinearSystem<N1, N1, N1>) plant,
                                            (Vector<N1>) m_modelStdDevs.orElseThrow(),
                                            (Vector<N1>) m_encoderStdDevs.orElseThrow(),
                                            m_period.in(Seconds));
      }
      case ARM, ELEVATOR -> /// 2 modeled states, position and velocity. Inputs are volts. Outputs are position.
      {
        return new KalmanFilter<N2, N1, N1>(Nat.N2(),
                                            Nat.N1(),
                                            (LinearSystem<N2, N1, N1>) plant,
                                            (Vector<N2>) m_modelStdDevs.orElseThrow(),
                                            (Vector<N1>) m_encoderStdDevs.orElseThrow(),
                                            m_period.in(Seconds));
      }
    }
    throw new IllegalStateException("Invalid LQR Type");
  }

  /**
   * Get the {@link LinearQuadraticRegulator}.
   *
   * @param plant {@link LinearSystem} for the LQR, fetched from {@link #getSystem()}.
   * @return {@link LinearQuadraticRegulator} for the LQR.
   */
  @SuppressWarnings("unchecked")
  public LinearQuadraticRegulator<?, ?, ?> getRegulator(LinearSystem<?, ?, ?> plant)
  {
    switch (m_type.orElseThrow())
    {
      case FLYWHEEL -> /// 1 modeled state, velocity. Inputs are volts. Outputs are velocity.
      {
        return new LinearQuadraticRegulator<N1, N1, N1>((LinearSystem<N1, N1, N1>) plant,
                                                        (Vector<N1>) m_qelms.orElseThrow(),
                                                        VecBuilder.fill(m_relms.in(Volts)),
                                                        m_period.in(Seconds));
      }
      case ARM, ELEVATOR -> /// 2 modeled states, position and velocity. Inputs are volts. Outputs are position.
      {
        return new LinearQuadraticRegulator<N2, N1, N1>((LinearSystem<N2, N1, N1>) plant,
                                                        (Vector<N2>) m_qelms.orElseThrow(),
                                                        VecBuilder.fill(m_relms.in(Volts)),
                                                        m_period.in(Seconds));
      }
    }
    throw new IllegalStateException("Invalid LQR Type");
  }

  /**
   * Get the {@link LinearSystemLoop}.
   *
   * @param plant      {@link LinearSystem} for the LQR, fetched from {@link #getSystem()}
   * @param controller {@link LinearQuadraticRegulator} for the LQR, fetched from {@link #getRegulator(LinearSystem)}
   * @param observer   {@link KalmanFilter} for the LQR, fetched from {@link #getKalmanFilter(LinearSystem)}
   * @return {@link LinearSystemLoop} for the LQR.
   */
  @SuppressWarnings("unchecked")
  public LinearSystemLoop<?, ?, ?> getLoop(LinearSystem<?, ?, ?> plant, LinearQuadraticRegulator<?, ?, ?> controller,
                                           KalmanFilter<?, ?, ?> observer)
  {
    switch (m_type.orElseThrow())
    {
      case FLYWHEEL -> /// 1 modeled state, velocity. Inputs are volts. Outputs are velocity.
      {
        return new LinearSystemLoop<N1, N1, N1>((LinearSystem<N1, N1, N1>) plant,
                                                (LinearQuadraticRegulator<N1, N1, N1>) controller,
                                                (KalmanFilter<N1, N1, N1>) observer,
                                                m_maxVoltage.in(Volts),
                                                m_period.in(Seconds));
      }
      case ARM, ELEVATOR -> /// 2 modeled states, position and velocity. Inputs are volts. Outputs are position.
      {
        return new LinearSystemLoop<N2, N1, N1>((LinearSystem<N2, N1, N1>) plant,
                                                (LinearQuadraticRegulator<N2, N1, N1>) controller,
                                                (KalmanFilter<N2, N1, N1>) observer,
                                                m_maxVoltage.in(Volts),
                                                m_period.in(Seconds));
      }
    }
    throw new IllegalStateException("Invalid LQR Type");
  }

  /**
   * Get the {@link LinearSystemLoop} with the currently configured LQR.
   *
   * @return {@link LinearSystemLoop} for the LQR.
   */
  @SuppressWarnings("unchecked")
  public LinearSystemLoop<?, ?, ?> getLoop()
  {
    var plant = getSystem();
    switch (m_type.orElseThrow())
    {
      case FLYWHEEL -> /// 1 modeled state, velocity. Inputs are volts. Outputs are velocity.
      {
        return (LinearSystemLoop<N1, N1, N1>) getLoop(plant,
                                                      (LinearQuadraticRegulator<N1, N1, N1>) getRegulator(plant),
                                                      (KalmanFilter<N1, N1, N1>) getKalmanFilter(plant));
      }
      case ARM, ELEVATOR -> /// 2 modeled states, position and velocity. Inputs are volts. Outputs are position.
      {
        return (LinearSystemLoop<N2, N1, N1>) getLoop(plant,
                                                      (LinearQuadraticRegulator<N2, N1, N1>) getRegulator(plant),
                                                      (KalmanFilter<N2, N1, N1>) getKalmanFilter(plant));

      }
    }
    throw new IllegalStateException("Invalid LQR Type");
  }

}
