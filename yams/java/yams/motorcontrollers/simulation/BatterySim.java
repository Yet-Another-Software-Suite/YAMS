// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MilliOhms;
import static edu.wpi.first.units.Units.Ohms;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.UUID;

/**
 * Battery Simulation that takes the entirety of the robot into account by storing all currently
 * used currents on the robot.
 */
public class BatterySim {
  /**
   * Hashmap holding all currents used by the robot.
   */
  private static HashMap<UUID, Double> currents = new HashMap<>();
  /**
   * Battery voltage.
   */
  private static Voltage batteryVoltage = Volts.of(12);
  /**
   * Battery resistance.
   */
  private static Resistance batteryResistance = MilliOhms.of(20);
  /**
   * Open circuit voltage of the battery as a function of state of charge (0 to 1), based on a
   * typical FRC sealed lead-acid battery discharge curve. Voltage stays relatively flat for most
   * of the discharge before sagging quickly near depletion.
   */
  private static InterpolatingDoubleTreeMap SOC_TO_VOLTAGE = new InterpolatingDoubleTreeMap();

  static {
    SOC_TO_VOLTAGE.put(0.00, 9.0);
    SOC_TO_VOLTAGE.put(0.05, 10.5);
    SOC_TO_VOLTAGE.put(0.10, 11.5);
    SOC_TO_VOLTAGE.put(0.20, 12.0);
    SOC_TO_VOLTAGE.put(0.40, 12.3);
    SOC_TO_VOLTAGE.put(0.60, 12.5);
    SOC_TO_VOLTAGE.put(0.80, 12.7);
    SOC_TO_VOLTAGE.put(0.90, 12.8);
    SOC_TO_VOLTAGE.put(1.00, 12.9);
  }

  /**
   * Whether discharge simulation is enabled.
   */
  private static boolean dischargeEnabled = false;
  /**
   * Capacity of the battery in amp-hours used for discharge simulation.
   */
  private static double batteryCapacityAmpHours = 18.0;
  /**
   * Amount of charge consumed from the battery so far, in amp-hours.
   */
  private static double ampHoursUsed = 0.0;
  /**
   * Timestamp of the last discharge integration step, in seconds. {@link Double#NaN} indicates no
   * previous step has been taken yet.
   */
  private static double lastTimestampSeconds = Double.NaN;

  /**
   * Replace the default state-of-charge &rarr; open circuit voltage interpolation table used when
   * discharge simulation is enabled with {@link #enableDischarge(double, Voltage, Resistance)}.
   *
   * <p>
   * Not every battery discharges like YAMS's built-in sealed lead-acid curve. Reach for this
   * method when you want to model something different, for example:
   * </p>
   * <ul>
   *   <li>A well-used competition battery that sags earlier and harder than a fresh one.</li>
   *   <li>Matching a curve you measured from an actual battery on a load tester, for the most
   *       accurate brownout predictions possible.</li>
   * </ul>
   *
   * <pre>{@code
   * // Model a well-used competition battery that sags earlier and more severely than a new one.
   * InterpolatingDoubleTreeMap wornBatteryCurve = new InterpolatingDoubleTreeMap();
   * wornBatteryCurve.put(0.00, 8.0);
   * wornBatteryCurve.put(0.05, 9.5);
   * wornBatteryCurve.put(0.10, 10.5);
   * wornBatteryCurve.put(0.20, 11.2);
   * wornBatteryCurve.put(0.40, 11.6);
   * wornBatteryCurve.put(0.60, 11.9);
   * wornBatteryCurve.put(0.80, 12.2);
   * wornBatteryCurve.put(0.90, 12.4);
   * wornBatteryCurve.put(1.00, 12.6);
   *
   * BatterySim.replaceSOCInterpolation(wornBatteryCurve);
   * // Pair with a reduced usable capacity and higher resistance to match a worn battery.
   * BatterySim.enableDischarge(15.0, Volts.of(12.6), Milliohms.of(28));
   * }</pre>
   *
   * @param socToVoltage Interpolation table mapping state of charge {@code [0, 1]} to open
   *                     circuit voltage. Call this before {@link #enableDischarge(double,
   *                     Voltage, Resistance)} so discharge simulation uses the new curve from the
   *                     start.
   */
  public static void replaceSOCInterpolation(InterpolatingDoubleTreeMap socToVoltage) {
    BatterySim.SOC_TO_VOLTAGE = socToVoltage;
  }

  /**
   * Enable realistic battery discharge simulation. As current is drawn from the battery over time
   * its state of charge will drop, reducing the open circuit voltage and increasing the internal
   * resistance to more realistically model a depleted battery.
   *
   * @param batteryCapacityAmpHours Capacity of the battery in amp-hours (Ah). A typical FRC
   *                                battery is around 18 Ah.
   * @param nomVoltage              Nominal (fully charged) open circuit voltage of the battery.
   * @param nomResistance           Nominal internal resistance of the battery.
   */
  public static void enableDischarge(
      double batteryCapacityAmpHours, Voltage nomVoltage, Resistance nomResistance) {
    dischargeEnabled = true;
    BatterySim.batteryCapacityAmpHours = batteryCapacityAmpHours;
    BatterySim.batteryResistance = nomResistance;
    BatterySim.batteryVoltage = nomVoltage;
  }

  /**
   * Disable battery discharge simulation, reverting to a constant {@link #batteryVoltage} and
   * {@link #batteryResistance}.
   */
  public static void disableDischarge() {
    dischargeEnabled = false;
  }

  /**
   * Reset the simulated battery back to a full charge.
   */
  public static void resetDischarge() {
    ampHoursUsed = 0.0;
    lastTimestampSeconds = Double.NaN;
  }

  /**
   * Get the simulated state of charge of the battery, from 0 (empty) to 1 (full).
   *
   * @return State of charge of the battery.
   */
  public static double getStateOfCharge() {
    return MathUtil.clamp(1.0 - (ampHoursUsed / batteryCapacityAmpHours), 0.0, 1.0);
  }

  /**
   * Integrate the total current draw of the robot over the elapsed time since the last call to
   * track amp-hours consumed from the battery.
   *
   * @param totalCurrentAmps Total current drawn by the robot in {@link
   *                         edu.wpi.first.units.Units#Amps Amps}.
   */
  private static void updateDischarge(double totalCurrentAmps) {
    double now = Timer.getFPGATimestamp();
    if (!Double.isNaN(lastTimestampSeconds)) {
      double dtHours = (now - lastTimestampSeconds) / 3600.0;
      if (dtHours > 0) {
        ampHoursUsed += totalCurrentAmps * dtHours;
        ampHoursUsed = MathUtil.clamp(ampHoursUsed, 0.0, batteryCapacityAmpHours);
      }
    }
    lastTimestampSeconds = now;
  }

  /**
   * Calculate the voltage based on the currents used by the robot.
   * @param id {@link UUID} of the simulation to calculate the voltage for.
   * @param current {@link edu.wpi.first.units.Units#Amps Amps} used by the robot.
   * @return Voltage of the robot.
   */
  public static double calculateVoltage(UUID id, double current) {
    currents.put(id, current);
    double[] currentDraws = currents.values().stream().mapToDouble(Double::doubleValue).toArray();

    double openCircuitVoltage = batteryVoltage.in(Volts);
    double internalResistance = batteryResistance.in(Ohms);
    if (dischargeEnabled) {
      double totalCurrentAmps = 0;
      for (double draw : currentDraws) {
        totalCurrentAmps += draw;
      }
      updateDischarge(totalCurrentAmps);

      double stateOfCharge = getStateOfCharge();
      openCircuitVoltage = SOC_TO_VOLTAGE.get(stateOfCharge);
      // Internal resistance rises as the battery depletes, exaggerating voltage sag under load.
      internalResistance *= 1.0 + (2.0 * (1.0 - stateOfCharge));
    }

    return edu.wpi.first.wpilibj.simulation.BatterySim.calculateLoadedBatteryVoltage(
        openCircuitVoltage, internalResistance, currentDraws);
  }

  /**
   * Calculate the voltage based on the currents used by the robot.
   * @param id {@link UUID} of the simulation to calculate the voltage for.
   * @param current {@link edu.wpi.first.units.Units#Amps Amps} used by the robot.
   * @return Voltage of the robot.
   */
  public static double calculateVoltage(UUID id, Current current) {
    return calculateVoltage(id, current.in(Amps));
  }
}
