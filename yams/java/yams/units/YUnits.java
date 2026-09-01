// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.units;

import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.InchesPerSecond;
import static org.wpilib.units.Units.Minutes;
import static org.wpilib.units.Units.Pound;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.derive;

import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularMomentumUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearMomentumUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.MomentOfInertiaUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * YUnits that are fun and useful!
 *
 * <p>This class extends the standard WPILib unit set with additional units that are either useful
 * for FRC robotics or simply entertaining. Beyond what WPILib provides out of the box, {@code YUnits}
 * adds:
 *
 * <ul>
 *   <li><b>Imperial distance units</b> — {@link #Hands}, {@link #Yards}, {@link #Cubits},
 *       {@link #Fathoms}, {@link #Chains}, {@link #Furlongs}, {@link #Miles}, {@link #Leagues},
 *       and even a {@link #FootlongSandwich}.</li>
 *   <li><b>Extended time units</b> — {@link #Hours}, {@link #Days}, {@link #Weeks},
 *       {@link #Fortnight}, and {@link #Years}.</li>
 *   <li><b>Derived velocity units</b> — {@link #MilesPerHour} (MPH), {@link #FurlongsPerFortnight}
 *       (FPF), and {@link #SandwichPerSecond} for all your footlong-based kinematics.</li>
 *   <li><b>Angular velocity and acceleration</b> — {@link #RotationsPerYear} (RPY) and
 *       {@link #RPMPerSecond} for angular acceleration in RPM/s.</li>
 *   <li><b>Linear and angular momentum units</b> — {@link #PoundFeetPerSecond},
 *       {@link #PoundInchesPerSecond}, {@link #PoundFeetSquaredPerSecond}, and
 *       {@link #PoundInchesSquaredPerSecond}.</li>
 *   <li><b>Moment of inertia units</b> — {@link #PoundSquareFeet} and {@link #PoundSquareInches}
 *       for imperial rotational inertia calculations.</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * import yams.units.YUnits;
 * import static org.wpilib.units.Units.Meters;
 * import static org.wpilib.units.Units.MetersPerSecond;
 *
 * // Convert a robot speed from MPH to meters per second
 * double speedMps = YUnits.MPH.of(10).in(MetersPerSecond);  // ~4.47 m/s
 *
 * // Express angular acceleration in RPM per second
 * var accel = YUnits.RPMPerSecond.of(50);  // 50 RPM/s
 *
 * // Use PoundSquareInches for a flywheel moment of inertia
 * var moi = YUnits.PoundSquareInches.of(0.004);
 * }</pre>
 */
public class YUnits
{
  // Angular Momentum Units
  /**
   * Pounds * Feet/Second
   */
  public static final LinearMomentumUnit PoundFeetPerSecond = Pound.mult(FeetPerSecond);
  /**
   * Pounds * Inches/Second
   */
  public static final LinearMomentumUnit PoundInchesPerSecond = Pound.mult(InchesPerSecond);
  // Linear Momentum Units
  /**
   * Pounds * Feet^2/Second
   */
  public static final AngularMomentumUnit PoundFeetSquaredPerSecond = PoundFeetPerSecond.mult(Feet);
  /**
   * Pounds * Inches^2/Second
   */
  public static final AngularMomentumUnit PoundInchesSquaredPerSecond = PoundInchesPerSecond.mult(Inches);
  // Moment of Inertia Units
  /**
   * Pounds^2 * Inches * Radians/Second
   */
  public static final MomentOfInertiaUnit PoundSquareFeet = MomentOfInertiaUnit.combine(PoundFeetSquaredPerSecond,RadiansPerSecond);
  /**
   * Pounds^2 * Inches *Radians/Second
   */
  public static final MomentOfInertiaUnit PoundSquareInches = MomentOfInertiaUnit.combine(PoundInchesSquaredPerSecond,RadiansPerSecond);

  /**
   * 101.6/1000 of a {@link org.wpilib.units.Units#Meters}, or 4 {@link org.wpilib.units.Units#Inches}.
   */
  public static final DistanceUnit Hands =
      derive(Inches).aggregate(4).named("Hand").symbol("hand").make();

  /**
   * 101.6/1000 of a {@link org.wpilib.units.Units#Meters}, or 4 {@link org.wpilib.units.Units#Inches}.
   */
  public static final DistanceUnit Hand = Hands;

  /**
   * 914.4/1000 of a {@link org.wpilib.units.Units#Meters}, or 3 {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Yards =
      derive(Feet).aggregate(3).named("Yard").symbol("yd").make();

  /**
   * 914.4/1000 of a {@link org.wpilib.units.Units#Meters}, or 3 {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Yard = Yards;

  /**
   * 457.2/1000 of a {@link org.wpilib.units.Units#Meters}, or 18 {@link org.wpilib.units.Units#Inches}.
   */
  public static final DistanceUnit Cubits =
      derive(Inches).aggregate(18).named("Cubit").symbol("cbt").make();

  /**
   * 457.2/1000 of a {@link org.wpilib.units.Units#Meters}, or 18 {@link org.wpilib.units.Units#Inches}.
   */
  public static final DistanceUnit Cubit = Cubits;

  /**
   * 1.8288 {@link org.wpilib.units.Units#Meters}s, or 6 {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Fathoms =
      derive(Feet).aggregate(6).named("Fathom").symbol("ftm").make();

  /**
   * 1 {@link org.wpilib.units.Units#Feet}, or <a href="https://myginosdeli.com/ginos-deli-menu/">Sandwich</a>
   */
  public static final DistanceUnit FootlongSandwich =
      derive(Feet).named("FootlongSandwich").symbol("1ftsub").make();

  /**
   * 1.8288 {@link org.wpilib.units.Units#Meters}s, or 6 {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Fathom = Fathoms; // alias

  /**
   * 20.1168 (Except in India, where it is 20 meters) {@link org.wpilib.units.Units#Meters}s, or 66
   * {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Chains =
      derive(Feet).aggregate(66).named("Chain").symbol("ch").make();

  /**
   * 20.1168 (Except in India, where it is 20 meters) {@link org.wpilib.units.Units#Meters}s, or 66
   * {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Chain = Chains; // alias

  /**
   * 201.168 {@link org.wpilib.units.Units#Meters}s, or 10 {@link #Chains}.
   */
  public static final DistanceUnit Furlongs =
      derive(Chain).aggregate(10).named("Furlong").symbol("fur").make();

  /**
   * 201.168 {@link org.wpilib.units.Units#Meters}s, or 10 {@link #Chains}.
   */
  public static final DistanceUnit Furlong = Furlongs; // alias

  /**
   * 1609.344 {@link org.wpilib.units.Units#Meters}s, or 5280 {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Miles =
      derive(Feet).aggregate(5280).named("Mile").symbol("mi").make();

  /**
   * 1609.344 {@link org.wpilib.units.Units#Meters}s, or 5280 {@link org.wpilib.units.Units#Feet}.
   */
  public static final DistanceUnit Mile = Miles; // alias

  /**
   * 4828.032 {@link org.wpilib.units.Units#Meters}s, or 3 {@link #Miles}.
   */
  public static final DistanceUnit Leagues =
      derive(Mile).aggregate(3).named("League").symbol("lea").make();

  /**
   * 4828.032 {@link org.wpilib.units.Units#Meters}s, or 3 {@link #Miles}.
   */
  public static final DistanceUnit League = Leagues; // alias

  /**
   * 60 {@link org.wpilib.units.Units#Minutes}.
   */
  public static final TimeUnit Hours =
      derive(Minutes).aggregate(60).named("Hour").symbol("hr").make();

  /**
   * 525600 {@link org.wpilib.units.Units#Minutes}.
   */
  public static final TimeUnit Years =
      derive(Minutes).aggregate(525600).named("Year").symbol("yr").make();

  /**
   * Alias for {@link #Hours} to make combined unit definitions read more smoothly.
   */
  public static final TimeUnit Hour = Hours; // singularized alias
  /**
   * 1 {@link #Miles} per {@link #Hour}.
   */
  public static final LinearVelocityUnit MilesPerHour = Miles.per(Hour);
  /**
   * 1 {@link #Miles} per {@link #Hour}.
   */
  public static final LinearVelocityUnit MPH = MilesPerHour;
  /**
   * Alias for {@link #Years} to make combined unit definitions read more smoothly.
   */
  public static final TimeUnit Year = Years; // singularized alias
  /**
   * 1 {@link org.wpilib.units.Units#Rotations} per {@link #Year}.
   */
  public static final AngularVelocityUnit RotationsPerYear = Rotations.per(Year);
  /**
   * 1 {@link org.wpilib.units.Units#Rotations} per {@link #Year}.
   */
  public static final AngularVelocityUnit RPY = RotationsPerYear;
  /**
   * 24 {@link #Hours}.
   */
  public static final TimeUnit Days = derive(Hours).aggregate(24).named("Days").symbol("days").make();
  /**
   * 7 {@link #Days}.
   */
  public static final TimeUnit Weeks = derive(Days).aggregate(7).named("Weeks").symbol("weeks").make();
  /**
   * 2 {@link #Weeks}.
   */
  public static final TimeUnit Fortnight = derive(Weeks).aggregate(2).named("Fortnights").symbol("fortnights").make();
  /**
   * 1 {@link #Furlongs} per {@link #Fortnight}.
   */
  public static final LinearVelocityUnit FurlongsPerFortnight = Furlongs.per(Fortnight);
  /**
   * 1 {@link #Furlongs} per {@link #Fortnight}.
   */
  public static final LinearVelocityUnit FPF = FurlongsPerFortnight;
  /**
   * 1 {@link #FootlongSandwich} per {@link org.wpilib.units.Units#Second}
   */
  public static final LinearVelocityUnit SandwichPerSecond = FootlongSandwich.per(Second);
  /**
   * 1 {@link org.wpilib.units.Units#RPM} per {@link org.wpilib.units.Units#Second}
   */
  public static final AngularAccelerationUnit RotationsPerMinutePerSecond = RPM.per(Second);
  /**
   * 1 {@link org.wpilib.units.Units#RPM} per {@link org.wpilib.units.Units#Second}
   */
  public static final AngularAccelerationUnit RPMPerSecond = RotationsPerMinutePerSecond;

}
