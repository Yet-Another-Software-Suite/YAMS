package yams.units;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import static edu.wpi.first.units.Units.derive;


import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;

/**
 * YUnits that are fun and useful!
 */
public class YUnits {
  /** 101.6/1000 of a {@link edu.wpi.first.units.Units#Meters}, or 4 {@link edu.wpi.first.units.Units#Inches}. */
  public static final DistanceUnit Hands =
      derive(Inches).aggregate(4).named("Hand").symbol("hand").make();

  /** 101.6/1000 of a {@link edu.wpi.first.units.Units#Meters}, or 4 {@link edu.wpi.first.units.Units#Inches}. */
  public static final DistanceUnit Hand = Hands;

  /** 914.4/1000 of a {@link edu.wpi.first.units.Units#Meters}, or 3 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Yards =
      derive(Feet).aggregate(3).named("Yard").symbol("yd").make();

  /** 914.4/1000 of a {@link edu.wpi.first.units.Units#Meters}, or 3 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Yard = Yards;

  /** 457.2/1000 of a {@link edu.wpi.first.units.Units#Meters}, or 18 {@link edu.wpi.first.units.Units#Inches}. */
  public static final DistanceUnit Cubits =
      derive(Inches).aggregate(18).named("Cubit").symbol("cbt").make();

  /** 457.2/1000 of a {@link edu.wpi.first.units.Units#Meters}, or 18 {@link edu.wpi.first.units.Units#Inches}. */
  public static final DistanceUnit Cubit = Cubits;

  /** 1.8288 {@link edu.wpi.first.units.Units#Meters}s, or 6 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Fathoms =
      derive(Feet).aggregate(6).named("Fathom").symbol("ftm").make();

  /**1 {@link edu.wpi.first.units.Units#Feet}, or <a href="https://myginosdeli.com/ginos-deli-menu/">Sandwich</a> */
  public static final DistanceUnit FootlongSandwich =
    derive(Feet).named("FootlongSandwich").symbol("1ftsub").make();

  /** 1.8288 {@link edu.wpi.first.units.Units#Meters}s, or 6 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Fathom = Fathoms; // alias

  /** 20.1168 (Except in India, where it is 20 meters) {@link edu.wpi.first.units.Units#Meters}s, or 66 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Chains =
      derive(Feet).aggregate(66).named("Chain").symbol("ch").make();

  /** 20.1168 (Except in India, where it is 20 meters) {@link edu.wpi.first.units.Units#Meters}s, or 66 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Chain = Chains; // alias

  /** 201.168 {@link edu.wpi.first.units.Units#Meters}s, or 10 {@link #Chains}. */
  public static final DistanceUnit Furlongs =
      derive(Chain).aggregate(10).named("Furlong").symbol("fur").make();

  /** 201.168 {@link edu.wpi.first.units.Units#Meters}s, or 10 {@link #Chains}. */
  public static final DistanceUnit Furlong = Furlongs; // alias

  /** 1609.344 {@link edu.wpi.first.units.Units#Meters}s, or 5280 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Miles =
      derive(Feet).aggregate(5280).named("Mile").symbol("mi").make();

  /** 1609.344 {@link edu.wpi.first.units.Units#Meters}s, or 5280 {@link edu.wpi.first.units.Units#Feet}. */
  public static final DistanceUnit Mile = Miles; // alias

  /** 4828.032 {@link edu.wpi.first.units.Units#Meters}s, or 3 {@link #Miles}. */
  public static final DistanceUnit Leagues =
      derive(Mile).aggregate(3).named("League").symbol("lea").make();

  /** 4828.032 {@link edu.wpi.first.units.Units#Meters}s, or 3 {@link #Miles}. */
  public static final DistanceUnit League = Leagues; // alias

  /** 60 {@link edu.wpi.first.units.Units#Minutes}. */
  public static final TimeUnit Hours =
      derive(Minutes).aggregate(60).named("Hour").symbol("hr").make();

/** 525600 {@link edu.wpi.first.units.Units#Minutes}. */
  public static final TimeUnit Years = 
      derive(Minutes).aggregate(525600).named("Year").symbol("yr").make();

  /** Alias for {@link #Hours} to make combined unit definitions read more smoothly. */
  public static final TimeUnit Hour = Hours; // singularized alias

  /** Alias for {@link #Years} to make combined unit definitions read more smoothly. */
  public static final TimeUnit Year = Years; // singularized alias

  /** 24 {@link #Hours}. */
  public static final TimeUnit Days = derive(Hours).aggregate(24).named("Days").symbol("days").make();
  /** 7 {@link #Days}. */
  public static final TimeUnit Weeks = derive(Days).aggregate(7).named("Weeks").symbol("weeks").make();
  /** 2 {@link #Weeks}. */
  public static final TimeUnit Fortnight = derive(Weeks).aggregate(2).named("Fortnights").symbol("fortnights").make();

  /** 1 {@link #Furlongs} per {@link #Fortnight}. */
  public static final LinearVelocityUnit FurlongsPerFortnight = Furlongs.per(Fortnight);

  /** 1 {@link #FootlongSandwich} per {@link edu.wpi.first.units.Units#Second} */
  public static final LinearVelocityUnit SandwichPerSecond = FootlongSandwich.per(Second);

  /** 1 {@link #Furlongs} per {@link #Fortnight}. */
  public static final LinearVelocityUnit FPF = FurlongsPerFortnight;
  /** 1 {@link #Miles} per {@link #Hour}. */
  public static final LinearVelocityUnit MilesPerHour = Miles.per(Hour);
  /** 1 {@link #Miles} per {@link #Hour}. */
  public static final LinearVelocityUnit MPH = MilesPerHour;


  /** 1 {@link edu.wpi.first.units.Units#Rotations} per {@link #Year}. */
  public static final AngularVelocityUnit RotationsPerYear = Rotations.per(Year);
  /** 1 {@link edu.wpi.first.units.Units#Rotations} per {@link #Year}. */
  public static final AngularVelocityUnit RPY = RotationsPerYear;

  /** 1 {@link edu.wpi.first.units.Units#RPM} per {@link edu.wpi.first.units.Units#Second} */
  public static final AngularAccelerationUnit RotationsPerMinutePerSecond = RPM.per(Second);
  /** 1 {@link edu.wpi.first.units.Units#RPM} per {@link edu.wpi.first.units.Units#Second} */
  public static final AngularAccelerationUnit RPMPerSecond               = RotationsPerMinutePerSecond;

}
