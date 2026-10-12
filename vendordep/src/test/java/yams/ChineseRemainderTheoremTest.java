// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import org.junit.jupiter.api.Test;
import edu.wpi.first.units.measure.Angle;
import yams.units.EasyCRT;

public class ChineseRemainderTheoremTest {
  private static final int SHARED_TEETH = 100;
  private static final int A_TEETH = 19;
  private static final int B_TEETH = 21;

  /** The reading an ideal absolute encoder gives, wrapped into one rotation. */
  private static Angle reading(double gearRotations, int pinionTeeth) {
    double encoderRotations = gearRotations * SHARED_TEETH / pinionTeeth;
    return Rotations.of(encoderRotations - Math.floor(encoderRotations));
  }

  @Test
  void uniqueRangeIsLcmOverSharedGear() {
    assertEquals(3.99, new EasyCRT(100, 19, 21).getUniqueRange().in(Rotations), 1e-9);
    // 20 and 30 share a factor of 10, so they only cover lcm(20, 30) = 60 teeth.
    assertEquals(0.6, new EasyCRT(100, 20, 30).getUniqueRange().in(Rotations), 1e-9);
  }

  @Test
  void solvesAcrossTheCenteredRange() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    double half = crt.getUniqueRange().in(Rotations) / 2.0;
    for (double gear = -half + 0.01; gear < half - 0.01; gear += 0.0137) {
      Optional<Angle> solved = crt.solve(reading(gear, A_TEETH), reading(gear, B_TEETH));
      assertTrue(solved.isPresent(), "No solution at " + gear);
      assertEquals(gear, solved.get().in(Rotations), 1e-6);
    }
  }

  @Test
  void solvesAcrossAnOffCenterRange() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    for (double gear = 0.0; gear <= 3.5; gear += 0.0137) {
      Optional<Angle> solved = crt.solve(reading(gear, A_TEETH), reading(gear, B_TEETH), Rotations.of(0), Rotations.of(3.5));
      assertTrue(solved.isPresent(), "No solution at " + gear);
      assertEquals(gear, solved.get().in(Rotations), 1e-6);
    }
  }

  @Test
  void acceptsUnwrappedAndNegativeReadings() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    double gear = -1.3;
    Angle a = Rotations.of(gear * SHARED_TEETH / A_TEETH);
    Angle b = Rotations.of(gear * SHARED_TEETH / B_TEETH);
    assertEquals(gear, crt.solve(a, b).orElseThrow().in(Rotations), 1e-6);
  }

  @Test
  void toleratesBacklashOnEitherEncoder() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    Angle slop = Degrees.of(2);
    for (double gear = -1.9; gear < 1.9; gear += 0.0137) {
      Optional<Angle> solved = crt.solve(reading(gear, A_TEETH).plus(slop), reading(gear, B_TEETH).minus(slop));
      assertTrue(solved.isPresent(), "No solution at " + gear);
      // The result follows encoder A, so its error is the slop seen through A's gearing.
      assertEquals(gear, solved.get().in(Rotations), slop.in(Rotations) * A_TEETH / SHARED_TEETH + 1e-6);
    }
  }

  @Test
  void emptyWhenTheEncodersDisagree() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    // Halfway between two candidates: 1 / (2 * 21) of a rotation off on encoder B.
    Angle wrong = reading(0.5, B_TEETH).plus(Rotations.of(0.5 / B_TEETH));
    assertTrue(crt.solve(reading(0.5, A_TEETH), wrong).isEmpty());
  }

  @Test
  void emptyForReadingsThatAreNotNumbers() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    assertTrue(crt.solve(Rotations.of(Double.NaN), Rotations.of(0.1)).isEmpty());
  }

  @Test
  void rejectsTravelLongerThanTheUniqueRange() {
    EasyCRT crt = new EasyCRT(SHARED_TEETH, A_TEETH, B_TEETH);
    assertThrows(IllegalArgumentException.class, () -> crt.solve(Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(4.0)));
    assertThrows(IllegalArgumentException.class, () -> crt.solve(Rotations.of(0), Rotations.of(0), Rotations.of(1), Rotations.of(0)));
  }

  @Test
  void rejectsToothCountsThatAreNotPositive() {
    assertThrows(IllegalArgumentException.class, () -> new EasyCRT(0, 19, 21));
    assertThrows(IllegalArgumentException.class, () -> new EasyCRT(100, -19, 21));
    assertThrows(IllegalArgumentException.class, () -> new EasyCRT(100, 19, 0));
  }
}
