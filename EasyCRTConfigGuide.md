# EasyCRT Absolute Encoder Configuration Guide

This guide shows how to configure the EasyCRT solver (`EasyCRT` + `EasyCRTConfig`)

## How the solver works
- You supply two absolute enocder angles (wrapping to [0,1) is taken care of by the config) plus their rotations-per-mechanism-rotation ratios.
- The solver enumerates every mechanism angle that fits encoder 1 within the allowed mechanism range, predicts what encoder 2 should read, and measures modular error.
- The best match inside your tolerance becomes the mechanism angle; near ties become `AMBIGUOUS`; no in-range match becomes `NO_SOLUTION`.
- The iteration count stays reasonable. Typical solves are tens of iterations. Log `getLastIterations()` to view this. The gear recommender also filters gear pairs whose theoretical iterations exceed your limit.

## Required inputs
- Provide two `Supplier<Angle>` values (encoder 1 and encoder 2). If a supplier returns `null`, it is treated as `NaN` and the solve will fail.
- All other settings are configured via `EasyCRTConfig` before passing it to `EasyCRT`.

## Ratio / gearing setup
Pick ONE path per encoder to define rotations per mechanism rotation.

- **Direct ratio**: `withEncoderRatios(enc1RotPerMechRot, enc2RotPerMechRot)` when you already computed the ratios. Example:
  ```java
  easyCrt.withEncoderRatios(/* enc1 */ 50.0, /* enc2 */ 18.3333);
  ```
- **Shared drive gear**: `withCommonDriveGear(commonRatio, driveGearTeeth, encoder1PinionTeeth, encoder2PinionTeeth)`
  - `commonRatio` = mechanism rotations : shared drive gear rotations (the gear that meshes with both encoder pinions).
  - Seeds prime tooth counts, the common scale `k`, and stage 1 gear teeth + stage 2 ratio for gear recommendations.
  - Example: turret gearbox is 12:50 -> 10:110; encoders use 30T and 31T pinions driven by the 50t gear. `commonRatio` is 110.0 / 10.0 = 11.0; shared drive gear is 50T:
    ```java
    easyCrt.withCommonDriveGear(/* commonRatio */ 11.0,
                            /* driveGearTeeth */ 50,
                            /* encoder1Pinion */ 30,
                            /* encoder2Pinion */ 31);
    ```
- **Gear chain (simple mesh sequence)**: `withAbsoluteEncoder1Gearing(teeth...)` / `withAbsoluteEncoder2Gearing(teeth...)`
  - Teeth listed from mechanism-side gear to encoder pinion. Example:
    ```java
    easyCrt.withAbsoluteEncoder1Gearing(50, 20, 40); // 50 drives 20, 20 drives 40
    easyCrt.withAbsoluteEncoder2Gearing(60, 20);     // 60 drives 20
    ```
- **Explicit stages (driver, driven pairs)**: `withAbsoluteEncoder1GearingStages(driver1, driven1, driver2, driven2, ...)` / `withAbsoluteEncoder2GearingStages(...)`
  - Use for compound or same-shaft trains. Example:
    ```java
    easyCrt.withAbsoluteEncoder1GearingStages(12, 36, 18, 60); // 12->36, then 18->60
    easyCrt.withAbsoluteEncoder2GearingStages(12, 60);         // single stage 12->60
    ```
- **Inversion**: Configure both in one place (preferred). We reccomend that offsets be applied in the config to avoid confusion, but if your vendor's encoder supports on-device offsets, that is fine, however DO NOT set them in both places:
  ```java
  easyCrt.withAbsoluteEncoderInversions(/* enc1 inverted */ false,
                                    /* enc2 inverted */ true);
  ```
  Leave device-side inversion at defaults; set inversion here as the single source of truth. The CTRE CANCoder configurator in YAMS resets device inversion to a known baseline, so you will not carry stale on-device inversion.

Sanity checks:
- Ratios must be finite and non-zero; tooth counts must be positive.
- If neither direct ratios nor gearing are provided, the configuration throws an error.
- When using chains or stages, verify the order (mechanism -> ... -> encoder). Reversed order yields the wrong ratio.

## Offsets and limits
- **Offsets before wrap**: `withAbsoluteEncoderOffsets(enc1Offset, enc2Offset)` (in rotations) shifts raw readings prior to wrapping into `[0, 1)`. Set offsets here after mechanical zeroing so both encoders read as close to 0.0 rotations as possible at your reference pose. Keep device-side offsets at defaults; the CTRE CANCoder configurator resets device offsets, so the config holds the offsets. Similar to inversion, set this where you want but DO NOT do it twice.
- **Motion limits**: `withMechanismRange(minAngle, maxAngle)` (rotations). The solver enumerates only within this window; narrower windows shrink the candidate set.
- **Match tolerance**: `withMatchTolerance(tolerance)` (rotations of encoder 2). Start near the expected backlash mapped through encoder 2’s ratio; too small -> `NO_SOLUTION`, too large -> risk of `AMBIGUOUS`. If you always preload the turret (or mechanism) to one side of its backlash before solving, effective backlash drops and you can run a smaller tolerance. Monitor `getLastErrorRotations()` while tuning.

## Coverage (unique range) and primes
- When you provide prime tooth counts and a common scale, `getUniqueCoverage()` returns the mechanism rotations you can uniquely represent.
- Provided automatically when you use `withCommonDriveGear` (it seeds `encoder1PrimeTeeth`, `encoder2PrimeTeeth`, and `commonScaleK = commonRatio * driveGearTeeth`).
- Formula: `coverageRot = lcm(encoder1PrimeTeeth, encoder2PrimeTeeth) / commonScaleK`.
- `coverageSatisfiesRange()` checks whether the computed coverage is greater than or equal to your configured `maxMechanismAngle`. Use this as a guardrail when choosing pinion teeth.
- Tip: pick pinion teeth that are coprime to maximize coverage; otherwise `lcm` shrinks and you may not cover your travel.

## Gear recommender workflow
Goal: find the smallest coprime-ish gear pair that yields enough unique coverage with acceptable solve iterations.
- Prereqs (often auto-set by `withCommonDriveGear`):
  - Stage 1 gear teeth (the shared drive gear for both encoders).
  - Stage 2 ratio (mechanism : drive gear).
- If you do NOT use `withCommonDriveGear`, you can still seed the recommender manually with `withCrtGearRecommendationInputs(stage1GearTeeth, stage2Ratio)` before calling `withCrtGearRecommendationConstraints(...)`.
- Configure constraints: `withCrtGearRecommendationConstraints(coverageMargin, minTeeth, maxTeeth, maxIterations)` and (if needed) `withCrtGearRecommendationInputs(stage1GearTeeth, stage2Ratio)`.
  - Runs only in simulation to avoid extra CPU on-robot.
  - `coverageMargin` multiplies your `maxMechanismAngle` to enforce slack (for example, 1.2 adds 20 percent headroom).
  - `maxIterations` rejects pairs that would require too many candidate checks per solve (the real count is visible via `getLastIterations()`).
- Call `getRecommendedCrtGearPair()` in sim; it returns `Optional<CrtGearPair>` with:
  - `gearA`, `gearB`: tooth counts.
  - `coverage`: resulting unique mechanism coverage (rotations).
  - `lcm`, `gcd`, and `theoreticalIterations`.
- Use the returned pinions as `encoder1PinionTeeth` / `encoder2PinionTeeth` in `withCommonDriveGear` (or your own chain/stage builders) and re-check `coverageSatisfiesRange()`.

## Practical gearing guidance
- Ratio equals the total reduction to the mechanism. Ensure backlash mapped to encoder 2 does not exceed your tolerance.
- Chain or belt stages: include every stage when you compute ratios or describe the chain; missing a stage collapses your coverage and can cause ambiguity.
- Prime tooth choice: pick small, coprime pinions (for example, 19T + 21T) for high coverage with minimal size. Avoid sharing factors (for example, 20T + 30T) unless coverage math still clears your travel.

## Calibration and bring-up
1) Mechanically zero the mechanism; log both absolute readings (raw rotations in [0, 1)).
2) Set offsets so that the zero pose reads the desired mechanism zero after wrap (set them in the config).
3) Set motion range to the actual usable travel (include soft stops if you have them).
4) Compute ratios via one of the gearing helpers; confirm `coverageSatisfiesRange()` if using prime-aware inputs.
5) Choose an initial `matchTolerance` based on backlash mapped to encoder 2: `backlash_mech_rot * ratio2`. If you preload to one side, you can shrink this.
6) Jog through the full travel; watch `getLastStatus()` and `getLastErrorRotations()`; ensure no `AMBIGUOUS` near boundary regions.
7) Power-cycle in multiple poses and confirm reconstructed angles stay consistent.

## Seeding a motor controller with EasyCRT
Use the solved mechanism angle to initialize your motor controller's encoder so it knows its absolute position after boot:
```java
var easyCrtConfig = ...; // build EasyCRTConfig as shown above
var easyCrtSolver = new EasyCRT(easyCrtConfig);
//If the mechanism is configured using YAMS, use setEncoderPosition
easyCrtSolver.getAngleOptional().ifPresent(mechAngle -> {
  motor.setEncoderPosition(mechAngle);
});
```
Call this once at startup (after sensors are ready) before running closed-loop control.

## Troubleshooting: Common pitfalls and how to avoid them
- Wrong ratio sign: mechanism turns positive but angle decreases -> set encoder inversion
- Missing a gear stage: forgetting chain or belt stages yields too-small ratios; candidates overlap -> `AMBIGUOUS`.
- Non-coprime pinions: shared factors shrink coverage; `coverageSatisfiesRange()` becomes false. Pick coprime counts or widen the coverage margin.
- Zeroing at a wrap edge: offsets that place the parked pose near 0 or 1 can cause wrap flips from noise. If your zero offset lands within a few hundredths of 0 or 1 rotations, physically re-phase the encoder (pull the pinion or magnet, rotate a tooth or two, and remesh) so your zero sits closer to mid-window, then reset offsets in the config.
- Tolerance too tight: backlash or magnet noise bigger than tolerance -> `NO_SOLUTION`. Increase tolerance or preload the mechanism to one side of backlash.
- Tolerance too loose: two candidates inside tolerance -> `AMBIGUOUS`. Reduce tolerance or widen range margins so only one candidate survives.
- Double inversion or offset: applying inversion or offset both on device and in config produces mirrored or shifted angles. Keep device at defaults; do inversion and offsets in the config. The CTRE CANCoder configurator resets device settings, preventing stale on-device values.
- Skipping power-cycle tests: CRT must reconstruct after reboot; always validate cold-start behavior.

## Example configuration (encoders driven through the mechanism reduction)
```java
// Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T and 23T pinions.
var easyCrt =
    new EasyCRTConfig(enc1Supplier, enc2Supplier)
        .withCommonDriveGear(
            /* commonRatio (mech:drive) */ 12.0,
            /* driveGearTeeth */ 50,
            /* encoder1Pinion */ 19,
            /* encoder2Pinion */ 23)
        .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
        .withMechanismRange(Rotations.of(-1.0), Rotations.of(2.0)) // -360 deg to +720 deg
        .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
        .withAbsoluteEncoderInversions(false, false)
        .withCrtGearRecommendationConstraints(
            /* coverageMargin */ 1.2,
            /* minTeeth */ 15,
            /* maxTeeth */ 45,
            /* maxIterations */ 30);

// you can inspect:
easyCrt.getUniqueCoverage();          // Optional<Angle> coverage from prime counts and common scale
easyCrt.coverageSatisfiesRange();     // Does coverage exceed maxMechanismAngle?
easyCrt.getRecommendedCrtGearPair();  // Suggested pair within constraints

// Create the solver:
var easyCrtSolver = new EasyCRT(easyCrt);
```

## More gearing examples
- **Encoders using explicit stage definitions** (no CRT inputs set by gearing; seed the recommender manually):
  ```java
  var easyCrt =
      new EasyCRTConfig(enc1, enc2)
          .withAbsoluteEncoder1GearingStages(14, 50, 16, 60) // compound to encoder 1
          .withAbsoluteEncoder2GearingStages(12, 36, 18, 54) // compound to encoder 2
          .withMechanismRange(Rotations.of(0.0), Rotations.of(2.0))
          .withAbsoluteEncoderOffsets(Rotations.of(0.01), Rotations.of(0.0)) // align to hard stop
          .withAbsoluteEncoderInversions(false, true)
          // Seed recommender since common drive inputs were not provided above
          .withCrtGearRecommendationInputs(
              /* stage1GearTeeth (shared driver) */ 48,
              /* stage2Ratio (mech:drive) */ 10.0)
          .withCrtGearRecommendationConstraints(
              /* coverageMargin */ 1.15,
              /* minTeeth */ 15,
              /* maxTeeth */ 55,
              /* maxIterations */ 30);

  easyCrt.getRecommendedCrtGearPair();
  ```

- **Encoders using chain helpers** (no CRT inputs; coverage helpers empty):
  ```java
  var easyCrt =
      new EasyCRTConfig(enc1, enc2)
          .withAbsoluteEncoder1Gearing(72, 24)        // 72 drives 24
          .withAbsoluteEncoder2Gearing(50, 20, 40)    // 50 drives 20, 20 drives 40
          .withMechanismRange(Rotations.of(0.0), Rotations.of(1.2))
          .withAbsoluteEncoderInversions(false, false);
  ```

## Validation checklist
- Observe `getLastStatus()` while commanding moves; it should report `OK` across travel.
- Ensure `getLastErrorRotations()` stays below your tolerance with margin.
- Jog across theoretical wrap boundaries; verify no sudden +/-1 rotation jumps in mechanism space.
- Run multiple power cycles in varied poses and confirm the resolved angle stays within your tolerance budget.
