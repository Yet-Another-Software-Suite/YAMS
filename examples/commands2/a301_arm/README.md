# A301 Arm

An arm driven by one FIRST A301, through its 64:1 gearbox and a 3:1 chain. The A301 has no closed
loop of its own, so YAMS runs the arm's position loop and trapezoidal profile on the SystemCore and
sends the A301 a voltage.

## Controls

| Input | Action |
| --- | --- |
| A (hold) | Level, 0° |
| B (hold) | 45° |
| Y (hold) | 80° |
| Nothing pressed | Stow at -80° |
| Autonomous | Swings through every setpoint, then stows |

## Simulation

Run `./gradlew run`, enable Autonomous in the Sim GUI, and watch the arm's Mechanism2d and
the "Arm" telemetry in AdvantageScope or Elastic.
