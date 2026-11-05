# Nova Arm

An arm driven by a NEO on a Thrifty Nova, through a 25:1 gearbox. The Nova runs the arm's position
loop and trapezoidal profile itself, with the arm feedforward, so the SystemCore only sends it the
setpoint. In simulation, YAMS runs the Nova's closed loop as the Nova does, since ThriftyLib's
simulation ignores the Nova's gains. This is the Commands v3 version: the arm is a `Mechanism`, and its
bindings and autonomous are opmodes.

## Controls

| Input | Action |
| --- | --- |
| A (hold) | Level, 0° |
| B (hold) | 45° |
| Y (hold) | 80° |
| Nothing pressed | Stow at -80° |
| Autonomous ("Arm Tour") | Swings through every setpoint, then stows |

## Simulation

Run `./gradlew run`, select the "Arm Tour" autonomous or the "Arm Teleop" teleop in the Sim GUI and
enable it, and watch the arm's Mechanism2d and the "Arm" telemetry in AdvantageScope or Elastic.
While the robot is disabled, the arm falls.
