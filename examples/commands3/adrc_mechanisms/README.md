# ADRC mechanisms (Commands v3)

The Commands v3 version of the [ADRC mechanisms example](../../commands2/adrc_mechanisms/README.md): an arm, an
elevator and a shooter run by ADRCs, each its own mechanism (`ADRCArmMechanism`, `ADRCElevatorMechanism`,
`ADRCShooterMechanism`). **The ADRC was written by FRC Team 8331**, who contributed it to YAMS. The ADRC settings and
live tuning are the same. The arm's Live Tuning command is named after the mechanism, at
`/Tuning/ADRCArmMechanism/Live Tuning`.

The bindings are in the `ADRC Teleop` opmode. While nothing else runs, the shooter coasts (its idle command).
