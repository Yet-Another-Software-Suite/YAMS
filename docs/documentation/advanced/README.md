---
description: >-
  Features most teams never need, explained as ideas and shown in complete
  examples, for when you are curious how they work.
icon: flask
---

# Advanced features

Most robots never need anything on these pages. A robot that scores reliably from a few known spots will win more matches than one that tries something clever and misses, so build that robot first.

These pages are for when you are curious anyway: you saw a team shoot while driving across the field and want to know how they did it, or you have a working robot and some time before the next event. Each one covers something that does not belong in YAMS itself, because every robot does it differently. Instead of a new YAMS feature, you get two things:

* **The idea**, in plain words, with links to the people who explain it best: blog posts, Chief Delphi threads, WPILib pages and other teams' code.
* **A complete example robot** in the YAMS repository, in Java with Commands v2 and Commands v3, that you can run in simulation, read, and take apart. Each page tells you which file and which method does each part of the idea.

You do not need to understand everything here to use it. Run the example first, watch what it does, then read the part of the page about whatever surprised you.

## Before you start

The examples build on the tutorials, so do the ones for the mechanisms an example uses first. Each page lists them. You will also want to be comfortable with:

* WPILib's [Coordinate System](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html): which way X, Y and a positive rotation point on the field.
* Running a robot program in simulation and graphing values in AdvantageScope: [How do I view my DataLog in AdvantageScope?](../how-to-simulation-and-logging/how-do-i-view-my-datalog-in-advantagescope.md)

## Running an example

The examples live in [`examples/`](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples) in the YAMS repository, one folder per example in `commands2/` and `commands3/`. Each folder holds only the robot's own code: the [examples README](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples#readme) explains how `make` turns it into a complete WPILib project you can open in VS Code and run with **WPILib: Simulate Robot Code**.

## In this section

| Page | What the robot does |
| --- | --- |
| [Shoot on the move](shoot-on-the-move/) | Aims, sets its shooter speed and feeds only when the shot is ready, while it keeps driving. |
| [Tilt-scaled odometry](tilt-scaled-odometry/) | Projects swerve wheel travel onto the horizontal field plane on ramps/bumps and gates 2D vision measurements while tilted. |
