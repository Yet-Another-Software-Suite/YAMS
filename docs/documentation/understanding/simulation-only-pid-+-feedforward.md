---
icon: robot
---

# Simulation-only PID + feedforward

A simulated mechanism is an ideal motor and load: no friction, no backlash, no cable drag, and exactly the mass you told YAMS. A real mechanism has all of those. So the gains that hold a real arm usually do something different in simulation, and the other way round. `withSimClosedLoopController` and `withSimFeedforward` let you keep one set of gains for each, so tuning one never breaks the other.

## An example

An arm tuned on the real robot needed `kS = 0.15 V` to get past friction and `kG = 0.35 V` to hold it level. In simulation there is no friction, and the simulated arm is a little lighter than the real one with its wiring and bolts:

```java
.withClosedLoopController(4, 0, 0.2)                 // real robot
.withFeedforward(new ArmFeedforward(0.15, 0.35, 1.2)) // real robot: kS, kG, kV
.withSimClosedLoopController(4, 0, 0.2)              // simulation: the same PID
.withSimFeedforward(new ArmFeedforward(0, 0.30, 1.2)) // simulation: no friction, a lighter arm
```

With the real `kS` in simulation, the arm would push 0.15 V harder than it needs to and overshoot every small move. With the real `kG`, it would creep upward when it should hold still. Each set is right for its own mechanism.

## When you need it

* **Leave them out at first.** Without them, simulation uses your real gains. That is fine while you are getting a mechanism moving.
* **Add them once the real robot is tuned**, if simulation then behaves worse than it did. Tune the simulation gains in simulation with [live tuning](../how-to-tuning/live-tuning/), the same way as the real ones.
* **Do not tune the real robot in simulation.** Gains found in simulation are a starting point at best: [Step 5 of the Arm tutorial](../tutorials/arm/tune.md) tunes them on the robot.

[`withSimClosedLoopController` and `withSimFeedforward`](../reference/editor/config/simulation.md#withsimclosedloopcontroller-and-withsimfeedforward) list the options, including slots.
