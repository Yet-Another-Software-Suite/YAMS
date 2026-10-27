// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.core.mechanisms.config.DoubleJointedArmConfig.ExclusionZone;

/**
 * Plans paths for a double jointed arm that keep both of its segments out of exclusion zones, with A* on a grid in
 * joint space.
 *
 * <p>Both angles are from horizontal, and the arm's frame has its origin at the shoulder with X away from it and Y up.
 * The lower segment runs from the shoulder to the elbow, and the upper segment from the elbow to the end of the arm.
 *
 * <p>Each step of a path moves to a neighboring grid point, and is checked over the whole box of joint angles it
 * spans, as each joint's closed loop controller can reach its next angle before or after the other. The box is checked
 * at its center with the exclusion zones grown by the farthest any point on the arm moves in it, so the check never
 * misses a collision. A step is:
 *
 * <ul>
 *   <li>never taken when the arm comes within the clearance of a zone in that box.</li>
 *   <li>taken at a much higher cost when the box widened by the following error the
 *   {@link yams.core.mechanisms.positional.DoubleJointedArmPathFollower} allows comes within the clearance, so paths
 *   keep that margin wherever they can, and only give it up near a start or goal that is close to a zone, or through a
 *   gap too tight for it.</li>
 * </ul>
 *
 * <p>When the arm starts within the clearance of an exclusion zone, the path leaves it along the shortest way out, and
 * never comes back.
 *
 * <p>Planning allocates nothing but the path it returns: its working arrays are kept and reused, and only grow when a
 * plan needs a bigger grid. So a planner is not thread safe.
 */
public class DoubleJointedArmPathPlanner {
  /** Cost of a step within the following error of the clearance, per radian. */
  private static final double kMarginStepCost    = 100;
  /** Cost of a step within the clearance, per radian, when leaving it after starting there. */
  private static final double kCollidingStepCost = 10000;
  /** Grid point not checked for exclusion zones yet. */
  private static final byte   kUnchecked         = 0;
  /** Grid point clear of the exclusion zones. */
  private static final byte   kClear             = 1;
  /** Grid point within the clearance of an exclusion zone. */
  private static final byte   kColliding         = 2;

  /** Lower segment length, in meters. */
  private final double   lowerLength;
  /** Upper segment length, in meters. */
  private final double   upperLength;
  /** Lowest lower arm angle, in radians. */
  private final double   lowerMin;
  /** Highest lower arm angle, in radians. */
  private final double   lowerMax;
  /** Lowest upper arm angle, in radians. */
  private final double   upperMin;
  /** Highest upper arm angle, in radians. */
  private final double   upperMax;
  /** Exclusion zones' lowest X, in meters. */
  private final double[] zoneMinX;
  /** Exclusion zones' lowest Y, in meters. */
  private final double[] zoneMinY;
  /** Exclusion zones' highest X, in meters. */
  private final double[] zoneMaxX;
  /** Exclusion zones' highest Y, in meters. */
  private final double[] zoneMaxY;
  /** Distance to keep from the exclusion zones, in meters. */
  private final double   clearance;
  /** Largest spacing of the grid, in radians. */
  private final double   resolution;
  /** Following error allowed in each joint, in radians. */
  private final double   tolerance;

  // ---- The plan being made. Kept between plans, so planning allocates nothing.

  /** Lowest lower arm angle on the grid, in radians. */
  private double      gridLowerMin;
  /** Lowest upper arm angle on the grid, in radians. */
  private double      gridUpperMin;
  /** Lower arm angle between grid points, in radians. */
  private double      lowerStep;
  /** Upper arm angle between grid points, in radians. */
  private double      upperStep;
  /** Grid points along the upper arm's angle. */
  private int         upperPoints;
  /** Grid points, which are followed by the start and goal nodes. */
  private int         gridNodes;
  /** Start lower arm angle, in radians. */
  private double      startLower;
  /** Start upper arm angle, in radians. */
  private double      startUpper;
  /** Goal lower arm angle, in radians. */
  private double      goalLower;
  /** Goal upper arm angle, in radians. */
  private double      goalUpper;
  /** Plan each node's entries below were last set in, so they don't have to be cleared for every plan. */
  private int[]       planOf       = new int[0];
  /** This plan's number. */
  private int         plan         = 0;
  /** Cost of the cheapest way found to each node. */
  private double[]    cost         = new double[0];
  /** Cost plus the heuristic, which the open set is ordered by. */
  private double[]    estimate     = new double[0];
  /** Node each node was reached from, or -1. */
  private int[]       cameFrom     = new int[0];
  /** Whether each node has been expanded. */
  private boolean[]   closed       = new boolean[0];
  /** Whether each node is in an exclusion zone: {@link #kUnchecked}, {@link #kClear} or {@link #kColliding}. */
  private byte[]      colliding    = new byte[0];
  /** Open set, a binary min-heap of nodes by {@link #estimate}. */
  private int[]       heap         = new int[0];
  /** Each node's index in {@link #heap} plus 1, or 0 when it isn't in it. */
  private int[]       heapIndex    = new int[0];
  /** Nodes in {@link #heap}. */
  private int         heapSize     = 0;
  /** Neighbors of the node being expanded. */
  private final int[] neighbors    = new int[13];
  /** Grid points at the corners of the start's cell. */
  private final int[] startCorners = new int[4];
  /** Grid points at the corners of the goal's cell. */
  private final int[] goalCorners  = new int[4];

  /**
   * Create a path planner.
   *
   * @param lowerLength Length of the lower segment.
   * @param upperLength Length of the upper segment.
   * @param lowerMin    Lowest lower arm angle, or negative infinity without a limit.
   * @param lowerMax    Highest lower arm angle, or positive infinity without a limit.
   * @param upperMin    Lowest upper arm angle, or negative infinity without a limit.
   * @param upperMax    Highest upper arm angle, or positive infinity without a limit.
   * @param zones       Exclusion zones.
   * @param clearance   Distance to keep from the exclusion zones.
   * @param resolution  Largest spacing of the joint space grid.
   * @param tolerance   Following error allowed in each joint, which paths keep away from the zones too.
   * @throws IllegalArgumentException if a joint's lowest angle is above its highest, or the resolution or tolerance is
   *                                  not above 0.
   */
  public DoubleJointedArmPathPlanner(Distance lowerLength, Distance upperLength, Angle lowerMin, Angle lowerMax, Angle upperMin, Angle upperMax,
                                     List<ExclusionZone> zones, Distance clearance, Angle resolution, Angle tolerance) {
    this.lowerLength = lowerLength.in(Meters);
    this.upperLength = upperLength.in(Meters);
    this.lowerMin = lowerMin.in(Radians);
    this.lowerMax = lowerMax.in(Radians);
    this.upperMin = upperMin.in(Radians);
    this.upperMax = upperMax.in(Radians);
    this.clearance = clearance.in(Meters);
    this.resolution = resolution.in(Radians);
    this.tolerance = tolerance.in(Radians);
    if (this.lowerMin > this.lowerMax || this.upperMin > this.upperMax) {
      throw new IllegalArgumentException("A joint's lowest angle is above its highest");
    }
    if (!(this.resolution > 0 && this.tolerance > 0)) {
      throw new IllegalArgumentException("Resolution and tolerance must be above 0");
    }
    zoneMinX = new double[zones.size()];
    zoneMinY = new double[zones.size()];
    zoneMaxX = new double[zones.size()];
    zoneMaxY = new double[zones.size()];
    for (int z = 0; z < zones.size(); z++) {
      zoneMinX[z] = zones.get(z).min().getX();
      zoneMinY[z] = zones.get(z).min().getY();
      zoneMaxX[z] = zones.get(z).max().getX();
      zoneMaxY[z] = zones.get(z).max().getY();
    }
  }

  /**
   * Whether the arm at these angles is in an exclusion zone, or closer to one than the clearance.
   *
   * @param lowerAngle Lower arm angle.
   * @param upperAngle Upper arm angle.
   * @return True if either segment is in or too close to an exclusion zone.
   */
  public boolean isInExclusionZone(Angle lowerAngle, Angle upperAngle) {
    return collides(lowerAngle.in(Radians), upperAngle.in(Radians), clearance);
  }

  /**
   * Plan a path between two sets of joint angles that never enters an exclusion zone.
   *
   * @param fromLower Lower arm angle to start at, in radians.
   * @param fromUpper Upper arm angle to start at, in radians.
   * @param toLower   Lower arm angle to end at, in radians.
   * @param toUpper   Upper arm angle to end at, in radians.
   * @return The angles to move through after the start, ending at the goal, or empty when the goal is outside the joint
   *     limits, in an exclusion zone, or can't be reached without entering one.
   */
  public Optional<DoubleJointedArmPath> plan(double fromLower, double fromUpper, double toLower, double toUpper) {
    if (toLower < lowerMin || toLower > lowerMax || toUpper < upperMin || toUpper > upperMax || collides(toLower, toUpper, clearance)) {
      return Optional.empty();
    }
    if (zoneMinX.length == 0) {
      return Optional.of(DoubleJointedArmPath.to(Radians.of(toLower), Radians.of(toUpper)));
    }
    startLower = fromLower;
    startUpper = fromUpper;
    goalLower = toLower;
    goalUpper = toUpper;

    // The grid spans the joint limits, and the start if the arm is past them. A joint without limits gets a turn either
    // way from the start and goal.
    gridLowerMin = Double.isInfinite(lowerMin) ? Math.min(fromLower, toLower) - Math.PI : Math.min(lowerMin, fromLower);
    gridUpperMin = Double.isInfinite(upperMin) ? Math.min(fromUpper, toUpper) - Math.PI : Math.min(upperMin, fromUpper);
    final double gridLowerMax = Double.isInfinite(lowerMax) ? Math.max(fromLower, toLower) + Math.PI : Math.max(lowerMax, fromLower);
    final double gridUpperMax = Double.isInfinite(upperMax) ? Math.max(fromUpper, toUpper) + Math.PI : Math.max(upperMax, fromUpper);
    final int lowerSteps = Math.max(1, (int) Math.ceil((gridLowerMax - gridLowerMin) / resolution));
    final int upperSteps = Math.max(1, (int) Math.ceil((gridUpperMax - gridUpperMin) / resolution));
    lowerStep = (gridLowerMax - gridLowerMin) / lowerSteps;
    upperStep = (gridUpperMax - gridUpperMin) / upperSteps;
    upperPoints = upperSteps + 1;
    gridNodes = (lowerSteps + 1) * upperPoints;
    final int start = gridNodes;
    final int goal  = gridNodes + 1;
    beginPlan(gridNodes + 2);

    cellCorners(fromLower, fromUpper, lowerSteps, upperSteps, startCorners);
    cellCorners(toLower, toUpper, lowerSteps, upperSteps, goalCorners);
    final boolean startColliding = isColliding(start);

    cost[start] = 0;
    push(start, heuristic(fromLower, fromUpper));
    while (heapSize > 0) {
      final int node = pop();
      if (node == goal) {
        return Optional.of(reconstruct(goal));
      }
      closed[node] = true;

      int count = 0;
      if (node == start) {
        for (int corner : startCorners) {
          neighbors[count++] = corner;
        }
        if (Math.abs(toLower - fromLower) <= lowerStep && Math.abs(toUpper - fromUpper) <= upperStep) {
          neighbors[count++] = goal;
        }
      } else {
        final int i = node / upperPoints;
        final int j = node % upperPoints;
        for (int di = -1; di <= 1; di++) {
          for (int dj = -1; dj <= 1; dj++) {
            if ((di != 0 || dj != 0) && i + di >= 0 && i + di <= lowerSteps && j + dj >= 0 && j + dj <= upperSteps) {
              neighbors[count++] = (i + di) * upperPoints + j + dj;
            }
          }
        }
        for (int corner : goalCorners) {
          if (corner == node) {
            neighbors[count++] = goal;
            break;
          }
        }
      }

      // Leaving the clearance of a zone the arm started in is allowed, but entering it is not.
      final boolean leaving   = startColliding && isColliding(node);
      final double  nodeLower = lowerOf(node);
      final double  nodeUpper = upperOf(node);
      for (int n = 0; n < count; n++) {
        final int next = neighbors[n];
        touch(next);
        if (closed[next]) {
          continue;
        }
        final double  nextLower     = lowerOf(next);
        final double  nextUpper     = upperOf(next);
        final double  lowerDelta    = nextLower - nodeLower;
        final double  upperDelta    = nextUpper - nodeUpper;
        final double  centerLower   = (nodeLower + nextLower) / 2;
        final double  centerUpper   = (nodeUpper + nextUpper) / 2;
        final double  stepMargin    = clearance + lowerLength * Math.abs(lowerDelta) / 2 + upperLength * Math.abs(upperDelta) / 2;
        final boolean stepColliding = collides(centerLower, centerUpper, stepMargin);
        if (stepColliding && !leaving) {
          continue;
        }
        final boolean inMargin = stepColliding || collides(centerLower, centerUpper, stepMargin + (lowerLength + upperLength) * tolerance);
        final double nextCost = cost[node] + Math.hypot(lowerDelta, upperDelta) * (stepColliding ? kCollidingStepCost : inMargin ? kMarginStepCost : 1);
        if (nextCost < cost[next]) {
          cost[next] = nextCost;
          cameFrom[next] = node;
          push(next, nextCost + heuristic(nextLower, nextUpper));
        }
      }
    }
    return Optional.empty();
  }

  /** Start a plan over this many nodes, growing the working arrays if they're too small. */
  private void beginPlan(int nodes) {
    if (planOf.length < nodes) {
      final int capacity = Math.max(nodes, planOf.length * 2);
      planOf = new int[capacity];
      cost = new double[capacity];
      estimate = new double[capacity];
      cameFrom = new int[capacity];
      closed = new boolean[capacity];
      colliding = new byte[capacity];
      heap = new int[capacity];
      heapIndex = new int[capacity];
      plan = 0;
    }
    plan++;
    if (plan == Integer.MAX_VALUE) {
      Arrays.fill(planOf, 0);
      plan = 1;
    }
    heapSize = 0;
  }

  /** Reset a node's entries the first time this plan uses it. */
  private void touch(int node) {
    if (planOf[node] != plan) {
      planOf[node] = plan;
      cost[node] = Double.POSITIVE_INFINITY;
      cameFrom[node] = -1;
      closed[node] = false;
      colliding[node] = kUnchecked;
      heapIndex[node] = 0;
    }
  }

  /** Lower arm angle of a node, in radians. */
  private double lowerOf(int node) {
    if (node < gridNodes) {
      return gridLowerMin + (node / upperPoints) * lowerStep;
    }
    return node == gridNodes ? startLower : goalLower;
  }

  /** Upper arm angle of a node, in radians. */
  private double upperOf(int node) {
    if (node < gridNodes) {
      return gridUpperMin + (node % upperPoints) * upperStep;
    }
    return node == gridNodes ? startUpper : goalUpper;
  }

  /** Whether a node is within the clearance of an exclusion zone, checked once a plan. */
  private boolean isColliding(int node) {
    touch(node);
    if (colliding[node] == kUnchecked) {
      colliding[node] = collides(lowerOf(node), upperOf(node), clearance) ? kColliding : kClear;
    }
    return colliding[node] == kColliding;
  }

  /** Put the grid points at the corners of the cell some angles are in into {@code corners}. */
  private void cellCorners(double lowerAngle, double upperAngle, int lowerSteps, int upperSteps, int[] corners) {
    final int i = Math.min(lowerSteps - 1, Math.max(0, (int) Math.floor((lowerAngle - gridLowerMin) / lowerStep)));
    final int j = Math.min(upperSteps - 1, Math.max(0, (int) Math.floor((upperAngle - gridUpperMin) / upperStep)));
    corners[0] = i * upperPoints + j;
    corners[1] = i * upperPoints + j + 1;
    corners[2] = (i + 1) * upperPoints + j;
    corners[3] = (i + 1) * upperPoints + j + 1;
  }

  /** Straight line distance to the goal in joint space, which a path can never be shorter than. */
  private double heuristic(double lowerAngle, double upperAngle) {
    return Math.hypot(goalLower - lowerAngle, goalUpper - upperAngle);
  }

  /**
   * The path to a node from the step after the start, with steps that move only one joint the same way merged: the
   * other joint holds still, so the arm can't stray from the line between them. Diagonal steps are kept, as the joints
   * can reach them at different times.
   */
  private DoubleJointedArmPath reconstruct(int node) {
    int length = 0;
    for (int current = node; cameFrom[current] != -1; current = cameFrom[current]) {
      length++;
    }
    final double[] lower = new double[length];
    final double[] upper = new double[length];
    int k = length;
    for (int current = node; cameFrom[current] != -1; current = cameFrom[current]) {
      k--;
      lower[k] = lowerOf(current);
      upper[k] = upperOf(current);
    }
    int kept = 0;
    for (k = 0; k < length; k++) {
      if (kept > 0 && k < length - 1) {
        final double  previousLower = lower[kept - 1];
        final double  previousUpper = upper[kept - 1];
        final boolean upperOnly     = previousLower == lower[k] && lower[k] == lower[k + 1] &&
                                      Math.signum(upper[k] - previousUpper) == Math.signum(upper[k + 1] - upper[k]);
        final boolean lowerOnly     = previousUpper == upper[k] && upper[k] == upper[k + 1] &&
                                      Math.signum(lower[k] - previousLower) == Math.signum(lower[k + 1] - lower[k]);
        if (upperOnly || lowerOnly) {
          continue;
        }
      }
      lower[kept] = lower[k];
      upper[kept] = upper[k];
      kept++;
    }
    return new DoubleJointedArmPath(Arrays.copyOf(lower, kept), Arrays.copyOf(upper, kept));
  }

  // ---- Open set: an indexed binary min-heap, so a node's estimate can be lowered in place.

  /** Add a node to the open set, or lower its estimate if it's already in it. */
  private void push(int node, double nodeEstimate) {
    estimate[node] = nodeEstimate;
    int index = heapIndex[node] - 1;
    if (index < 0) {
      index = heapSize++;
      heap[index] = node;
      heapIndex[node] = index + 1;
    }
    siftUp(index);
  }

  /** Remove and return the node with the lowest estimate. */
  private int pop() {
    final int top = heap[0];
    heapIndex[top] = 0;
    heapSize--;
    if (heapSize > 0) {
      heap[0] = heap[heapSize];
      heapIndex[heap[0]] = 1;
      siftDown(0);
    }
    return top;
  }

  /** Move a heap entry up until its parent's estimate is no higher. */
  private void siftUp(int index) {
    final int node = heap[index];
    while (index > 0) {
      final int parent = (index - 1) / 2;
      if (estimate[heap[parent]] <= estimate[node]) {
        break;
      }
      heap[index] = heap[parent];
      heapIndex[heap[index]] = index + 1;
      index = parent;
    }
    heap[index] = node;
    heapIndex[node] = index + 1;
  }

  /** Move a heap entry down until neither child's estimate is lower. */
  private void siftDown(int index) {
    final int node = heap[index];
    while (true) {
      int child = 2 * index + 1;
      if (child >= heapSize) {
        break;
      }
      if (child + 1 < heapSize && estimate[heap[child + 1]] < estimate[heap[child]]) {
        child++;
      }
      if (estimate[heap[child]] >= estimate[node]) {
        break;
      }
      heap[index] = heap[child];
      heapIndex[heap[index]] = index + 1;
      index = child;
    }
    heap[index] = node;
    heapIndex[node] = index + 1;
  }

  /**
   * Whether either segment of the arm is within a distance of an exclusion zone.
   *
   * @param lowerAngle Lower arm angle, in radians.
   * @param upperAngle Upper arm angle, in radians.
   * @param margin     Distance to grow the exclusion zones by, in meters.
   * @return True if a segment touches a grown exclusion zone.
   */
  private boolean collides(double lowerAngle, double upperAngle, double margin) {
    final double elbowX = lowerLength * Math.cos(lowerAngle);
    final double elbowY = lowerLength * Math.sin(lowerAngle);
    final double endX   = elbowX + upperLength * Math.cos(upperAngle);
    final double endY   = elbowY + upperLength * Math.sin(upperAngle);
    for (int z = 0; z < zoneMinX.length; z++) {
      final double minX = zoneMinX[z] - margin;
      final double minY = zoneMinY[z] - margin;
      final double maxX = zoneMaxX[z] + margin;
      final double maxY = zoneMaxY[z] + margin;
      if (segmentIntersectsBox(0, 0, elbowX, elbowY, minX, minY, maxX, maxY) || segmentIntersectsBox(elbowX, elbowY, endX, endY, minX, minY, maxX, maxY)) {
        return true;
      }
    }
    return false;
  }

  /** Whether a segment touches a box, by clipping the segment to each side of the box in turn (Liang-Barsky). */
  private static boolean segmentIntersectsBox(double x0, double y0, double x1, double y1, double minX, double minY, double maxX, double maxY) {
    final double dx   = x1 - x0;
    final double dy   = y1 - y0;
    double       tIn  = 0;
    double       tOut = 1;
    for (int side = 0; side < 4; side++) {
      // The segment is inside this side where p * t <= q.
      final double p = switch (side) {
        case 0 -> -dx;
        case 1 -> dx;
        case 2 -> -dy;
        default -> dy;
      };
      final double q = switch (side) {
        case 0 -> x0 - minX;
        case 1 -> maxX - x0;
        case 2 -> y0 - minY;
        default -> maxY - y0;
      };
      if (p == 0) {
        if (q < 0) {
          return false;
        }
      } else {
        final double t = q / p;
        if (p < 0) {
          tIn = Math.max(tIn, t);
        } else {
          tOut = Math.min(tOut, t);
        }
        if (tIn > tOut) {
          return false;
        }
      }
    }
    return true;
  }
}
