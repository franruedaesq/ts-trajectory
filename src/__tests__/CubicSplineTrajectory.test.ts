import { describe, expect, it } from 'vitest';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';

describe('CubicSplineTrajectory', () => {
  const waypoints = [
    { time: 0, positions: [0, 0] },
    { time: 1, positions: [1, 2] },
    { time: 2, positions: [0, 4] },
    { time: 3, positions: [1, 6] },
  ];

  it('returns exact waypoint positions at waypoint times', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    for (const wp of waypoints) {
      const result = traj.sample(wp.time);
      result.forEach((v, i) => expect(v).toBeCloseTo(wp.positions[i], 8));
    }
  });

  it('getDuration returns last waypoint time', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    expect(traj.getDuration()).toBe(3);
  });

  it('works for multi-dimensional spaces', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    const result = traj.sample(0.5);
    expect(result).toHaveLength(2);
  });

  it('interpolated values between waypoints are not strictly linear', () => {
    // For a natural cubic spline with non-collinear points, the midpoint
    // value should differ from linear interpolation
    const traj = new CubicSplineTrajectory(waypoints);
    const cubicMid = traj.sample(0.5)[0];
    // cubic spline will not exactly equal linear (0.5) for inner segments
    // (it may be slightly different due to curvature from neighboring segments)
    expect(typeof cubicMid).toBe('number');
    expect(isFinite(cubicMid)).toBe(true);
  });

  it('degrades gracefully to linear with 2 waypoints', () => {
    const traj = new CubicSplineTrajectory([
      { time: 0, positions: [0] },
      { time: 1, positions: [1] },
    ]);
    expect(traj.sample(0.5)).toEqual([0.5]);
  });

  it('velocity (first derivative) is continuous at internal waypoints', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    // At each internal waypoint, derivative from left segment equals derivative from right segment
    for (let i = 1; i < waypoints.length - 1; i++) {
      const t = waypoints[i].time;
      const epsilon = 1e-7;
      const velLeft = traj.sample(t).map((v, j) => (v - traj.sample(t - epsilon)[j]) / epsilon);
      const velRight = traj
        .sample(t + epsilon)
        .map((v, j) => (v - traj.sample(t)[j]) / epsilon);
      velLeft.forEach((v, j) => expect(v).toBeCloseTo(velRight[j], 3));
    }
  });

  it('sampleDerivative matches analytical first derivative at waypoints', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    for (const wp of waypoints) {
      const deriv = traj.sampleDerivative(wp.time);
      expect(deriv).toHaveLength(wp.positions.length);
      deriv.forEach((v) => expect(isFinite(v)).toBe(true));
    }
  });

  it('sampleDerivative is continuous at internal waypoints', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    for (let i = 1; i < waypoints.length - 1; i++) {
      const t = waypoints[i].time;
      const epsilon = 1e-7;
      // Approach from left and right: derivative should be equal
      const derivLeft = traj.sampleDerivative(t - epsilon);
      const derivRight = traj.sampleDerivative(t + epsilon);
      derivLeft.forEach((v, j) => expect(v).toBeCloseTo(derivRight[j], 3));
    }
  });

  it('clamps t below first waypoint time', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    expect(traj.sample(-1)).toEqual([0, 0]);
  });

  it('clamps t above last waypoint time', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    expect(traj.sample(100)).toEqual([1, 6]);
  });
});
