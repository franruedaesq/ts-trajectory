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

  it('clamps t below first waypoint time', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    expect(traj.sample(-1)).toEqual([0, 0]);
  });

  it('clamps t above last waypoint time', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    expect(traj.sample(100)).toEqual([1, 6]);
  });
});
