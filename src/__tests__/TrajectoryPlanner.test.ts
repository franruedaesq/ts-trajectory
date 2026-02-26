import { describe, expect, it } from 'vitest';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';
import { LinearTrajectory } from '../LinearTrajectory';
import { TrajectoryPlanner } from '../TrajectoryPlanner';
import { isPlannerConfig, isTrajectory, isWaypoint } from '../types';

describe('TrajectoryPlanner', () => {
  const planner = new TrajectoryPlanner();

  const waypoints = [
    { time: 0, positions: [0, 0] },
    { time: 1, positions: [1, 2] },
    { time: 2, positions: [3, 4] },
  ];

  it('creates LinearTrajectory when interpolationType is linear', () => {
    const traj = planner.plan(waypoints, { interpolationType: 'linear' });
    expect(traj).toBeInstanceOf(LinearTrajectory);
  });

  it('creates CubicSplineTrajectory when interpolationType is cubic', () => {
    const traj = planner.plan(waypoints, { interpolationType: 'cubic' });
    expect(traj).toBeInstanceOf(CubicSplineTrajectory);
  });

  it('throws with fewer than 2 waypoints', () => {
    expect(() =>
      planner.plan([{ time: 0, positions: [0] }], { interpolationType: 'linear' }),
    ).toThrow();
  });

  it('throws if waypoint times are not strictly increasing', () => {
    expect(() =>
      planner.plan(
        [
          { time: 0, positions: [0] },
          { time: 0, positions: [1] },
        ],
        { interpolationType: 'linear' },
      ),
    ).toThrow();
  });

  it('throws if waypoint dimensions are inconsistent', () => {
    expect(() =>
      planner.plan(
        [
          { time: 0, positions: [0, 1] },
          { time: 1, positions: [2] },
        ],
        { interpolationType: 'linear' },
      ),
    ).toThrow();
  });
});

describe('isWaypoint', () => {
  it('returns true for a valid waypoint', () => {
    expect(isWaypoint({ time: 0, positions: [0, 1] })).toBe(true);
  });

  it('returns true for a waypoint with velocities', () => {
    expect(isWaypoint({ time: 1, positions: [0], velocities: [1] })).toBe(true);
  });

  it('returns false for null', () => {
    expect(isWaypoint(null)).toBe(false);
  });

  it('returns false when time is missing', () => {
    expect(isWaypoint({ positions: [0] })).toBe(false);
  });

  it('returns false when positions is not a number array', () => {
    expect(isWaypoint({ time: 0, positions: ['a'] })).toBe(false);
  });

  it('returns false when velocities is not a number array', () => {
    expect(isWaypoint({ time: 0, positions: [0], velocities: ['a'] })).toBe(false);
  });
});

describe('isTrajectory', () => {
  it('returns true for a LinearTrajectory', () => {
    const traj = new LinearTrajectory([
      { time: 0, positions: [0] },
      { time: 1, positions: [1] },
    ]);
    expect(isTrajectory(traj)).toBe(true);
  });

  it('returns true for a CubicSplineTrajectory', () => {
    const traj = new CubicSplineTrajectory([
      { time: 0, positions: [0] },
      { time: 1, positions: [1] },
    ]);
    expect(isTrajectory(traj)).toBe(true);
  });

  it('returns false for null', () => {
    expect(isTrajectory(null)).toBe(false);
  });

  it('returns false for an object missing getDuration', () => {
    expect(isTrajectory({ sample: () => [] })).toBe(false);
  });

  it('returns false for an object missing sample', () => {
    expect(isTrajectory({ getDuration: () => 1 })).toBe(false);
  });
});

describe('isPlannerConfig', () => {
  it('returns true for a linear config', () => {
    expect(isPlannerConfig({ interpolationType: 'linear' })).toBe(true);
  });

  it('returns true for a cubic config with optional fields', () => {
    expect(
      isPlannerConfig({ interpolationType: 'cubic', maxVelocity: [1], maxAcceleration: [2] }),
    ).toBe(true);
  });

  it('returns false for null', () => {
    expect(isPlannerConfig(null)).toBe(false);
  });

  it('returns false for an unknown interpolationType', () => {
    expect(isPlannerConfig({ interpolationType: 'spline' })).toBe(false);
  });

  it('returns false when interpolationType is missing', () => {
    expect(isPlannerConfig({ maxVelocity: [1] })).toBe(false);
  });

  it('returns false when maxVelocity is not a number array', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxVelocity: ['fast'] })).toBe(false);
  });

  it('returns false when maxAcceleration is not a number array', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxAcceleration: [null] })).toBe(false);
  });
});
