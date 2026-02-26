import { describe, expect, it } from 'vitest';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';
import { LinearTrajectory } from '../LinearTrajectory';
import { TrajectoryBuilder } from '../TrajectoryBuilder';
import { isPlannerConfig, isTrajectory, isWaypoint } from '../types';

describe('TrajectoryBuilder', () => {
  const planner = new TrajectoryBuilder();

  const waypoints = [
    { time: 0, positions: [0, 0] },
    { time: 1, positions: [1, 2] },
    { time: 2, positions: [3, 4] },
  ];

  it('throws when a waypoint has velocities (not yet supported in V1)', () => {
    expect(() =>
      planner.plan(
        [
          { time: 0, positions: [0], velocities: [1] },
          { time: 1, positions: [1] },
        ],
        { interpolationType: 'linear' },
      ),
    ).toThrow('Waypoint velocities are not yet supported in V1.');
  });

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

  it('throws if maxVelocity dimensions do not match waypoints', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'linear', maxVelocity: [1] }),
    ).toThrow('maxVelocity must have the same number of dimensions as waypoints.');
  });

  it('throws if maxAcceleration dimensions do not match waypoints', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'linear', maxAcceleration: [1] }),
    ).toThrow('maxAcceleration must have the same number of dimensions as waypoints.');
  });

  it('does not throw when linear trajectory is within maxVelocity', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'linear', maxVelocity: [10, 10] }),
    ).not.toThrow();
  });

  it('throws when linear trajectory exceeds maxVelocity', () => {
    // segment 0->1: vel_dim0 = 1/1 = 1, vel_dim1 = 2/1 = 2
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'linear', maxVelocity: [10, 1] }),
    ).toThrow('maxVelocity');
  });

  it('does not throw when linear trajectory is within maxAcceleration (always zero)', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'linear', maxAcceleration: [0, 0] }),
    ).not.toThrow();
  });

  it('does not throw when cubic trajectory is within maxVelocity', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'cubic', maxVelocity: [100, 100] }),
    ).not.toThrow();
  });

  it('throws when cubic trajectory exceeds maxVelocity', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'cubic', maxVelocity: [0.001, 0.001] }),
    ).toThrow('maxVelocity');
  });

  it('does not throw when cubic trajectory is within maxAcceleration', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'cubic', maxAcceleration: [1000, 1000] }),
    ).not.toThrow();
  });

  it('throws when cubic trajectory exceeds maxAcceleration', () => {
    expect(() =>
      planner.plan(waypoints, { interpolationType: 'cubic', maxAcceleration: [0.001, 0.001] }),
    ).toThrow('maxAcceleration');
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

  it('returns false when time is NaN', () => {
    expect(isWaypoint({ time: NaN, positions: [0] })).toBe(false);
  });

  it('returns false when time is Infinity', () => {
    expect(isWaypoint({ time: Infinity, positions: [0] })).toBe(false);
  });

  it('returns false when a position is NaN', () => {
    expect(isWaypoint({ time: 0, positions: [NaN, 0] })).toBe(false);
  });

  it('returns false when a position is Infinity', () => {
    expect(isWaypoint({ time: 0, positions: [Infinity, 0] })).toBe(false);
  });

  it('returns false when a velocity is NaN', () => {
    expect(isWaypoint({ time: 0, positions: [0], velocities: [NaN] })).toBe(false);
  });

  it('returns false when positions is an empty array', () => {
    expect(isWaypoint({ time: 0, positions: [] })).toBe(false);
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

  it('returns false when maxVelocity contains NaN', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxVelocity: [NaN] })).toBe(false);
  });

  it('returns false when maxVelocity contains Infinity', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxVelocity: [Infinity] })).toBe(false);
  });

  it('returns false when maxAcceleration contains NaN', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxAcceleration: [NaN] })).toBe(false);
  });

  it('returns false when maxVelocity is an empty array', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxVelocity: [] })).toBe(false);
  });

  it('returns false when maxAcceleration is an empty array', () => {
    expect(isPlannerConfig({ interpolationType: 'linear', maxAcceleration: [] })).toBe(false);
  });
});
