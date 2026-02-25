import { describe, expect, it } from 'vitest';
import { LinearTrajectory } from '../LinearTrajectory';

describe('LinearTrajectory', () => {
  const waypoints = [
    { time: 0, positions: [0, 0, 0] },
    { time: 1, positions: [2, 4, 6] },
    { time: 3, positions: [8, 10, 12] },
  ];

  it('returns exact waypoint positions at waypoint times', () => {
    const traj = new LinearTrajectory(waypoints);
    expect(traj.sample(0)).toEqual([0, 0, 0]);
    expect(traj.sample(1)).toEqual([2, 4, 6]);
    expect(traj.sample(3)).toEqual([8, 10, 12]);
  });

  it('interpolates correctly at the midpoint', () => {
    const traj = new LinearTrajectory(waypoints);
    const mid = traj.sample(0.5);
    expect(mid).toEqual([1, 2, 3]);
  });

  it('works for 1D', () => {
    const t = new LinearTrajectory([
      { time: 0, positions: [0] },
      { time: 10, positions: [100] },
    ]);
    expect(t.sample(5)).toEqual([50]);
  });

  it('getDuration returns last waypoint time', () => {
    const traj = new LinearTrajectory(waypoints);
    expect(traj.getDuration()).toBe(3);
  });

  it('clamps t below first waypoint time', () => {
    const traj = new LinearTrajectory(waypoints);
    expect(traj.sample(-5)).toEqual([0, 0, 0]);
  });

  it('clamps t above last waypoint time', () => {
    const traj = new LinearTrajectory(waypoints);
    expect(traj.sample(100)).toEqual([8, 10, 12]);
  });
});
