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

  it('works for N-dimensional waypoints', () => {
    const dims = 6;
    const start = Array.from({ length: dims }, (_, i) => i * 1.0);
    const end = Array.from({ length: dims }, (_, i) => i * 2.0);
    const traj = new LinearTrajectory([
      { time: 0, positions: start },
      { time: 1, positions: end },
    ]);
    const mid = traj.sample(0.5);
    expect(mid).toHaveLength(dims);
    mid.forEach((v, i) => expect(v).toBeCloseTo(start[i] + 0.5 * (end[i] - start[i])));
  });

  it('produces correct results for sequential (monotonically increasing) t queries', () => {
    const traj = new LinearTrajectory(waypoints);
    const times = [0.1, 0.2, 0.5, 0.9, 1.0, 1.5, 2.0, 2.5, 2.9];
    for (const t of times) {
      const result = traj.sample(t);
      expect(result).toHaveLength(3);
      result.forEach((v) => expect(isFinite(v)).toBe(true));
    }
    // Verify specific sequential values match expected interpolation
    const traj2 = new LinearTrajectory(waypoints);
    expect(traj2.sample(0.5)).toEqual([1, 2, 3]);
    expect(traj2.sample(1.0)).toEqual([2, 4, 6]);
    expect(traj2.sample(2.0)).toEqual([5, 7, 9]);
  });
});
