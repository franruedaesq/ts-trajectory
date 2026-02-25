import { describe, expect, it } from 'vitest';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';
import { LinearTrajectory } from '../LinearTrajectory';
import { TrajectoryPlanner } from '../TrajectoryPlanner';

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
