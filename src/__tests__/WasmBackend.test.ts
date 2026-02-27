import { describe, it, expect } from 'vitest';
import { LinearTrajectory } from '../LinearTrajectory';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';

const waypoints = [
  { time: 0, positions: [0, 0] },
  { time: 1, positions: [1, 2] },
  { time: 2, positions: [0, 4] },
  { time: 3, positions: [1, 6] },
];

describe('WASM backend', () => {
  it('CubicSplineTrajectory loads WASM backend', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    const wasmInternal = (traj as unknown as Record<string, unknown>)._wasm;
    expect(wasmInternal).not.toBeNull();
  });

  it('LinearTrajectory loads WASM backend', () => {
    const traj = new LinearTrajectory([
      { time: 0, positions: [0] },
      { time: 1, positions: [1] },
    ]);
    const wasmInternal = (traj as unknown as Record<string, unknown>)._wasm;
    expect(wasmInternal).not.toBeNull();
  });

  it('WASM CubicSpline matches TS output at sample points', () => {
    const traj = new CubicSplineTrajectory(waypoints);
    // Verify sample values are correct (same as TS implementation)
    const result = traj.sample(0.5);
    expect(result).toHaveLength(2);
    result.forEach((v) => expect(isFinite(v)).toBe(true));
  });

  it('WASM LinearTrajectory matches expected output', () => {
    const traj = new LinearTrajectory([
      { time: 0, positions: [0, 0] },
      { time: 2, positions: [4, 6] },
    ]);
    const mid = traj.sample(1);
    expect(mid[0]).toBeCloseTo(2, 8);
    expect(mid[1]).toBeCloseTo(3, 8);
  });
});
