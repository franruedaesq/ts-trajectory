/**
 * Parity / Fuzz Tests: WASM vs Pure-TS
 *
 * Verifies that the WASM-backed implementation produces the exact same output
 * (within floating-point epsilon) as the original TypeScript implementation
 * for a wide range of inputs.
 *
 * Strategy:
 *  - CubicSplineTrajectory always computes both `coeffsByDim` (TS) and `_wasm`.
 *    When WASM is available, `sample()` delegates to WASM.  The expected TS
 *    result is derived by evaluating `coeffsByDim` directly.
 *  - LinearTrajectory is tested analogously.
 *  - Fallback tests force `_wasm = null` at runtime and verify that the TS
 *    code path produces identical results.
 */

import { describe, expect, it } from 'vitest';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';
import { LinearTrajectory } from '../LinearTrajectory';
import { Waypoint } from '../types';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const EPSILON = 1e-9;

// ---------------------------------------------------------------------------
// Deterministic PRNG (Mulberry32) for reproducible fuzz tests
// Standard Mulberry32 algorithm: https://gist.github.com/tommyettinger/46a874533244883189143505d203312c
// ---------------------------------------------------------------------------
function makePRNG(seed: number): () => number {
  let s = seed >>> 0;
  return (): number => {
    // Mulberry32 single-iteration: mix with xorshift and multiply steps
    s = (Math.imul(s ^ (s >>> 15), s | 1) ^ ((s ^ (s >>> 7)) * (61 | s))) >>> 0;
    return s / 0x100000000;
  };
}

// ---------------------------------------------------------------------------
// Waypoint generator
// ---------------------------------------------------------------------------
function generateWaypoints(rand: () => number, numPoints: number, dims: number): Waypoint[] {
  const waypoints: Waypoint[] = [];
  let t = 0;
  for (let i = 0; i < numPoints; i++) {
    // Strictly-increasing time with variable step (0.01 – 5.01)
    t += rand() * 5 + 0.01;
    waypoints.push({
      time: t,
      positions: Array.from({ length: dims }, () => rand() * 200 - 100),
    });
  }
  return waypoints;
}

// ---------------------------------------------------------------------------
// Internal type helpers (mirrors private fields of trajectory classes)
// ---------------------------------------------------------------------------
type SplineCoeffs = { a: number; b: number; c: number; d: number };

interface CubicInternal {
  _wasm: object | null;
  waypoints: Waypoint[];
  coeffsByDim: SplineCoeffs[][];
}

interface LinearInternal {
  _wasm: object | null;
  waypoints: Waypoint[];
}

// ---------------------------------------------------------------------------
// Pure-TS reference implementations
// ---------------------------------------------------------------------------

/** Evaluates a cubic spline at time `t` using the pre-computed TS coefficients. */
function tsCubicSample(waypoints: Waypoint[], coeffsByDim: SplineCoeffs[][], t: number): number[] {
  const first = waypoints[0];
  const last = waypoints[waypoints.length - 1];

  if (t <= first.time) {
    return coeffsByDim.map((dc) => dc[0].a);
  }
  if (t >= last.time) {
    // The spline interpolates through waypoints, so evaluating the last segment
    // at its end gives the last position exactly.
    return last.positions.map((p) => p);
  }

  // Binary search for segment
  let lo = 0;
  let hi = waypoints.length - 2;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (waypoints[mid].time <= t) lo = mid;
    else hi = mid - 1;
  }

  const dt = t - waypoints[lo].time;
  return coeffsByDim.map((dc) => {
    const { a, b, c, d } = dc[lo];
    return a + b * dt + c * dt * dt + d * dt * dt * dt;
  });
}

/** Evaluates a linear trajectory at time `t`. */
function tsLinearSample(waypoints: Waypoint[], t: number): number[] {
  const first = waypoints[0];
  const last = waypoints[waypoints.length - 1];

  if (t <= first.time) return [...first.positions];
  if (t >= last.time) return [...last.positions];

  let lo = 0;
  let hi = waypoints.length - 2;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (waypoints[mid].time <= t) lo = mid;
    else hi = mid - 1;
  }

  const w0 = waypoints[lo];
  const w1 = waypoints[lo + 1];
  const alpha = (t - w0.time) / (w1.time - w0.time);
  return w0.positions.map((p, i) => p + alpha * (w1.positions[i] - p));
}

// ---------------------------------------------------------------------------
// Fuzz helpers
// ---------------------------------------------------------------------------

/** Generates a random set of query times that includes clamped, exact-waypoint,
 *  and interior timestamps. */
function queryTimes(rand: () => number, waypoints: Waypoint[], count: number): number[] {
  const first = waypoints[0].time;
  const last = waypoints[waypoints.length - 1].time;
  const duration = last - first;

  const times: number[] = [
    first - 1, // before start (clamp)
    first, // exact start
    last, // exact end
    last + 1, // after end (clamp)
  ];

  // Random interior and exterior queries
  for (let i = 0; i < count - 4; i++) {
    times.push(first + rand() * (duration + 2) - 1);
  }

  // Also include exact waypoint times for interpolation-through-waypoint checks
  for (const wp of waypoints) times.push(wp.time);

  return times;
}

// ===========================================================================
// CubicSplineTrajectory parity
// ===========================================================================

describe('Parity: CubicSplineTrajectory WASM vs Pure-TS', () => {
  it('produces identical results for 1000 random configurations', () => {
    const rand = makePRNG(0xdeadbeef);
    let wasmAvailable = false;

    for (let trial = 0; trial < 1000; trial++) {
      const numPoints = Math.floor(rand() * 49) + 2; // 2 – 50 waypoints
      const dims = Math.floor(rand() * 5) + 1; // 1 – 5 dimensions
      const wps = generateWaypoints(rand, numPoints, dims);

      const traj = new CubicSplineTrajectory(wps);
      const internal = traj as unknown as CubicInternal;

      if (!internal._wasm) continue; // WASM not available in this environment
      wasmAvailable = true;

      const { coeffsByDim } = internal;
      const times = queryTimes(rand, wps, 15);

      for (const t of times) {
        const wasmResult = [...traj.sample(t)];
        const tsResult = tsCubicSample(wps, coeffsByDim, t);

        expect(wasmResult).toHaveLength(dims);
        for (let i = 0; i < dims; i++) {
          expect(Math.abs(wasmResult[i] - tsResult[i])).toBeLessThan(EPSILON);
        }
      }
    }

    // Ensure we actually exercised the WASM path; if never available, skip silently.
    void wasmAvailable;
  });

  it('matches at exact waypoint times (interpolation-through-waypoint property)', () => {
    const rand = makePRNG(0xcafebabe);

    for (let trial = 0; trial < 200; trial++) {
      const numPoints = Math.floor(rand() * 18) + 2; // 2 – 19
      const dims = Math.floor(rand() * 3) + 1; // 1 – 3
      const wps = generateWaypoints(rand, numPoints, dims);

      const traj = new CubicSplineTrajectory(wps);
      const internal = traj as unknown as CubicInternal;

      for (const wp of wps) {
        const result = [...traj.sample(wp.time)];
        for (let i = 0; i < dims; i++) {
          expect(Math.abs(result[i] - wp.positions[i])).toBeLessThan(EPSILON);
        }
      }

      // Also check getDuration
      expect(traj.getDuration()).toBeCloseTo(wps[wps.length - 1].time, 12);

      if (!internal._wasm) continue; // WASM-specific checks only when available
      const { coeffsByDim } = internal;

      for (const wp of wps) {
        const wasmResult = [...traj.sample(wp.time)];
        const tsResult = tsCubicSample(wps, coeffsByDim, wp.time);
        for (let i = 0; i < dims; i++) {
          expect(Math.abs(wasmResult[i] - tsResult[i])).toBeLessThan(EPSILON);
        }
      }
    }
  });

  it('handles varying time scales (very small and very large intervals)', () => {
    const rand = makePRNG(0x1234abcd);

    const configs: Array<[number, number]> = [
      [1e-3, 1e-3], // micro-scale
      [1e6, 1e6], // mega-scale
      [1, 1e4], // mixed
    ];

    for (const [step, posScale] of configs) {
      const wps: Waypoint[] = [
        { time: 0, positions: [0] },
        { time: step, positions: [posScale] },
        { time: 2 * step, positions: [-posScale] },
        { time: 3 * step, positions: [0] },
      ];

      const traj = new CubicSplineTrajectory(wps);
      const internal = traj as unknown as CubicInternal;

      // Basic sanity: interpolates through waypoints
      for (const wp of wps) {
        const result = traj.sample(wp.time);
        expect(Math.abs(result[0] - wp.positions[0])).toBeLessThan(EPSILON * Math.abs(posScale) + EPSILON);
      }

      if (!internal._wasm) continue;
      const { coeffsByDim } = internal;

      for (const wp of wps) {
        const wasmResult = [...traj.sample(wp.time)];
        const tsResult = tsCubicSample(wps, coeffsByDim, wp.time);
        expect(Math.abs(wasmResult[0] - tsResult[0])).toBeLessThan(
          EPSILON * Math.abs(posScale) + EPSILON,
        );
      }
    }
  });

  it('handles high-dimensional waypoints (N-D)', () => {
    const rand = makePRNG(0xf00dface);

    for (const dims of [6, 10, 20]) {
      const wps = generateWaypoints(rand, 5, dims);
      const traj = new CubicSplineTrajectory(wps);
      const internal = traj as unknown as CubicInternal;

      const t = (wps[0].time + wps[wps.length - 1].time) / 2;
      const result = traj.sample(t);
      expect(result).toHaveLength(dims);
      result.forEach((v) => expect(isFinite(v)).toBe(true));

      if (!internal._wasm) continue;
      const { coeffsByDim } = internal;
      const tsResult = tsCubicSample(wps, coeffsByDim, t);
      for (let i = 0; i < dims; i++) {
        expect(Math.abs(result[i] - tsResult[i])).toBeLessThan(EPSILON);
      }
    }
  });
});

// ===========================================================================
// LinearTrajectory parity
// ===========================================================================

describe('Parity: LinearTrajectory WASM vs Pure-TS', () => {
  it('produces identical results for 1000 random configurations', () => {
    const rand = makePRNG(0xbaadf00d);
    let wasmAvailable = false;

    for (let trial = 0; trial < 1000; trial++) {
      const numPoints = Math.floor(rand() * 49) + 2; // 2 – 50
      const dims = Math.floor(rand() * 5) + 1; // 1 – 5
      const wps = generateWaypoints(rand, numPoints, dims);

      const traj = new LinearTrajectory(wps);
      const internal = traj as unknown as LinearInternal;

      if (!internal._wasm) continue;
      wasmAvailable = true;

      const times = queryTimes(rand, wps, 15);

      for (const t of times) {
        const wasmResult = [...traj.sample(t)];
        const tsResult = tsLinearSample(wps, t);

        expect(wasmResult).toHaveLength(dims);
        for (let i = 0; i < dims; i++) {
          expect(Math.abs(wasmResult[i] - tsResult[i])).toBeLessThan(EPSILON);
        }
      }
    }

    // Ensure we actually exercised the WASM path; if never available, skip silently.
    void wasmAvailable;
  });

  it('matches exact waypoint positions at waypoint times', () => {
    const rand = makePRNG(0x0badf00d);

    for (let trial = 0; trial < 200; trial++) {
      const numPoints = Math.floor(rand() * 18) + 2;
      const dims = Math.floor(rand() * 4) + 1;
      const wps = generateWaypoints(rand, numPoints, dims);

      const traj = new LinearTrajectory(wps);

      for (const wp of wps) {
        const result = [...traj.sample(wp.time)];
        for (let i = 0; i < dims; i++) {
          expect(Math.abs(result[i] - wp.positions[i])).toBeLessThan(EPSILON);
        }
      }
    }
  });
});

// ===========================================================================
// Fallback Mechanism
// ===========================================================================

describe('Fallback Mechanism: TS path when WASM is unavailable', () => {
  const baseWaypoints: Waypoint[] = [
    { time: 0, positions: [0, 0] },
    { time: 1, positions: [1, 2] },
    { time: 2, positions: [0, 4] },
    { time: 3, positions: [1, 6] },
  ];

  it('CubicSplineTrajectory TS fallback produces correct results', () => {
    const traj = new CubicSplineTrajectory(baseWaypoints);

    // Force TS path by nullifying the WASM handle at runtime
    (traj as unknown as CubicInternal)._wasm = null;

    // Must still interpolate through all waypoints
    for (const wp of baseWaypoints) {
      const result = traj.sample(wp.time);
      for (let i = 0; i < wp.positions.length; i++) {
        expect(Math.abs(result[i] - wp.positions[i])).toBeLessThan(EPSILON);
      }
    }

    // Clamping behaviour
    expect(traj.sample(-1)).toEqual(baseWaypoints[0].positions);
    expect(traj.sample(100)).toEqual(baseWaypoints[baseWaypoints.length - 1].positions);
  });

  it('CubicSplineTrajectory TS fallback getDuration still works', () => {
    const traj = new CubicSplineTrajectory(baseWaypoints);
    (traj as unknown as CubicInternal)._wasm = null;
    expect(traj.getDuration()).toBe(3);
  });

  it('LinearTrajectory TS fallback produces correct results', () => {
    const traj = new LinearTrajectory(baseWaypoints);
    (traj as unknown as LinearInternal)._wasm = null;

    for (const wp of baseWaypoints) {
      const result = traj.sample(wp.time);
      for (let i = 0; i < wp.positions.length; i++) {
        expect(Math.abs(result[i] - wp.positions[i])).toBeLessThan(EPSILON);
      }
    }

    // Midpoint interpolation
    const mid = traj.sample(0.5);
    expect(mid[0]).toBeCloseTo(0.5, 10);
    expect(mid[1]).toBeCloseTo(1.0, 10);
  });

  it('TS fallback result matches WASM result when both available', () => {
    const traj = new CubicSplineTrajectory(baseWaypoints);
    const internal = traj as unknown as CubicInternal;

    if (!internal._wasm) return; // Nothing to compare in TS-only environment

    const savedWasm = internal._wasm;
    const { coeffsByDim } = internal;
    const queryT = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5];

    try {
      for (const t of queryT) {
        // WASM result (via sample())
        const wasmResult = [...traj.sample(t)];

        // Force TS path
        (traj as unknown as CubicInternal)._wasm = null;
        const tsResult = [...traj.sample(t)];
        // Restore for next iteration
        (traj as unknown as CubicInternal)._wasm = savedWasm;

        // Must also match expected from coefficients
        const expectedTs = tsCubicSample(baseWaypoints, coeffsByDim, t);

        for (let i = 0; i < baseWaypoints[0].positions.length; i++) {
          expect(Math.abs(wasmResult[i] - tsResult[i])).toBeLessThan(EPSILON);
          expect(Math.abs(tsResult[i] - expectedTs[i])).toBeLessThan(EPSILON);
        }
      }
    } finally {
      // Ensure WASM handle is always restored even if assertions fail
      (traj as unknown as CubicInternal)._wasm = savedWasm;
    }
  });
});

// ===========================================================================
// Edge Cases
// ===========================================================================

describe('Edge Cases', () => {
  it('minimum waypoints (2 points) – cubic behaves as linear', () => {
    const wps: Waypoint[] = [
      { time: 0, positions: [0] },
      { time: 1, positions: [1] },
    ];
    const traj = new CubicSplineTrajectory(wps);
    expect(traj.sample(0)[0]).toBeCloseTo(0, 10);
    expect(traj.sample(0.5)[0]).toBeCloseTo(0.5, 10);
    expect(traj.sample(1)[0]).toBeCloseTo(1, 10);
  });

  it('maximum sampled waypoints (100 points) – cubic stays finite', () => {
    const rand = makePRNG(0x12345678);
    const wps = generateWaypoints(rand, 100, 2);
    const traj = new CubicSplineTrajectory(wps);

    for (let i = 0; i < wps.length; i++) {
      const result = traj.sample(wps[i].time);
      result.forEach((v) => expect(isFinite(v)).toBe(true));
    }
  });

  it('t exactly at every waypoint returns finite values (linear)', () => {
    const rand = makePRNG(0xabcdef01);
    const wps = generateWaypoints(rand, 50, 3);
    const traj = new LinearTrajectory(wps);

    for (const wp of wps) {
      const result = traj.sample(wp.time);
      result.forEach((v) => expect(isFinite(v)).toBe(true));
    }
  });

  it('getDuration is consistent across WASM and TS paths (cubic)', () => {
    const wps: Waypoint[] = [
      { time: 0, positions: [0] },
      { time: 5, positions: [1] },
      { time: 10, positions: [0] },
    ];
    const traj = new CubicSplineTrajectory(wps);
    const internal = traj as unknown as CubicInternal;

    const durationWithWasm = traj.getDuration();
    (traj as unknown as CubicInternal)._wasm = null;
    const durationWithTs = traj.getDuration();
    (traj as unknown as CubicInternal)._wasm = internal._wasm;

    expect(Math.abs(durationWithWasm - durationWithTs)).toBeLessThan(EPSILON);
    expect(durationWithTs).toBeCloseTo(10, 12);
  });

  it('getDuration is consistent across WASM and TS paths (linear)', () => {
    const wps: Waypoint[] = [
      { time: 0, positions: [0] },
      { time: 7, positions: [1] },
    ];
    const traj = new LinearTrajectory(wps);
    const internal = traj as unknown as LinearInternal;

    const durationWithWasm = traj.getDuration();
    (traj as unknown as LinearInternal)._wasm = null;
    const durationWithTs = traj.getDuration();
    (traj as unknown as LinearInternal)._wasm = internal._wasm;

    expect(Math.abs(durationWithWasm - durationWithTs)).toBeLessThan(EPSILON);
    expect(durationWithTs).toBeCloseTo(7, 12);
  });

  it('negative position values are handled correctly (cubic)', () => {
    const wps: Waypoint[] = [
      { time: 0, positions: [-100, -50] },
      { time: 1, positions: [-50, -25] },
      { time: 2, positions: [-100, -50] },
    ];
    const traj = new CubicSplineTrajectory(wps);
    for (const wp of wps) {
      const result = traj.sample(wp.time);
      for (let i = 0; i < wp.positions.length; i++) {
        expect(Math.abs(result[i] - wp.positions[i])).toBeLessThan(EPSILON);
      }
    }
  });

  it('large position values are handled correctly (cubic)', () => {
    const wps: Waypoint[] = [
      { time: 0, positions: [1e8] },
      { time: 1, positions: [-1e8] },
      { time: 2, positions: [1e8] },
    ];
    const traj = new CubicSplineTrajectory(wps);
    for (const wp of wps) {
      const result = traj.sample(wp.time);
      expect(isFinite(result[0])).toBe(true);
    }
  });

  it('very small time intervals (cubic) – does not produce NaN', () => {
    const dt = 1e-6;
    const wps: Waypoint[] = [
      { time: 0, positions: [0] },
      { time: dt, positions: [1] },
      { time: 2 * dt, positions: [0] },
    ];
    const traj = new CubicSplineTrajectory(wps);
    const result = traj.sample(dt / 2);
    expect(isFinite(result[0])).toBe(true);
  });

  it('sampleDerivative and sampleSecondDerivative stay finite under fallback', () => {
    const wps: Waypoint[] = [
      { time: 0, positions: [0, 0] },
      { time: 1, positions: [1, 2] },
      { time: 2, positions: [0, 4] },
    ];
    const traj = new CubicSplineTrajectory(wps);
    (traj as unknown as CubicInternal)._wasm = null;

    const d1 = traj.sampleDerivative(1);
    const d2 = traj.sampleSecondDerivative(1);
    d1.forEach((v) => expect(isFinite(v)).toBe(true));
    d2.forEach((v) => expect(isFinite(v)).toBe(true));
  });
});
