import { describe, it, expect, vi, beforeAll, afterAll } from 'vitest';
import { CubicSplineTrajectory } from '../CubicSplineTrajectory';
import { LinearTrajectory } from '../LinearTrajectory';
import { Waypoint } from '../types';

// Utility to generate random waypoints
function generateRandomWaypoints(numPoints: number, dims: number): Waypoint[] {
  const waypoints: Waypoint[] = [];
  let currentTime = 0;

  for (let i = 0; i < numPoints; i++) {
    const positions = Array.from({ length: dims }, () => Math.random() * 200 - 100);
    waypoints.push({ time: currentTime, positions });
    currentTime += Math.random() * 5 + 0.1; // Ensure strictly increasing time
  }
  return waypoints;
}

// Utility to check if arrays are close
function expectArraysClose(arr1: number[], arr2: number[], epsilon = 1e-9) {
  expect(arr1.length).toBe(arr2.length);
  for (let i = 0; i < arr1.length; i++) {
    // Relaxed epsilon for high values or accumulated floating point errors
    const diff = Math.abs(arr1[i] - arr2[i]);
    if (diff > epsilon) {
       // Log detailed failure info to help debug
       console.error(`Mismatch at index ${i}: WASM=${arr1[i]}, TS=${arr2[i]}, Diff=${diff}`);
       expect(diff).toBeLessThan(epsilon);
    }
  }
}

describe('Parity / Fuzz Tests (WASM vs Pure TS)', () => {

  const NUM_FUZZ_ITERATIONS = 50;

  it('CubicSplineTrajectory: WASM output matches Pure TS output', async () => {

    for (let i = 0; i < NUM_FUZZ_ITERATIONS; i++) {
      const numPoints = Math.floor(Math.random() * 10) + 3; // 3 to 12 points
      const dims = Math.floor(Math.random() * 3) + 1; // 1 to 3 dimensions
      const waypoints = generateRandomWaypoints(numPoints, dims);

      const wasmTraj = new CubicSplineTrajectory(waypoints);

      // Ensure we are testing WASM
      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      if (!(wasmTraj as any)._wasm) {
        console.warn('WASM backend not loaded! Parity test is comparing TS vs TS.');
      }

      // Generate random sample times
      const duration = wasmTraj.getDuration();
      const sampleTimes = [
        0,
        duration,
        duration / 2,
        waypoints[1].time, // Exact waypoint match
        Math.random() * duration,
        -0.1, // Out of bounds
        duration + 0.1
      ];

      for (const t of sampleTimes) {
        const wasmResult = wasmTraj.sample(t);
        const refResult = referenceCubicSplineSample(waypoints, t);

        // Use a slightly larger epsilon (1e-6) because WASM (f64) and JS (f64)
        // might have subtle order-of-operation differences in the tridiagonal solver
        // or polynomial evaluation, especially with accumulated errors.
        expectArraysClose(wasmResult, refResult, 1e-6);
      }
    }
  });

  it('LinearTrajectory: WASM output matches Pure TS output', () => {
    for (let i = 0; i < NUM_FUZZ_ITERATIONS; i++) {
      const numPoints = Math.floor(Math.random() * 10) + 2;
      const dims = Math.floor(Math.random() * 3) + 1;
      const waypoints = generateRandomWaypoints(numPoints, dims);

      const wasmTraj = new LinearTrajectory(waypoints);

      // eslint-disable-next-line @typescript-eslint/no-explicit-any
      if (!(wasmTraj as any)._wasm) {
        console.warn('WASM backend not loaded! Parity test is comparing TS vs TS.');
      }

      const duration = wasmTraj.getDuration();
      const sampleTimes = [
        0,
        duration,
        Math.random() * duration,
        waypoints[1].time,
        -1.0,
        duration + 1.0
      ];

      for (const t of sampleTimes) {
        const wasmResult = wasmTraj.sample(t);
        const refResult = referenceLinearSample(waypoints, t);
        expectArraysClose(wasmResult, refResult, 1e-9);
      }
    }
  });
});

// ---------------------------------------------------------------------------
// Reference Implementations (Pure TS)
// ---------------------------------------------------------------------------

function referenceLinearSample(waypoints: Waypoint[], t: number): number[] {
  const dims = waypoints[0].positions.length;
  const result = new Array(dims).fill(0);
  const first = waypoints[0];
  const last = waypoints[waypoints.length - 1];

  if (t <= first.time) return [...first.positions];
  if (t >= last.time) return [...last.positions];

  // Find segment
  let lo = 0;
  let hi = waypoints.length - 2;
  let idx = 0;
  // Binary search
  while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (waypoints[mid].time <= t) {
          idx = mid;
          lo = mid + 1;
      } else {
          hi = mid - 1;
      }
  }
  // Adjust because binary search finds the element <= t, we want the segment start
  if (waypoints[idx].time > t && idx > 0) idx--;
  if (idx >= waypoints.length - 1) idx = waypoints.length - 2;

  const w0 = waypoints[idx];
  const w1 = waypoints[idx + 1];

  // Guard against division by zero if times are identical (should typically be validated)
  const dt = w1.time - w0.time;
  const alpha = dt === 0 ? 0 : (t - w0.time) / dt;

  for (let i = 0; i < dims; i++) {
      result[i] = w0.positions[i] + alpha * (w1.positions[i] - w0.positions[i]);
  }
  return result;
}

function solveTridiagonal(
  lower: number[],
  diag: number[],
  upper: number[],
  rhs: number[],
): number[] {
  const n = diag.length;
  // Clone to avoid side effects if modifying inputs
  const d = [...diag];
  const r = [...rhs];

  for (let i = 1; i < n; i++) {
    const w = lower[i] / d[i - 1];
    d[i] -= w * upper[i - 1];
    r[i] -= w * r[i - 1];
  }

  const x = new Array<number>(n).fill(0);
  x[n - 1] = r[n - 1] / d[n - 1];
  for (let i = n - 2; i >= 0; i--) {
    x[i] = (r[i] - upper[i] * x[i + 1]) / d[i];
  }
  return x;
}

interface SplineCoeffs {
  a: number;
  b: number;
  c: number;
  d: number;
}

function computeReferenceCoeffs(times: number[], values: number[]): SplineCoeffs[] {
  const n = times.length - 1;
  const h = times.slice(0, n).map((t, i) => times[i + 1] - t);
  const M = new Array(n + 1).fill(0);

  if (n >= 2) {
      const size = n - 1;
      const lower = new Array(size).fill(0);
      const diag = new Array(size).fill(0);
      const upper = new Array(size).fill(0);
      const rhs = new Array(size).fill(0);

      for (let i = 0; i < size; i++) {
          const idx = i + 1;
          diag[i] = 2 * (h[idx - 1] + h[idx]);
          if (i > 0) lower[i] = h[idx - 1];
          if (i < size - 1) upper[i] = h[idx];
          rhs[i] = 6 * ((values[idx + 1] - values[idx]) / h[idx] - (values[idx] - values[idx - 1]) / h[idx - 1]);
      }
      const x = solveTridiagonal(lower, diag, upper, rhs);
      for (let i = 0; i < size; i++) M[i + 1] = x[i];
  }

  const coeffs: SplineCoeffs[] = [];
  for (let i = 0; i < n; i++) {
      const a = values[i];
      const c = M[i] / 2;
      const d = (M[i + 1] - M[i]) / (6 * h[i]);
      const b = (values[i + 1] - values[i]) / h[i] - (h[i] * (2 * M[i] + M[i + 1])) / 6;
      coeffs.push({ a, b, c, d });
  }
  return coeffs;
}

function referenceCubicSplineSample(waypoints: Waypoint[], t: number): number[] {
  const times = waypoints.map(w => w.time);
  const dims = waypoints[0].positions.length;

  // Precompute coeffs (inefficient for sampling, but fine for reference test)
  const coeffsByDim = Array.from({ length: dims }, (_, dim) => {
      const values = waypoints.map(w => w.positions[dim]);
      return computeReferenceCoeffs(times, values);
  });

  const first = waypoints[0];
  const last = waypoints[waypoints.length - 1];
  const result = new Array(dims).fill(0);

  if (t <= first.time) return [...first.positions];
  if (t >= last.time) return [...last.positions];

  // Find segment
  let idx = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
      if (t >= waypoints[i].time && t < waypoints[i + 1].time) {
          idx = i;
          break;
      }
  }

  const dt = t - waypoints[idx].time;
  for (let i = 0; i < dims; i++) {
      const { a, b, c, d } = coeffsByDim[i][idx];
      result[i] = a + b * dt + c * dt * dt + d * dt * dt * dt;
  }
  return result;
}
