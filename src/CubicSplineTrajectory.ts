import { Trajectory, Waypoint } from './types';

interface SplineCoeffs {
  a: number;
  b: number;
  c: number;
  d: number;
}

function computeCoeffs(times: number[], values: number[]): SplineCoeffs[] {
  const n = times.length - 1;
  const h = times.slice(0, n).map((t, i) => times[i + 1] - t);

  // Natural spline: M[0] = M[n] = 0
  // Solve tridiagonal system for M[1]..M[n-1]
  const M = new Array<number>(n + 1).fill(0);

  if (n >= 2) {
    const size = n - 1;
    const lower = new Array<number>(size).fill(0);
    const diag = new Array<number>(size).fill(0);
    const upper = new Array<number>(size).fill(0);
    const rhs = new Array<number>(size).fill(0);

    for (let i = 0; i < size; i++) {
      const idx = i + 1;
      diag[i] = 2 * (h[idx - 1] + h[idx]);
      if (i > 0) lower[i] = h[idx - 1];
      if (i < size - 1) upper[i] = h[idx];
      rhs[i] =
        6 *
        ((values[idx + 1] - values[idx]) / h[idx] -
          (values[idx] - values[idx - 1]) / h[idx - 1]);
    }

    // Thomas algorithm
    const c2 = [...upper];
    const d2 = [...rhs];
    for (let i = 1; i < size; i++) {
      const w = lower[i] / diag[i - 1];
      diag[i] -= w * c2[i - 1];
      d2[i] -= w * d2[i - 1];
    }
    const x = new Array<number>(size).fill(0);
    x[size - 1] = d2[size - 1] / diag[size - 1];
    for (let i = size - 2; i >= 0; i--) {
      x[i] = (d2[i] - c2[i] * x[i + 1]) / diag[i];
    }
    for (let i = 0; i < size; i++) {
      M[i + 1] = x[i];
    }
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

export class CubicSplineTrajectory implements Trajectory {
  private readonly waypoints: Waypoint[];
  private readonly coeffsByDim: SplineCoeffs[][];

  constructor(waypoints: Waypoint[]) {
    this.waypoints = waypoints;
    const times = waypoints.map((w) => w.time);
    const dims = waypoints[0].positions.length;
    this.coeffsByDim = Array.from({ length: dims }, (_, dim) => {
      const values = waypoints.map((w) => w.positions[dim]);
      return computeCoeffs(times, values);
    });
  }

  getDuration(): number {
    return this.waypoints[this.waypoints.length - 1].time;
  }

  sample(t: number): number[] {
    const first = this.waypoints[0];
    const last = this.waypoints[this.waypoints.length - 1];

    if (t <= first.time) return [...first.positions];
    if (t >= last.time) return [...last.positions];

    // Binary search for segment
    let lo = 0;
    let hi = this.waypoints.length - 2;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.waypoints[mid].time <= t) lo = mid;
      else hi = mid - 1;
    }

    const dt = t - this.waypoints[lo].time;
    return this.coeffsByDim.map((coeffs) => {
      const { a, b, c, d } = coeffs[lo];
      return a + b * dt + c * dt * dt + d * dt * dt * dt;
    });
  }
}
