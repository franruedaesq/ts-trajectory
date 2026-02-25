/** Coefficients for a cubic polynomial f(t) = a*t^3 + b*t^2 + c*t + d */
export interface CubicCoeffs {
  a: number;
  b: number;
  c: number;
  d: number;
}

/**
 * Linearly interpolates a 1D value between p0 (at t0) and p1 (at t1).
 * @param p0 - value at t0
 * @param p1 - value at t1
 * @param t0 - start time
 * @param t1 - end time
 * @param t  - query time
 */
export function linearInterpolate1D(
  p0: number,
  p1: number,
  t0: number,
  t1: number,
  t: number,
): number {
  const alpha = (t - t0) / (t1 - t0);
  return p0 + alpha * (p1 - p0);
}

/**
 * Computes cubic Hermite polynomial coefficients for a single 1D segment.
 * The polynomial f(τ) = a*τ^3 + b*τ^2 + c*τ + d is parameterized over
 * τ ∈ [0, 1] where τ = (t - t0) / (t1 - t0).
 * Satisfies: f(0) = p0, f(1) = p1, f'(0) = v0, f'(1) = v1.
 * @param p0 - position at the start of the segment
 * @param p1 - position at the end of the segment
 * @param v0 - velocity at the start (in units per normalized interval)
 * @param v1 - velocity at the end (in units per normalized interval)
 */
export function cubicCoeffs1D(
  p0: number,
  p1: number,
  v0: number,
  v1: number,
): CubicCoeffs {
  const d = p0;
  const c = v0;
  const b = -3 * p0 + 3 * p1 - 2 * v0 - v1;
  const a = 2 * p0 - 2 * p1 + v0 + v1;
  return { a, b, c, d };
}

/**
 * Evaluates a cubic polynomial f(t) = a*t^3 + b*t^2 + c*t + d at the given t.
 * @param coeffs - polynomial coefficients
 * @param t      - query value
 */
export function evaluateCubic1D(coeffs: CubicCoeffs, t: number): number {
  const { a, b, c, d } = coeffs;
  return a * t * t * t + b * t * t + c * t + d;
}

/**
 * Solves a tridiagonal linear system A*x = rhs using the Thomas algorithm.
 * @param lower - subdiagonal coefficients (lower[0] is ignored)
 * @param diag  - main diagonal coefficients (modified in place; pass a copy if needed)
 * @param upper - superdiagonal coefficients (upper[n-1] is ignored)
 * @param rhs   - right-hand side vector (modified in place; pass a copy if needed)
 * @returns solution vector x
 */
export function solveTridiagonal(
  lower: number[],
  diag: number[],
  upper: number[],
  rhs: number[],
): number[] {
  const n = diag.length;
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
