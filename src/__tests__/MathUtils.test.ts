import { describe, expect, it } from 'vitest';
import {
  CubicCoeffs,
  cubicCoeffs1D,
  evaluateCubic1D,
  linearInterpolate1D,
} from '../MathUtils';

describe('linearInterpolate1D', () => {
  it('returns the exact interpolated position at t=0.5 between two points', () => {
    expect(linearInterpolate1D(0, 1, 0, 1, 0.5)).toBe(0.5);
  });

  it('returns p0 at t=t0', () => {
    expect(linearInterpolate1D(2, 8, 0, 1, 0)).toBe(2);
  });

  it('returns p1 at t=t1', () => {
    expect(linearInterpolate1D(2, 8, 0, 1, 1)).toBe(8);
  });

  it('correctly interpolates with non-zero start time', () => {
    expect(linearInterpolate1D(10, 20, 5, 15, 10)).toBe(15);
  });

  it('works with negative values', () => {
    expect(linearInterpolate1D(-4, 4, 0, 1, 0.5)).toBe(0);
  });
});

describe('cubicCoeffs1D', () => {
  it('calculates coefficients so that f(0)=p0 and f(1)=p1', () => {
    const coeffs: CubicCoeffs = cubicCoeffs1D(0, 1, 0, 0);
    // f(0) = d = p0
    expect(coeffs.d).toBe(0);
    // f(1) = a + b + c + d = p1
    expect(coeffs.a + coeffs.b + coeffs.c + coeffs.d).toBeCloseTo(1);
  });

  it('calculates coefficients so that f\'(0)=v0 and f\'(1)=v1', () => {
    const coeffs: CubicCoeffs = cubicCoeffs1D(0, 1, 0, 0);
    // f'(0) = c = v0
    expect(coeffs.c).toBe(0);
    // f'(1) = 3a + 2b + c = v1
    expect(3 * coeffs.a + 2 * coeffs.b + coeffs.c).toBeCloseTo(0);
  });

  it('returns the exact interpolated position at t=0.5 (zero velocities)', () => {
    const coeffs = cubicCoeffs1D(0, 1, 0, 0);
    // By symmetry with zero velocities, the midpoint equals (p0+p1)/2
    expect(evaluateCubic1D(coeffs, 0.5)).toBeCloseTo(0.5);
  });

  it('respects non-zero velocities in the coefficients', () => {
    const coeffs = cubicCoeffs1D(0, 1, 1, 0);
    // f'(0) = c = 1
    expect(coeffs.c).toBe(1);
    // f(0) = d = 0
    expect(coeffs.d).toBe(0);
    // f(1) = a + b + c + d = 1
    expect(coeffs.a + coeffs.b + coeffs.c + coeffs.d).toBeCloseTo(1);
  });
});

describe('evaluateCubic1D', () => {
  it('evaluates a known polynomial correctly', () => {
    // f(t) = t^3 => a=1, b=0, c=0, d=0
    const coeffs: CubicCoeffs = { a: 1, b: 0, c: 0, d: 0 };
    expect(evaluateCubic1D(coeffs, 2)).toBe(8);
    expect(evaluateCubic1D(coeffs, 0)).toBe(0);
    expect(evaluateCubic1D(coeffs, 1)).toBe(1);
  });

  it('evaluates constant polynomial correctly', () => {
    const coeffs: CubicCoeffs = { a: 0, b: 0, c: 0, d: 5 };
    expect(evaluateCubic1D(coeffs, 0.5)).toBe(5);
  });
});
