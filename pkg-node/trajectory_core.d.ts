/* tslint:disable */
/* eslint-disable */

export class CubicSplineTrajectory {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Returns spline coefficients for use by the TypeScript adapter for validation.
     * Returns a flat array: for each dim, for each segment: [a, b, c, d, ...]
     */
    get_coeffs_flat(): Float64Array;
    get_duration(): number;
    constructor(waypoints_js: any);
    sample(t: number): Float64Array;
    sample_derivative(t: number): Float64Array;
    sample_second_derivative(t: number): Float64Array;
}

export class LinearTrajectory {
    free(): void;
    [Symbol.dispose](): void;
    get_duration(): number;
    constructor(waypoints_js: any);
    sample(t: number): Float64Array;
}
