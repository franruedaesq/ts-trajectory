import type { TrajectoryError } from './errors';

/** Represents an N-dimensional state at a given time */
export interface Waypoint {
  /** Time value (e.g. seconds). Must be strictly increasing across waypoints. */
  time: number;
  /** N-dimensional position vector. All waypoints must share the same length. */
  positions: number[];
  /** Optional velocity hints (V2 roadmap – not yet supported). */
  velocities?: number[];
}

/** The immutable output of the planner, optimized for the render loop */
export interface Trajectory {
  /** Returns the total duration of the trajectory (last waypoint time minus first). */
  getDuration(): number;
  /**
   * Returns the interpolated position array at the given time `t`.
   * If `t` is before the first waypoint it clamps to the start;
   * if `t` is after the last waypoint it clamps to the end.
   * @param t - Query time.
   * @returns Pre-allocated position array (same reference on every call).
   */
  sample(t: number): number[];
}

/** Configuration for generating a path */
export interface PlannerConfig {
  /** Interpolation algorithm to use. */
  interpolationType: 'linear' | 'cubic';
  /** Maximum absolute velocity per dimension. Length must equal waypoint dimension count. */
  maxVelocity?: number[];
  /** Maximum absolute acceleration per dimension. Length must equal waypoint dimension count. */
  maxAcceleration?: number[];
}

/**
 * Observability hooks for `TrajectoryBuilder`.
 * All callbacks are optional; provide only those you need.
 *
 * @example
 * ```typescript
 * const builder = new TrajectoryBuilder({
 *   onPlanStart: (wps, cfg) => console.log('Planning…', cfg.interpolationType),
 *   onPlanComplete: (traj, ms) => console.log(`Done in ${ms.toFixed(2)} ms`),
 * });
 * ```
 */
export interface TrajectoryBuilderOptions {
  /**
   * Called immediately before trajectory computation begins.
   * @param waypoints - The waypoints passed to `plan()`.
   * @param config    - The planner config passed to `plan()`.
   */
  onPlanStart?: (waypoints: Readonly<Waypoint[]>, config: Readonly<PlannerConfig>) => void;
  /**
   * Called after trajectory computation succeeds.
   * @param trajectory  - The resulting trajectory.
   * @param durationMs  - Wall-clock time spent computing the trajectory, in milliseconds.
   */
  onPlanComplete?: (trajectory: Trajectory, durationMs: number) => void;
  /**
   * Called when `plan()` throws due to a validation or constraint error.
   * The error is still re-thrown after this callback returns.
   * @param error - The error that caused the failure.
   */
  onPlanError?: (error: TrajectoryError) => void;
}

/** Returns true if value is a valid Waypoint */
export function isWaypoint(value: unknown): value is Waypoint {
  if (typeof value !== 'object' || value === null) return false;
  const v = value as Record<string, unknown>;
  if (!Number.isFinite(v.time as number)) return false;
  if (!Array.isArray(v.positions) || v.positions.length === 0 || !v.positions.every((p) => Number.isFinite(p))) return false;
  if (v.velocities !== undefined) {
    if (!Array.isArray(v.velocities) || !v.velocities.every((p) => Number.isFinite(p)))
      return false;
  }
  return true;
}

/** Returns true if value implements the Trajectory interface */
export function isTrajectory(value: unknown): value is Trajectory {
  if (typeof value !== 'object' || value === null) return false;
  const v = value as Record<string, unknown>;
  return typeof v.getDuration === 'function' && typeof v.sample === 'function';
}

/** Returns true if value is a valid PlannerConfig */
export function isPlannerConfig(value: unknown): value is PlannerConfig {
  if (typeof value !== 'object' || value === null) return false;
  const v = value as Record<string, unknown>;
  if (v.interpolationType !== 'linear' && v.interpolationType !== 'cubic') return false;
  if (v.maxVelocity !== undefined) {
    if (!Array.isArray(v.maxVelocity) || v.maxVelocity.length === 0 || !v.maxVelocity.every((x) => Number.isFinite(x)))
      return false;
  }
  if (v.maxAcceleration !== undefined) {
    if (
      !Array.isArray(v.maxAcceleration) ||
      v.maxAcceleration.length === 0 ||
      !v.maxAcceleration.every((x) => Number.isFinite(x))
    )
      return false;
  }
  return true;
}
