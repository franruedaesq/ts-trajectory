/** Represents an N-dimensional state at a given time */
export interface Waypoint {
  time: number;
  positions: number[];
  velocities?: number[];
}

/** The immutable output of the planner, optimized for the render loop */
export interface Trajectory {
  getDuration(): number;
  sample(t: number): number[];
}

/** Configuration for generating a path */
export interface PlannerConfig {
  interpolationType: 'linear' | 'cubic';
  maxVelocity?: number[];
  maxAcceleration?: number[];
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
