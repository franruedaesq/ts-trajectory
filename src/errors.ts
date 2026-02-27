/**
 * Base error class for all ts-trajectory errors.
 * Extend this class to catch any error thrown by this library.
 *
 * @example
 * ```typescript
 * import { TrajectoryError } from 'ts-trajectory';
 * try {
 *   builder.plan(waypoints, config);
 * } catch (err) {
 *   if (err instanceof TrajectoryError) {
 *     console.error('Trajectory error:', err.message);
 *   }
 * }
 * ```
 */
export class TrajectoryError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'TrajectoryError';
    // Restore prototype chain for instanceof checks across transpilation targets.
    Object.setPrototypeOf(this, new.target.prototype);
  }
}

/**
 * Thrown when input waypoints or planner configuration fail validation.
 * For example: fewer than 2 waypoints, non-increasing times, or mismatched dimensions.
 */
export class ValidationError extends TrajectoryError {
  constructor(message: string) {
    super(message);
    this.name = 'ValidationError';
    Object.setPrototypeOf(this, new.target.prototype);
  }
}

/**
 * Thrown when a generated trajectory violates the provided kinematic constraints
 * (`maxVelocity` or `maxAcceleration` in `PlannerConfig`).
 *
 * @example
 * ```typescript
 * import { KinematicConstraintError } from 'ts-trajectory';
 * try {
 *   builder.plan(waypoints, { interpolationType: 'cubic', maxVelocity: [1] });
 * } catch (err) {
 *   if (err instanceof KinematicConstraintError) {
 *     console.warn('Constraint violated:', err.message, 'dimension:', err.dimension);
 *   }
 * }
 * ```
 */
export class KinematicConstraintError extends TrajectoryError {
  /** Zero-based index of the dimension in which the constraint was violated. */
  readonly dimension: number;
  /** Zero-based index of the trajectory segment in which the constraint was violated. */
  readonly segment: number;

  constructor(message: string, dimension: number, segment: number) {
    super(message);
    this.name = 'KinematicConstraintError';
    this.dimension = dimension;
    this.segment = segment;
    Object.setPrototypeOf(this, new.target.prototype);
  }
}
