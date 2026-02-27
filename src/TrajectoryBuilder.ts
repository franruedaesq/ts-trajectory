import { CubicSplineTrajectory } from './CubicSplineTrajectory';
import { KinematicConstraintError, ValidationError } from './errors';
import { LinearTrajectory } from './LinearTrajectory';
import { PlannerConfig, Trajectory, TrajectoryBuilderOptions, Waypoint } from './types';

/**
 * Builds a trajectory from waypoints and a planner config.
 *
 * Validates inputs and enforces optional kinematic constraints
 * (`maxVelocity`, `maxAcceleration`). Throws a {@link ValidationError}
 * for malformed inputs and a {@link KinematicConstraintError} when a
 * generated trajectory violates kinematic limits.
 *
 * @example
 * ```typescript
 * import { TrajectoryBuilder } from 'ts-trajectory';
 *
 * const builder = new TrajectoryBuilder({
 *   onPlanComplete: (traj, ms) => console.log(`planned in ${ms.toFixed(1)} ms`),
 * });
 *
 * const trajectory = builder.plan(
 *   [
 *     { time: 0, positions: [0, 0] },
 *     { time: 1, positions: [10, 5] },
 *   ],
 *   { interpolationType: 'cubic' },
 * );
 * ```
 */
export class TrajectoryBuilder {
  private readonly _options: TrajectoryBuilderOptions;

  /**
   * @param options - Optional observability hooks. All callbacks are optional.
   */
  constructor(options: TrajectoryBuilderOptions = {}) {
    this._options = options;
  }

  /**
   * Plans a trajectory through the supplied waypoints according to `config`.
   *
   * @param waypoints - Ordered list of waypoints. Must contain at least 2 entries
   *   with strictly increasing `time` values and consistent `positions` length.
   * @param config    - Interpolation type and optional kinematic constraints.
   * @returns An immutable {@link Trajectory} ready for repeated sampling.
   * @throws {@link ValidationError}         on malformed inputs.
   * @throws {@link KinematicConstraintError} when a kinematic limit is exceeded.
   */
  plan(waypoints: Waypoint[], config: PlannerConfig): Trajectory {
    this._options.onPlanStart?.(waypoints, config);
    const t0 = performance.now();

    try {
      if (waypoints.length < 2) {
        throw new ValidationError('At least 2 waypoints are required.');
      }

      for (const waypoint of waypoints) {
        if (waypoint.velocities !== undefined) {
          throw new ValidationError('Waypoint velocities are not yet supported in V1.');
        }
      }

      const dims = waypoints[0].positions.length;
      for (let i = 0; i < waypoints.length; i++) {
        if (waypoints[i].positions.length !== dims) {
          throw new ValidationError('All waypoints must have the same number of dimensions.');
        }
        if (i > 0 && waypoints[i].time <= waypoints[i - 1].time) {
          throw new ValidationError('Waypoint times must be strictly increasing.');
        }
      }

      if (config.maxVelocity !== undefined && config.maxVelocity.length !== dims) {
        throw new ValidationError('maxVelocity must have the same number of dimensions as waypoints.');
      }
      if (config.maxAcceleration !== undefined && config.maxAcceleration.length !== dims) {
        throw new ValidationError('maxAcceleration must have the same number of dimensions as waypoints.');
      }

      let trajectory: Trajectory;
      if (config.interpolationType === 'cubic') {
        trajectory = new CubicSplineTrajectory(waypoints);
      } else {
        trajectory = new LinearTrajectory(waypoints);
      }

      if (config.maxVelocity !== undefined) {
        this.validateVelocity(trajectory, waypoints, config.maxVelocity);
      }
      if (config.maxAcceleration !== undefined) {
        this.validateAcceleration(trajectory, waypoints, config.maxAcceleration);
      }

      this._options.onPlanComplete?.(trajectory, performance.now() - t0);
      return trajectory;
    } catch (err) {
      if (err instanceof ValidationError || err instanceof KinematicConstraintError) {
        this._options.onPlanError?.(err);
      }
      throw err;
    }
  }

  private validateVelocity(
    trajectory: Trajectory,
    waypoints: Waypoint[],
    maxVelocity: number[],
  ): void {
    if (trajectory instanceof CubicSplineTrajectory) {
      // Analytical O(1) validation per segment.
      // f'(dt) = b + 2c*dt + 3d*dt^2 (quadratic); peak occurs at dt = -c/(3d) when d != 0.
      const coeffsByDim = trajectory.getCoeffsByDim();
      const dims = maxVelocity.length;
      for (let seg = 0; seg < waypoints.length - 1; seg++) {
        const h = waypoints[seg + 1].time - waypoints[seg].time;
        for (let dim = 0; dim < dims; dim++) {
          const { b, c, d } = coeffsByDim[dim][seg];
          const checkPoints = [0, h];
          if (d !== 0) {
            const dtPeak = -c / (3 * d);
            if (dtPeak > 0 && dtPeak < h) checkPoints.push(dtPeak);
          }
          for (const dt of checkPoints) {
            const vel = b + 2 * c * dt + 3 * d * dt * dt;
            if (Math.abs(vel) > maxVelocity[dim]) {
              throw new KinematicConstraintError(
                `Trajectory exceeds maxVelocity in dimension ${dim}: ` +
                  `|${vel}| > ${maxVelocity[dim]} in segment ${seg}.`,
                dim,
                seg,
              );
            }
          }
        }
      }
    } else {
      // Linear trajectory: velocity is constant per segment
      for (let i = 0; i < waypoints.length - 1; i++) {
        const dt = waypoints[i + 1].time - waypoints[i].time;
        for (let dim = 0; dim < maxVelocity.length; dim++) {
          const vel = Math.abs(waypoints[i + 1].positions[dim] - waypoints[i].positions[dim]) / dt;
          if (vel > maxVelocity[dim]) {
            throw new KinematicConstraintError(
              `Trajectory exceeds maxVelocity in dimension ${dim}: ` +
                `${vel} > ${maxVelocity[dim]} in segment ${i}.`,
              dim,
              i,
            );
          }
        }
      }
    }
  }

  private validateAcceleration(
    trajectory: Trajectory,
    waypoints: Waypoint[],
    maxAcceleration: number[],
  ): void {
    // Linear trajectories have zero acceleration within segments; no validation needed.
    if (trajectory instanceof CubicSplineTrajectory) {
      // Analytical O(1) validation per segment.
      // f''(dt) = 2c + 6d*dt (linear); peak occurs at segment boundaries.
      const coeffsByDim = trajectory.getCoeffsByDim();
      const dims = maxAcceleration.length;
      for (let seg = 0; seg < waypoints.length - 1; seg++) {
        const h = waypoints[seg + 1].time - waypoints[seg].time;
        for (let dim = 0; dim < dims; dim++) {
          const { c, d } = coeffsByDim[dim][seg];
          for (const dt of [0, h]) {
            const accel = 2 * c + 6 * d * dt;
            if (Math.abs(accel) > maxAcceleration[dim]) {
              throw new KinematicConstraintError(
                `Trajectory exceeds maxAcceleration in dimension ${dim}: ` +
                  `|${accel}| > ${maxAcceleration[dim]} in segment ${seg}.`,
                dim,
                seg,
              );
            }
          }
        }
      }
    }
  }
}
