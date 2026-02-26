import { CubicSplineTrajectory } from './CubicSplineTrajectory';
import { LinearTrajectory } from './LinearTrajectory';
import { PlannerConfig, Trajectory, Waypoint } from './types';

const KINEMATIC_SAMPLE_COUNT = 100;

export class TrajectoryPlanner {
  plan(waypoints: Waypoint[], config: PlannerConfig): Trajectory {
    if (waypoints.length < 2) {
      throw new Error('At least 2 waypoints are required.');
    }

    const dims = waypoints[0].positions.length;
    for (let i = 0; i < waypoints.length; i++) {
      if (waypoints[i].positions.length !== dims) {
        throw new Error('All waypoints must have the same number of dimensions.');
      }
      if (i > 0 && waypoints[i].time <= waypoints[i - 1].time) {
        throw new Error('Waypoint times must be strictly increasing.');
      }
    }

    if (config.maxVelocity !== undefined && config.maxVelocity.length !== dims) {
      throw new Error('maxVelocity must have the same number of dimensions as waypoints.');
    }
    if (config.maxAcceleration !== undefined && config.maxAcceleration.length !== dims) {
      throw new Error('maxAcceleration must have the same number of dimensions as waypoints.');
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

    return trajectory;
  }

  private validateVelocity(
    trajectory: Trajectory,
    waypoints: Waypoint[],
    maxVelocity: number[],
  ): void {
    if (trajectory instanceof CubicSplineTrajectory) {
      const startTime = waypoints[0].time;
      const duration = trajectory.getDuration() - startTime;
      for (let s = 0; s <= KINEMATIC_SAMPLE_COUNT; s++) {
        const t = startTime + (s / KINEMATIC_SAMPLE_COUNT) * duration;
        const vel = trajectory.sampleDerivative(t);
        for (let dim = 0; dim < vel.length; dim++) {
          if (Math.abs(vel[dim]) > maxVelocity[dim]) {
            throw new Error(
              `Trajectory exceeds maxVelocity in dimension ${dim}: ` +
                `|${vel[dim]}| > ${maxVelocity[dim]} at t=${t}.`,
            );
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
            throw new Error(
              `Trajectory exceeds maxVelocity in dimension ${dim}: ` +
                `${vel} > ${maxVelocity[dim]} in segment ${i}.`,
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
      const startTime = waypoints[0].time;
      const duration = trajectory.getDuration() - startTime;
      for (let s = 0; s <= KINEMATIC_SAMPLE_COUNT; s++) {
        const t = startTime + (s / KINEMATIC_SAMPLE_COUNT) * duration;
        const accel = trajectory.sampleSecondDerivative(t);
        for (let dim = 0; dim < accel.length; dim++) {
          if (Math.abs(accel[dim]) > maxAcceleration[dim]) {
            throw new Error(
              `Trajectory exceeds maxAcceleration in dimension ${dim}: ` +
                `|${accel[dim]}| > ${maxAcceleration[dim]} at t=${t}.`,
            );
          }
        }
      }
    }
  }
}
