import { CubicSplineTrajectory } from './CubicSplineTrajectory';
import { LinearTrajectory } from './LinearTrajectory';
import { PlannerConfig, Trajectory, Waypoint } from './types';

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

    if (config.interpolationType === 'cubic') {
      return new CubicSplineTrajectory(waypoints);
    }
    return new LinearTrajectory(waypoints);
  }
}
