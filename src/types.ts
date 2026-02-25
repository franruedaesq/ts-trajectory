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
