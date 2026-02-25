import { Trajectory, Waypoint } from './types';

export class LinearTrajectory implements Trajectory {
  private readonly waypoints: Waypoint[];

  constructor(waypoints: Waypoint[]) {
    this.waypoints = waypoints;
  }

  getDuration(): number {
    return this.waypoints[this.waypoints.length - 1].time;
  }

  sample(t: number): number[] {
    const first = this.waypoints[0];
    const last = this.waypoints[this.waypoints.length - 1];

    if (t <= first.time) return [...first.positions];
    if (t >= last.time) return [...last.positions];

    // Binary search for segment
    let lo = 0;
    let hi = this.waypoints.length - 2;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.waypoints[mid].time <= t) lo = mid;
      else hi = mid - 1;
    }

    const w0 = this.waypoints[lo];
    const w1 = this.waypoints[lo + 1];
    const alpha = (t - w0.time) / (w1.time - w0.time);

    return w0.positions.map((p, i) => p + alpha * (w1.positions[i] - p));
  }
}
