import { Trajectory, Waypoint } from './types';

export class LinearTrajectory implements Trajectory {
  private readonly waypoints: Waypoint[];
  private readonly _result: number[];

  constructor(waypoints: Waypoint[]) {
    this.waypoints = waypoints;
    this._result = new Array(waypoints[0].positions.length);
  }

  getDuration(): number {
    return this.waypoints[this.waypoints.length - 1].time;
  }

  sample(t: number): number[] {
    const first = this.waypoints[0];
    const last = this.waypoints[this.waypoints.length - 1];
    const dims = this._result.length;

    if (t <= first.time) {
      for (let i = 0; i < dims; i++) this._result[i] = first.positions[i];
      return this._result;
    }
    if (t >= last.time) {
      for (let i = 0; i < dims; i++) this._result[i] = last.positions[i];
      return this._result;
    }

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

    for (let i = 0; i < dims; i++) {
      this._result[i] = w0.positions[i] + alpha * (w1.positions[i] - w0.positions[i]);
    }
    return this._result;
  }
}
