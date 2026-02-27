import { Trajectory, Waypoint } from './types';

// Attempt to load the Rust/WASM backend. If unavailable (e.g. WASM not
// supported in the current environment), the pure-TypeScript implementation
// is used transparently.
interface WasmLinearTrajectory {
  get_duration(): number;
  sample(t: number): Float64Array;
  free(): void;
}
interface WasmLinearTrajectoryConstructor {
  new (waypoints: unknown): WasmLinearTrajectory;
}
let WasmBackend: WasmLinearTrajectoryConstructor | null = null;
try {
  // eslint-disable-next-line @typescript-eslint/no-require-imports
  const mod = require('../pkg-node/trajectory_core') as {
    LinearTrajectory: WasmLinearTrajectoryConstructor;
  };
  WasmBackend = mod.LinearTrajectory;
} catch {
  // WASM unavailable; pure-TS fallback will be used.
}

export class LinearTrajectory implements Trajectory {
  private readonly waypoints: Waypoint[];
  private readonly _result: number[];
  private _lastSegmentIndex: number = 0;
  private readonly _wasm: WasmLinearTrajectory | null;

  constructor(waypoints: Waypoint[]) {
    this.waypoints = waypoints;
    this._result = new Array(waypoints[0].positions.length);
    this._wasm = WasmBackend ? new WasmBackend(waypoints) : null;
  }

  private _findSegment(t: number): number {
    const idx = this._lastSegmentIndex;
    if (idx < this.waypoints.length - 1 && this.waypoints[idx].time <= t && t < this.waypoints[idx + 1].time) {
      return idx;
    }
    let lo = 0;
    let hi = this.waypoints.length - 2;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.waypoints[mid].time <= t) lo = mid;
      else hi = mid - 1;
    }
    this._lastSegmentIndex = lo;
    return lo;
  }

  getDuration(): number {
    if (this._wasm) return this._wasm.get_duration();
    return this.waypoints[this.waypoints.length - 1].time;
  }

  sample(t: number): number[] {
    if (this._wasm) {
      const raw = this._wasm.sample(t);
      for (let i = 0; i < this._result.length; i++) this._result[i] = raw[i];
      return this._result;
    }

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

    // Find segment using cache, fall back to binary search
    const lo = this._findSegment(t);

    const w0 = this.waypoints[lo];
    const w1 = this.waypoints[lo + 1];
    const alpha = (t - w0.time) / (w1.time - w0.time);

    for (let i = 0; i < dims; i++) {
      this._result[i] = w0.positions[i] + alpha * (w1.positions[i] - w0.positions[i]);
    }
    return this._result;
  }
}
