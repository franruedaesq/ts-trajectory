# ts-trajectory

A lightweight, highly performant, zero-dependency TypeScript library for generating multi-dimensional, time-parameterized trajectories.

Ideal for robotics, animation, game development, and any application requiring smooth, continuous motion planning between waypoints.

## Features

- **Zero Dependencies:** Keeps your bundle size small and avoids dependency hell.
- **Multi-dimensional:** Supports creating trajectories in 1D, 2D, 3D, or N-dimensional space.
- **Interpolation Types:**
  - `linear`: Constant velocity between waypoints.
  - `cubic`: Cubic spline interpolation for smooth velocity and continuous acceleration.
- **Kinematic Constraints:** Validates trajectories against maximum velocity and acceleration limits.
- **Optimized for Render Loops:** Pre-computes splines for $O(1)$ sampling performance per dimension during runtime.
- **Strictly Typed:** First-class TypeScript support.
- **Universal:** Works in Node.js and the browser (ESM/CommonJS compatible).

## Installation

```bash
npm install ts-trajectory
```

## Quick Start
```typescript
import { TrajectoryBuilder } from 'ts-trajectory';
import type { Waypoint, PlannerConfig } from 'ts-trajectory';

// Define your waypoints (time, positions array)
const waypoints: Waypoint[] = [
  { time: 0, positions: [0, 0] },
  { time: 1, positions: [10, 5] },
  { time: 2, positions: [10, 10] },
];

const config: PlannerConfig = {
  interpolationType: 'cubic',
  maxVelocity: [15, 10],      // Optional: enforce velocity limits per dimension
  maxAcceleration: [30, 20],  // Optional: enforce acceleration limits per dimension
};

const builder = new TrajectoryBuilder();
const trajectory = builder.plan(waypoints, config);

// Sample the trajectory at any given time
const t = 1.5;
const currentPosition = trajectory.sample(t);

console.log(`Position at t=${t}:`, currentPosition);
// Output will be an array of length 2 (since positions are 2D)
```

## API Reference

### `TrajectoryBuilder`
The main class responsible for validating inputs and generating the trajectory.

#### `plan(waypoints: Waypoint[], config: PlannerConfig): Trajectory`
Generates a trajectory from the given waypoints.
- Throws an error if `maxVelocity` or `maxAcceleration` constraints are violated.
- Throws an error if waypoint times are not strictly increasing.
- Throws an error if the dimensions of waypoints are mismatched.

### `Trajectory`
The immutable output of the `TrajectoryBuilder`, designed for fast, repeated sampling in render or control loops.

#### `getDuration(): number`
Returns the total duration of the trajectory (from the first waypoint's time to the last).

#### `sample(time: number): number[]`
Returns the interpolated position array at the exact given `time`. If `time` is out of bounds, it clamps to the first or last waypoint.

### Types

#### `Waypoint`
```typescript
interface Waypoint {
  time: number;       // Time in seconds (or custom unit)
  positions: number[]; // N-dimensional position array
  velocities?: number[]; // (V2 Roadmap - Currently unsupported)
}
```

#### `PlannerConfig`
```typescript
interface PlannerConfig {
  interpolationType: 'linear' | 'cubic';
  maxVelocity?: number[];     // Max absolute velocity per dimension
  maxAcceleration?: number[]; // Max absolute acceleration per dimension
}
```

## Use Cases

- **Robotics Path Planning:** Generating smooth joint trajectories (N-dimensional) for robot arms, ensuring they don't exceed motor velocity/acceleration limits.
- **Game Development:** Moving cameras or entities smoothly along a predefined path over time using spline interpolation.
- **UI Animation:** Creating complex, multi-property CSS/Canvas animations that need to hit specific keyframes at exact times.

## License
[MIT](LICENSE)
