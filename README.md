# ts-trajectory

A lightweight, highly performant, zero-dependency TypeScript library for generating multi-dimensional, time-parameterized trajectories.

Ideal for robotics, animation, game development, and any application requiring smooth, continuous motion planning between waypoints.

[![CI](https://github.com/franruedaesq/ts-trajectory/actions/workflows/ci.yml/badge.svg)](https://github.com/franruedaesq/ts-trajectory/actions/workflows/ci.yml)
[![npm version](https://img.shields.io/npm/v/ts-trajectory.svg)](https://www.npmjs.com/package/ts-trajectory)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Features

- **Zero Dependencies:** Keeps your bundle size small and avoids dependency hell.
- **Multi-dimensional:** Supports creating trajectories in 1D, 2D, 3D, or N-dimensional space.
- **Interpolation Types:**
  - `linear`: Constant velocity between waypoints.
  - `cubic`: Cubic spline interpolation for smooth velocity and continuous acceleration.
- **Kinematic Constraints:** Validates trajectories against maximum velocity and acceleration limits.
- **Observability Hooks:** Built-in `onPlanStart`, `onPlanComplete`, and `onPlanError` callbacks for logging and debugging.
- **Custom Error Classes:** Exported `ValidationError` and `KinematicConstraintError` for precise error handling.
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

## Error Handling

All errors thrown by this library extend the exported `TrajectoryError` base class, allowing precise `catch` blocks:

```typescript
import {
  TrajectoryBuilder,
  TrajectoryError,
  ValidationError,
  KinematicConstraintError,
} from 'ts-trajectory';

const builder = new TrajectoryBuilder();
try {
  builder.plan(waypoints, config);
} catch (err) {
  if (err instanceof KinematicConstraintError) {
    console.error(`Constraint violated in dimension ${err.dimension}, segment ${err.segment}`);
  } else if (err instanceof ValidationError) {
    console.error('Bad input:', err.message);
  } else if (err instanceof TrajectoryError) {
    console.error('Library error:', err.message);
  }
}
```

## Observability

Pass lifecycle callbacks to `TrajectoryBuilder` to log or instrument trajectory planning:

```typescript
import { TrajectoryBuilder } from 'ts-trajectory';

const builder = new TrajectoryBuilder({
  onPlanStart: (waypoints, config) => {
    console.log(`[plan] Starting ${config.interpolationType} trajectory with ${waypoints.length} waypoints`);
  },
  onPlanComplete: (trajectory, durationMs) => {
    console.log(`[plan] Done in ${durationMs.toFixed(2)} ms. Duration: ${trajectory.getDuration()}s`);
  },
  onPlanError: (error) => {
    console.warn('[plan] Failed:', error.message);
  },
});

const trajectory = builder.plan(waypoints, config);
```

## API Reference

### `TrajectoryBuilder`
The main class responsible for validating inputs and generating the trajectory.

#### `new TrajectoryBuilder(options?)`
Creates a new builder. The optional `options` object accepts observability hooks:

| Hook | Signature | Description |
|------|-----------|-------------|
| `onPlanStart` | `(waypoints, config) => void` | Called before planning begins. |
| `onPlanComplete` | `(trajectory, durationMs) => void` | Called after successful planning. |
| `onPlanError` | `(error: TrajectoryError) => void` | Called when planning throws; error is still re-thrown. |

#### `plan(waypoints: Waypoint[], config: PlannerConfig): Trajectory`
Generates a trajectory from the given waypoints.
- Throws a `ValidationError` if `maxVelocity`/`maxAcceleration` dimensions don't match, waypoint times are not strictly increasing, or dimensions are mismatched.
- Throws a `KinematicConstraintError` when a trajectory exceeds `maxVelocity` or `maxAcceleration`.

### `Trajectory`
The immutable output of the `TrajectoryBuilder`, designed for fast, repeated sampling in render or control loops.

#### `getDuration(): number`
Returns the total duration of the trajectory (from the first waypoint's time to the last).

#### `sample(time: number): number[]`
Returns the interpolated position array at the exact given `time`. If `time` is out of bounds, it clamps to the first or last waypoint.

### Error Classes

| Class | Extends | Description |
|-------|---------|-------------|
| `TrajectoryError` | `Error` | Base class for all library errors. |
| `ValidationError` | `TrajectoryError` | Thrown for malformed inputs. |
| `KinematicConstraintError` | `TrajectoryError` | Thrown when kinematic limits are exceeded. Has `.dimension` and `.segment` properties. |

### Types

#### `Waypoint`
```typescript
interface Waypoint {
  time: number;          // Time in seconds (or custom unit)
  positions: number[];   // N-dimensional position array
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

#### `TrajectoryBuilderOptions`
```typescript
interface TrajectoryBuilderOptions {
  onPlanStart?: (waypoints: Readonly<Waypoint[]>, config: Readonly<PlannerConfig>) => void;
  onPlanComplete?: (trajectory: Trajectory, durationMs: number) => void;
  onPlanError?: (error: TrajectoryError) => void;
}
```

## Use Cases

- **Robotics Path Planning:** Generating smooth joint trajectories (N-dimensional) for robot arms, ensuring they don't exceed motor velocity/acceleration limits.
- **Game Development:** Moving cameras or entities smoothly along a predefined path over time using spline interpolation.
- **UI Animation:** Creating complex, multi-property CSS/Canvas animations that need to hit specific keyframes at exact times.

## License
[MIT](LICENSE)
