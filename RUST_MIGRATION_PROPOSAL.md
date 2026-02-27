# Proposal: Migrating `ts-trajectory` Core Logic to Rust

This document outlines a strategy to migrate the core logic of `ts-trajectory` to Rust, leveraging WebAssembly (WASM) for performance while maintaining 100% backward compatibility with the existing TypeScript API.

## Goal

The primary goal is to rewrite the computationally intensive parts of the library (trajectory generation and sampling) in Rust to potentially improve performance and safety, without requiring existing users to refactor their code.

## Architecture

We will use **Rust** with **`wasm-bindgen`** to compile our logic into WebAssembly. On the TypeScript side, we will use the **Adapter Pattern** to wrap the WASM modules, exposing them through the exact same interfaces (`Trajectory`, `Waypoint`, `PlannerConfig`) that users currently use.

### High-Level Overview

1.  **Rust Crate**: A new Rust crate (e.g., `trajectory-core`) will implement the math and logic.
2.  **WASM Bindings**: `wasm-bindgen` will expose Rust structs and functions to JavaScript.
3.  **TypeScript Adapters**: The existing classes `LinearTrajectory` and `CubicSplineTrajectory` will be updated to act as wrappers around their Rust counterparts.

## Implementation Steps

### 1. Environment Setup

*   Initialize a new Rust crate within the project root (or a `native/` subdirectory).
*   Add `wasm-bindgen` to `Cargo.toml`.
*   Configure `wasm-pack` for building the project.

### 2. Rust Implementation (`src/lib.rs` & modules)

We will port the following core components:

*   **Data Structures**: Define `Waypoint` and `PlannerConfig` equivalents in Rust.
    *   *Note*: We may need to use `serde` and `serde-wasm-bindgen` for easy data passing between JS and Rust.
*   **MathUtils**: Port the `solveTridiagonal` function and cubic spline coefficient calculation.
*   **LinearTrajectory**: Implement the binary search and linear interpolation logic.
*   **CubicSplineTrajectory**: Implement the spline generation and sampling logic.

**Example Rust Struct (Conceptual):**

```rust
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RustCubicSpline {
    // Internal fields...
}

#[wasm_bindgen]
impl RustCubicSpline {
    #[wasm_bindgen(constructor)]
    pub fn new(waypoints: JsValue) -> Result<RustCubicSpline, JsValue> {
        // Deserialize waypoints and compute splines
    }

    pub fn sample(&self, t: f64) -> Vec<f64> {
        // Return sampled point
    }
}
```

### 3. TypeScript Integration (The "Adapter")

We will modify `src/CubicSplineTrajectory.ts` (and `LinearTrajectory.ts`) to wrap the WASM implementation.

**Current TypeScript:**

```typescript
export class CubicSplineTrajectory implements Trajectory {
  // ... pure TS implementation
  sample(t: number): number[] { /* ... */ }
}
```

**Proposed TypeScript (Adapter):**

```typescript
// Import the WASM module (exact import depends on build tool, e.g., vite/webpack/rollup)
import { RustCubicSpline } from '../pkg/trajectory_core';
import { Trajectory, Waypoint } from './types';

export class CubicSplineTrajectory implements Trajectory {
  private internal: RustCubicSpline;

  constructor(waypoints: Waypoint[]) {
    // Initialize the Rust object
    this.internal = new RustCubicSpline(waypoints);
  }

  getDuration(): number {
    return this.internal.get_duration();
  }

  sample(t: number): number[] {
    // Delegate to Rust
    return Array.from(this.internal.sample(t));
  }
}
```

This ensures that `import { CubicSplineTrajectory } from 'ts-trajectory'` continues to work exactly as before.

### 4. Build System

*   Update `package.json` scripts to include a `build:wasm` step (e.g., `wasm-pack build --target web`).
*   Ensure the generated WASM and JS bindings are properly included in the distribution (`dist/`).
*   Update `tsup.config.ts` or other bundler configs to handle WASM loading.

## Challenges & Considerations

1.  **WASM Initialization**: WASM modules often need to be asynchronously loaded (e.g., `init()`). We need a strategy to handle this, either by:
    *   Requiring an async `init()` call from the user (breaking change).
    *   Inlining the WASM (increases bundle size).
    *   Using synchronous instantiation if running in Node.js or compatible environments.
    *   *Recommendation*: For a "zero-refactor" goal, we might need to inline the WASM or use a sync loading mechanism if feasible, or wrap the factory pattern (`TrajectoryBuilder`) to be async. **However**, sticking to the existing synchronous constructor API is tricky with standard WASM on the web. We may need to investigate `wasm-pack` targets carefully.

2.  **Data Marshaling Overhead**: Passing large arrays of waypoints across the JS/WASM boundary has a cost. Since trajectory generation happens once, this is acceptable. Sampling (`sample(t)`) is high-frequency, so we must ensure the return type (small array of numbers) is optimized (e.g., returning a pointer to memory or a Float64Array view).

## Roadmap

1.  **Prototype**: Create a simple "Hello World" WASM module and call it from the existing TS code.
2.  **Port Math**: Port `solveTridiagonal` and verify correctness with tests.
3.  **Port Linear**: Implement `LinearTrajectory` in Rust.
4.  **Port Cubic**: Implement `CubicSplineTrajectory` in Rust.
5.  **Benchmarks**: Compare performance between pure TS and Rust+WASM to justify the complexity.
