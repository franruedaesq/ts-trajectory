# Test Plan: Rust/WASM Migration for Semantic State Estimator

## 1. Goal
The primary objective of this migration is to rewrite the core logic of the `SemanticStateEngine` (mapped to `TrajectoryBuilder` and `CubicSplineTrajectory` in the codebase) in Rust (compiled to WASM). This aims to improve performance and safety while maintaining **100% backward compatibility**.

Users who were using the previous pure-TypeScript version must be able to upgrade to the new version without any code changes. The public API (`TrajectoryBuilder`, `WorkerManager` abstractions, types) must remain identical.

## 2. Scope
The migration affects the following core components:
-   **`TrajectoryBuilder`**: The main entry point for creating trajectories.
-   **`CubicSplineTrajectory`**: The implementation of cubic spline interpolation.
-   **`LinearTrajectory`**: The implementation of linear interpolation.
-   **Internal Math Utilities**: Replaced by Rust implementations.

## 3. Testing Strategy

To ensure a seamless transition and verify the correctness of the new implementation, we will employ a multi-layered testing strategy.

### 3.1. Unit Tests (Existing)
*   **Action**: Run the existing test suite (`src/__tests__/*.test.ts`).
*   **Goal**: Verify that the new WASM-backed implementation passes all original test cases designed for the TypeScript version.
*   **Coverage**: Basic functionality, edge cases (min/max points), error handling (invalid inputs), and API contract adherence.

### 3.2. Parity / Fuzz Testing (New)
*   **Action**: Create a new test suite `src/__tests__/Parity.test.ts`.
*   **Goal**: Mathematically prove that the WASM implementation produces the *exact same* output (within floating-point epsilon) as the original TypeScript implementation for a vast range of inputs.
*   **Methodology**:
    1.  **Dual Instantiation**: Logic to instantiate both the WASM version and the Pure-TS fallback version (by mocking the WASM module loading).
    2.  **Fuzzing**: Generate thousands of random, valid `Waypoint[]` configurations:
        *   Varying number of waypoints (2 to 100+).
        *   Varying dimensions (1D, 2D, 3D, ... N-D).
        *   Varying time intervals (uniform, non-uniform, very small, very large).
        *   Varying position values (positive, negative, zero, large numbers).
    3.  **Sampling**: For each configuration, sample both implementations at random timestamps `t` (including `t < start`, `t > end`, and `t` exactly at waypoints).
    4.  **Assertion**: `|WASM_output - TS_output| < 1e-9`.

### 3.3. Edge Case Testing
*   **Action**: Specifically target potential WASM boundary issues.
*   **Scenarios**:
    *   **NaN/Infinity**: Ensure handling matches TS (or fails gracefully/identically).
    *   **Memory Management**: Although JS garbage collects the wrapper, we must ensure the underlying WASM memory is managed correctly (using `free()` where applicable in the internal implementation, though the TS wrapper handles this for the user).
    *   **Fallback Mechanism**: Verify that if WASM fails to load (e.g., in an environment without WASM support), the system automatically and silently falls back to the Pure-TS implementation.

### 3.4. Performance Benchmarking (Optional)
*   While not strictly a correctness test, we should verify that the WASM implementation is indeed faster or at least not slower for large datasets.

## 4. Execution Plan
1.  **Implement Parity Tests**: Write `src/__tests__/Parity.test.ts` implementing the strategy in 3.2.
2.  **Run All Tests**: Execute `npm test`.
3.  **Verify Fallback**: Manually force WASM failure in a test case to ensure the TS path is taken.

## 5. Success Criteria
*   All existing unit tests pass.
*   Parity tests pass with a strict epsilon.
*   No changes to the public API signature.
