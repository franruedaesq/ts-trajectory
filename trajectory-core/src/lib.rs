use wasm_bindgen::prelude::*;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waypoint {
    pub time: f64,
    pub positions: Vec<f64>,
}

// ---------------------------------------------------------------------------
// MathUtils
// ---------------------------------------------------------------------------

/// Solves a tridiagonal linear system A*x = rhs using the Thomas algorithm.
/// `lower`, `diag`, `upper`, and `rhs` are modified in place.
fn solve_tridiagonal(
    lower: &mut Vec<f64>,
    diag: &mut Vec<f64>,
    upper: &mut Vec<f64>,
    rhs: &mut Vec<f64>,
) -> Result<Vec<f64>, String> {
    let n = diag.len();
    for i in 1..n {
        if diag[i - 1].abs() < f64::EPSILON {
            return Err(format!("solve_tridiagonal: near-zero pivot at index {}", i - 1));
        }
        let w = lower[i] / diag[i - 1];
        diag[i] -= w * upper[i - 1];
        rhs[i] -= w * rhs[i - 1];
    }

    let mut x = vec![0.0f64; n];
    if diag[n - 1].abs() < f64::EPSILON {
        return Err(format!("solve_tridiagonal: near-zero pivot at index {}", n - 1));
    }
    x[n - 1] = rhs[n - 1] / diag[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = (rhs[i] - upper[i] * x[i + 1]) / diag[i];
    }
    Ok(x)
}

// ---------------------------------------------------------------------------
// LinearTrajectory (WASM-exported)
// ---------------------------------------------------------------------------

#[wasm_bindgen]
pub struct LinearTrajectory {
    waypoints: Vec<Waypoint>,
    dims: usize,
    last_segment_index: usize,
}

#[wasm_bindgen]
impl LinearTrajectory {
    #[wasm_bindgen(constructor)]
    pub fn new(waypoints_js: JsValue) -> Result<LinearTrajectory, JsValue> {
        let waypoints: Vec<Waypoint> = serde_wasm_bindgen::from_value(waypoints_js)
            .map_err(|e| JsValue::from_str(&e.to_string()))?;

        if waypoints.len() < 2 {
            return Err(JsValue::from_str("At least 2 waypoints are required."));
        }
        let dims = waypoints[0].positions.len();
        for (i, wp) in waypoints.iter().enumerate() {
            if wp.positions.len() != dims {
                return Err(JsValue::from_str(
                    "All waypoints must have the same number of dimensions.",
                ));
            }
            if i > 0 && waypoints[i].time <= waypoints[i - 1].time {
                return Err(JsValue::from_str(
                    "Waypoint times must be strictly increasing.",
                ));
            }
        }
        Ok(LinearTrajectory { waypoints, dims, last_segment_index: 0 })
    }

    pub fn get_duration(&self) -> f64 {
        self.waypoints[self.waypoints.len() - 1].time
    }

    fn find_segment(&mut self, t: f64) -> usize {
        let idx = self.last_segment_index;
        if idx < self.waypoints.len() - 1
            && self.waypoints[idx].time <= t
            && t < self.waypoints[idx + 1].time
        {
            return idx;
        }
        let mut lo = 0usize;
        let mut hi = self.waypoints.len() - 2;
        while lo < hi {
            let mid = (lo + hi + 1) / 2;
            if self.waypoints[mid].time <= t {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        self.last_segment_index = lo;
        lo
    }

    pub fn sample(&mut self, t: f64) -> Vec<f64> {
        let n = self.waypoints.len();
        let first = &self.waypoints[0];
        let last = &self.waypoints[n - 1];

        if t <= first.time {
            return first.positions.clone();
        }
        if t >= last.time {
            return last.positions.clone();
        }

        let lo = self.find_segment(t);
        let w0 = &self.waypoints[lo];
        let w1 = &self.waypoints[lo + 1];
        let alpha = (t - w0.time) / (w1.time - w0.time);

        let mut result = vec![0.0f64; self.dims];
        for i in 0..self.dims {
            result[i] = w0.positions[i] + alpha * (w1.positions[i] - w0.positions[i]);
        }
        result
    }
}

// ---------------------------------------------------------------------------
// CubicSplineTrajectory (WASM-exported)
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct SplineCoeffs {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
}

fn compute_coeffs(times: &[f64], values: &[f64]) -> Result<Vec<SplineCoeffs>, String> {
    let n = times.len() - 1;
    let h: Vec<f64> = (0..n).map(|i| times[i + 1] - times[i]).collect();

    let mut m = vec![0.0f64; n + 1];

    if n >= 2 {
        let size = n - 1;
        let mut lower = vec![0.0f64; size];
        let mut diag = vec![0.0f64; size];
        let mut upper = vec![0.0f64; size];
        let mut rhs = vec![0.0f64; size];

        for i in 0..size {
            let idx = i + 1;
            diag[i] = 2.0 * (h[idx - 1] + h[idx]);
            if i > 0 {
                lower[i] = h[idx - 1];
            }
            if i < size - 1 {
                upper[i] = h[idx];
            }
            rhs[i] = 6.0
                * ((values[idx + 1] - values[idx]) / h[idx]
                    - (values[idx] - values[idx - 1]) / h[idx - 1]);
        }

        let x = solve_tridiagonal(&mut lower, &mut diag, &mut upper, &mut rhs)?;
        for i in 0..size {
            m[i + 1] = x[i];
        }
    }

    let mut coeffs = Vec::with_capacity(n);
    for i in 0..n {
        let a = values[i];
        let c = m[i] / 2.0;
        let d = (m[i + 1] - m[i]) / (6.0 * h[i]);
        let b = (values[i + 1] - values[i]) / h[i] - (h[i] * (2.0 * m[i] + m[i + 1])) / 6.0;
        coeffs.push(SplineCoeffs { a, b, c, d });
    }
    Ok(coeffs)
}

#[wasm_bindgen]
pub struct CubicSplineTrajectory {
    waypoints: Vec<Waypoint>,
    coeffs_by_dim: Vec<Vec<SplineCoeffs>>,
    dims: usize,
    last_segment_index: usize,
}

#[wasm_bindgen]
impl CubicSplineTrajectory {
    #[wasm_bindgen(constructor)]
    pub fn new(waypoints_js: JsValue) -> Result<CubicSplineTrajectory, JsValue> {
        let waypoints: Vec<Waypoint> = serde_wasm_bindgen::from_value(waypoints_js)
            .map_err(|e| JsValue::from_str(&e.to_string()))?;

        if waypoints.len() < 2 {
            return Err(JsValue::from_str("At least 2 waypoints are required."));
        }
        for i in 1..waypoints.len() {
            if waypoints[i].time <= waypoints[i - 1].time {
                return Err(JsValue::from_str(
                    "Waypoint times must be strictly increasing.",
                ));
            }
        }
        let times: Vec<f64> = waypoints.iter().map(|w| w.time).collect();
        let dims = waypoints[0].positions.len();
        let mut coeffs_by_dim = Vec::with_capacity(dims);
        for dim in 0..dims {
            let values: Vec<f64> = waypoints.iter().map(|w| w.positions[dim]).collect();
            let c = compute_coeffs(&times, &values)
                .map_err(|e| JsValue::from_str(&e))?;
            coeffs_by_dim.push(c);
        }
        Ok(CubicSplineTrajectory { waypoints, coeffs_by_dim, dims, last_segment_index: 0 })
    }

    pub fn get_duration(&self) -> f64 {
        self.waypoints[self.waypoints.len() - 1].time
    }

    fn find_segment(&mut self, t: f64) -> usize {
        let idx = self.last_segment_index;
        if idx < self.waypoints.len() - 1
            && self.waypoints[idx].time <= t
            && t < self.waypoints[idx + 1].time
        {
            return idx;
        }
        let mut lo = 0usize;
        let mut hi = self.waypoints.len() - 2;
        while lo < hi {
            let mid = (lo + hi + 1) / 2;
            if self.waypoints[mid].time <= t {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        self.last_segment_index = lo;
        lo
    }

    pub fn sample(&mut self, t: f64) -> Vec<f64> {
        let n = self.waypoints.len();
        let first = &self.waypoints[0];
        let last = &self.waypoints[n - 1];

        if t <= first.time {
            return first.positions.clone();
        }
        if t >= last.time {
            return last.positions.clone();
        }

        let lo = self.find_segment(t);
        let dt = t - self.waypoints[lo].time;

        let mut result = vec![0.0f64; self.dims];
        for i in 0..self.dims {
            let SplineCoeffs { a, b, c, d } = self.coeffs_by_dim[i][lo];
            result[i] = a + b * dt + c * dt * dt + d * dt * dt * dt;
        }
        result
    }

    pub fn sample_derivative(&mut self, t: f64) -> Vec<f64> {
        let n = self.waypoints.len();
        let first = &self.waypoints[0];
        let last = &self.waypoints[n - 1];

        if t <= first.time {
            return (0..self.dims)
                .map(|i| self.coeffs_by_dim[i][0].b)
                .collect();
        }
        if t >= last.time {
            let seg = n - 2;
            let h = last.time - self.waypoints[seg].time;
            return (0..self.dims)
                .map(|i| {
                    let SplineCoeffs { b, c, d, .. } = self.coeffs_by_dim[i][seg];
                    b + 2.0 * c * h + 3.0 * d * h * h
                })
                .collect();
        }

        let lo = self.find_segment(t);
        let dt = t - self.waypoints[lo].time;

        (0..self.dims)
            .map(|i| {
                let SplineCoeffs { b, c, d, .. } = self.coeffs_by_dim[i][lo];
                b + 2.0 * c * dt + 3.0 * d * dt * dt
            })
            .collect()
    }

    pub fn sample_second_derivative(&mut self, t: f64) -> Vec<f64> {
        let n = self.waypoints.len();
        let first = &self.waypoints[0];
        let last = &self.waypoints[n - 1];

        if t <= first.time {
            return (0..self.dims)
                .map(|i| 2.0 * self.coeffs_by_dim[i][0].c)
                .collect();
        }
        if t >= last.time {
            let seg = n - 2;
            let h = last.time - self.waypoints[seg].time;
            return (0..self.dims)
                .map(|i| {
                    let SplineCoeffs { c, d, .. } = self.coeffs_by_dim[i][seg];
                    2.0 * c + 6.0 * d * h
                })
                .collect();
        }

        let lo = self.find_segment(t);
        let dt = t - self.waypoints[lo].time;

        (0..self.dims)
            .map(|i| {
                let SplineCoeffs { c, d, .. } = self.coeffs_by_dim[i][lo];
                2.0 * c + 6.0 * d * dt
            })
            .collect()
    }

    /// Returns spline coefficients for use by the TypeScript adapter for validation.
    /// Returns a flat array: for each dim, for each segment: [a, b, c, d, ...]
    pub fn get_coeffs_flat(&self) -> Vec<f64> {
        let mut out = Vec::new();
        for dim_coeffs in &self.coeffs_by_dim {
            for seg in dim_coeffs {
                out.push(seg.a);
                out.push(seg.b);
                out.push(seg.c);
                out.push(seg.d);
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solve_tridiagonal_identity() {
        // Solve [2 -1 0; -1 2 -1; 0 -1 2] * x = [1, 0, 1]
        let mut lower = vec![0.0, -1.0, -1.0];
        let mut diag = vec![2.0, 2.0, 2.0];
        let mut upper = vec![-1.0, -1.0, 0.0];
        let mut rhs = vec![1.0, 0.0, 1.0];
        let x = solve_tridiagonal(&mut lower, &mut diag, &mut upper, &mut rhs).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 1.0).abs() < 1e-10);
        assert!((x[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_compute_coeffs_linear() {
        // For two equidistant points, spline should be linear
        let times = vec![0.0, 1.0];
        let values = vec![0.0, 1.0];
        let coeffs = compute_coeffs(&times, &values).unwrap();
        assert_eq!(coeffs.len(), 1);
        // f(0) = a = 0
        assert!((coeffs[0].a - 0.0).abs() < 1e-10);
        // f'(0) = b = 1 (linear slope)
        assert!((coeffs[0].b - 1.0).abs() < 1e-10);
        // c and d are zero for linear (natural spline with 2 points)
        assert!(coeffs[0].c.abs() < 1e-10);
        assert!(coeffs[0].d.abs() < 1e-10);
    }

    #[test]
    fn test_compute_coeffs_exact_at_waypoints() {
        let times = vec![0.0, 1.0, 2.0, 3.0];
        let values = vec![0.0, 1.0, 0.0, 1.0];
        let coeffs = compute_coeffs(&times, &values).unwrap();
        // Evaluate at each waypoint time
        for (i, &t) in times.iter().enumerate() {
            if i < coeffs.len() {
                let dt = t - times[i];
                let v = coeffs[i].a + coeffs[i].b * dt + coeffs[i].c * dt * dt + coeffs[i].d * dt * dt * dt;
                assert!((v - values[i]).abs() < 1e-10, "segment {} start: expected {}, got {}", i, values[i], v);
            }
            // Check end of previous segment
            if i > 0 {
                let seg = i - 1;
                let h = times[i] - times[i - 1];
                let v = coeffs[seg].a + coeffs[seg].b * h + coeffs[seg].c * h * h + coeffs[seg].d * h * h * h;
                assert!((v - values[i]).abs() < 1e-10, "segment {} end: expected {}, got {}", seg, values[i], v);
            }
        }
    }
}
