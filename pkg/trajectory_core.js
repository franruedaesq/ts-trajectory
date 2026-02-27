/* @ts-self-types="./trajectory_core.d.ts" */

import * as wasm from "./trajectory_core_bg.wasm";
import { __wbg_set_wasm } from "./trajectory_core_bg.js";
__wbg_set_wasm(wasm);
wasm.__wbindgen_start();
export {
    CubicSplineTrajectory, LinearTrajectory
} from "./trajectory_core_bg.js";
