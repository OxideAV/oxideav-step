//! Read an arbitrary byte string as a STEP file end to end: the
//! physical-file parser, the EXPRESS typing, the product-structure walk,
//! the B-rep / tessellated geometry meshing and the styling must never
//! panic, loop or allocate without bound on hostile input.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_step::{read_step_with, GeometryLimits, ReadOptions, StepLimits, Tolerance};

fuzz_target!(|data: &[u8]| {
    let opts = ReadOptions {
        parse_limits: StepLimits {
            max_input_len: 64 * 1024,
            max_instances: 4096,
            max_depth: 16,
            max_string_len: 1024,
        },
        geometry_limits: GeometryLimits {
            max_triangles: 200_000,
            max_curve_samples: 4096,
        },
        tolerance: Tolerance {
            relative: 1e-2,
            ..Tolerance::default()
        },
        keep_invisible: false,
    };
    // Results are irrelevant; only the absence of panics / runaway work
    // matters.
    let _ = read_step_with(data, &opts);
});
