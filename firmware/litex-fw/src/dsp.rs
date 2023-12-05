/// Examples of some fixed-point DSP algorithms.

use fixed::{FixedI32, types::extra::U16};

/// Fixed point DSP below should use 32-bit integers with a 16.16 split.
/// This could be made generic below, but isn't to reduce noise...
pub type Fix = FixedI32<U16>;

/// Implementation of fixed-point DSP for a resonant low-pass filter.
pub struct KarlsenLpf {
    rezz: Fix,
    sat : Fix,
    a1  : Fix,
    a2  : Fix,
    a3  : Fix,
    a4  : Fix,
    dc_block : DcBlock,
}

/// Implementation of fixed-point DSP for a DC block.
struct DcBlock {
    x_k1: Fix,
    y_k1: Fix,
}

impl DcBlock {
    fn new() -> Self {
        DcBlock {
            x_k1: Fix::from_num(0),
            y_k1: Fix::from_num(0),
        }
    }

    /// Execute the DC Block function on a single sample, returning the next sample.
    /// -1 <= x <= 1 (audio sample)
    fn proc(&mut self, x_k: Fix) -> Fix {
        self.y_k1 = (x_k - self.x_k1) + Fix::from_num(0.99f32) * self.y_k1;
        self.x_k1 = x_k;
        self.y_k1
    }
}

impl KarlsenLpf {
    pub fn new() -> Self {
        KarlsenLpf {
            rezz: Fix::from_num(0),
            sat: Fix::from_num(0),
            a1: Fix::from_num(0),
            a2: Fix::from_num(0),
            a3: Fix::from_num(0),
            a4: Fix::from_num(0),
            dc_block: DcBlock::new(),
        }
    }

    /// Execute the LPF function on a single sample, returning the next sample.
    /// -1 <= x <= 1 (audio sample)
    /// 0 <= g <= 1 (cutoff)
    /// 0 <= res <= 1 (resonance)
    pub fn proc(&mut self, x: Fix, g: Fix, res: Fix) -> Fix {
        let gmax = Fix::max(Fix::from_num(0), g);
        let res_scaled = Fix::max(Fix::from_num(0), res * 4);
        self.rezz = x - ((self.a4 - x) * res_scaled);
        self.sat = Fix::min(Fix::ONE, self.rezz);
        self.sat = Fix::max(-Fix::ONE, self.rezz);
        self.a1 = self.a1 + ((-self.a1 + self.sat) * gmax);
        self.a2 = self.a2 + ((-self.a2 + self.a1) * gmax);
        self.a3 = self.a3 + ((-self.a3 + self.a2) * gmax);
        self.a4 = self.a4 + ((-self.a4 + self.a3) * gmax);
        self.dc_block.proc(self.a4)
    }
}
