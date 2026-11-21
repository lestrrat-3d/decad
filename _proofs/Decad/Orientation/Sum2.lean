module

public import Decad.Float.Model

/-!
# Sign decision of `proof.orientSignSum2`

`proof.OrientSignExpansion` writes the 3×3 determinant of the admitted differences as the exact sum of 24
floats: six triple products, each split into four components by `tripleComponents`. `proof.orientSignSum2`
sums them with Sum2 (Ogita, Rump and Oishi) and decides the sign only when the result clears
`2^-96 · mag`. This file proves:

* `sum24_eq_det`: the 24 components add up to the exact determinant.
* `scale_bound`: the constant `2^-96` is at least `g·(1+3ε)/(1-γ(5))`, the factor the Go doc comment derives.
* `sum2_sign_sound`: a result beyond `2^-96 · mag` has the sign of the exact sum.

The Sum2 error bound (A12) and the bound `S ≤ (1+3ε)/(1-γ(5)) · mag` (A13) are hypotheses.
-/

public section

namespace Decad.Orientation

open Decad.Float

/-- The four components `proof.tripleComponents` returns for `x·y·z`: `xy` is the rounded product, `xyTail`
its exact FMA remainder, `hi` the rounded `xy·z`, `lo` its remainder, and `tailHi`, `tailLo` the same split of
`xyTail·z`. The remainders are exact by A10. -/
@[expose] noncomputable def tripleSum (M : FloatModel) (x y z : ℝ) : ℝ :=
  let xy := M.rn (x * y)
  let xyTail := x * y - xy
  let hi := M.rn (xy * z)
  let lo := xy * z - hi
  let tailHi := M.rn (xyTail * z)
  let tailLo := xyTail * z - tailHi
  hi + lo + tailHi + tailLo

/-- The four components of a triple product add up to exactly `x·y·z`. -/
theorem tripleSum_eq (M : FloatModel) (x y z : ℝ) : tripleSum M x y z = x * y * z := by
  simp only [tripleSum]
  ring

/-- The 24 components of the six triple products in `proof.orientSignSum2` add up to the exact determinant of
the matrix with rows `b`, `c`, `d`. -/
theorem sum24_eq_det (M : FloatModel) (bx by' bz cx cy cz dx dy dz : ℝ) :
    tripleSum M bx cy dz + tripleSum M by' cz dx + tripleSum M bz cx dy
      + tripleSum M (-bx) cz dy + tripleSum M (-by') cx dz + tripleSum M (-bz) cy dx =
    bx * (cy * dz - cz * dy) + by' * (cz * dx - cx * dz) + bz * (cx * dy - cy * dx) := by
  simp only [tripleSum_eq]
  ring

/-- Unit roundoff `ε = 2^-53`. -/
noncomputable def eps : ℝ := (2 : ℝ) ^ (-53 : ℤ)

/-- `γ(k) = kε / (1 - kε)`. -/
noncomputable def gam (k : ℕ) : ℝ := k * eps / (1 - k * eps)

/-- The proof's constant `g = γ(23)²` for `n = 24` components. -/
noncomputable def gConst : ℝ := gam 23 ^ 2

/-- `2^-96` is at least `g·(1+3ε)/(1-γ(5))`. The Go doc comment states it is below `6.6e-30` while `2^-96`
exceeds `1.26e-29`. -/
theorem scale_bound : gConst * ((1 + 3 * eps) / (1 - gam 5)) ≤ (2 : ℝ) ^ (-96 : ℤ) := by
  unfold gConst gam eps
  norm_num

/-- `proof.orientSignSum2` is sound. `s` is the exact sum of the 24 components, `res` the Sum2 result, `S` the
sum of the components' absolute values and `mag` the float sum of the six `|hi|`. Hypothesis `hSum2` is A12
(Ogita, Rump and Oishi, Proposition 4.5, `n = 24`); `hS` is A13. A result above `2^-96 · mag` gives a positive
exact sum, and below `-(2^-96 · mag)` a negative one. -/
theorem sum2_sign_sound {s res S mag : ℝ}
    (hmag : 0 ≤ mag)
    (hSum2 : |res - s| ≤ eps * |s| + gConst * S)
    (hS : S ≤ (1 + 3 * eps) / (1 - gam 5) * mag) :
    ((2 : ℝ) ^ (-96 : ℤ) * mag < res → 0 < s) ∧
    (res < -((2 : ℝ) ^ (-96 : ℤ) * mag) → s < 0) := by
  have heps0 : 0 < eps := by unfold eps; positivity
  have heps1 : eps < 1 := by unfold eps; norm_num
  have hg : 0 ≤ gConst := by unfold gConst; positivity
  have hgS : gConst * S ≤ (2 : ℝ) ^ (-96 : ℤ) * mag := by
    calc gConst * S ≤ gConst * ((1 + 3 * eps) / (1 - gam 5) * mag) := by gcongr
      _ = (gConst * ((1 + 3 * eps) / (1 - gam 5))) * mag := by ring
      _ ≤ (2 : ℝ) ^ (-96 : ℤ) * mag := by gcongr; exact scale_bound
  constructor
  · intro h
    by_contra hs
    have hs' : s ≤ 0 := not_lt.mp hs
    have h1 := le_abs_self (res - s)
    rw [abs_of_nonpos hs'] at hSum2
    have h2 : (1 - eps) * s ≤ 0 := mul_nonpos_of_nonneg_of_nonpos (by linarith) hs'
    nlinarith
  · intro h
    by_contra hs
    have hs' : 0 ≤ s := not_lt.mp hs
    have h1 := neg_le_abs (res - s)
    rw [abs_of_nonneg hs'] at hSum2
    have h2 : 0 ≤ (1 - eps) * s := mul_nonneg (by linarith) hs'
    nlinarith

end Decad.Orientation

end
