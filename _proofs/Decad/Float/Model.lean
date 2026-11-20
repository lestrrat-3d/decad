module

public import Mathlib

/-!
# Abstract binary64 model

Abstract model of the finite binary64 floats that `internal/proof` and `internal/proofbound` compute with.
Overflow to `+Inf` is not modeled; `+Inf` is at least every finite value, so it only widens a bound.

* `F`   the finite floats, closed under negation, containing 0.
* `rn`  round-to-nearest: a float at least as close to `x` as any float.
* `nu`  `proof.NextUp`: the least float strictly above `f`.
* `δ`   the smallest positive float; every sum of two floats is 0 or at least `δ` in magnitude, because
        all floats are multiples of `δ`.

`Decad.Float.gridModel` shows the fields are satisfiable. A concrete binary64 instance would discharge the
float assumptions of `agent-docs/proofs-design.md` (A1 to A3); theorems that take `M : FloatModel` need no edit.
-/

public section

namespace Decad.Float

structure FloatModel where
  F : Set ℝ
  rn : ℝ → ℝ
  nu : ℝ → ℝ
  δ : ℝ
  zero_mem : (0 : ℝ) ∈ F
  neg_mem : ∀ a ∈ F, -a ∈ F
  δ_pos : 0 < δ
  δ_mem : δ ∈ F
  rn_mem : ∀ x, rn x ∈ F
  rn_near : ∀ x, ∀ f ∈ F, |rn x - x| ≤ |f - x|
  nu_mem : ∀ f ∈ F, nu f ∈ F
  lt_nu : ∀ f ∈ F, f < nu f
  nu_min : ∀ f ∈ F, ∀ g ∈ F, f < g → nu f ≤ g
  add_grid : ∀ a ∈ F, ∀ b ∈ F, a + b = 0 ∨ δ ≤ |a + b|

namespace FloatModel
variable (M : FloatModel)

/-- Rounding to nearest then stepping up never lands below `x`. -/
theorem up_ge (x : ℝ) : x ≤ M.nu (M.rn x) := by
  by_contra h
  push Not at h
  have hf := M.rn_mem x
  have hlt := M.lt_nu _ hf
  have hnu := M.nu_mem _ hf
  have hnear := M.rn_near x _ hnu
  have h1 : M.rn x < x := by linarith
  rw [abs_of_neg (by linarith : M.rn x - x < 0),
      abs_of_neg (by linarith : M.nu (M.rn x) - x < 0)] at hnear
  linarith

/-- Rounding a non-negative real to nearest gives a non-negative float. -/
theorem rn_nonneg {x : ℝ} (hx : 0 ≤ x) : 0 ≤ M.rn x := by
  have h0 := M.rn_near x 0 M.zero_mem
  have e : |(0:ℝ) - x| = x := by rw [zero_sub, abs_neg, abs_of_nonneg hx]
  rw [e] at h0
  by_contra h
  push Not at h
  rw [abs_of_neg (by linarith : M.rn x - x < 0)] at h0
  linarith

end FloatModel

end Decad.Float

end
