module

public import Decad.Float.Model

/-!
# Sign of `proof.Orient2SignExpansion`

`proof.Orient2SignExpansion` decides the sign of `(bu - au)(cv - av) - (bv - av)(cu - au)` from two float
products and four `TwoSum` steps (Shewchuk's Two_Two_Sum). This file models that code over `ℝ` and proves:

* `parts_sum`: the four output components add up to the exact determinant `p + q`, where `p = bau·cav` and
  `q = -(bav·cau)` are the two exact products.
* `expansionSign_eq`: when the four components are nonoverlapping (assumption A11), the sign of the highest
  nonzero component is the sign of their sum.
* `orient2Sign_sound`: the Go result is the sign of the exact determinant.

The products and `TwoSum` errors are exact by assumptions A9 and A10: the model writes each error as the exact
real difference instead of the float operations that compute it.
-/

public section

namespace Decad.Orientation

open Decad.Float

/-- The sign of a real as `1`, `-1` or `0`. Model of `proof.floatSign`. -/
@[expose] noncomputable def sgn (x : ℝ) : ℤ := if 0 < x then 1 else if x < 0 then -1 else 0

/-- The four components `x3, x2, x1, x0` that `proof.Orient2SignExpansion` produces. -/
structure Parts where
  x3 : ℝ
  x2 : ℝ
  x1 : ℝ
  x0 : ℝ

/-- Model of the float steps of `proof.Orient2SignExpansion`, from the exact products `p = bau·cav` and
`q = -(bav·cau)`. Each `a1`, `b1` is the rounded product and `a0`, `b0` its exact FMA remainder (A10). Each
`TwoSum` returns the rounded sum and its exact error (A9). -/
@[expose] noncomputable def parts (M : FloatModel) (p q : ℝ) : Parts :=
  let a1 := M.rn p
  let a0 := p - a1
  let b1 := M.rn q
  let b0 := q - b1
  let i := M.rn (a0 + b0)
  let x0 := a0 + b0 - i
  let j := M.rn (a1 + i)
  let z := a1 + i - j
  let k := M.rn (z + b1)
  let x1 := z + b1 - k
  let x3 := M.rn (j + k)
  let x2 := j + k - x3
  ⟨x3, x2, x1, x0⟩

/-- The four components add up to the exact determinant `p + q`. -/
theorem parts_sum (M : FloatModel) (p q : ℝ) :
    (parts M p q).x3 + (parts M p q).x2 + (parts M p q).x1 + (parts M p q).x0 = p + q := by
  simp only [parts]
  ring

/-- Model of the final `switch` of `proof.Orient2SignExpansion`: the sign of the highest nonzero component. -/
@[expose] noncomputable def expansionSign (x3 x2 x1 x0 : ℝ) : ℤ :=
  if x3 ≠ 0 then sgn x3 else if x2 ≠ 0 then sgn x2 else if x1 ≠ 0 then sgn x1 else sgn x0

/-- Assumption A11 as a predicate: each nonzero component, taken as the highest, strictly outweighs the sum of
the components below it. Shewchuk's Theorem 12 gives this for the output of Two_Two_Sum. -/
def NonOverlap (x3 x2 x1 x0 : ℝ) : Prop :=
  (x3 ≠ 0 → |x2 + x1 + x0| < |x3|) ∧
  (x3 = 0 → x2 ≠ 0 → |x1 + x0| < |x2|) ∧
  (x3 = 0 → x2 = 0 → x1 ≠ 0 → |x0| < |x1|)

/-- A term that strictly outweighs the rest decides the sign of the sum. -/
theorem sgn_add_of_abs_lt {t r : ℝ} (h : |r| < |t|) : sgn (t + r) = sgn t := by
  unfold sgn
  rcases lt_trichotomy t 0 with ht | ht | ht
  · rw [abs_of_neg ht] at h
    have h1 := (abs_lt.mp h).2
    have h2 : t + r < 0 := by linarith
    simp only [not_lt.mpr ht.le, not_lt.mpr h2.le, ht, h2, ↓reduceIte]
  · rw [ht, abs_zero] at h
    exact absurd h (not_lt.mpr (abs_nonneg r))
  · rw [abs_of_pos ht] at h
    have h1 := (abs_lt.mp h).1
    have h2 : 0 < t + r := by linarith
    simp only [ht, h2, ↓reduceIte]

/-- For nonoverlapping components (A11), the sign of the highest nonzero one is the sign of the sum. -/
theorem expansionSign_eq {x3 x2 x1 x0 : ℝ} (h : NonOverlap x3 x2 x1 x0) :
    expansionSign x3 x2 x1 x0 = sgn (x3 + x2 + x1 + x0) := by
  obtain ⟨h3, h2, h1⟩ := h
  unfold expansionSign
  by_cases a : x3 = 0
  · by_cases b : x2 = 0
    · by_cases c : x1 = 0
      · subst a b c
        simp
      · have e := sgn_add_of_abs_lt (h1 a b c)
        subst a b
        simp only [ne_eq, not_true_eq_false, ↓reduceIte, c, not_false_eq_true, zero_add]
        exact e.symm
    · have e := sgn_add_of_abs_lt (h2 a b)
      subst a
      simp only [ne_eq, not_true_eq_false, ↓reduceIte, b, not_false_eq_true, zero_add]
      rw [add_assoc]
      exact e.symm
  · have e := sgn_add_of_abs_lt (h3 a)
    simp only [ne_eq, a, not_false_eq_true, ↓reduceIte]
    rw [add_assoc, add_assoc]
    rw [add_assoc] at e
    exact e.symm

/-- `proof.Orient2SignExpansion` returns the sign of the exact determinant `p + q` of its admitted inputs,
including exact zero, provided Two_Two_Sum's output is nonoverlapping (A11). The exact products and `TwoSum`
errors are A9 and A10. -/
theorem orient2Sign_sound (M : FloatModel) (p q : ℝ)
    (h : NonOverlap (parts M p q).x3 (parts M p q).x2 (parts M p q).x1 (parts M p q).x0) :
    expansionSign (parts M p q).x3 (parts M p q).x2 (parts M p q).x1 (parts M p q).x0 = sgn (p + q) := by
  rw [expansionSign_eq h, parts_sum]

end Decad.Orientation

end
