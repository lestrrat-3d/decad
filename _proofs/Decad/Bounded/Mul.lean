module

public import Decad.Bounded.Upper

/-!
# Soundness of `proofbound.BoundedMul`

Held operands `v`, `w` carry bounds `Ea`, `Eb` of at least 0, meaning the true values `x`, `y` satisfy
`|x - v| ≤ Ea` and `|y - w| ≤ Eb`. `held` is the float product `fl(v·w)` and `r` is a float at least
`|v·w - held|`, which the Go code computes with `proof.MulRoundError` (assumption A4 in
`agent-docs/proofs-design.md`). The three product terms use `proofbound.ProductUpper` and are summed with
`proofbound.AbsSumUpper`; `MeasuredScalar` returns the same bound.
-/

public section

namespace Decad.Bounded

open Decad.Float

/-- The real-number core of `proofbound.BoundedMul`: the product of two enclosed values stays inside
`|v|·Eb + |w|·Ea + Ea·Eb` of the product of the held values. -/
theorem mul_enclosure (v w x y Ea Eb : ℝ)
    (hx : |x - v| ≤ Ea) (hy : |y - w| ≤ Eb) :
    |x * y - v * w| ≤ |v| * Eb + |w| * Ea + Ea * Eb := by
  have key : x * y - v * w = v * (y - w) + w * (x - v) + (x - v) * (y - w) := by ring
  have hEa : 0 ≤ Ea := le_trans (abs_nonneg _) hx
  have hEb : 0 ≤ Eb := le_trans (abs_nonneg _) hy
  rw [key]
  calc |v * (y - w) + w * (x - v) + (x - v) * (y - w)|
      ≤ |v * (y - w) + w * (x - v)| + |(x - v) * (y - w)| := abs_add_le _ _
    _ ≤ (|v * (y - w)| + |w * (x - v)|) + |(x - v) * (y - w)| := by
        gcongr; exact abs_add_le _ _
    _ ≤ |v| * Eb + |w| * Ea + Ea * Eb := by
        rw [abs_mul, abs_mul, abs_mul]
        gcongr

/-- `proofbound.BoundedMul` and `proofbound.MeasuredScalar` publish a sound bound: the result covers the
distance from the held product to every true product. The bound is `AbsSumUpper` of the three
`ProductUpper` terms and the rounding error `r`. Hypothesis `hr` is A4; `hrF` says `r` is a float; `hEa0`
and `hEb0` are A6. -/
theorem boundedMul_bound_sound (M : FloatModel)
    {v w x y Ea Eb held r : ℝ}
    (hrF : r ∈ M.F)
    (hEa0 : 0 ≤ Ea) (hEb0 : 0 ≤ Eb)
    (hx : |x - v| ≤ Ea) (hy : |y - w| ≤ Eb)
    (hr : |v * w - held| ≤ r) :
    |x * y - held| ≤
      absSum M [productUpper M |v| Eb, productUpper M |w| Ea, productUpper M Ea Eb, r] 0 := by
  have hmem : ∀ u ∈ [productUpper M |v| Eb, productUpper M |w| Ea, productUpper M Ea Eb, r],
      u ∈ M.F := by
    intro u hu
    simp only [List.mem_cons, List.not_mem_nil, or_false] at hu
    rcases hu with rfl | rfl | rfl | rfl
    · exact productUpper_mem M _ _
    · exact productUpper_mem M _ _
    · exact productUpper_mem M _ _
    · exact hrF
  have hsum := absSum_ge M _ 0 hmem M.zero_mem le_rfl
  have h1 := productUpper_ge M (abs_nonneg v) hEb0
  have h2 := productUpper_ge M (abs_nonneg w) hEa0
  have h3 := productUpper_ge M hEa0 hEb0
  have core := mul_enclosure v w x y Ea Eb hx hy
  simp only [List.map_cons, List.map_nil, List.sum_cons, List.sum_nil] at hsum
  have := le_abs_self (productUpper M |v| Eb)
  have := le_abs_self (productUpper M |w| Ea)
  have := le_abs_self (productUpper M Ea Eb)
  have := le_abs_self r
  calc |x * y - held|
      = |(x * y - v * w) + (v * w - held)| := by ring_nf
    _ ≤ |x * y - v * w| + |v * w - held| := abs_add_le _ _
    _ ≤ _ := by linarith

end Decad.Bounded

end
