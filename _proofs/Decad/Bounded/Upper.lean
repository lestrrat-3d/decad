module

public import Decad.Float.Model

/-!
# Upward-rounding helpers

Models of the upward-rounding helpers behind `proofbound.BoundedMul`: `proofbound.UpRound`,
`proofbound.ProvenUpRound` (and `proof.ProvenUpRound`), `proofbound.ProductUpper` and
`proofbound.AbsSumUpper`. Each theorem states that the helper's result is at least its exact value, and that the
result is a float. All of them take the float model `M` explicitly; the Go helpers' `+Inf` refusal branches are
not modeled (assumption A7 in `agent-docs/proofs-design.md`).
-/

public section

namespace Decad.Bounded

open Decad.Float

variable (M : FloatModel)

/-- Model of `proofbound.UpRound`: steps a positive sum up to the next float and leaves other values alone. -/
@[expose] noncomputable def upRound (s : ℝ) : ℝ := if 0 < s then M.nu s else s

theorem upRound_mem {s : ℝ} (hs : s ∈ M.F) : upRound M s ∈ M.F := by
  unfold upRound; split_ifs
  · exact M.nu_mem _ hs
  · exact hs

/-- A held sum of two non-negative floats, rounded then stepped up by `proofbound.UpRound`, covers the exact
sum. Hypotheses `ha`, `hb`, `ha0`, `hb0` are the operand facts of `proofbound.AbsSumUpper`'s fold; the grid
fact comes from A3. -/
theorem add_up {a b : ℝ} (ha : a ∈ M.F) (hb : b ∈ M.F) (ha0 : 0 ≤ a) (hb0 : 0 ≤ b) :
    a + b ≤ upRound M (M.rn (a + b)) := by
  have hx : 0 ≤ a + b := by linarith
  have hs0 := M.rn_nonneg hx
  unfold upRound
  split_ifs with hpos
  · exact M.up_ge _
  · have hs : M.rn (a + b) = 0 := le_antisymm (not_lt.mp hpos) hs0
    rcases M.add_grid a ha b hb with h | h
    · rw [hs]; linarith
    · exfalso
      rw [abs_of_nonneg hx] at h
      have hn := M.rn_near (a + b) M.δ M.δ_mem
      rw [hs, abs_of_nonpos (by linarith : (0:ℝ) - (a + b) ≤ 0),
          abs_of_nonpos (by linarith : M.δ - (a + b) ≤ 0)] at hn
      linarith [M.δ_pos]

/-- Model of `proofbound.AbsSumUpper`'s fold: adds `|v|` for each `v` in turn, rounding each partial sum
upward. -/
@[expose] noncomputable def absSum (L : List ℝ) (t : ℝ) : ℝ :=
  L.foldl (fun t v => upRound M (M.rn (t + |v|))) t

/-- The fold of `proofbound.AbsSumUpper` over floats is at least the exact sum of absolute values. -/
theorem absSum_ge (L : List ℝ) :
    ∀ t : ℝ, (∀ v ∈ L, v ∈ M.F) → t ∈ M.F → 0 ≤ t →
      t + (L.map abs).sum ≤ absSum M L t := by
  induction L with
  | nil => intro t _ _ _; simp [absSum]
  | cons v L ih =>
    intro t hL ht ht0
    have hv : v ∈ M.F := hL v (by simp)
    have hav : |v| ∈ M.F := by
      rcases abs_choice v with h | h <;> rw [h]
      · exact hv
      · exact M.neg_mem _ hv
    have hstep := add_up M ht hav ht0 (abs_nonneg v)
    have hmem := upRound_mem M (M.rn_mem (t + |v|))
    have hnn : 0 ≤ upRound M (M.rn (t + |v|)) := by
      linarith [abs_nonneg v]
    have := ih _ (fun w hw => hL w (by simp [hw])) hmem hnn
    simp only [absSum, List.foldl_cons, List.map_cons, List.sum_cons] at this ⊢
    linarith

/-- Model of `proofbound.ProvenUpRound` and `proof.ProvenUpRound`: steps a positive value up, maps zero to the
smallest positive float, and leaves negative values alone. -/
@[expose] noncomputable def provenUp (s : ℝ) : ℝ :=
  if 0 < s then M.nu s else if s = 0 then M.δ else s

/-- Model of `proofbound.ProductUpper`, finite operands only: the `+Inf` refusal branch is not modeled. -/
@[expose] noncomputable def productUpper (a b : ℝ) : ℝ :=
  if a ≤ 0 ∨ b ≤ 0 then 0 else provenUp M (M.rn (a * b))

theorem productUpper_mem (a b : ℝ) : productUpper M a b ∈ M.F := by
  unfold productUpper provenUp
  split_ifs
  · exact M.zero_mem
  · exact M.nu_mem _ (M.rn_mem _)
  · exact M.δ_mem
  · exact M.rn_mem _

/-- `proofbound.ProductUpper` of two non-negative values is at least their exact product. -/
theorem productUpper_ge {a b : ℝ} (ha : 0 ≤ a) (hb : 0 ≤ b) :
    a * b ≤ productUpper M a b := by
  unfold productUpper
  split_ifs with h
  · rcases h with h | h
    · have : a = 0 := le_antisymm h ha
      simp [this]
    · have : b = 0 := le_antisymm h hb
      simp [this]
  · push Not at h
    have hx : 0 < a * b := mul_pos h.1 h.2
    have hs0 := M.rn_nonneg hx.le
    unfold provenUp
    split_ifs with hp hz
    · exact M.up_ge _
    · have hn := M.rn_near (a * b) M.δ M.δ_mem
      rw [hz, abs_of_nonpos (by linarith : (0:ℝ) - a * b ≤ 0)] at hn
      rcases le_or_gt M.δ (a * b) with hle | hlt
      · rw [abs_of_nonpos (by linarith : M.δ - a * b ≤ 0)] at hn
        linarith [M.δ_pos]
      · rw [abs_of_nonneg (by linarith : 0 ≤ M.δ - a * b)] at hn
        linarith
    · exfalso
      exact hz (le_antisymm (not_lt.mp hp) hs0)

end Decad.Bounded

end
