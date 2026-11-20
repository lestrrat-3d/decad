module

public import Decad.Float.Model

/-!
# A grid instance of `FloatModel`

`gridModel δ` takes the floats to be the multiples `δ·k` of a positive step `δ`, rounds to the nearest multiple,
and steps up by `δ`. It does not model binary64. It shows the `FloatModel` fields are jointly satisfiable, so no
theorem about an arbitrary `M : FloatModel` holds only because the structure is inconsistent.
-/

public section

namespace Decad.Float

/-- The floats are the integer multiples of `δ`; `rn` rounds to the nearest multiple; `nu f = f + δ`. -/
noncomputable def gridModel (δ : ℝ) (hδ : 0 < δ) : FloatModel where
  F := {x | ∃ k : ℤ, x = δ * k}
  rn x := δ * (round (x / δ) : ℤ)
  nu f := f + δ
  δ := δ
  zero_mem := ⟨0, by simp⟩
  neg_mem := by
    rintro a ⟨k, rfl⟩
    exact ⟨-k, by push_cast; ring⟩
  δ_pos := hδ
  δ_mem := ⟨1, by simp⟩
  rn_mem x := ⟨round (x / δ), rfl⟩
  rn_near := by
    intro x f hf
    obtain ⟨k, rfl⟩ := hf
    have h := round_le (x / δ) k
    have e1 : δ * (round (x / δ) : ℤ) - x = -(δ * (x / δ - round (x / δ))) := by
      field_simp
      ring
    have e2 : δ * (k : ℝ) - x = -(δ * (x / δ - k)) := by
      field_simp
      ring
    rw [e1, e2, abs_neg, abs_neg, abs_mul, abs_mul, abs_of_pos hδ]
    exact mul_le_mul_of_nonneg_left h hδ.le
  nu_mem := by
    rintro f ⟨k, rfl⟩
    exact ⟨k + 1, by push_cast; ring⟩
  lt_nu := fun f _ => by linarith
  nu_min := by
    rintro f ⟨k, rfl⟩ g ⟨m, rfl⟩ hfg
    have hkm : k < m := by
      have : (k : ℝ) < m := lt_of_mul_lt_mul_left hfg hδ.le
      exact_mod_cast this
    have hk1 : (k + 1 : ℤ) ≤ m := hkm
    have : ((k + 1 : ℤ) : ℝ) ≤ m := by exact_mod_cast hk1
    push_cast at this
    nlinarith
  add_grid := by
    rintro a ⟨k, rfl⟩ b ⟨m, rfl⟩
    by_cases h : k + m = 0
    · left
      have : (k : ℝ) + m = 0 := by exact_mod_cast h
      linear_combination δ * this
    · right
      have h1 : (1 : ℝ) ≤ |((k + m : ℤ) : ℝ)| := by
        have : (1 : ℤ) ≤ |k + m| := Int.one_le_abs h
        exact_mod_cast this
      push_cast at h1
      have e : δ * (k : ℝ) + δ * m = δ * ((k : ℝ) + m) := by ring
      rw [e, abs_mul, abs_of_pos hδ]
      nlinarith

end Decad.Float

end
