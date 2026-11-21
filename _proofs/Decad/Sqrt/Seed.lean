module

public import Decad.Float.Model

/-!
# Quotient seed of `proof.ratioSqrtSeedLeading`

`proof.ratioSqrtSeedLeading` seeds a square root from the leading 64 bits `num`, `den` of a numerator and a
denominator. It divides them with `bits.Div64`, takes the quotient `q`, and accepts the seed only when
`float64(q-8)` and `float64(q+8)` are equal. The accepted claim is that every 64-bit quotient the full
`big.Float` division could produce converts to that same float. This file proves, over `ℝ` and `ℕ`:

* `seedQ_bounds`: `q` lies in `[2^63, 2^64)`, and the `Div64` precondition holds, in both scale cases.
* `seed_enclosure`: the exact scaled quotient is within 2 of `q + r` for a fraction `r < 1`, so a 64-bit
  rounding `r` of it is within `3.5` of `q`, inside the `[q-8, q+8]` window.
* `seed_no_wrap`: the guard `q ≤ 2^64 - 9` keeps `q + 8` from wrapping `uint64`.
* `seed_sound`: when `float64(q-8) = float64(q+8)`, every real in the window converts to that float.

The conversion is modeled by `FloatModel.rn` with the extra hypothesis that it is monotone (A17).
Not covered: the scaling of the exponent, `math.Sqrt` and `math.Ldexp`, and the claim that the full-division
path yields the same float.
-/

public section

namespace Decad.Sqrt

open Decad.Float

/-- Model of the quotient `bits.Div64` returns in `proof.ratioSqrtSeedLeading`: `⌊num·2^k / den⌋` with `k = 63`
when `num ≥ den` (the dividend is `num / 2` and `num << 63`) and `k = 64` when `num < den` (the dividend is
`num` and `0`). A18 states that `Div64` returns the floor quotient. -/
@[expose] def seedQ (num den k : ℕ) : ℕ := num * 2 ^ k / den

/-- The scale choice of `proof.ratioSqrtSeedLeading`. -/
def ScaleOk (num den k : ℕ) : Prop := (k = 63 ∧ den ≤ num) ∨ (k = 64 ∧ num < den)

/-- Both inputs are 64-bit words with the top bit set. -/
def Leading (x : ℕ) : Prop := 2 ^ 63 ≤ x ∧ x < 2 ^ 64

theorem seedQ_bounds {num den k : ℕ} (hn : Leading num) (hd : Leading den) (hk : ScaleOk num den k) :
    2 ^ 63 ≤ seedQ num den k ∧ seedQ num den k < 2 ^ 64 := by
  obtain ⟨hn1, hn2⟩ := hn
  obtain ⟨hd1, hd2⟩ := hd
  have hden : 0 < den := by omega
  unfold seedQ
  rcases hk with ⟨rfl, h⟩ | ⟨rfl, h⟩
  · constructor
    · rw [Nat.le_div_iff_mul_le hden]
      have := Nat.mul_le_mul_right (2 ^ 63) h
      linarith
    · rw [Nat.div_lt_iff_lt_mul hden]
      have h2 : num < 2 * den := by omega
      calc num * 2 ^ 63 < 2 * den * 2 ^ 63 := by
            exact Nat.mul_lt_mul_of_pos_right h2 (by positivity)
        _ = 2 ^ 64 * den := by ring
  · constructor
    · rw [Nat.le_div_iff_mul_le hden]
      have h2 : den ≤ 2 * num := by omega
      calc 2 ^ 63 * den ≤ 2 ^ 63 * (2 * num) := Nat.mul_le_mul_left _ h2
        _ = num * 2 ^ 64 := by ring
    · rw [Nat.div_lt_iff_lt_mul hden]
      calc num * 2 ^ 64 < den * 2 ^ 64 := Nat.mul_lt_mul_of_pos_right h (by positivity)
        _ = 2 ^ 64 * den := by ring

/-- The `bits.Div64` precondition `hi < den` holds for both dividends of `proof.ratioSqrtSeedLeading`. -/
theorem div64_precondition {num den : ℕ} (hn : Leading num) (hd : Leading den) :
    (den ≤ num → num / 2 < den) ∧ (num < den → num < den) := by
  obtain ⟨hn1, hn2⟩ := hn
  obtain ⟨hd1, hd2⟩ := hd
  refine ⟨fun _ => ?_, fun h => h⟩
  omega

/-- The guard `q > 2^64 - 1 - 8` refuses every quotient whose `q + 8` would wrap `uint64`; an accepted `q` has
`q - 8` and `q + 8` as exact naturals below `2^64`. -/
theorem seed_no_wrap {q : ℕ} (hq : 2 ^ 63 ≤ q) (hg : ¬ q > 2 ^ 64 - 1 - 8) :
    8 ≤ q ∧ q + 8 < 2 ^ 64 := by
  constructor <;> omega

/-- The exact scaled quotient lies within 2 of the integer quotient plus its fraction. `u` and `v` are the
numerator and denominator scaled so their leading words are `num` and `den`: each is within one unit above. -/
theorem seed_enclosure {num den k : ℕ} {u v : ℝ} (hn : Leading num) (hd : Leading den)
    (hk : ScaleOk num den k)
    (hu : (num : ℝ) ≤ u ∧ u ≤ num + 1) (hv : (den : ℝ) ≤ v ∧ v ≤ den + 1) :
    |u / v * 2 ^ k - (seedQ num den k : ℝ)| < 3 := by
  obtain ⟨hn1, hn2⟩ := hn
  obtain ⟨hd1, hd2⟩ := hd
  have hden : 0 < den := by omega
  have hdR : (0 : ℝ) < den := by exact_mod_cast hden
  have hn1R : ((2 : ℝ) ^ 63) ≤ num := by exact_mod_cast hn1
  have hn2R : (num : ℝ) < 2 ^ 64 := by exact_mod_cast hn2
  have hd1R : ((2 : ℝ) ^ 63) ≤ den := by exact_mod_cast hd1
  have hd2R : (den : ℝ) < 2 ^ 64 := by exact_mod_cast hd2
  -- the integer quotient and its remainder
  have hdiv := Nat.div_add_mod (num * 2 ^ k) den
  have hmod : num * 2 ^ k % den < den := Nat.mod_lt _ hden
  set q := seedQ num den k with hq
  set r := num * 2 ^ k % den with hr
  have hdiv' : (den : ℝ) * q + r = num * 2 ^ k := by exact_mod_cast hdiv
  have hrR : (r : ℝ) < den := by exact_mod_cast hmod
  have hr0 : (0 : ℝ) ≤ r := Nat.cast_nonneg _
  have hv0 : 0 < v := by linarith [hv.1]
  have h2k : (2 : ℝ) ^ k ≤ 2 ^ 64 := by
    rcases hk with ⟨rfl, _⟩ | ⟨rfl, _⟩ <;> norm_num
  have h2k0 : (0 : ℝ) < 2 ^ k := by positivity
  -- Q = num·2^k/den = q + r/den
  have hQ : (num : ℝ) * 2 ^ k / den = q + r / den := by
    field_simp
    linarith
  have hfrac : (r : ℝ) / den < 1 := by rw [div_lt_one hdR]; exact hrR
  have hfrac0 : 0 ≤ (r : ℝ) / den := div_nonneg hr0 hdR.le
  -- upper: T ≤ (num+1)·2^k/den ≤ Q + 2
  have hup : u / v * 2 ^ k ≤ ((num : ℝ) + 1) * 2 ^ k / den := by
    rw [div_mul_eq_mul_div, div_le_div_iff₀ hv0 hdR]
    have a : u * den ≤ ((num : ℝ) + 1) * den := mul_le_mul_of_nonneg_right hu.2 hdR.le
    have b : ((num : ℝ) + 1) * den ≤ ((num : ℝ) + 1) * v :=
      mul_le_mul_of_nonneg_left hv.1 (by positivity)
    have c := mul_le_mul_of_nonneg_left (a.trans b) h2k0.le
    linarith
  have hup2 : ((num : ℝ) + 1) * 2 ^ k / den ≤ (num : ℝ) * 2 ^ k / den + 2 := by
    have e : ((num : ℝ) + 1) * 2 ^ k / den = (num : ℝ) * 2 ^ k / den + 2 ^ k / den := by ring
    have : (2 : ℝ) ^ k / den ≤ 2 := by
      rw [div_le_iff₀ hdR]
      have : (2 : ℝ) ^ 64 = 2 * 2 ^ 63 := by norm_num
      linarith
    linarith
  -- lower: T ≥ num·2^k/(den+1) ≥ Q - 2
  have hlow : (num : ℝ) * 2 ^ k / (den + 1) ≤ u / v * 2 ^ k := by
    rw [div_mul_eq_mul_div, div_le_div_iff₀ (by linarith) hv0]
    have u0 : 0 ≤ u := le_trans (Nat.cast_nonneg _) hu.1
    have a : (num : ℝ) * v ≤ u * v := mul_le_mul_of_nonneg_right hu.1 hv0.le
    have b : u * v ≤ u * (den + 1) := mul_le_mul_of_nonneg_left hv.2 u0
    have c := mul_le_mul_of_nonneg_left (a.trans b) h2k0.le
    linarith
  have hP : (num : ℝ) * 2 ^ k ≤ 2 * den * (den + 1) := by
    rcases hk with ⟨rfl, _⟩ | ⟨rfl, hlt⟩
    · have hltR : (num : ℝ) < 2 ^ 64 := hn2R
      have : (2 : ℝ) ^ 63 * 2 ^ 63 ≤ den * den := mul_le_mul hd1R hd1R (by positivity) hdR.le
      have e : (2 : ℝ) ^ 64 * 2 ^ 63 = 2 * (2 ^ 63 * 2 ^ 63) := by norm_num
      nlinarith [mul_pos hdR hdR]
    · have hltR : (num : ℝ) < den := by exact_mod_cast hlt
      have e : (2 : ℝ) ^ 64 = 2 * 2 ^ 63 := by norm_num
      nlinarith [mul_pos hdR hdR, mul_le_mul_of_nonneg_left hd1R hdR.le]
  have hlow2 : (num : ℝ) * 2 ^ k / den - 2 ≤ (num : ℝ) * 2 ^ k / (den + 1) := by
    have hd1' : (0 : ℝ) < den + 1 := by linarith
    have key : (num : ℝ) * 2 ^ k / den - (num : ℝ) * 2 ^ k / (den + 1) =
        (num : ℝ) * 2 ^ k / (den * (den + 1)) := by
      field_simp
      ring
    have : (num : ℝ) * 2 ^ k / (den * (den + 1)) ≤ 2 := by
      rw [div_le_iff₀ (by positivity)]
      linarith
    linarith
  have hT : (num : ℝ) * 2 ^ k / den - 2 ≤ u / v * 2 ^ k := le_trans hlow2 hlow
  rw [abs_lt]
  constructor <;> linarith

/-- When `float64(q-8) = float64(q+8)` (the check `lower != upper` passes), every 64-bit rounding `r` of the exact
scaled quotient converts to that same float. `hmono` is A17: the conversion is monotone. -/
theorem seed_sound (M : FloatModel) (hmono : ∀ x y : ℝ, x ≤ y → M.rn x ≤ M.rn y)
    {num den k : ℕ} {u v r : ℝ} (hn : Leading num) (hd : Leading den) (hk : ScaleOk num den k)
    (hu : (num : ℝ) ≤ u ∧ u ≤ num + 1) (hv : (den : ℝ) ≤ v ∧ v ≤ den + 1)
    (hr : |r - u / v * 2 ^ k| ≤ 1 / 2)
    (heq : M.rn ((seedQ num den k : ℝ) - 8) = M.rn ((seedQ num den k : ℝ) + 8)) :
    M.rn r = M.rn ((seedQ num den k : ℝ) - 8) := by
  have h := seed_enclosure hn hd hk hu hv
  rw [abs_lt] at h
  rw [abs_le] at hr
  have h1 : (seedQ num den k : ℝ) - 8 ≤ r := by linarith
  have h2 : r ≤ (seedQ num den k : ℝ) + 8 := by linarith
  apply le_antisymm
  · calc M.rn r ≤ M.rn ((seedQ num den k : ℝ) + 8) := hmono _ _ h2
      _ = M.rn ((seedQ num den k : ℝ) - 8) := heq.symm
  · exact hmono _ _ h1

end Decad.Sqrt

end
