module

public import Mathlib

/-!
# Outward dyadic rounding of a fraction

Models of `proof.lazyRound`, which `proof.LazyRoundDown` and `proof.LazyRoundUp` call
(`internal/proof/lazy_rat.go`). For `x = n/d` with `d > 0` the Go code picks the scale
`s = bits - (len(n) - len(d))` with `len` the `big.Int.BitLen`, shifts `n` or `d` by `|s|` so the scaled
fraction `n'/d'` equals `x·2^s`, divides with `big.Int.QuoRem` (assumption A8 in
`agent-docs/proofs-design.md`), corrects the quotient by one toward the requested side when the remainder is
nonzero, and returns `m/2^s`.

The file proves, in order: the corrected quotient is the floor or ceiling (`down_is_floor`, `up_is_ceil`);
rounding is outward and within `2^-s` (`round_outward`, `round_error`); the scale puts `|x·2^s|` strictly
between `2^(bits-1)` and `2^(bits+1)` (`scale_range`); the sign survives (`round_sign`); the relative error is
below `2^(1-bits)` (`round_rel_error`); and the branch-for-branch model gives all of these (`lazyRound_sound`).
-/

public section

namespace Decad.Rational

/-- Down: the corrected truncating quotient is the floor of `n/d`. The remainder bounds `h1`, `h2` and the
decomposition `h` are what `big.Int.QuoRem` returns (A8). -/
theorem down_is_floor (n d q r : ℤ) (hd : 0 < d) (h : n = d * q + r)
    (h1 : -d < r) (h2 : r < d) :
    (if r < 0 then q - 1 else q) = ⌊(n : ℚ) / d⌋ := by
  have hdq : (0 : ℚ) < d := by exact_mod_cast hd
  have hn : (n : ℚ) = d * q + r := by exact_mod_cast h
  have h1' : (-(d : ℚ)) < r := by exact_mod_cast h1
  have h2' : (r : ℚ) < d := by exact_mod_cast h2
  symm
  rw [Int.floor_eq_iff]
  by_cases hr : r < 0
  · simp only [hr, ↓reduceIte]
    have hr' : (r : ℚ) < 0 := by exact_mod_cast hr
    push_cast
    constructor
    · rw [le_div_iff₀ hdq]; nlinarith
    · rw [div_lt_iff₀ hdq]; nlinarith
  · simp only [hr, ↓reduceIte]
    have hr' : (0 : ℚ) ≤ r := by exact_mod_cast not_lt.mp hr
    constructor
    · rw [le_div_iff₀ hdq]; nlinarith
    · rw [div_lt_iff₀ hdq]; nlinarith

/-- Up: the corrected truncating quotient is the ceiling of `n/d`. -/
theorem up_is_ceil (n d q r : ℤ) (hd : 0 < d) (h : n = d * q + r)
    (h1 : -d < r) (h2 : r < d) :
    (if 0 < r then q + 1 else q) = ⌈(n : ℚ) / d⌉ := by
  have hdq : (0 : ℚ) < d := by exact_mod_cast hd
  have hn : (n : ℚ) = d * q + r := by exact_mod_cast h
  have h1' : (-(d : ℚ)) < r := by exact_mod_cast h1
  have h2' : (r : ℚ) < d := by exact_mod_cast h2
  symm
  rw [Int.ceil_eq_iff]
  by_cases hr : 0 < r
  · simp only [hr, ↓reduceIte]
    have hr' : (0 : ℚ) < r := by exact_mod_cast hr
    push_cast
    constructor
    · rw [lt_div_iff₀ hdq]; nlinarith
    · rw [div_le_iff₀ hdq]; nlinarith
  · simp only [hr, ↓reduceIte]
    have hr' : (r : ℚ) ≤ 0 := by exact_mod_cast not_lt.mp hr
    constructor
    · rw [lt_div_iff₀ hdq]; nlinarith
    · rw [div_le_iff₀ hdq]; nlinarith

/-- Outward rounding is exact: floor and ceiling of the scaled value, scaled back, bracket `x`. -/
theorem round_outward (x : ℚ) (s : ℤ) :
    ((⌊x * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s ≤ x ∧ x ≤ ((⌈x * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s := by
  have hp : (0 : ℚ) < 2 ^ s := zpow_pos (by norm_num) s
  constructor
  · rw [div_le_iff₀ hp]; exact Int.floor_le _
  · rw [le_div_iff₀ hp]; exact Int.le_ceil _

/-- The rounded values are within `2^-s` of `x`. -/
theorem round_error (x : ℚ) (s : ℤ) :
    x - ((⌊x * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s < 1 / 2 ^ s ∧
    ((⌈x * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s - x < 1 / 2 ^ s := by
  have hp : (0 : ℚ) < 2 ^ s := zpow_pos (by norm_num) s
  have hne : (2 : ℚ) ^ s ≠ 0 := hp.ne'
  constructor
  · have e : x - ((⌊x * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s = (x * 2 ^ s - (⌊x * 2 ^ s⌋ : ℤ)) / 2 ^ s := by
      field_simp
    rw [e, div_lt_div_iff_of_pos_right hp]
    linarith [Int.lt_floor_add_one (x * 2 ^ s)]
  · have e : ((⌈x * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s - x = ((⌈x * 2 ^ s⌉ : ℤ) - x * 2 ^ s) / 2 ^ s := by
      field_simp
    rw [e, div_lt_div_iff_of_pos_right hp]
    linarith [Int.ceil_lt_add_one (x * 2 ^ s)]

/-- Sign is kept: `|x·2^s| > 1` makes both roundings nonzero with the sign of `x`. -/
theorem round_sign (x : ℚ) (s : ℤ) (hx : 1 < |x * 2 ^ s|) :
    (0 < x → 0 < ⌊x * 2 ^ s⌋ ∧ 0 < ⌈x * 2 ^ s⌉) ∧
    (x < 0 → ⌊x * 2 ^ s⌋ < 0 ∧ ⌈x * 2 ^ s⌉ < 0) := by
  have hp : (0 : ℚ) < 2 ^ s := zpow_pos (by norm_num) s
  constructor
  · intro h
    have hy : 0 < x * 2 ^ s := mul_pos h hp
    rw [abs_of_pos hy] at hx
    exact ⟨Int.floor_pos.mpr hx.le, Int.ceil_pos.mpr hy⟩
  · intro h
    have hy : x * 2 ^ s < 0 := mul_neg_of_neg_of_pos h hp
    rw [abs_of_neg hy] at hx
    constructor
    · exact Int.floor_lt.mpr (by push_cast; linarith)
    · have : ⌈x * 2 ^ s⌉ ≤ -1 := Int.ceil_le.mpr (by push_cast; linarith)
      omega

/-- The scale `s = bits - (len n - len d)` of `proof.lazyRound`, with `len` the bit length (A8), puts
`|n/d·2^s|` strictly between `2^(bits-1)` and `2^(bits+1)`. -/
theorem scale_range (n d : ℤ) (bits : ℕ) (s : ℤ) (hn : n ≠ 0) (hd : 0 < d)
    (hs : s = (bits : ℤ) - ((Nat.size n.natAbs : ℤ) - (Nat.size d.natAbs : ℤ))) :
    (2 : ℚ) ^ ((bits : ℤ) - 1) < |(n : ℚ) / d * 2 ^ s| ∧
      |(n : ℚ) / d * 2 ^ s| < (2 : ℚ) ^ ((bits : ℤ) + 1) := by
  set a := Nat.size n.natAbs with ha
  set b := Nat.size d.natAbs with hb
  have hnpos : 0 < n.natAbs := Int.natAbs_pos.mpr hn
  have hdpos : 0 < d.natAbs := Int.natAbs_pos.mpr hd.ne'
  have ha1 : 0 < a := Nat.size_pos.mpr hnpos
  have hb1 : 0 < b := Nat.size_pos.mpr hdpos
  have hNlo : 2 ^ (a - 1) ≤ n.natAbs := Nat.lt_size.mp (by omega)
  have hNhi : n.natAbs < 2 ^ a := Nat.lt_size_self _
  have hDlo : 2 ^ (b - 1) ≤ d.natAbs := Nat.lt_size.mp (by omega)
  have hDhi : d.natAbs < 2 ^ b := Nat.lt_size_self _
  have hNq : ((n.natAbs : ℕ) : ℚ) = |(n : ℚ)| := by
    rw [← Int.cast_abs, ← Int.natCast_natAbs]; simp
  have hDq : ((d.natAbs : ℕ) : ℚ) = (d : ℚ) := by
    rw [← Int.cast_natCast, Int.natCast_natAbs, abs_of_pos hd]
  have hNlo' : (2 : ℚ) ^ (a - 1) ≤ |(n : ℚ)| := by rw [← hNq]; exact_mod_cast hNlo
  have hNhi' : |(n : ℚ)| < 2 ^ a := by rw [← hNq]; exact_mod_cast hNhi
  have hDlo' : (2 : ℚ) ^ (b - 1) ≤ d := by rw [← hDq]; exact_mod_cast hDlo
  have hDhi' : (d : ℚ) < 2 ^ b := by rw [← hDq]; exact_mod_cast hDhi
  have hdq : (0 : ℚ) < d := by exact_mod_cast hd
  have hA : (0 : ℚ) < 2 ^ a := by positivity
  have hB : (0 : ℚ) < 2 ^ b := by positivity
  have hAe : (2 : ℚ) ^ a = 2 * 2 ^ (a - 1) := by
    rw [← pow_succ']; congr 1; omega
  have hBe : (2 : ℚ) ^ b = 2 * 2 ^ (b - 1) := by
    rw [← pow_succ']; congr 1; omega
  have h2s : (2 : ℚ) ^ s = 2 ^ bits * 2 ^ b / 2 ^ a := by
    have : s = (bits : ℤ) + (b : ℤ) - (a : ℤ) := by rw [hs]; ring
    rw [this, zpow_sub₀ (by norm_num), zpow_add₀ (by norm_num)]
    simp only [zpow_natCast]
  have hbits : (2 : ℚ) ^ ((bits : ℤ) - 1) = 2 ^ bits / 2 := by
    rw [zpow_sub₀ (by norm_num)]; simp
  have hbits' : (2 : ℚ) ^ ((bits : ℤ) + 1) = 2 ^ bits * 2 := by
    rw [zpow_add₀ (by norm_num)]; simp
  have hP : (0 : ℚ) < 2 ^ bits := by positivity
  have habs : |(n : ℚ) / d * 2 ^ s| = |(n : ℚ)| / d * (2 ^ bits * 2 ^ b / 2 ^ a) := by
    rw [abs_mul, abs_div, abs_of_pos hdq, h2s,
      abs_of_pos (show (0 : ℚ) < 2 ^ bits * 2 ^ b / 2 ^ a by positivity)]
  rw [habs, hbits, hbits']
  constructor
  · -- |n|/d > 2^(a-1)/2^b, strictly because d < 2^b
    have h1 : (2 : ℚ) ^ (a - 1) / 2 ^ b < |(n : ℚ)| / d := by
      rw [div_lt_div_iff₀ hB hdq]
      nlinarith [pow_pos (by norm_num : (0 : ℚ) < 2) (a - 1)]
    have h2 : (2 : ℚ) ^ (a - 1) / 2 ^ b * (2 ^ bits * 2 ^ b / 2 ^ a) = 2 ^ bits / 2 := by
      rw [hAe]; field_simp
    rw [← h2]
    exact mul_lt_mul_of_pos_right h1 (by positivity)
  · -- |n|/d < 2^a/2^(b-1), strictly because |n| < 2^a
    have h1 : |(n : ℚ)| / d < 2 ^ a / 2 ^ (b - 1) := by
      rw [div_lt_div_iff₀ hdq (by positivity)]
      nlinarith [pow_pos (by norm_num : (0 : ℚ) < 2) (b - 1)]
    have h2 : (2 : ℚ) ^ a / 2 ^ (b - 1) * (2 ^ bits * 2 ^ b / 2 ^ a) = 2 ^ bits * 2 := by
      rw [hBe]; field_simp
    rw [← h2]
    exact mul_lt_mul_of_pos_right h1 (by positivity)

/-- With `2^(bits-1) < |x·2^s|`, the floor and ceiling results are each within `2^(1-bits)·|x|` of `x`. This
is the relative-error claim in the doc comment of `proof.LazyRoundDown`. -/
theorem round_rel_error (x : ℚ) (s : ℤ) (bits : ℕ)
    (hx : (2 : ℚ) ^ ((bits : ℤ) - 1) < |x * 2 ^ s|) :
    x - ((⌊x * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s < (2 : ℚ) ^ (1 - (bits : ℤ)) * |x| ∧
    ((⌈x * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s - x < (2 : ℚ) ^ (1 - (bits : ℤ)) * |x| := by
  have hp : (0 : ℚ) < 2 ^ s := zpow_pos (by norm_num) s
  have habs : |x * 2 ^ s| = |x| * 2 ^ s := by rw [abs_mul, abs_of_pos hp]
  have hprod : (2 : ℚ) ^ (1 - (bits : ℤ)) * 2 ^ ((bits : ℤ) - 1) = 1 := by
    rw [← zpow_add₀ (by norm_num)]; simp
  have hpos : (0 : ℚ) < 2 ^ (1 - (bits : ℤ)) := zpow_pos (by norm_num) _
  have key : 1 < (2 : ℚ) ^ (1 - (bits : ℤ)) * |x| * 2 ^ s := by
    rw [mul_assoc, ← habs]
    nlinarith
  have hfin : 1 / (2 : ℚ) ^ s < (2 : ℚ) ^ (1 - (bits : ℤ)) * |x| := by
    rw [div_lt_iff₀ hp]; linarith
  obtain ⟨e1, e2⟩ := round_error x s
  exact ⟨lt_trans e1 hfin, lt_trans e2 hfin⟩

/-- Model of `proof.lazyRound` on `x = n/d` with `d > 0`, branch for branch: the early return, the scale, the
shift of `n` or `d`, the truncating `QuoRem` (`Int.tdiv`, `Int.tmod`, A8), the correction by one, and the
result `m/2^s`. The Go code also returns `m·2^|s|` over one when `s ≤ 0`; both denote `m/2^s`. -/
@[expose] def lazyRoundModel (n d : ℤ) (bits : ℕ) (up : Bool) : ℚ :=
  if n = 0 ∨ Nat.size n.natAbs + Nat.size d.natAbs ≤ 4 * bits then (n : ℚ) / d
  else
    let s : ℤ := (bits : ℤ) - ((Nat.size n.natAbs : ℤ) - (Nat.size d.natAbs : ℤ))
    let n' : ℤ := if 0 ≤ s then n * 2 ^ s.toNat else n
    let d' : ℤ := if 0 ≤ s then d else d * 2 ^ (-s).toNat
    let q := n'.tdiv d'
    let r := n'.tmod d'
    let m : ℤ := if up = true ∧ 0 < r then q + 1 else if up = false ∧ r < 0 then q - 1 else q
    (m : ℚ) / 2 ^ s

/-- The shifted fraction of `proof.lazyRound` equals `x·2^s`, and its denominator stays positive. -/
theorem scaled_frac (n d : ℤ) (s : ℤ) (hd : 0 < d) :
    0 < (if 0 ≤ s then d else d * 2 ^ (-s).toNat) ∧
    (((if 0 ≤ s then n * 2 ^ s.toNat else n : ℤ) : ℚ) /
      ((if 0 ≤ s then d else d * 2 ^ (-s).toNat : ℤ) : ℚ)) = (n : ℚ) / d * 2 ^ s := by
  have hdq : (d : ℚ) ≠ 0 := by exact_mod_cast hd.ne'
  by_cases hs : 0 ≤ s
  · simp only [hs, ↓reduceIte]
    refine ⟨hd, ?_⟩
    have : (2 : ℚ) ^ s = 2 ^ s.toNat := by
      rw [← zpow_natCast, Int.toNat_of_nonneg hs]
    push_cast
    rw [this]; field_simp
  · simp only [hs, ↓reduceIte]
    refine ⟨by positivity, ?_⟩
    have hneg : 0 ≤ -s := by omega
    have : (2 : ℚ) ^ s = ((2 : ℚ) ^ (-s).toNat)⁻¹ := by
      rw [← zpow_natCast, Int.toNat_of_nonneg hneg, zpow_neg, inv_inv]
    push_cast
    rw [this]; field_simp

/-- The corrected truncating quotient of `n'/d'` is the floor (down) or ceiling (up) of `x·2^s` when
`n'/d' = x·2^s`, so `proof.lazyRound` returns `⌊x·2^s⌋/2^s` or `⌈x·2^s⌉/2^s`. -/
theorem round_via (x : ℚ) (s : ℤ) (n' d' : ℤ) (hd : 0 < d') (hfrac : (n' : ℚ) / d' = x * 2 ^ s)
    (up : Bool) :
    (((if up = true ∧ 0 < n'.tmod d' then n'.tdiv d' + 1
        else if up = false ∧ n'.tmod d' < 0 then n'.tdiv d' - 1 else n'.tdiv d' : ℤ) : ℚ) / 2 ^ s) =
      if up = true then ((⌈x * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s else ((⌊x * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s := by
  have hdec : n' = d' * n'.tdiv d' + n'.tmod d' := by
    have := Int.tmod_add_tdiv_mul n' d'; linarith
  have h1 := Int.lt_tmod_of_pos n' hd
  have h2 := Int.tmod_lt_of_pos n' hd
  rw [← hfrac]
  cases up
  · simp only [Bool.false_eq_true, false_and, ↓reduceIte, true_and]
    rw [down_is_floor n' d' _ _ hd hdec h1 h2]
  · simp only [true_and, Bool.true_eq_false, false_and, ↓reduceIte]
    rw [up_is_ceil n' d' _ _ hd hdec h1 h2]

/-- `proof.LazyRoundDown` and `proof.LazyRoundUp` (through `proof.lazyRound`) round outward with a bounded
relative error. For `x = n/d`, `d > 0` and `bits ≥ 1`: the down result is at most `x` and the up result is at
least `x`. When the early return does not fire (`n ≠ 0` and `4·bits` below the combined bit length), both
results are nonzero with the sign of `x` and within `2^(1-bits)·|x|` of `x`. The `QuoRem` and `BitLen` facts
the model uses are A8. -/
theorem lazyRound_sound (n d : ℤ) (bits : ℕ) (up : Bool) (hd : 0 < d) (hb : 1 ≤ bits) :
    (up = false → lazyRoundModel n d bits up ≤ (n : ℚ) / d) ∧
    (up = true → (n : ℚ) / d ≤ lazyRoundModel n d bits up) ∧
    (n ≠ 0 → 4 * bits < Nat.size n.natAbs + Nat.size d.natAbs →
      ((0 : ℚ) < (n : ℚ) / d → 0 < lazyRoundModel n d bits up) ∧
      ((n : ℚ) / d < 0 → lazyRoundModel n d bits up < 0) ∧
      |lazyRoundModel n d bits up - (n : ℚ) / d| < (2 : ℚ) ^ (1 - (bits : ℤ)) * |(n : ℚ) / d|) := by
  by_cases hearly : n = 0 ∨ Nat.size n.natAbs + Nat.size d.natAbs ≤ 4 * bits
  · have hm : lazyRoundModel n d bits up = (n : ℚ) / d := by
      unfold lazyRoundModel; simp only [hearly, ↓reduceIte]
    rw [hm]
    refine ⟨fun _ => le_rfl, fun _ => le_rfl, ?_⟩
    intro hn h4
    rcases hearly with h | h
    · exact absurd h hn
    · omega
  · have hn : n ≠ 0 := fun h => hearly (Or.inl h)
    have h4 : 4 * bits < Nat.size n.natAbs + Nat.size d.natAbs := by
      by_contra h; exact hearly (Or.inr (by omega))
    set s : ℤ := (bits : ℤ) - ((Nat.size n.natAbs : ℤ) - (Nat.size d.natAbs : ℤ)) with hs
    obtain ⟨hdpos, hfrac⟩ := scaled_frac n d s hd
    have hmodel : lazyRoundModel n d bits up =
        if up = true then ((⌈(n : ℚ) / d * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s
        else ((⌊(n : ℚ) / d * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s := by
      unfold lazyRoundModel
      simp only [hearly, ↓reduceIte]
      exact round_via _ s _ _ hdpos hfrac up
    obtain ⟨hlo, -⟩ := scale_range n d bits s hn hd hs
    set x : ℚ := (n : ℚ) / d with hx
    obtain ⟨hout1, hout2⟩ := round_outward x s
    have hbpos : (1 : ℚ) ≤ 2 ^ ((bits : ℤ) - 1) := by
      apply one_le_zpow₀ (by norm_num); omega
    have hsign := round_sign x s (lt_of_le_of_lt hbpos hlo)
    obtain ⟨herr1, herr2⟩ := round_rel_error x s bits hlo
    have hp : (0 : ℚ) < 2 ^ s := zpow_pos (by norm_num) s
    refine ⟨?_, ?_, fun _ _ => ?_⟩
    · intro hu; rw [hmodel, hu]; simpa using hout1
    · intro hu; rw [hmodel, hu]; simpa using hout2
    · cases up
      · have hmodel' : lazyRoundModel n d bits false = ((⌊x * 2 ^ s⌋ : ℤ) : ℚ) / 2 ^ s := by
          rw [hmodel]; simp
        rw [hmodel']
        refine ⟨fun hpos => ?_, fun hneg => ?_, ?_⟩
        · exact div_pos (by exact_mod_cast (hsign.1 hpos).1) hp
        · exact div_neg_of_neg_of_pos (by exact_mod_cast (hsign.2 hneg).1) hp
        · rw [abs_sub_comm, abs_of_nonneg (by linarith)]; exact herr1
      · have hmodel' : lazyRoundModel n d bits true = ((⌈x * 2 ^ s⌉ : ℤ) : ℚ) / 2 ^ s := by
          rw [hmodel]; simp
        rw [hmodel']
        refine ⟨fun hpos => ?_, fun hneg => ?_, ?_⟩
        · exact div_pos (by exact_mod_cast (hsign.1 hpos).2) hp
        · exact div_neg_of_neg_of_pos (by exact_mod_cast (hsign.2 hneg).2) hp
        · rw [abs_of_nonneg (by linarith)]; exact herr2

end Decad.Rational

end
