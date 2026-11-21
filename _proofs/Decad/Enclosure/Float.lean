module

public import Decad.Float.Model

/-!
# Outward float enclosures

Models of `proof.FloatBounds`, `floatDown`, `floatUp`, `floatMulDown`, `floatMulUp` and
`proof.DotSubEnclosure`. Each directed operation rounds one operation to nearest and steps one float
outward, so the exact result lies inside. `DotSubEnclosure` then brackets `x·y - c` over a box of `x`, a box
of `y` and an interval of `c`.

`proof.FloatBounds` returns the same float twice when `Dyadic.Float64` reports an exact conversion, and the
two neighbours of the rounded value otherwise. Overflow to an infinity is not modeled (A7 in
`agent-docs/proofs-design.md`), so every theorem holds for finite values. The Go code widens an overflowed
end to `MaxFloat64`, which also encloses; that branch is not proved here.
-/

public section

namespace Decad.Enclosure

open Decad.Float

variable (M : FloatModel)

/-- Model of `math.Nextafter(x, -Inf)` on a float: the float just below `f` (A15). -/
@[expose] noncomputable def nd (f : ℝ) : ℝ := -M.nu (-f)

theorem nd_mem {f : ℝ} (hf : f ∈ M.F) : nd M f ∈ M.F :=
  M.neg_mem _ (M.nu_mem _ (M.neg_mem _ hf))

theorem nd_lt {f : ℝ} (hf : f ∈ M.F) : nd M f < f := by
  unfold nd
  have := M.lt_nu _ (M.neg_mem _ hf)
  linarith

/-- Rounding to nearest then stepping down never lands above `x`. -/
theorem down_le (x : ℝ) : nd M (M.rn x) ≤ x := by
  by_contra h
  push Not at h
  have hf := M.rn_mem x
  have hlt := nd_lt M hf
  have hnd := nd_mem M hf
  have hnear := M.rn_near x _ hnd
  rw [abs_of_pos (by linarith : 0 < M.rn x - x),
      abs_of_pos (by linarith : 0 < nd M (M.rn x) - x)] at hnear
  linarith

/-- Model of `floatDown (rn x)`: one operation rounded to nearest, then stepped down. -/
@[expose] noncomputable def stepDown (x : ℝ) : ℝ := nd M (M.rn x)

/-- Model of `floatUp (rn x)`: one operation rounded to nearest, then stepped up. -/
@[expose] noncomputable def stepUp (x : ℝ) : ℝ := M.nu (M.rn x)

theorem stepDown_mem (x : ℝ) : stepDown M x ∈ M.F := nd_mem M (M.rn_mem x)

theorem stepUp_mem (x : ℝ) : stepUp M x ∈ M.F := M.nu_mem _ (M.rn_mem x)

/-- `floatDown` of a rounded result is at most the exact result. -/
theorem stepDown_le (x : ℝ) : stepDown M x ≤ x := down_le M x

/-- `floatUp` of a rounded result is at least the exact result. -/
theorem stepUp_ge (x : ℝ) : x ≤ stepUp M x := M.up_ge x

/-- Model of `floatMulDown`: the product rounded to nearest, stepped down. The explicit `float64()` conversion
in the Go keeps the product from fusing into a later add (A1). -/
@[expose] noncomputable def mulDown (a b : ℝ) : ℝ := stepDown M (a * b)

/-- Model of `floatMulUp`. -/
@[expose] noncomputable def mulUp (a b : ℝ) : ℝ := stepUp M (a * b)

/-- `floatMulDown a b` is a float at most `a·b`. -/
theorem mulDown_le (a b : ℝ) : mulDown M a b ≤ a * b := stepDown_le M _

/-- `floatMulUp a b` is a float at least `a·b`. -/
theorem mulUp_ge (a b : ℝ) : a * b ≤ mulUp M a b := stepUp_ge M _

theorem min_mem {x y : ℝ} (hx : x ∈ M.F) (hy : y ∈ M.F) : min x y ∈ M.F := by
  rcases min_choice x y with h | h <;> rw [h] <;> assumption

theorem max_mem {x y : ℝ} (hx : x ∈ M.F) (hy : y ∈ M.F) : max x y ∈ M.F := by
  rcases max_choice x y with h | h <;> rw [h] <;> assumption

/-- Model of `proof.FloatBounds`. `ex` is the exact flag of `Dyadic.Float64`; the conversion is `rn d`. -/
@[expose] noncomputable def floatBounds (d : ℝ) (ex : Bool) : ℝ × ℝ :=
  if ex then (M.rn d, M.rn d) else (nd M (M.rn d), M.nu (M.rn d))

/-- `proof.FloatBounds` brackets `d` between two floats. Hypothesis `hex` is A16: an exact flag means the
rounded conversion equals `d`. -/
theorem floatBounds_enclose (d : ℝ) (ex : Bool) (hex : ex = true → M.rn d = d) :
    (floatBounds M d ex).1 ≤ d ∧ d ≤ (floatBounds M d ex).2 ∧
      (floatBounds M d ex).1 ∈ M.F ∧ (floatBounds M d ex).2 ∈ M.F := by
  unfold floatBounds
  cases ex with
  | true =>
    have := hex rfl
    simp only [↓reduceIte]
    exact ⟨le_of_eq this, le_of_eq this.symm, M.rn_mem d, M.rn_mem d⟩
  | false =>
    simp only [Bool.false_eq_true, ↓reduceIte]
    exact ⟨down_le M d, M.up_ge d, nd_mem M (M.rn_mem d), M.nu_mem _ (M.rn_mem d)⟩

/-- A product of values in two intervals is at least any `L` that is at most all four corner products. -/
theorem prod_ge {a b c d x y L : ℝ} (hax : a ≤ x) (hxb : x ≤ b) (hcy : c ≤ y) (hyd : y ≤ d)
    (h1 : L ≤ a * c) (h2 : L ≤ a * d) (h3 : L ≤ b * c) (h4 : L ≤ b * d) : L ≤ x * y := by
  rcases le_total 0 y with hy | hy
  · have h5 : a * y ≤ x * y := mul_le_mul_of_nonneg_right hax hy
    rcases le_total 0 a with ha | ha
    · have : a * c ≤ a * y := mul_le_mul_of_nonneg_left hcy ha
      linarith
    · have : a * d ≤ a * y := by nlinarith [mul_nonneg (neg_nonneg.mpr ha) (sub_nonneg.mpr hyd)]
      linarith
  · have h5 : b * y ≤ x * y := by nlinarith [mul_nonneg (sub_nonneg.mpr hxb) (neg_nonneg.mpr hy)]
    rcases le_total 0 b with hb | hb
    · have : b * c ≤ b * y := mul_le_mul_of_nonneg_left hcy hb
      linarith
    · have : b * d ≤ b * y := by nlinarith [mul_nonneg (neg_nonneg.mpr hb) (sub_nonneg.mpr hyd)]
      linarith

/-- A product of values in two intervals is at most any `U` that is at least all four corner products. -/
theorem prod_le {a b c d x y U : ℝ} (hax : a ≤ x) (hxb : x ≤ b) (hcy : c ≤ y) (hyd : y ≤ d)
    (h1 : a * c ≤ U) (h2 : a * d ≤ U) (h3 : b * c ≤ U) (h4 : b * d ≤ U) : x * y ≤ U := by
  have := prod_ge (a := a) (b := b) (c := -d) (d := -c) (x := x) (y := -y) (L := -U)
    hax hxb (by linarith) (by linarith)
    (by rw [mul_neg]; linarith) (by rw [mul_neg]; linarith)
    (by rw [mul_neg]; linarith) (by rw [mul_neg]; linarith)
  rw [mul_neg] at this
  linarith

/-- Lower end of one axis's product range in `proof.DotSubEnclosure`. -/
@[expose] noncomputable def axisLo (a b c d : ℝ) : ℝ :=
  min (min (mulDown M a c) (mulDown M a d)) (min (mulDown M b c) (mulDown M b d))

/-- Upper end of one axis's product range in `proof.DotSubEnclosure`. -/
@[expose] noncomputable def axisHi (a b c d : ℝ) : ℝ :=
  max (max (mulUp M a c) (mulUp M a d)) (max (mulUp M b c) (mulUp M b d))

theorem axisLo_mem (a b c d : ℝ) : axisLo M a b c d ∈ M.F :=
  min_mem M (min_mem M (stepDown_mem M _) (stepDown_mem M _))
    (min_mem M (stepDown_mem M _) (stepDown_mem M _))

theorem axisHi_mem (a b c d : ℝ) : axisHi M a b c d ∈ M.F :=
  max_mem M (max_mem M (stepUp_mem M _) (stepUp_mem M _))
    (max_mem M (stepUp_mem M _) (stepUp_mem M _))

/-- The lower end of an axis's range is at most `x·y` for `x` in `[a, b]` and `y` in `[c, d]`. -/
theorem axisLo_le {a b c d x y : ℝ} (hax : a ≤ x) (hxb : x ≤ b) (hcy : c ≤ y) (hyd : y ≤ d) :
    axisLo M a b c d ≤ x * y := by
  have e1 := mulDown_le M a c
  have e2 := mulDown_le M a d
  have e3 := mulDown_le M b c
  have e4 := mulDown_le M b d
  refine prod_ge hax hxb hcy hyd ?_ ?_ ?_ ?_ <;> unfold axisLo
  · exact le_trans (min_le_left _ _) (le_trans (min_le_left _ _) e1)
  · exact le_trans (min_le_left _ _) (le_trans (min_le_right _ _) e2)
  · exact le_trans (min_le_right _ _) (le_trans (min_le_left _ _) e3)
  · exact le_trans (min_le_right _ _) (le_trans (min_le_right _ _) e4)

/-- The upper end of an axis's range is at least `x·y`. -/
theorem axisHi_ge {a b c d x y : ℝ} (hax : a ≤ x) (hxb : x ≤ b) (hcy : c ≤ y) (hyd : y ≤ d) :
    x * y ≤ axisHi M a b c d := by
  have e1 := mulUp_ge M a c
  have e2 := mulUp_ge M a d
  have e3 := mulUp_ge M b c
  have e4 := mulUp_ge M b d
  refine prod_le hax hxb hcy hyd ?_ ?_ ?_ ?_ <;> unfold axisHi
  · exact le_trans e1 (le_trans (le_max_left _ _) (le_max_left _ _))
  · exact le_trans e2 (le_trans (le_max_right _ _) (le_max_left _ _))
  · exact le_trans e3 (le_trans (le_max_left _ _) (le_max_right _ _))
  · exact le_trans e4 (le_trans (le_max_right _ _) (le_max_right _ _))

/-- Model of `proof.DotSubEnclosure`'s lower end: `floatDown(lo - cHi)` after three
`lo = floatDown(lo + plo)` steps from zero. `p0, p1, p2` are the three axes' `plo`. -/
@[expose] noncomputable def dotLo (p0 p1 p2 cHi : ℝ) : ℝ :=
  stepDown M (stepDown M (stepDown M (stepDown M (0 + p0) + p1) + p2) - cHi)

/-- Model of `proof.DotSubEnclosure`'s upper end: `floatUp(hi - cLo)`. -/
@[expose] noncomputable def dotHi (q0 q1 q2 cLo : ℝ) : ℝ :=
  stepUp M (stepUp M (stepUp M (stepUp M (0 + q0) + q1) + q2) - cLo)

/-- `proof.DotSubEnclosure` brackets `x·y - c` for every pair of points in the two boxes and every `c` in
`[cLo, cHi]`. The box ends and `cLo`, `cHi` are floats; overflow is not modeled (A7). Every `lo`/`hi`
update in the Go is one rounded add stepped outward, so each partial bound stays on the correct side. -/
theorem dotSub_enclosure
    {a0 b0 c0 d0 a1 b1 c1 d1 a2 b2 c2 d2 x0 y0 x1 y1 x2 y2 cLo cHi c : ℝ}
    (hx0 : a0 ≤ x0 ∧ x0 ≤ b0) (hy0 : c0 ≤ y0 ∧ y0 ≤ d0)
    (hx1 : a1 ≤ x1 ∧ x1 ≤ b1) (hy1 : c1 ≤ y1 ∧ y1 ≤ d1)
    (hx2 : a2 ≤ x2 ∧ x2 ≤ b2) (hy2 : c2 ≤ y2 ∧ y2 ≤ d2)
    (hc : cLo ≤ c ∧ c ≤ cHi) :
    dotLo M (axisLo M a0 b0 c0 d0) (axisLo M a1 b1 c1 d1) (axisLo M a2 b2 c2 d2) cHi
        ≤ x0 * y0 + x1 * y1 + x2 * y2 - c ∧
    x0 * y0 + x1 * y1 + x2 * y2 - c ≤
      dotHi M (axisHi M a0 b0 c0 d0) (axisHi M a1 b1 c1 d1) (axisHi M a2 b2 c2 d2) cLo := by
  have l0 := axisLo_le M hx0.1 hx0.2 hy0.1 hy0.2
  have l1 := axisLo_le M hx1.1 hx1.2 hy1.1 hy1.2
  have l2 := axisLo_le M hx2.1 hx2.2 hy2.1 hy2.2
  have u0 := axisHi_ge M hx0.1 hx0.2 hy0.1 hy0.2
  have u1 := axisHi_ge M hx1.1 hx1.2 hy1.1 hy1.2
  have u2 := axisHi_ge M hx2.1 hx2.2 hy2.1 hy2.2
  constructor
  · unfold dotLo
    have s1 := stepDown_le M (0 + axisLo M a0 b0 c0 d0)
    have s2 := stepDown_le M (stepDown M (0 + axisLo M a0 b0 c0 d0) + axisLo M a1 b1 c1 d1)
    have s3 := stepDown_le M (stepDown M (stepDown M (0 + axisLo M a0 b0 c0 d0)
      + axisLo M a1 b1 c1 d1) + axisLo M a2 b2 c2 d2)
    have s4 := stepDown_le M (stepDown M (stepDown M (stepDown M (0 + axisLo M a0 b0 c0 d0)
      + axisLo M a1 b1 c1 d1) + axisLo M a2 b2 c2 d2) - cHi)
    linarith [hc.2]
  · unfold dotHi
    have s1 := stepUp_ge M (0 + axisHi M a0 b0 c0 d0)
    have s2 := stepUp_ge M (stepUp M (0 + axisHi M a0 b0 c0 d0) + axisHi M a1 b1 c1 d1)
    have s3 := stepUp_ge M (stepUp M (stepUp M (0 + axisHi M a0 b0 c0 d0)
      + axisHi M a1 b1 c1 d1) + axisHi M a2 b2 c2 d2)
    have s4 := stepUp_ge M (stepUp M (stepUp M (stepUp M (0 + axisHi M a0 b0 c0 d0)
      + axisHi M a1 b1 c1 d1) + axisHi M a2 b2 c2 d2) - cLo)
    linarith [hc.1]

end Decad.Enclosure

end
