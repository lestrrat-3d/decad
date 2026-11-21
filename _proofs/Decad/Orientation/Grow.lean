module

public import Decad.Orientation.Orient2

/-!
# Exact fallback of `proof.OrientSignExpansion`

When `proof.orientSignSum2` cannot decide, `proof.orientSignGrow` builds the determinant as an exact
zero-eliminating expansion with `orientationExpansion.add` (Shewchuk's Grow-Expansion) and reads the sign
of its highest component. This file models that code over `ℝ` and proves:

* `growAux_sum`: one `add` keeps the exact sum of the expansion plus the added value (A9).
* `growAux_length`: one `add` grows the expansion by at most one component, so six triples need at most 24
  slots and `orientationExpansion.terms` never overflows.
* `growAux_ne_zero`: every stored component is nonzero, so the final sign read is never of a zero.
* `tripleAdd_sum`: the four `add` calls of one triple add exactly `x·y·z` (A9, A10).
* `orientGrow_sound`: `proof.orientSignGrow` returns the sign of the exact determinant, provided the final
  expansion's highest component outweighs the sum of the others (A14).
-/

public section

namespace Decad.Orientation

open Decad.Float

/-- Model of one pass of `orientationExpansion.add`: each stored component and `q` go through `TwoSum`; a
nonzero error is stored, and the final `q` is stored when nonzero. The component order is the storage order,
lowest first. -/
@[expose] noncomputable def growAux (M : FloatModel) : List ℝ → ℝ → List ℝ
  | [], q => if q ≠ 0 then [q] else []
  | c :: cs, q =>
    (if q + c - M.rn (q + c) ≠ 0 then [q + c - M.rn (q + c)] else []) ++ growAux M cs (M.rn (q + c))

/-- Model of `orientationExpansion.add`: a zero `q` leaves the expansion alone. -/
@[expose] noncomputable def addTerm (M : FloatModel) (e : List ℝ) (q : ℝ) : List ℝ :=
  if q = 0 then e else growAux M e q

/-- One `add` pass keeps the exact sum. The `TwoSum` error is exact (A9). -/
theorem growAux_sum (M : FloatModel) (e : List ℝ) (q : ℝ) :
    (growAux M e q).sum = e.sum + q := by
  induction e generalizing q with
  | nil =>
    unfold growAux
    by_cases h : q = 0 <;> simp [h]
  | cons c cs ih =>
    unfold growAux
    by_cases h : q + c - M.rn (q + c) = 0
    · simp only [h, ne_eq, not_true_eq_false, ↓reduceIte, List.nil_append, List.sum_cons, ih]
      linarith
    · simp only [h, ne_eq, not_false_eq_true, ↓reduceIte, List.cons_append, List.nil_append,
        List.sum_cons, ih]
      ring

theorem addTerm_sum (M : FloatModel) (e : List ℝ) (q : ℝ) :
    (addTerm M e q).sum = e.sum + q := by
  unfold addTerm
  by_cases h : q = 0
  · simp [h]
  · simp only [h, ↓reduceIte, growAux_sum]

/-- One `add` pass adds at most one component. -/
theorem growAux_length (M : FloatModel) (e : List ℝ) (q : ℝ) :
    (growAux M e q).length ≤ e.length + 1 := by
  induction e generalizing q with
  | nil =>
    unfold growAux
    split_ifs <;> simp
  | cons c cs ih =>
    unfold growAux
    have := ih (M.rn (q + c))
    split_ifs <;> simp <;> omega

theorem addTerm_length (M : FloatModel) (e : List ℝ) (q : ℝ) :
    (addTerm M e q).length ≤ e.length + 1 := by
  unfold addTerm
  split_ifs
  · omega
  · exact growAux_length M e q

/-- Every component `add` stores is nonzero. -/
theorem growAux_ne_zero (M : FloatModel) (e : List ℝ) (q : ℝ) : ∀ x ∈ growAux M e q, x ≠ 0 := by
  induction e generalizing q with
  | nil =>
    intro x hx
    unfold growAux at hx
    split_ifs at hx with h
    · simp only [List.mem_singleton] at hx
      rw [hx]; exact h
    · simp at hx
  | cons c cs ih =>
    intro x hx
    unfold growAux at hx
    rw [List.mem_append] at hx
    rcases hx with hx | hx
    · split_ifs at hx with h
      · simp only [List.mem_singleton] at hx
        rw [hx]; exact h
      · simp at hx
    · exact ih _ x hx

theorem addTerm_ne_zero (M : FloatModel) {e : List ℝ} (he : ∀ x ∈ e, x ≠ 0) (q : ℝ) :
    ∀ x ∈ addTerm M e q, x ≠ 0 := by
  unfold addTerm
  split_ifs
  · exact he
  · exact growAux_ne_zero M e q

/-- Model of one triple's contribution in `proof.orientSignGrow`: the four `add` calls in order `tailLo`,
`tailHi`, `lo`, `hi`, the first two only when `xyTail` is nonzero. The FMA remainders are exact (A10). -/
@[expose] noncomputable def tripleAdd (M : FloatModel) (e : List ℝ) (t : ℝ × ℝ × ℝ) : List ℝ :=
  let xy := M.rn (t.1 * t.2.1)
  let xyTail := t.1 * t.2.1 - xy
  let hi := M.rn (xy * t.2.2)
  let lo := xy * t.2.2 - hi
  let e1 := if xyTail ≠ 0 then
      addTerm M (addTerm M e (xyTail * t.2.2 - M.rn (xyTail * t.2.2))) (M.rn (xyTail * t.2.2))
    else e
  addTerm M (addTerm M e1 lo) hi

/-- One triple adds exactly `x·y·z` to the expansion. -/
theorem tripleAdd_sum (M : FloatModel) (e : List ℝ) (t : ℝ × ℝ × ℝ) :
    (tripleAdd M e t).sum = e.sum + t.1 * t.2.1 * t.2.2 := by
  obtain ⟨x, y, z⟩ := t
  simp only [tripleAdd]
  by_cases h : x * y - M.rn (x * y) = 0
  · simp only [h, ne_eq, not_true_eq_false, ↓reduceIte, addTerm_sum]
    have h' : x * y = M.rn (x * y) := by linarith
    have h2 : x * y * z = M.rn (x * y) * z := by rw [← h']
    linarith
  · simp only [h, ne_eq, not_false_eq_true, ↓reduceIte, addTerm_sum]
    ring

theorem addTerm_length_le (M : FloatModel) {e : List ℝ} {q : ℝ} {n : ℕ} (h : e.length ≤ n) :
    (addTerm M e q).length ≤ n + 1 :=
  (addTerm_length M e q).trans (by omega)

theorem tripleAdd_length (M : FloatModel) (e : List ℝ) (t : ℝ × ℝ × ℝ) :
    (tripleAdd M e t).length ≤ e.length + 4 := by
  obtain ⟨x, y, z⟩ := t
  simp only [tripleAdd]
  split_ifs
  · exact le_trans (addTerm_length_le M (addTerm_length_le M (addTerm_length_le M
      (addTerm_length_le M (le_refl e.length))))) (by omega)
  · exact le_trans (addTerm_length_le M (addTerm_length_le M (le_refl e.length))) (by omega)

theorem tripleAdd_ne_zero (M : FloatModel) {e : List ℝ} (he : ∀ x ∈ e, x ≠ 0) (t : ℝ × ℝ × ℝ) :
    ∀ x ∈ tripleAdd M e t, x ≠ 0 := by
  obtain ⟨x, y, z⟩ := t
  simp only [tripleAdd]
  split_ifs
  · exact addTerm_ne_zero M (addTerm_ne_zero M (addTerm_ne_zero M (addTerm_ne_zero M he _) _) _) _
  · exact addTerm_ne_zero M (addTerm_ne_zero M he _) _

/-- The expansion `proof.orientSignGrow` builds: the six triples folded in order into an empty expansion. -/
@[expose] noncomputable def orientGrow (M : FloatModel) (ts : List (ℝ × ℝ × ℝ)) : List ℝ :=
  ts.foldl (tripleAdd M) []

/-- Model of the final read of `proof.orientSignGrow`: zero for an empty expansion, else the sign of the
highest component. -/
@[expose] noncomputable def growSign (E : List ℝ) : ℤ :=
  match E.getLast? with
  | none => 0
  | some t => if 0 < t then 1 else -1

theorem orientGrow_sum (M : FloatModel) (ts : List (ℝ × ℝ × ℝ)) :
    (orientGrow M ts).sum = (ts.map fun t => t.1 * t.2.1 * t.2.2).sum := by
  unfold orientGrow
  suffices h : ∀ (e : List ℝ),
      (ts.foldl (tripleAdd M) e).sum = e.sum + (ts.map fun t => t.1 * t.2.1 * t.2.2).sum by
    simpa using h []
  induction ts with
  | nil => intro e; simp
  | cons t ts ih =>
    intro e
    simp only [List.foldl_cons, List.map_cons, List.sum_cons]
    rw [ih, tripleAdd_sum]
    ring

theorem orientGrow_ne_zero (M : FloatModel) (ts : List (ℝ × ℝ × ℝ)) :
    ∀ x ∈ orientGrow M ts, x ≠ 0 := by
  unfold orientGrow
  suffices h : ∀ (e : List ℝ), (∀ x ∈ e, x ≠ 0) → ∀ x ∈ ts.foldl (tripleAdd M) e, x ≠ 0 by
    exact h [] (by simp)
  induction ts with
  | nil => intro e he; exact he
  | cons t ts ih =>
    intro e he
    exact ih _ (tripleAdd_ne_zero M he t)

theorem orientGrow_length (M : FloatModel) (ts : List (ℝ × ℝ × ℝ)) :
    (orientGrow M ts).length ≤ 4 * ts.length := by
  unfold orientGrow
  suffices h : ∀ (e : List ℝ), (ts.foldl (tripleAdd M) e).length ≤ e.length + 4 * ts.length by
    simpa using h []
  induction ts with
  | nil => intro e; simp
  | cons t ts ih =>
    intro e
    simp only [List.foldl_cons, List.length_cons]
    have := ih (tripleAdd M e t)
    have := tripleAdd_length M e t
    omega

/-- The sign read from a nonzero-component expansion whose highest component outweighs the rest is the sign
of the exact sum. -/
theorem growSign_eq {E : List ℝ} (hne : ∀ x ∈ E, x ≠ 0)
    (hdom : ∀ t, E.getLast? = some t → |E.dropLast.sum| < |t|) :
    growSign E = sgn E.sum := by
  unfold growSign
  cases hl : E.getLast? with
  | none =>
    have : E = [] := List.getLast?_eq_none_iff.mp hl
    subst this
    simp [sgn]
  | some t =>
    obtain ⟨ys, hE⟩ := List.getLast?_eq_some_iff.mp hl
    subst hE
    have hd := hdom t hl
    rw [List.dropLast_concat] at hd
    have ht : t ≠ 0 := hne t (by simp)
    have e1 := sgn_add_of_abs_lt hd
    have hsum : (ys ++ [t]).sum = t + ys.sum := by simp [add_comm]
    rw [hsum, e1]
    unfold sgn
    rcases lt_or_gt_of_ne ht with h | h
    · simp [not_lt.mpr h.le, h]
    · simp [h]

/-- `proof.orientSignGrow` returns the sign of the exact determinant of its admitted differences, including
zero, provided the final expansion's highest component strictly outweighs the sum of the rest (A14). The
`TwoSum` errors and FMA remainders are exact (A9, A10). -/
theorem orientGrow_sound (M : FloatModel) (bx by' bz cx cy cz dx dy dz : ℝ)
    (hdom : ∀ t, (orientGrow M [(bx, cy, dz), (by', cz, dx), (bz, cx, dy),
        (-bx, cz, dy), (-by', cx, dz), (-bz, cy, dx)]).getLast? = some t →
      |(orientGrow M [(bx, cy, dz), (by', cz, dx), (bz, cx, dy),
        (-bx, cz, dy), (-by', cx, dz), (-bz, cy, dx)]).dropLast.sum| < |t|) :
    growSign (orientGrow M [(bx, cy, dz), (by', cz, dx), (bz, cx, dy),
        (-bx, cz, dy), (-by', cx, dz), (-bz, cy, dx)]) =
      sgn (bx * (cy * dz - cz * dy) + by' * (cz * dx - cx * dz) + bz * (cx * dy - cy * dx)) := by
  rw [growSign_eq (orientGrow_ne_zero M _) hdom, orientGrow_sum]
  congr 1
  simp only [List.map_cons, List.map_nil, List.sum_cons, List.sum_nil]
  ring

end Decad.Orientation

end
