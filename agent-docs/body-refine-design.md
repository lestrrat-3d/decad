# Body Refine: surface bounds for held Union meshes

`Refine` re-certifies the held mesh of a mesh-Boolean `Union` result against the true solids its leaves denote,
and returns a fresh body whose `Mesh.Bound()` is that certified surface bound. It changes no held vertex.
Companion to `agent-docs/faceted-vertex-bounds-design.md` ("vertex-bounds §N"), whose §3.2 rim rule and §3.6
shallow-crossing note this design answers, and to `agent-docs/tessellation-design.md` ("tessellation §N"), whose
§7 restatement refusal it lifts for a refined body. Status: design, with a measured feasibility probe (§8).
Nothing here has landed.

## 1. Problem

A chain of `Union` calls over placed solid cylinders publishes a held bound far above the chording that built it.
Six spans of radius 0.35 mm joined end to end at bends (the Gallery yarn leader's construction, reproduced in
§8's probe with a 0.1 mm chain tolerance) gives, per left-fold step:

| Step | Joint | `MeshBound` | worst rim: `δA`, `δB`, `sin θ`, `nA·nB` |
|---|---|---|---|
| 1 | collinear | analytic (stacked prism) | none |
| 2 | 90° bend | 0.150 | 0.00357, 0.00357, 0.0476, 0.9989 |
| 3 | bend | 0.150 | 0.00714, 0.00357, 0.1055, 0.9944 |
| 4 | bend | 0.2423 | 0.0338, 0.00478, 0.1594, 0.9872 |
| 5 | bend | 0.2423 | 0.0972, 0.00478, 0.9856, 0.1689 |

`Tessellate` then refuses every tolerance below 0.2423 mm (tessellation §7). The operand chord sagittas are
0.0036–0.0048 mm. Two mechanisms produce the factor of fifty:

1. **Near-tangent rims.** At a bend the two walls are tangent at `P ± r·b` (`P` the shared end point, `b` the
   bend binormal) with equal outward normals, and their crease runs out of that point. Facet pairs near it cross at
   `sin θ` of 0.05–0.16, and vertex-bounds §3.2 divides `δA + δB` by it. Every step's worst pair has `nA·nB > 0.98`.
2. **Corner-maximum spread.** A cylinder wall facet runs the span's whole length. Once a rim vertex with a large
   `β` becomes one of its corners, `δ(t)` (the corner maximum) carries that `β` to the far joint, so the next
   contact reads `δA = 0.0338` or `0.0972` instead of the operand's own 0.0036.

The same shape holds for the Gallery brake band (composite sweep cord plus revolved eye, 0.2713 mm) and, through
`Cut`, for the flexible connector (0.0758 mm against a 0.05 mm request).

## 2. What does not fix it

- **Finer uniform chords.** Near the tangency the facet crossing angle shrinks with the chord angle, so the rim
  bound falls only linearly, and its value depends on where stations happen to fall. Gallery measured the leader
  at half the chord: 0.059, 0.106, then 0.333 and a refusal; at a quarter, "every parity ray was ambiguous".
- **Contact-local refinement.** Same erratic `sin θ`, plus a non-uniform chording contract for every payload.
- **Chain tolerance.** `WithChainTolerance` moves the gate, not the bound.
- **The flat-corner rule of vertex-bounds §3.6.** A prototype that replaces the rim bound by
  `√2·max(δA, δB)` when `nA·nB ≥ 0` gives 0.0172 mm on the leader and 0.0076 mm on the band, so the bound is
  geometrically there. The rule is unsound as written (§3.6 lists why). Its natural premise, one graph direction
  over a ball of the tube radius, fails at exactly the leader's worst rims: the cap facets lie inside that ball,
  and the smallest facet normal component along the bisector there is −0.02 to −0.07.
- **A rim bound from the true surfaces.** The true crease near the tangency really does move by
  `≈ δ/θ`. Any bound on where the crossing curve is stays large. Only the distance to the true *surface* is small.

## 3. Two bounds

A held mesh after this design carries two readings:

| Reading | Meaning | Read by |
|---|---|---|
| crossing bound `β(v)` | vertex-bounds §2 unchanged: a rim vertex stands for the true crossing point | `Vertex.Bound`, `FacetedCurve.Bound`, edge length, face area, volume, the next Boolean's operand mesh |
| surface bound `σ(f)` | every point of facet `f` lies within `σ(f)` of the true boundary, and every true boundary point whose nearest held point lies on `f` lies within `σ(f)` of it | `Mesh.Bound()` and `sourceBound` of a public `Tessellate`, and the §7 refusal |

`σ(f) ≤ δ(f)` always holds, because `δ(f)` already proves both directions. Only `Refine` writes `σ`. The next
Boolean keeps reading the crossing record, which stays sound and only stays as loose as today (§9 R6).

## 4. Input class

A **convex leaf** is a solid whose true set `C` is convex and whose producer proves three facts about the held
polyhedron `A'` the Boolean used:

- **(L) inscription**: `A' ⊂ C^{+ρ}`, `ρ` the held mesh's coordinate and placement rounding;
- **(T) coverage**: `C ⊂ A'^{+δ}`, `δ` that mesh's published two-sided bound;
- **(U) outer piece**: an exact convex polyhedron `U ⊇ C`, held as rational vertices and planes.

The first slice admits one leaf kind: a prism payload whose section is one full circle, any extrusion, any rigid
placement. The chord polygon is inscribed, so (L) and (T) are its existing proofs. `U` is a circumscribed `m`-gon
prism built from rational directions `u_k` with offsets `c_k ≥ r·|u_k|` (checked as `c_k² ≥ r²·u_k·u_k`), both
cap levels widened by their recorded displacement, and mapped by the stored frame and placement read as exact
rationals. Since an affine map keeps containment, `U ⊇ C` holds whether or not the stored matrix is exactly
orthogonal. A stacked payload of coaxial circle slabs is one leaf per slab.

A **convex union** is a `Union` tree whose every leaf is a convex leaf. The mesh Boolean composes its record
(§6). `Cut`, `Intersect`, modify operations and placement drop it.

## 5. The certificate

Let `X = ∪ C_i` be the true union and `X'` the held mesh. Let `W` be the summed per-step weld displacement, and
`e_deep = Σ_k (w_k + 2ρ_k / s_k)` with `s_k` step `k`'s smallest proven crossing sine.

**(i) Held to true.** `(L)` composes through `Union` and each weld, so `X' ⊂ X^{+e}` with `e = max ρ_i + W`.
Take a piece `P` of a held facet and a rational translation `t` with `|t| ≤ h`. If `P + t` misses every `U_i`,
then every `p ∈ P` lies within `max(h, e)` of `∂X`. If `p ∈ X`, the segment to `p + t ∉ X` crosses `∂X`.
Otherwise `dist(p, ∂X) = dist(p, X) ≤ e`. No crossing angle enters.

**(ii) True to held.** Take `y ∈ ∂X`.

- If `y ∉ int X'`: `y ∈ C_i ⊂ A'_i^{+δ_i}` by (T), and `∪ A'_i ⊂ X'^{+W}`, because offsets compose through union
  and each weld. So `dist(y, ∂X') ≤ max δ_i + W`.
- If `y ∈ int X'`: points of `X'` deeper than `e_deep` lie in `X`. Inside one step, a point shallow in both
  operands but deep in their union lies within `2ρ/sin θ` of a crossing line on the union boundary. Pairs that
  come within slack without a proven crossing are already refused (tessellation §11 step 4). So `y`'s depth is at
  most `e_deep`, or `y` would be interior to `X`.

So `H = max(max δ_i + W, e_deep)` bounds the true-to-held side everywhere. Rounding is the only amplified term,
and with `ρ ≈ 1e-13` it stays negligible.

**Per-facet bound.** `σ(f) = upRound(max(h(f), e, H))`, where `h(f)` is the largest `h` its pieces needed.

**Search.** For each held facet, try candidate translations in a fixed order: the facet normal; the outward plane
normals of every `U_i` whose box comes within `h_max` of the facet; and the normalized pairwise sums of those.
Try lengths on the ladder `h_j = h_0·1.25^j`. A float pass picks the translation and a separating axis (a `U`
face normal, the piece normal, or an edge-edge cross product). The exact pass then compares the piece's three
translated rational vertices with `U`'s rational vertices on that one axis. A piece that no candidate certifies at
`maxBound` is split at its edge midpoints, to a fixed depth. Work is charged per exact axis test against a fixed
ceiling. Running out of ceiling or depth refuses with `ErrUnsupported`. No ceiling is a parameter.

The search is reject-only in the `CLAUDE.md` sense: a float result only proposes, and only the exact separating
axis admits a piece. The certificate needs no knowledge of which part is being modelled. It reads the leaf
records and the held mesh.

## 6. Records

- `internal/occupancy` (new): the leaf piece (`U` in V- and H-representation), its construction from a circle
  prism record under its frame and placement, the exact piece-misses-`U` test, and the facet search. Plain data
  in and out, per `agent-docs/module-boundary.md`.
- `facetedrecord.Payload` gains `Occupancy *OccupancyRecord`, which holds the leaves (`U_i`, `ρ_i`, `δ_i`), `W`
  and `e_deep`. `Union` sets it when both operands carry one, either a convex leaf's own or a prior record, adding
  that step's largest weld and smallest proven sine. Every other producer leaves it nil.
- `facetedrecord.Payload` gains `SurfaceBound []float64` (one per facet). Only `Refine` sets it.
  `facetedrecord.Restate`, for a public `Tessellate`, publishes `faceBound[f] = max σ` over the face and no vertex
  record. The Boolean operand path keeps today's restatement. The one-entry cache key gains the record kind, so
  the two restatements never share an entry.

## 7. Public contract

```go
// Refine returns a fresh body holding source's mesh with a certified surface bound no larger than maxBound.
func Refine(ctx context.Context, source *Body, maxBound units.Value) (*Body, error)
```

- `maxBound` is a length magnitude: wrong kind → `ErrUnitKind`; non-finite → `ErrNotFinite`; negative →
  `ErrNegativeMagnitude`; zero → `ErrDegenerate`. A nil or retired `source` is refused.
- `source` must be a live `BodySolid` whose payload carries an occupancy record. Anything else, or a certificate
  above `maxBound` within the ceilings, returns `ErrUnsupported` naming the facet bound reached.
- On success the result belongs to `source`'s document, has fresh face identities and the same held vertices,
  triangles, topology, `Volume`, `Area`, vertex and edge readings. The operation records as `Refine` and retires
  `source`. Failure or cancellation changes no history and no live body. `ctx.Err()` is returned unchanged.
- `Tessellate(tol)` on the result succeeds for every `tol ≥ max σ`, and its `Mesh.Bound()` is `max σ`.
- A later Boolean on the result reads the crossing record (§3), so refining never loosens what a chain admits.

`docs/api.md`, `docs/data-structures.md`, an `examples/` file and an `apitest` case land with the public slice.

## 8. Feasibility probe

The probe is an uncommitted scratch test (`.tmp/refine-probe/` in the worktree that wrote this document). It builds the six-span leader through the public `Extrude`, `Transform` and `Union`. It reads the held mesh,
measures each held point's distance out of the true union (exact line–cylinder intervals over 4000 directions),
and checks the held volume interval against the closed form

```text
V = Σ π r² L_i − Σ_joints (4/3) r³ tan(α_j / 2)
```

(each joint's overlap of two butt-ended cylinders with turn angle `α_j`, valid when every span is longer than
`r·tan(α_j/2)`).

| Quantity | Value |
|---|---|
| held `MeshBound` (today) | 0.2423 mm |
| largest held-to-true distance over the held mesh | 0.0061 mm |
| `max δ_i` (true-to-held side, before `W` and `e_deep`) | 0.00478 mm |
| held volume interval | 142.2414 ± 2.1660 mm³, containing the closed form 144.4022 mm³ |

Neither figure in the middle two rows is a proof. Both measure the quantities §5 certifies, so the certified
`max σ` should land near 0.006 mm against today's 0.2423 mm and the 0.12 mm request. The closed form landing at the
top of the interval is expected, because every leaf is inscribed.

## 9. Increments

| Inc | Lands | Red leg |
|---|---|---|
| R1 | `internal/occupancy`: circle-prism `U` under a rotated placement; exact piece-misses-`U`; facet search with ceiling | a point just inside the true circle is never separated, and one past the polygon's circumradius always is; a chord facet is never certified below its sagitta; a facet inside two leaves must clear the deeper; a work ceiling of 10 refuses |
| R2 | `OccupancyRecord` composed by `Union` (weld, smallest sine); dropped elsewhere | a `Cut` result carries no record |
| R3 | `Refine`, surface restatement, cache key, docs, example, apitest: two- and six-span bent cylinder chains at two radii and two bend sets, volume against §8's closed form | restating `δ` instead of `σ` keeps the refusal |
| R4 | `Cut` and `Intersect` with convex tools: the upper set `A_U \ B_L`, with the tool's inscribed mesh as `B_L` | — |
| R5 | swept and revolved circle tubes as unions of convex cell pieces, and the bulge term where chords leave the true tube | — |
| R6 | the next Boolean reads `σ` as its slab half-width | — |

## 10. Risks

- (ii)'s deep-held step leans on the existing hidden-tangency refusal for held surfaces that come within rounding
  of each other without crossing. R2 must restate that dependency as a checked condition, or the proof has a gap.
- Gallery's band and connector are outside R1–R3. The band needs R5, whose leaves are not inscribed: a chord along
  a bent tube's inner side leaves the true tube, so (L) gains a sagitta-sized `λ`. Then `e_deep` becomes
  `λ / sin θ`, which is mechanism 1 again unless R5 bounds the bulge per side. The connector needs R4.
- The certificate costs exact work per facet near other leaves. On the leader that is a few hundred facets. On
  the connector's 16 064-facet intermediate meshes the ceiling decides.
