# Body Refine: surface bounds for held Union meshes

`Refine` re-certifies the held mesh of a mesh-Boolean `Union` result against the true solids its leaves denote,
and returns a fresh body whose `Mesh.Bound()` is that certified surface bound. It changes no held vertex.
Companion to `agent-docs/faceted-vertex-bounds-design.md` ("vertex-bounds §N"), whose §3.2 rim rule and §3.6
shallow-crossing note this design answers, and to `agent-docs/tessellation-design.md` ("tessellation §N"), whose
§7 restatement refusal it lifts for a refined body. Increments R1–R3 (§9) implement it for Unions of extruded
circles; R4–R6 are design only.

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

Let `X = ∪ C_i` be the true union and `X'` the held mesh. `occupancy.Record` carries three figures, each proven
for the held mesh it is bound to:

- `Excess`: `X' ⊂ X^{+Excess}`;
- `Drift`: `∪ A'_i ⊂ X'^{+Drift}`, the held leaf meshes against the held result;
- `Deep`: every point of `X'` deeper than `Deep` lies in `X`.

A leaf starts at `Excess = Deep = ρ`, `Drift = 0`. A ball inside `C^{+ρ}` around a point outside the convex `C`
has radius at most `ρ`, which gives its `Deep`. One Union step with largest weld `w`, smallest proven crossing
sine `s` and smallest hidden-tangency slack `b` composes (`occupancy.Compose`):

```text
Excess = max(Excess_A, Excess_B) + w
Drift  = max(Drift_A, Drift_B) + w
Deep   = 2·max(Deep_A, Deep_B)/s + w        (max(Deep_A, Deep_B) + w when no segment crosses)
```

The `Deep` step rests on one fact. A point shallow in both held operands but deep in their union lies within
`max Deep` of both held surfaces, so within `2·max Deep / sin θ` of a crossing line on the union's boundary.
That holds only if every pair of held surfaces that come that close is a proven crossing. The hidden-tangency
pre-pass (tessellation §11 step 4) refuses any face pair within its slack `b` that has no proven crossing deeper
than `b`. Compose therefore checks `b > 2·max Deep` and returns no record otherwise. `b` is read as the sum of
the two meshes' smallest face bounds, stepped down one float. A step with a segment that has no proven positive
sine also returns no record.

**(i) Held to true.** Take a piece `P` of a held facet and a translation `t` with `|t| ≤ h`. If `P + t` misses
every `U_i`, then every `p ∈ P` lies within `max(h, Excess)` of `∂X`. If `p ∈ X`, the segment to `p + t ∉ X`
crosses `∂X`. Otherwise `dist(p, ∂X) = dist(p, X) ≤ Excess`. No crossing angle enters.

**(ii) True to held.** Take `y ∈ ∂X`.

- If `y ∉ int X'`, then `y ∈ C_i ⊂ A'_i^{+δ_i} ⊂ X'^{+Drift}`. So `dist(y, ∂X') ≤ max δ_i + Drift`.
- If `y ∈ int X'`, its depth is at most `Deep`, or `y` would be interior to `X`.

So `H = max(max δ_i + Drift, Deep)` (`Record.TrueToHeld`) bounds the true-to-held side everywhere. Only rounding is
amplified. On the leader, five steps leave `Deep` at 3.7e-8 mm and `Excess` at 8.8e-13 mm.

**Per-facet bound.** A facet whose crossing bound `δ(f)` exceeds a quarter of `maxBound` is certified, and gets
`σ(f) = min(δ(f), upRound(max(h(f), Excess, H)))`. Every other facet keeps `σ(f) = δ(f)`, which already proves both
directions.

**Search.** For each certified facet, candidate translations are tried in a fixed order: the facet normal; the
three faces of every near piece the facet's centroid is least deep behind; then the normalized sums of those.
Lengths follow the ladder from `max(floor, maxBound/64)` by a factor of 1.25 up to `maxBound`. The cheap
certificate comes first: one of the piece's own local half-spaces (a side or a level) that all three moved corners
lie strictly beyond. Floats rank the half-spaces, and the corners' exact local coordinates decide. Only when no
half-space separates does a float pass rank world axes (piece face normals, the piece normal, edge-edge cross
products), and the exact projections decide. A piece that no candidate certifies at `maxBound` is bisected across
its longest edge, to depth 24, with eight ulps of slack per split for the float midpoint. Work is charged per
exact test against a fixed ceiling. Running out of ceiling or depth refuses with `ErrUnsupported`. Facets run on
`GOMAXPROCS` workers.

The search is reject-only in the `CLAUDE.md` sense: a float result only proposes, and only an exact half-space or
axis admits a piece. The certificate needs no knowledge of which part is being modelled. It reads the leaf
records and the held mesh.

## 6. Records

- `internal/occupancy`: the leaf piece in V-, H- and local half-space form; `CircleLeaf`, which builds it from a
  circle and its maps and computes `ρ` exactly from the held mesh; `Record`, `Compose` and `Binding`; and the facet
  search with `Certify`. Plain data in and out, per `agent-docs/module-boundary.md`.
- `internal/occupancyleaf`: reads an operand as a record. A prism of one whole CCW circle is a leaf, its true solid
  the exact image of the section under its frame `(U, V, N = U×V)` and then its placement, both read as rational
  (`prismrecord.Payload.Point`). A stack of slabs of one circle at contiguous levels is a single leaf. A faceted
  payload contributes its own record. The piece is a 128-gon, widened by the record's section and level
  displacements plus a relative `1e-12` slack.
- `facetedrecord.Payload.Occupancy` is set by `booleaneval` on the ordinary mesh Union path only (no replay, no
  shift, no cap or cone certificate) when both operands carry a record.
- `facetedrecord.Payload.SurfaceBound` is set by `Refine`.
- Both records are bound to the identity of the payload's `Verts` and `Tris` arrays (`occupancy.Binding`). Every
  producer that changes the held geometry allocates new arrays, so a record copied along with a placed or
  rewritten payload never matches and is never read. Rigid placement (`facetedrecord.Payload.Placed`) drops the
  occupancy record. It carries the surface bounds instead: each `σ(f)` grows by the largest growth the placement
  charged to the vertex bound of one of its corners, and is bound again to the placed arrays. A placement that
  reorders a facet's corners, as a reflection does, drops them.
- `facetedrecord.Restate` serves the surface record only for a chord below the crossing floor `MeshBound`. There it
  publishes each face's largest `σ` and no vertex record. Every Boolean operand request sits at or above that floor
  (`booloperand.HeldFloor`), so the Boolean keeps today's crossing restatement. Which restatement a request gets is
  a function of its tolerance alone, so the one-entry cache needs no new key: its key already holds the tolerance.

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
- `Tessellate(tol)` on the result succeeds for every `tol ≥ max σ`. Below the crossing floor its `Mesh.Bound()`
  is `max σ`. At or above the floor it is the crossing bound, as before.
- A later Boolean on the result reads the crossing record (§3), so refining never loosens what a chain admits.
- A rigid placement of the result keeps its surface bounds, grown by the placement's per-vertex rounding (§6).
  Refining after placement is refused, because placement drops the occupancy record.

`docs/api.md` (Booleans), `docs/data-structures.md` (Meshes), `examples/refine_example_test.go` and
`apitest/refine_test.go` cover the public slice.

## 8. Feasibility probe

The probe is an uncommitted scratch test (`.tmp/refine-probe/` in the worktree that wrote this document). It
builds the six-span leader through the public `Extrude`, `Transform` and `Union`. It reads the held mesh,
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
| R1's exact `FacetBound` over all 914 held facets (64-gon pieces, ladder from 0.001 mm, depth 20) | 0.0355 mm, none refused |
| `max δ_i` (true-to-held side, before `W` and `e_deep`) | 0.00478 mm |
| held volume interval | 142.2414 ± 2.1660 mm³, containing the closed form 144.4022 mm³ |

The distance row is a measurement, not a proof. The `FacetBound` row is the exact certificate (i) over pieces the
probe built itself. Through the production path (leaf records from `occupancyleaf`, `Refine` at 0.12 mm on 24
workers) the same leader certifies a `Mesh.Bound()` of 0.0446 mm in 1.7 s. `apitest/refine_test.go` runs it as the
`leader` case. The search direction set, not the geometry, accounts for the gap to 0.0061 mm. The closed form
landing at the top of the interval is expected, because every leaf is inscribed.

## 9. Increments

| Inc | Lands | Red leg |
|---|---|---|
| R1 | `internal/occupancy`: circle-prism `U` under a rotated placement; exact piece-misses-`U`; facet search with ceiling | a point just inside the true circle is never separated, and one past the polygon's circumradius always is; a chord facet is never certified below its sagitta; a facet inside two leaves must clear the deeper; a work ceiling of 10 refuses |
| R2 | the occupancy record composed by the ordinary mesh `Union` (weld, smallest sine, checked tangency slack), bound to its held mesh | a placed Union result no longer carries a readable record, so `Refine` refuses it |
| R3 | `Refine`, surface restatement below the crossing floor, docs, example, apitest: four bent chains (a right-angle pair, an oblique zigzag, four bends at r = 0.5, the six-span leader) with volume against §8's closed form | before `Refine`, the leader's `Tessellate` at 0.12 mm refuses |
| R4 | `Cut` and `Intersect` with convex tools: the upper set `A_U \ B_L`, with the tool's inscribed mesh as `B_L` | — |
| R5 | swept and revolved circle tubes as unions of convex cell pieces, and the bulge term where chords leave the true tube | — |
| R6 | the next Boolean reads `σ` as its slab half-width | — |

## 10. Risks

- `Deep` grows by `2/s` per step, which is exponential in chain length. It reaches 3.7e-8 mm after the leader's
  five steps. A long chain of shallow crossings can push it past `maxBound`, and `Refine` then refuses.
- A chain must first survive its own folds. The chain-depth gate (vertex-bounds §5) still reads the crossing
  bounds, so a thicker or shorter-span chain can be refused mid-fold before `Refine` is reachable. R6 lifts that.
- Gallery's band and connector are outside R1–R3. The band needs R5, whose leaves are not inscribed: a chord along
  a bent tube's inner side leaves the true tube, so (L) gains a sagitta-sized `λ`. Then `Deep` grows by
  `λ / sin θ`, which is mechanism 1 again unless R5 bounds the bulge per side. The connector needs R4.
- Certificate cost scales with the facets near other leaves. The leader takes 1.7 s on 24 workers. The connector's
  16 064-facet meshes would meet the work ceiling and wall time first.
