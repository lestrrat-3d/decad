# Body Refine: surface bounds for held Union meshes

`Refine` re-certifies the held mesh of a chain of mesh-Boolean `Union` and `Cut` results against the true solids
its leaves denote, and returns a fresh body whose `Mesh.Bound()` is that certified surface bound. It changes no
held vertex.
Companion to `agent-docs/faceted-vertex-bounds-design.md` ("vertex-bounds §N"), whose §3.2 rim rule and §3.6
shallow-crossing note this design answers, and to `agent-docs/tessellation-design.md` ("tessellation §N"), whose
§7 restatement refusal it lifts for a refined body. Increments R1–R6 (§9) implement it.

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
Boolean reads `σ` as the facet's slab half-width, raised by any gap shift's move, for its rim composition and its
chain-depth gate: a slab only states where the true boundary near the facet lies, which `σ` proves (§9 R6). The
proximity gate reads each source face's largest `σ` in place of its face bound when that is smaller
(`BoolMesh.FaceSurface`): `σ` bounds the distance both ways, which is all that slack needs. Vertex
bounds keep the crossing record, so every reading of the result's older rims stays as loose as before.

## 4. Input class

A **leaf** is a solid whose true set `C` the record can read, held as:

- **outer pieces**: exact convex polyhedra, rational vertices and planes, whose union contains `C`;
- a **shape** that bounds, for any held triangle, how far its points lie outside `C` (its *poke*);
- **(T) coverage**: `C ⊂ A'^{+δ}`, `δ` the published two-sided bound of the held mesh `A'` the Boolean used;
- for a leaf a `Cut` may use as a tool, **inner pieces**: exact convex polyhedra inside `C`.

Leaves come from these payloads (`internal/occupancyleaf`):

| Payload | Leaf | Outer pieces | Poke |
|---|---|---|---|
| prism of one whole CCW circle | the placed cylinder | one circumscribed 128-gon prism; an inscribed 128-gon prism inside | exact: each corner taken back to local coordinates, radial plus axial excess (both convex, so corners bound the facet), stretched by the maps' Frobenius norms; a corner the float inverse puts inside by 1e-6 of the section skips the exact reading |
| stack of slabs of one circle at contiguous levels | one cylinder | as above | as above |
| prism, or stack slab by slab, whose regions are a whole circle or a convex polygon of whole lines, with whole circular holes (either winding) | one leaf per region's outer loop, and one hole leaf per hole through that region's levels; a stack's holes meet other slabs only on shared level planes, which a regularized solid ignores | the polygon prism itself, each side pushed out by the section displacement over `|u|₁`; convexity checked exactly | float: the sum of side excesses over the sine of the sharpest corner, plus the axial excess |
| Revolve of one whole circle clear of its axis | an arc tube about the spine circle, full or partial | one 64-gon prism per angular cell about the cell's chord, widened by the arc's sagitta over the cell and extended by `ρ·d/2` at both ends; cells of sagitta at most 1e-4 mm | float, below |
| Sweep of one whole circle along lines and arcs | one leaf per span: a cylinder for a line, an arc tube for an arc (the spans' own prism and Revolve reductions) | as for each kind | as for each kind |
| Coil of one whole circle | a helix tube: every point lies within the wire radius of the centre helix at its own angle, because the disc sits in the axis plane there (`agent-docs/helix-design.md` Table CP) | one 32-gon prism per angular cell about the centre helix's chord, widened by the helix sagitta `ρc·d²/8` and extended by `ρw·(lead + d)` at both ends | none: every facet keeps its own bound as its bulge |

A circumscribed polygon uses rational directions `u_k` with offsets `c_k ≥ r·|u_k|` (checked as
`c_k² ≥ r²·u_k·u_k`), and every corner is checked against every side. An inscribed one uses float corners proven
exactly inside the circle. The record's section and level displacements widen the outer pieces and narrow the
inner ones. The maps' stretch is bounded by the square root of the largest absolute row sum of `MᵀM` (just above 1
for a placement), not by the Frobenius norm. The prism maps (frame `(U, V, N = U×V)`, then placement) are read as
exact rationals
(`prismrecord.Payload.Point`). The Revolve maps follow `revolvemesh.RevolveLift.Point`.

**Arc tube poke.** Write a point's meridian offset from the spine as `P = (r − R, h)`. Over a triangle `h` is
affine and `r` convex; across corners spanning an angle `α` about the axis, `r` lies between its corner
interpolation `r_lin` and `r_lin·cos(α/2)`. So `P = (A − c, h)`, with `A` affine and `0 ≤ c ≤ r_max(1 − cos(α/2))`,
and `|P|² ≤ M² + 2Nc + c²`. Here `M` is the largest corner `|(A, h)|` and `N` the largest corner `R − r`, floored
at zero. An outer-side facet pokes out by at most `c²/(2ρ)`, an inner-side one by about `c`. A corner outside a
partial arc's range, or corners spanning half a turn, read `+Inf`, and the facet falls back to its own bound.

**The arc tube reading is in floats, not rationals.** Spine points, cell frames and the poke use float
trigonometry, widened by a relative `1e-12` of their magnitude plus the record's displacements. That margin
exceeds float evaluation by four orders of magnitude, but it is a charged allowance, not an exact proof. The
cylinder reading is exact.

## 5. The certificate

`X` is the true solid: the union of the record's leaves less the union of its holes. `X'` is the held mesh. The
record (`occupancy.Record`) carries:

- `Drift`: `∪ A'_i ⊂ X'^{+Drift}`, the summed weld drift;
- `Cross`: the accumulated crossing depth;
- `Sites`: one float box per contact segment of every earlier step;
- `Holes`: the tool leaves every `Cut` removed. `X = (∪ leaves) ∖ (∪ holes)` holds for Unions followed by Cuts.
  A Union of two records with holes is admitted only when both name the same tools, as a distributive Cut retry
  produces, because `(A ∖ H) ∪ (C ∖ H) = (A ∪ C) ∖ H`.

**Bulge.** For a held mesh of the record, every point of `X'` outside `X` lies within `λ(t)` of some facet `t`
(`Record.Bulges`). `λ(t)` is the smallest of the facet's own two-sided bound and the largest of three readings:
its poke out of the nearest leaf; the bound of any hole whose wall it may come near, because the true hole reaches
past the held tool wall by at most the tool mesh's bound; and `Cross` when it is near an earlier site. "Near the
wall" means within the reach plus twice that bound of the true tool's boundary. The cylinder reads that distance
from below at a box's centre, so a contact far from a bore never reads the bore's shortfall.

**One step.** At a contact segment of facets `t` and `u` with proven sine `s`, a point shallow in both held operands
but deep in their union lies within `2·max(λ(t), λ(u))/s` of the crossing line, which is on the result's boundary.
For each segment, `λ` is read at that segment (`Record.LocalBulgeAt`, from the facet's leaf poke that
`Record.BulgesPokes` already read). Hole and site sources count only within
`SiteReach` (1 mm) of it, and the step's term must stay below that reach. A Cut tool facet reads its own bound. The
step composes `Cross = max(Cross_A, Cross_B, deep term) + w` and `Drift = max(...) + w`, and appends its segment
boxes to `Sites`.

The step needs one more fact. Every pair of held facets that come within `2·max(λ)` of each other without a proven
crossing must have been refused. The hidden-tangency pre-pass refuses any face pair within
`δ(face(t)) + δ(face(u))` that has no proven crossing deeper than that (tessellation §11 step 4). So it is enough
that every facet near the other operand has `2·λ(t)` below its own face's bound. Where that fails, as it always
does for a Cut tool, the same pre-pass runs again over face bounds raised to twice each face's largest bulge. A
refusal there drops the record and leaves the Boolean as it was. The argument rests on the pre-pass's own
soundness. The attribution of near-crease points to facets and sites is argued, as the existing per-vertex model
argues its true pieces, and is not separately proven.

**(i) Held to true.** Take a piece `P` of a held facet and a translation `t` with `|t| ≤ h`. Suppose `P + t`
misses every outer piece, or lies wholly inside one hole's inner piece. Then every `p ∈ P` lies within
`max(h, Excess)` of `∂X`. `Excess` is the largest, over facets, of `λ(t)` plus the facet's reach outside `X`:
its poke, or a hole's bound near that hole's wall. An earlier crossing raises a bulge but not a reach, because
the crossing line lies on held facets (`Record.BulgeParts`, `Record.MeshBounds`).

**(ii) True to held.** `H = max(max δ_i + Drift, max λ)`, plus `Cross` when the record has holes, because a true
point a Cut exposed may sit near a held rim (`Record.MeshBounds`).

**Per-facet bound.** A facet whose crossing bound `δ(f)` exceeds a quarter of `maxBound` is certified, and gets
`σ(f) = min(δ(f), upRound(max(h(f), Excess, H)))`. Every other facet keeps `σ(f) = δ(f)`, which already proves both
directions.

**Search.** For each certified facet, candidate translations are tried in a fixed order: the facet normal; the
three faces of every near piece that the facet's centroid is least deep behind; then the normalized sums of those.
Lengths follow the ladder from `max(floor, maxBound/64)` by a factor of 1.25 up to `maxBound`. Containment in a
hole's inner piece is tried first. Next comes one of a piece's own local half-spaces that all three moved corners
lie strictly beyond. Only then do world axes get ranked (piece face normals, the piece normal, edge-edge cross
products). Floats rank, and exact local coordinates or projections decide. A piece that nothing certifies at
`maxBound` is bisected across its longest edge, to depth 24, with eight ulps of slack per split. Work is charged
per exact test against a fixed ceiling, and running out refuses with `ErrUnsupported`. Facets run on `GOMAXPROCS`
workers. Exact pieces are built only when `Refine` first asks for them.

The search is reject-only in the `CLAUDE.md` sense: a float result only proposes, and only an exact half-space,
containment or axis admits a piece. The certificate needs no knowledge of which part is being modelled.

## 6. Records

- `internal/occupancy`: pieces (outer, inscribed), leaves (`CircleLeaf`, `ArcTubeLeaf`), `Record` with `Compose`,
  `CutCompose`, `Bulges`, `BulgesPokes`, `LocalBulge`, `LocalBulgeAt`, `MeshBounds`, the site index and `Binding`,
  and the facet search with `Certify`. Plain data in and out, per `agent-docs/module-boundary.md`.
- `internal/occupancyleaf`: reads an operand payload as a record (§4), and a mesh's per-facet bounds.
- `internal/meshbool`: `BoolMesh.Bulge` and `Stats.Contacts` (facet pair, proven sine and float box of every
  contact segment) from `MeshBooleanStats`.
- `facetedrecord.Payload.Occupancy` is set by `booleaneval` on the ordinary mesh Union or Cut path only (no replay,
  no shift, no cap or cone certificate) when both operands carry a record.
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
- `source` must be a live `BodySolid` whose payload carries an occupancy record: mesh Unions of extruded, revolved
  or swept circles, then Cuts by extruded circles. Anything else, or a certificate
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
| R4 | `Cut` by extruded-circle tools: holes, inscribed tool pieces, tool facets bulging by their own bound, the widened re-gate, shared holes in a Union of two cut halves | a bored bend (two thick legs, two Cuts) refuses `Tessellate` at 0.05 mm before `Refine` and meets 0.01 mm after |
| R5 | revolved and swept circle tubes: per-cell outer pieces, the sign-aware tube poke, per-facet bulges with crossing sites read per contact | a straight wire and a swept cord, each leaving a ring, meet 0.05 mm; an outer-equator facet barely pokes, an inner one at least as far as its edge midpoint, and a facet past a partial arc falls back to its bound |
| R6 | the next Boolean reads `σ` as its slab half-width when the restated mesh is the held one | under a 0.02 mm chain tolerance the leader's fold refuses at its third Union, and completes when each held step is refined first |

### Gallery parts

| Part | Held bound | After `Refine` | `Refine` time | `Refine` call |
|---|---|---|---|---|
| `yarn-leader` | 0.2423 mm | 0.0446 mm | 1.7 s | `yarn.go`: `return b.refine(b.union(spans...), 0.12)` |
| `brake-band` | 0.2713 mm | 0.0297 mm | 0.09 s | `brake_cord.go`: `return b.refine(b.union(cord, leftEye, rightEye), 0.12)` |
| `treadle-flex-connector` | 0.0758 mm | 0.0071 mm | about 4.5 s | `connector.go`: `return b.refine(b.cut(body, ...), 0.01)` around the second Cut |
| `brake-spring-*` | 2.075 mm | 0.0889 mm at the default pose | about 8.5 s | `brake.go`: `return b.brakeRotate(b.refine(b.union(coil, frontLeg, rearLeg, frontEye, rearEye), 0.12), front, phase)` |

Each call sits inside the part's constructor, before `builder.part` applies the head tilt or `brakeRotate` turns
the spring. A Union result placed after its last Union has no record, and placing the refined body keeps its
bound. `b.refine` wraps `decad.Refine(b.ctx, body, units.Millimeters(bound))` and `b.fail`. The connector refines
at 0.01 mm, which serves both its 0.05 mm test and the 0.01 mm clearance screen.

With these four calls the Gallery native default Build tessellates all 190 parts at 0.12 mm, and the connector
test passes. The full default test asks `footman` for 0.01 mm. Footman is a rectangular bar with two holes, cut by
two cylinders. Chorded at its whole 476 mm pair's tolerance, its leaf bounds would be 0.009 mm and its floor
`δ(1 + 2/sin)` 0.0335 mm, which no re-certification lowers: splitting a held facet keeps it on the same plane, and a
finer mesh needs new vertices on the true crossing of the socket and the hole, which means running the Cut again.
The floor meets 0.01 mm only when the chord comes from a box of about 130 mm. A public Cut by a curved tool whose
operand boxes overlap in at most 1/8 of the pair chords its fresh operands by that overlap
(agent-docs/chained-cut-design.md §4.1), so footman's last Cut is chorded finely enough and its 0.01 mm check passes.

At 0.05 mm the facet search certifies the pieces on the 2.2 mm hole's wall where the socket crosses it only by a
move into the hole's inner piece. Three rules make that reachable:

- Every split piece reads its candidate directions again, from its own centroid.
- A piece also tries the moves into a near inner piece, across one face or two at once.
- A stack's hole that runs through joined slabs is one hole leaf, so a wall facet that spans the slab interface
  sits inside one hole.

The four Gallery variants (small-seven, large-nine, twelve, small-seventeen) build the yarn leader by a chain of
Unions whose operands carry crossing bounds of 0.20–0.36 mm where each pair meets, above the document's 0.1 mm chain
tolerance. With R6, and the leader refining each held step at 0.1 mm before the next Union, the chain completes, and
the proximity gate reads the refined face bounds. The variants' first failures are:

| Variant | First failure |
|---|---|
| small-seven | `head-leg` Cut: the proximity gate cannot decide held facets within the chord tolerance |
| large-nine | `yarn-leader` Union: every parity ray is ambiguous (the oblique parity rays now decide it) |
| twelve | `treadle-heel-rail` Cut: the same proximity refusal |
| small-seventeen | `brake-spring-left` Union: the revolve facet contact audit hits its work ceiling, and every coarser rung's parity rays are ambiguous (the oblique parity rays now decide them) |

In small-seven the leg's planar end face carries a 0.0137 mm face bound, the chord of its heel hole, against a rail
face that crosses it far from the hole. Every facet of a face carries the face's whole bound, so the gate reads the
chord where no chord is.

**Gap-shift retries.** When an ordinary Union meets an isolated point touch, Decad retries with a gap shift. The
retry moves the second operand's held mesh by at most a measured displacement and leaves the true operand where it
was (agent-docs/gap-tolerance-design.md §4). The record follows it: the moved operand's record grows every leaf's
and hole's bound, and its drift, by that displacement (`Record.Shifted`). Its bulges are then read on the moved
mesh. The default spring's rear eye takes this path.

**Exact pieces near certified facets only.** `Refine` builds the exact pieces only for cells whose float boxes
come within `maxBound` of a facet it certifies (`Record.PiecesNear`). A spring's 1900 helix cells mostly stay
unbuilt.

## 10. Risks

- `Cross` grows by `2/s` per step where crossings sit near earlier sites, which is exponential in such a chain.
  It reaches 2.2e-8 mm after the leader's five steps.
- A facet's bulge is one number over the whole facet. A wall facet runs a whole span, so a site or hole near one
  end reaches the other. Reading each contact's bulge at its own segment (`LocalBulgeAt`) keeps the deep term local,
  but `MeshBounds` still reads whole facets.
- The arc tube reading and the cylinder fast path are float with charged margins, not exact rational proofs (§4).
- A chain must first survive its own folds. The chain-depth gate (vertex-bounds §5) still reads the crossing
  bounds unless an operand was refined, so a chain must call `Refine` between folds to use R6.
- Tracking costs every ordinary Union and Cut of tracked operands a bulge pass, and sometimes a second pre-pass.
  The Gallery default Build takes 60.6 s with tracking against 37.7 s without.
- Coil facets bound no poke of their own: each keeps its own two-sided bound as its bulge.
- Replay and ordered-shift paths are still not tracked.
