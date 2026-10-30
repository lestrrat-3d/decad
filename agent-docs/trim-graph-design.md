# Trim Graph Design: general modify and Boolean reach over analytic results

This document ("trim §N" elsewhere) is the companion to `agent-docs/api-design.md` ("core §N"),
`agent-docs/evaluator-design.md` §9 (the mesh Boolean), `agent-docs/faceted-source-retention-design.md`
("retention §N"), `agent-docs/retained-edge-blend-design.md` ("blend §N"), `agent-docs/modify-design.md`
("modify §N") and `agent-docs/modify-reach-design.md` ("reach §N").

## Navigation (not authority)

| Question | Section |
|---|---|
| Which gap rows does this close, and why do they share a cause? | Gap inventory |
| Which architecture, and what else was weighed? | Approach |
| What is a trim graph, and how is it read from a body? | Design → Component 1 |
| How are offsets, vertices and curves computed and bounded? | Design → Components 2, 3 |
| How are new faces meshed and the result proven? | Design → Components 4, 5 |
| What exactly does increment 1 build? | Design → Component 6 |
| What is refused, never repaired? | Soundness rules, Table TG |
| What lands in which PR? | Rollout |

## Problem

Fillet, Chamfer, Shell and Draft build only on recognised shape classes, and a mesh Boolean result reaches them
only through per-surface-pair routes (blend R1–R6, reach §4–§8). Shell, Draft and Offset of a mesh Boolean result
refuse outright, because the held mesh records polygons and no face trims. Blends between crossing cylinders outside
blend R7's saddles, on cones or tori, and on oblique plane–cylinder rims refuse because their blend surface is outside the vocabulary.
Variable-radius and face-to-face fillets have no entry point. This design adds one general path: lift any body whose
faces carry exact analytic tags into a **trim graph** (faces on tags, edges on exact or certified curves, vertices
with bounds), run the modify operation on the graph, mesh only the faces it changed, and prove the result.

Success criteria:

- Increment 1: `Shell(ctx, b, nil, t, WithNoOpenings())` builds the closed inward shell of a convex mesh Boolean
  result with planar faces (the 45° bevelled plate of `apitest/faceted_retained_blend_test.go`). Its volume equals
  the closed form within the published bound, its void shell has one `Plane` face per outer face, and
  `Tessellate(VerifyAll)` passes.
- Each later increment closes or narrows the `agent-docs/missing-features.md` rows the Rollout table names, with a
  computed-geometry test per row.
- No result that builds today changes: every existing specific route runs first.
- Every trim-graph result is a faceted payload with retained tags, so later Booleans and placements consume it like
  any retained body. A face whose stored tag is a rounding of its true surface states that departure in
  `NormalBound` (Component 6 step 7), so exact-plane readers and a later Lift decline that face.

**Out of scope:** Exact (zero-bound) volumes on trim-graph results; Boolean contact refusals (coplanar overlap,
tangent touch, graze); free-form (`NURBSSurface`) faces; level denotations; sketch-side arrangements; STEP analytic
export of the new blend surfaces; topology-changing offsets (merge, split, vanishing face).

## Assumptions

- "Exact" in the request means an exact analytic representation: every face on an exact tag, every edge on an exact
  closed-form curve or a certified intersection curve, with bounded measurements. Measurements stay `Approximate`.
- "Analytic" means the `Plane`, `Cylinder`, `Cone`, `Sphere` and `Torus` tags, plus the blend surfaces this design adds.
- Shell keeps the repo's ball-erosion semantics (modify §8 offset table: a convex corner stays sharp, a reflex corner
  rounds with radius `t`), lifted to 3D: offset faces, a pipe of radius `t` about each concave edge, a sphere of
  radius `t` at each all-concave vertex. Decisions records the alternative.
- The mesh Boolean keeps deciding Boolean topology (core §2.1). This design never replaces it.
- The trim graph is lifted at the consuming call and is not stored on the payload. Blend faces this design mints
  carry self-describing internal tags (a pipe tag holds its spine), so a later lift reads them.
- Sizes that do not convert exactly to millimetres (`tDelta != 0`, `rDelta != 0`) refuse, as the retained blend does
  (`internal/retainedblend/route.go`), until an increment charges them.
- Cavity faces take role `cavity(k)`, opening lip faces `lip(k)`, with `k` the outer face's index in
  `topobuild.Faces` order, as the brep record's `wall(k)` roles index its faces (`internal/breprecord/payload.go`).
  `k` therefore follows the receiver's build order. Blends keep `fillet(k)` and `chamfer(k)`.
- Shell keeps two corner semantics after increment 3a, split by receiver: an axis-square faceted solid with no vertex
  bound keeps the orthogonal shell's mitred cube erosion (`shell_orthogonal.go`, which runs first), and every
  trim-graph receiver takes ball erosion. `shell.go` and trim §1 state the split. Decisions records it.
- Increments 2 and 3a carry staged tasks (Staged tasks). Increments 3b–12 are outlines: the PR that implements one
  first replaces its outline with tasks in this document, in the same PR.
- New internal packages get rows in a new part file `agent-docs/layout/internal-trim.md`, because
  `agent-docs/layout/internal-core.md` (12208 bytes) and `internal-checks.md` (11939 bytes) are near the 12288-byte
  budget.
- Fixtures reuse the apitest helpers `wedgedPlate`, `blendPlate` and `newRetentionScene`
  (`apitest/faceted_retained_blend_test.go`, `apitest/faceted_source_retention_test.go`).

## Gap inventory

Rows are named by their leading words in `agent-docs/missing-features.md`. Each refusal site is named.

### RC1 — no recorded trims on a mesh result

A modify operation that moves a face needs that face re-trimmed against its neighbours. The held mesh records only
polygons, so every face-moving operation refuses on a mesh result.

| Row | Refusal site |
|---|---|
| Modify: "`Shell` of a mesh Boolean result outside the two-prism nested-boss Union route" | `internal/brepmodify/shell.go:151` "this evaluator shells a straight prism or its one-line Sweep reduction only" |
| Modify: "Closed shell (`WithNoOpenings`) outside a full revolve, a straight prism…" (faceted part) | `internal/stackedshell/closed_refusal.go:29` (SX9) |
| Feature: "`Draft` of a mesh Boolean result" | `internal/draftrecord/gates.go:25` (SD23) |
| Sheets: "`Offset` of a solid other than an axis-square faceted one" | `offset.go:149` |
| Also consider: "Direct edits: … move a face of a solid that is not an orthogonal mesh solid" | `move_face.go` (orthogonal only) |

### RC2 — blend geometry enumerated per surface pair

Each blend class is a hand-written closed form (blend Table RR). A pair outside the table, or whose rolling-ball
surface is not a `Cylinder`, `Cone` or `Torus`, refuses.

| Row | Refusal site |
|---|---|
| Modify: "Fillet/chamfer of most faceted boolean results" | the S3 text "this evaluator fillets a straight prism, its one-line Sweep reduction, a revolve, or a matching-section axial loft only" (`internal/brepmodify/fillet.go`) after `retainedblend.SaddleFillet` and `retainedblend.Fillet` decline |
| Modify: "Retained edge blend between crossing cylinders (a `Fillet` outside the R7 fillet row, a `Chamfer` outside R7), or on a `Cone` or `Torus` face" | blend SR1: `retainedblend.Unread` (`internal/retainedblend/unread.go`) names the edge after `looksLikeSaddleFillet` (`saddle_fillet.go`), the `looksLike*` classification (`route.go`) and `looksLikeSaddle` (`saddle.go`) decline it |
| Modify: "Fillet/chamfer of a class-B crossing result's edges outside brep-modify §4.4" | brep-modify SB1 |
| Modify: "Variable-radius fillet, face-to-face fillet" | no entry point (`fillet.go` has no option for it) |
| Also consider: "Fillet by chord length … on a torus, with setback corners" | `fillet.go` |

### RC3 — no trim of a new face against the rest of the body

A blend strip that meets any other part of the body, or two blends that share a vertex, refuses, because no route
trims the new face by other faces.

| Row | Refusal site |
|---|---|
| Modify: "Retained edge blend or tangent chain whose new strip would cross another part" | `internal/retainedblend/strip_clear.go:173` |
| Modify: "Open tangent chain on a mesh Boolean result whose concave end edge is an arc" | `internal/retainedblend/chain_arc_end.go:45`, `chain_open.go` |
| (blend SR2) two selected edges sharing a vertex | `internal/retainedblend/route.go` "two selected edges share a corner" |

### RC4 — a held mesh cannot be re-tessellated

A mesh result keeps the chord bounds of the Boolean that built it. A later Boolean or `Verify` that needs a finer
mesh refuses.

| Row | Refusal site |
|---|---|
| Booleans: "Held mesh operand coarser than the pair tolerance after a Boolean chain" | `internal/meshbool/staging_error.go:21` |
| Booleans: "A body whose boolean rims cross at a shallow angle … verified at a tolerance below its mesh bound" | `faceted-vertex-bounds-design.md` §3.6 refusal |

### RC5 — analytic receivers whose specific routes do not reach the selection

The prism, brep, revolve, loft and sweep routes each read their own record. A selection outside the record's
reading refuses even though every face is analytic. A trim graph lifted from the receiver's tessellation would
reach these (increment 12).

Rows: Modify "Other Sweep Fillet/Chamfer targets", "Fillet of a loft with different sections", "Fillet/chamfer of a
brep or stacked result's single edges sharing a vertex", "Fillet or chamfer of independent straight brep edges",
"Fillet or chamfer of a loop whose neighbouring face is curved", "Fillet/chamfer of a revolve cap edge", "Fillet or
chamfer of a cap edge outside the convex prism cutter", "Chamfer of a prism's partial cap loop", "Asymmetric brep or
stacked chamfer selection", Feature "`Draft` of a wall subset that moves one of two walls".

### Not addressed here (different root cause)

Contact on the mesh path, tangent and coplanar contact, level denotations (prism `Cut` levels, blind cuts at displaced
levels), sketch arrangements of trimmed sections, the tangent junction that misses tangency by a rounding (stays
refused by soundness rule S6), an edge ending at a tangent junction whose continuation is not selected (a correct
refusal in `agent-docs/missing-features.md`), free-form walls, STEP rows, sweep and loft topology rows.

## Approach

**Mesh decides topology, tags decide geometry, certificates connect them.** The trim graph takes faces, loops, edge
incidences and vertex incidences from the receiver's topology, which the mesh Boolean decided with exact predicates
(evaluator §9) or a feature built by construction. It takes every surface from the face's exact retained tag
(retention Claim R) and every curve from a closed form (retention §10) or a certified enclosure (Component 3). An
operation changes tags (offset, rotate, add a blend face), recomputes the curves and vertices it touched, meshes the
faces it touched, and proves the new held mesh against the new true solid. Specific routes run first; this route runs
only where they decline, before the existing refusal.

### Candidates

| | A. Per-class routes (today) | B. Trim graph over retained tags (recommended) | C. Exact algebraic B-rep kernel |
|---|---|---|---|
| What it is | One closed-form route per (operation, surface pair, configuration), as blend R1–R6 and reach §4–§8 | Shared primitives: tag offsets, certified intersection points and curves, a trimmed-face mesher, an embedding gate; operations are graph rewrites | Rational/algebraic surfaces and curves with exact topology decided by resultants and root isolation (QSIC-style); mesh only for output |
| Gain | Each PR closes one row with a closed-form proof; some results stay `Exact` | One primitive set serves Shell, Draft, Fillet, Chamfer and Boolean refresh on every pair of the five tag kinds; results compose (each is a retained body); Boolean topology stays on the proven mesh path | Exact topology, including tangent contact; exact readings where the geometry is algebraic |
| Cost | Rows multiply by pair × operation × configuration; Shell and Draft of mesh results can never build (no trims); crossing-cylinder and oblique blends need surfaces the vocabulary lacks; `facetedrecord.Payload` already carries about 30 special records | About 7 new internal packages over 13 PRs; new proof surface (interval Krawczyk certificates) needing heavy tests; measurements stay `Approximate`; certification and re-meshing cost time per call; 3 new `Surface` variants, 1 `Curve` variant, 1 option, 1 function | Torus–torus curves reach degree 16; pipe and canal blends are not low-degree algebraic, so modify stays approximate anyway; replaces the mesh Boolean; years of work |

**Recommendation: B.** It is the only candidate that reaches RC1 (Shell and Draft need trims, which A cannot supply)
without replacing the proven mesh Boolean, and C's exactness does not survive the first blend.

### Alternatives considered

- **D. Push the modify down to an operand and redo the Boolean.** Rejected: a Boolean rim (hole mouth, crossing edge)
  belongs to no operand, and Shell does not commute with a Boolean.
- **E. Promote the mesh result to a `brepPayload`.** Rejected: a brep keeps every frame a signed permutation of one
  reference frame (general-boolean §4.1); rotated and oblique results are exactly the ones that are not.
- **F. Fit surfaces or curves to held polygons.** Rejected: it blesses geometry from the polygons it should be checked
  against (CLAUDE.md; retention §12).

## Design

### Dispatch order

For each operation the trim-graph route sits after every specific route and before the existing refusal text, so a
receiver it declines keeps its current error.

| Operation | Insertion point | Increment |
|---|---|---|
| `Shell`, closed | `internal/brepmodify/shell.go`, the `call.NoOpenings` branch, immediately before `return nil, stackedshell.RefuseClosed(b.Payload)` | 1 |
| `Shell`, openings | `internal/brepmodify/shell.go`, after the `fp.PrismUnionFillet` arm and before `Receiver(...)` | 2 |
| `Fillet`, `Chamfer` | `internal/brepmodify/fillet.go` and `chamfer.go`, after `retainedblend.Fillet` / `retainedblend.Chamfer` returns `recognized == false` | 4 |
| `Draft` | `draft.go` / `internal/draftrecord/gates.go`, before SD23 for a `facetedrecord.Payload` | 6 |
| Boolean operand meshing | `internal/booloperand`, where a restating operand is asked at the pair tolerance | 11 |

A route **declines** (`matched == false`, no error) when the receiver does not lift or the selection is outside the
increment's scope. Once it matches, its failures are its own refusals (Table TG).

### Component 1 — `internal/trimgraph`: the graph and its lift

Responsibility: read a body's topology and tags into a graph that names, for every face, edge and vertex, which
exact geometry it lies on. It computes no new geometry.

Interface (increment 1):

```go
package trimgraph

// ErrNotLiftable marks a receiver outside the graph's admitted vocabulary. Callers decline on it.
var ErrNotLiftable = errors.New("trimgraph: the receiver is outside the trim graph")

// Graph is the lifted receiver. Faces[i] is the i-th face of topobuild.Faces(b), so fp.FaceOf indexes it.
type Graph struct {
	Faces    []Face
	Edges    []Edge
	Vertices []Vertex
}

// Face is one receiver face on its exact tag.
type Face struct {
	Node     *topo.Face
	Group    int                 // the fp.Groups index its facets carry (fp.Src)
	Tag      surfacegeom.Surface // Plane in increment 1
	Reversed bool
	Loops    [][]Use             // outer loop first, each in its topo walk order
}

// Use is one directed use of a graph edge in a loop.
type Use struct {
	Edge    int
	Forward bool
}

// Edge is one receiver edge between two distinct faces.
type Edge struct {
	Node       *topo.Edge
	Faces      [2]int // Faces[0] walks it Forward
	Start, End int    // vertex indices
	Convex     bool   // decided from the tags (L6), never from held facets
	Curve      CurveKind
}

// CurveKind names the geometry an edge lies on.
type CurveKind int

const (
	CurveLine CurveKind = iota // the line of two plane tags; later increments add kinds
)

// Vertex is one receiver vertex of valence 3 or more.
type Vertex struct {
	Node  *topo.Vertex
	Pos   r3.Vec  // held position
	Bound float64 // held bound, mm
	Edges []int   // incident graph edges
	Faces []int   // incident faces, in the cyclic order around the vertex
}

// Lift reads b, a body over fp, into a graph. It returns ErrNotLiftable (wrapped with the reason) when a lift rule
// fails, and ctx.Err() when ctx is done; it never returns a partial graph.
func Lift(ctx context.Context, b *topo.Body, fp facetedrecord.Payload) (Graph, error)
```

Lift rules (Table L). A failed rule returns `ErrNotLiftable`.

| # | Rule | Increment |
|---|---|---|
| L1 | `b.Solid`; `len(b.Lumps) == 1`; the lump has one shell, not open | 1 (later: several shells) |
| L2 | Every face has `RetainedTag`, `TagDeparture == 0`, `NormalBound == 0`, `Denoted == nil`, no stated `AxialDelta` above zero (`HasAxialDelta && AxialDelta != 0` declines: the true plane may sit off the tag), and a `Plane` tag | 1 (3b adds `Cylinder`, `Sphere`, `Cone`, `Torus` and denoted faces whose tag is the record, by `retainedblend`'s `tagIsRecord` rule) |
| L3 | Every topo edge has exactly two distinct faces and curve `Line3` | 1 (3b: the closed forms of retention §10.1; 5: certified curves) |
| L5 | Every vertex has valence 3. A valence-2 vertex cannot come from `facetedrecord.Build` except where the held hinge sign changes (`internal/facetedtopology/build.go` `breakAt`), which L6 rejects anyway, so it declines here | 1 (5 admits higher valence with a certified vertex) |
| L6 | Convexity from tags: for edge `e` with faces `a = Faces[0]`, `b = Faces[1]`, exact outward normals `Na`, `Nb` (`U×V` of the tag frame, negated when `Reversed`) and held end points `p0`, `p1`, let `s = (Na × Nb) · (p1 − p0)` in exact rationals. Require `|s| > |Na × Nb|_up · (β0 + β1)`; `e` is convex when `s > 0`. The result must equal `topo.Edge.Convex`, else ErrNotLiftable ("the tags and the held mesh disagree on the edge's convexity") | 1 |

`Group` comes from `fp.Src` of any facet whose `fp.FaceOf` is the face; every facet of one face carries one group,
else ErrNotLiftable.

Dependencies: `internal/topo`, `internal/topobuild` (`Faces`), `internal/facetedrecord`, `internal/surfacegeom`,
`internal/proofbound` (rational products and `RatSqrtUp`).

Failure behavior: `ErrNotLiftable`, or `ctx.Err()` on cancellation, which the caller returns unchanged. Never
`ErrUnsupported`, so dispatch can decline.

### Component 2 — `internal/anasurf`: exact tags, offsets and vertices

Responsibility: represent a tag in exact rationals with interval parameters, offset it, and meet tags at vertices,
returning enclosures. A derived tag (an offset, a rotation) is never rounded silently: its parameters are intervals
that contain the exact value.

Interface (increment 1, planes):

```go
package anasurf

// ErrSingular marks three planes whose normals are linearly dependent in exact arithmetic.
var ErrSingular = errors.New("anasurf: the planes do not meet in one point")

// Plane is {x : N·x = c} for one c in C, with the material on the side N·x < c. N is exact and not unit.
type Plane struct {
	N [3]*big.Rat
	C proofbound.RatInterval
}

// PlaneOf reads a tag: N = U×V of the frame in exact rationals, negated when reversed; C is the point interval N·O.
func PlaneOf(tag surfacegeom.Plane, reversed bool) (Plane, error)

// Inward moves p a distance t into its material: C − t·|N|, |N| enclosed by proofbound.IntervalSqrt of N·N.
// t must be positive.
func (p Plane) Inward(t *big.Rat) (Plane, error)

// Point is an enclosure of one exact point.
type Point struct {
	Box proofbound.IvVec3
}

// Meet3 encloses the common point of a, b, c by Cramer's rule over exact N rows and interval C. ErrSingular when the
// exact determinant is zero.
func Meet3(a, b, c Plane) (Point, error)

// Held returns the nearest float vector to the box midpoint and an upward bound on its distance from every box point.
func (p Point) Held() (r3.Vec, float64)

// StrictlyInside reports whether every point of the box satisfies N·x < c for every c in p.C:
// max over the box of N·x is below C.Lo, decided exactly.
func StrictlyInside(pt Point, p Plane) bool
```

Later increments add `Cylinder`, `Sphere`, `Cone`, `Torus` with interval radii and the offset rules below; each kind
gets the same `Inward`/`Outward` pair and an implicit form for Component 3.

| Tag | Offset by `t` into the material | Stays in vocabulary |
|---|---|---|
| Plane | level `c − t·|N|` | yes |
| Cylinder, radius `R`, material outside (hole wall) / inside (shaft) | `R + t` / `R − t`; `R − t ≤ 0` is TG4 | yes |
| Sphere | `R ± t` as the cylinder | yes |
| Cone, half-angle `α` | same axis and angle, apex moved `t / sin α` along the axis | yes |
| Torus, tube `m` | same centre, axis and major radius, tube `m ± t`; `m − t ≤ 0` is TG4 | yes |

Erosion pieces (Shell inward; Shell outward swaps convex and concave):

| Receiver feature | Cavity piece | Increment |
|---|---|---|
| Face | the face's tag offset by `t` | 1 |
| Convex edge | none: the two offset faces meet along their intersection | 1 |
| Concave edge | pipe of radius `t` about the edge curve: a `Cylinder` about a line, a `Torus` about a circle, a pipe tag otherwise | 3a (line), 3b (circle), 4 (other) |
| Vertex, all edges convex | none: the point where the offset faces meet | 1 |
| Vertex, all edges concave, valence 3 | `Sphere` of radius `t` about the vertex | 3a |
| Vertex, exactly one concave edge, valence 3 | none: the edge's pipe ends on the third face's offset | 3a |
| Vertex, two concave edges and one convex | two pipes that cross: TG2 until increment 5 | 5 |

A vertex contributes a sphere piece exactly when the material cone at the vertex contains an open half-space (the
nearest boundary point of a cavity point is the vertex only then). For valence 3 that is the all-concave case. The
proof lands in trim §3 with Task 12.

### Component 3 — `internal/certcurve`: certified points and curves (increment 5)

Responsibility: enclose a point of `T1 ∩ T2 ∩ T3`, or an arc of `T1 ∩ T2`, where no closed form exists.

- **Point certificate.** `F(x) = (g1(x), g2(x), g3(x))` with `g_i` the implicit form of tag `i` (plane `N·x − c`;
  cylinder `|x⊥|² − R²`; sphere `|x − C|² − R²`; cone and torus their standard quadric and quartic forms), whose
  parameters are rational intervals. A Krawczyk test `K(X) ⊂ int X`, evaluated in `proofbound.RatInterval`
  arithmetic with `Y` a float inverse of the Jacobian at the box centre, proves one zero of `F` in `X` for every
  parameter value in the intervals. A failed test refuses (TG10). It never falls back to the float Newton point.
- **Tube certificate.** An edge from vertex box `X0` to `Xn` is a chain of boxes `B_k`, each a parallelotope along the
  chord `p_k → p_{k+1}`. In `B_k` the one-parameter system `(g1, g2, u_k·(x − p_k) − σ)` passes Krawczyk for the
  whole interval `σ ∈ [0, |p_{k+1} − p_k|]`, so `T1 ∩ T2 ∩ B_k` is one arc that is a graph over the chord. Consecutive
  boxes share the zero at their common slice. The chord's bound is `B_k`'s half-width across the chord.
- **Coverage rule.** For an edge lifted from a held chain with `ChainBound` `b`, every `B_k` must contain the held
  chain's points between its slices widened by `b`. The true edge lies within `b` of the held chain (retention
  Claim E), so the certified arc is the true edge. A tube that does not cover refuses (TG11).
- Seeds (held vertices, chain points, offsets of them) only start Newton. They admit nothing.

### Component 4 — `internal/facemesh`: meshing a trimmed face

Responsibility: mesh one face of a trim graph from its tag and its boundary points, and state each facet's bound.

Interface (increment 1, planes):

```go
package facemesh

// Loop is one boundary loop: held points with their bounds, in the face's walk order (outer counter-clockwise
// about the outward normal, holes clockwise).
type Loop struct {
	Points []r3.Vec
	Bounds []float64
}

// Planar triangulates a planar face. n is the exact outward normal (anasurf.Plane.N); u, v is the tag frame used only
// to project for internal/triangulation.Triangulate. When (u×v)·n < 0 it projects with (v, u) instead, so the outer
// loop is counter-clockwise in the projection, as Triangulate requires. Indices refer to the concatenated loop
// points. It refuses
// (ErrUnsupported, TG5) when Triangulate drops a loop point, or when a triangle (a, b, c) fails the orientation
// margin: s = n·((b−a)×(c−a)) exact must satisfy s > |n|_up·(2β(|b−a|_up + |c−a|_up) + 4β²), β the largest of its
// corner bounds.
func Planar(ctx context.Context, n [3]*big.Rat, u, v r3.Vec, loops []Loop) ([][3]int, error)
```

A planar facet's bound is its largest corner bound: the true face is the polygon of the true corners, and the margin
keeps every triangle positively oriented under any corner move within the bounds, so the same triangulation tiles
the true polygon.

Increment 3a adds `Cylinder` (chart `(θ, z)`, a θ-monotone domain zipped column by column at angular step `Δθ` with
`R(1 − cos(Δθ/2)) ≤ τ`) and `Sphere` (gnomonic chart about the patch's mean direction, 4-to-1 splits until every
edge's angle `e` gives `R(1 − cos e) ≤ τ`, for patches within one hemisphere). Each chart states its facet bound as
sagitta plus corner bound; trim §4 carries the proofs. Increment 3b adds `Cone` and `Torus`.

### Component 5 — `internal/trimrebuild`: assembling and proving the result

Responsibility: splice new faces' facets into the receiver's held mesh, compose every bound, run the gates, and build
the body through `facetedrecord.Build`.

Interface (increment 1):

```go
package trimrebuild

// NewFace is one face the operation adds: its group (Surface, Retained, Reversed, Planar, NormalBound, Origins,
// Source) and its facets. Tris index the receiver's fp.Verts followed by Splice.Verts.
type NewFace struct {
	Group facetedrecord.Group
	Tris  [][3]int
}

// Splice is the rebuilt held mesh: the receiver's kept facets plus the new faces.
type Splice struct {
	Keep   []int     // receiver facet indices kept unchanged, with their groups
	Verts  []r3.Vec  // new vertices; index len(fp.Verts)+i in NewFace.Tris
	Bounds []float64 // their bounds
	Faces  []NewFace
}

// Build assembles the payload, composes its bounds, runs the held-embedding gate over every facet that touches a
// new vertex, and builds the body. A failed gate or held audit is ErrUnsupported (TG5, TG8).
func Build(ctx context.Context, env topo.Env, fp facetedrecord.Payload, s Splice) (*topo.Body, error)
```

Bound composition (pattern: `internal/retainedblend/route.go` `build`):

| Field | Value |
|---|---|
| `Verts`, `VertexBound`, `Tris`, `Src`, `Groups` | kept receiver facets and groups, then new faces; unused vertices compacted |
| `MeshBound` | `facetproof.FacetBoundMax` over the new mesh |
| `VolSymDiff` | receiver's, plus `proofbound.SweptVolumeAllow(βnew, proofbound.PerturbedAreaUpper(newVerts, newTris, βnew))`, `βnew` the largest bound over every vertex of every new facet (receiver vertices included), plus the sagitta term `τ·A_up` per curved new face (Task 11). The receiver's term still covers the kept facets: the identity behind `SweptVolumeAllow` holds facet by facet, so dropping facets (increment 2's removed face) only removes terms whose surface the new facets replace |
| `AreaSlack` | receiver's, plus `PerturbedAreaUpper(new, βnew) − A_held(new)` rounded up, plus `proofbound.FacetAreaTermSlop` per new facet, plus, for a splice that drops receiver facets, the shortfall of the rebuilt faces' geometric area terms against the receiver's by source group (`agent-docs/retained-edge-blend-design.md` §6.3, Area) |
| `DPair` | `facetedrecord.SourceDiameterUpper(verts, MeshBound)` |
| `Xform`, `ExactPlaced` | receiver's |
| every special record | dropped, as `retainedblend` does |

Gates:

- **Held embedding (TG5).** `meshbool.HasHeldFold` over the whole new mesh. `Exact` holds `proof.XptOf` of every
  vertex's held position (pattern: `internal/booleaneval/shift.go`, the `HeldRounding` it fills), and `Moved` is
  true for every new vertex. A fold refuses.
- **Convex cavity (Claim C1, increment 1).** See Component 6.
- **Claim T (increment 3a).** The general gate of Component 6, increment 3a.

### Component 6 — operations

#### Increment 1: closed inward shell of a convex planar receiver (`internal/trimshell`)

```go
package trimshell

// Call is a Shell request already past modify §4 stage 1.
type Call struct {
	Inward bool
	T      float64 // mm
	TDelta float64
}

// Closed builds the closed shell of b over fp. matched is false when b does not lift (trimgraph.ErrNotLiftable),
// when Call.Inward is false, or when some edge of the graph is concave; the caller then keeps its refusal.
func Closed(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, call Call) (*topo.Body, bool, error)
```

Construction:

1. `g := trimgraph.Lift(ctx, b, fp)`; decline on `ErrNotLiftable`, on `!call.Inward`, or on any `!Edge.Convex`.
2. `call.TDelta != 0` → TG7. `t, ok := proofbound.RatOf(call.T)`; `!ok` → TG7.
3. For each face `i`: `P_i = anasurf.PlaneOf(tag, Reversed)`, `Q_i = P_i.Inward(t)`.
4. For each vertex `v` with faces `a, b, c`: `w_v = anasurf.Meet3(Q_a, Q_b, Q_c)`; `ErrSingular` → TG3.
5. Gate C1: for every vertex `v` and every face `j` not incident to `v`, `anasurf.StrictlyInside(w_v, Q_j)`; else TG6.
6. Cavity face `i`: its loop is the outer loop's vertex cycle with each `v` replaced by `w_v`, walked in reverse
   (the cavity face's outward normal is `−N_i`). Mesh it with `facemesh.Planar(ctx, −N_i, …)`.
7. Group of cavity face `i`: `Surface` a `Plane` with the receiver tag's frame vectors `U`, `V` and origin `O'`, the
   nearest float vector to the exact `O − t·(N/|N|)` taken at the midpoint of the `1/|N|` enclosure;
   `Retained = true`, `Reversed = !Reversed_i`, `Planar = true`, `Origins = [{env.Ref, "cavity(i)"}]`,
   `Source = facetedrecord.FaceSource(env.Ref, 0, i, face_i.Node)`, no `CapRecord`.
   `NormalBound = d`, with `d` the upward bound on `|N·O' − c'| / |N|` over every `c'` in `Q_i.C`: the distance
   between the stored plane and the true one. This follows retention §3.4, which folds a placed plane's origin
   rounding `P` into `NormalBound` so that zero keeps meaning "the tag is the exact plane". `d = 0` exactly when the
   offset plane is a float plane; then the cavity face is an exact retained plane. Otherwise every exact-plane
   reader (blend R1, `MirrorFace`, retention §11's slab bound, L2) declines it, and `NormalAt` over-states its
   normal bound by `d`, which is sound.
8. `trimrebuild.Build` with `Keep` = every receiver facet.

**Claim C1.** Let the receiver's true solid be `K`. If every graph edge is convex (L6), `K` is convex (a connected,
closed, locally convex set is convex; at an edge local convexity is the edge's convexity, and at a valence-3 vertex
whose three edges are convex the vertex cone is a spherical triangle with every angle below π, so it is convex
too), and `K = ∩ H_i` with `H_i = {N_i·x ≤ c_i}` over its faces. Its erosion by a
ball of radius `t` is `C = ∩ {N_i·x ≤ c_i − t|N_i|}`. If every `w_v` lies strictly inside every non-incident
offset half-space (step 5), then `C`'s vertices are exactly the `w_v`, its facets are exactly the offset planes, and
face `i` of `C` is the polygon of `w_v` over the vertices of receiver face `i`, in the same cyclic order.

*Proof.* Each `w_v` meets three constraints with equality and the rest strictly, so it is a simple vertex of `C`. For
an edge `(u, v)` of faces `a, b`, with `c` the third face at `u` and `d` at `v`, the segment `w_u w_v` lies on
`Q_a ∩ Q_b`; `c` is tight at `w_u` and strict at `w_v`, so by linearity strict on `(w_u, w_v]`, and likewise `d`;
every other constraint is strict at both ends. So the open segment has exactly two tight constraints and is an edge of
`C`. The graph of these segments is 3-regular, and every `C`-edge at a `w_v` is one of its three. It is therefore a
union of components of `C`'s 1-skeleton, which is connected (Balinski), so it is all of it. ∎

Strictness is proven over the whole enclosure of each `w_v` against the lower end of each `c'_j`, so it holds for the
true values. The held cavity then follows Component 4's margin and Component 5's embedding gate.

#### Increment 2: inward shell with openings, convex planar receiver

`trimshell.Open(ctx, env, b, fp, removed []*topo.Face, call) (*topo.Body, bool, error)`. Removed faces keep their
plane unshifted: the cavity is `C = ∩_{kept} {N_i·x ≤ c_i − t|N_i|} ∩ ∩_{removed} {N_j·x ≤ c_j}`. Claim C1 holds for
`C` with the removed planes as constraints. Each removed face `F` becomes a lip face on `F`'s tag: outer loop `F`'s
loop, one hole loop the cavity's face on `F`'s plane reversed. Two removed faces that share an edge decline (TG1)
until a later increment; every face removed declines.

#### Increment 3a: erosion pieces on planar receivers, outward convex shells

Concave line edges get `Cylinder` pieces, all-concave valence-3 vertices get `Sphere` pieces, and an outward shell of
a convex receiver uses the dilation table. New closed-form curves: foot lines (tangent, by construction), circle and
`Ellipse3` arcs where a cylinder piece meets an offset plane (retention §10.5 closed form), great-circle `Arc3`
where a sphere piece meets a cylinder piece (tangent, by construction).

**Claim T.** Let `E` be the true erosion and `Σ` the constructed cavity surface, made of one piece `P_X` per generating
feature `X` over a domain `D_X` bounded by the piece curves. (i) Every true point of `D_X` lies at distance `t` from
`X` by construction. (ii) If the open ball of radius `t` about every true point of `D_X` misses `∂K`, then
`D_X ⊂ region_X`, the set of points of `∂E` whose nearest boundary feature is `X`. (iii) `region_X ⊂ D_X`, because the
transitions between nearest features are exactly the piece curves bounding `D_X`. So `Σ = ∂E`.

Gates that prove (ii) on held data:

| Gate | Check |
|---|---|
| T-far | for every new facet `f` of piece `P_X` and every receiver facet `g` whose face shares no vertex with `X`: `meshbool.DisjointTriDistanceAbove(f, g, t + δ_f + β_g)`, box-pruned |
| T-local | for every receiver face `g` sharing a vertex with `X`, the exact closed-form condition of trim Table TL (Task 11): `g`'s polygon lies in the half-plane of `g`'s plane bounded by each concave edge it shares with `X` (every held vertex on the side, with margin `β`), and `P_X` lies in the offset half-space of `g` where `g` is not a contact face |
| T-pieces | new facets of two pieces that share no vertex: `DisjointTriDistanceAbove(f, f', δ_f + δ_f')` |
| Held | `HasHeldFold` as in Component 5 |

The full proof of (ii)–(iii) and Table TL land in trim §5 with Task 11. A condition that cannot be decided refuses.

#### Increments 4–12 (outline)

| Inc | Operation | Construction |
|---|---|---|
| 4 | `Fillet` of an edge whose spine has a closed form | spine `= offset(T1, s·r) ∩ offset(T2, s·r)` (`s` by convexity), blend surface the pipe of radius `r` about the spine (`Cylinder`, `Torus`, or the new pipe tag), feet `c − s·r·n_i(c)` on each face, ends on the end faces by closed-form intersection; first reach: closed oblique plane–cylinder rims (`Ellipse3` spine), coaxial plane–cone and cone–cylinder rims |
| 5 | Certified curves | Component 3; lift of non-closed-form rims (crossing cylinders, off-axis sphere–cylinder), valence > 3 vertices, two-concave vertices in Shell; Increments 1–4 extend to them |
| 6 | `Draft` | each drafted plane rotated about its line on the neutral plane; a cylinder parallel to the pull becomes a `Cone`; neighbours re-trimmed by Component 3 |
| 7 | `Chamfer` of a general edge | setback curves `T_i ∩ pipe(edge, d)`, chamfer face ruled between them (new ruled tag) |
| 8 | Blend trimming and vertex blends | a strip that meets another face is trimmed by `pipe ∩ T_g`; three equal-radius blends at a valence-3 vertex end on a `Sphere`; two end on `pipe ∩ pipe` |
| 9 | Variable-radius `Fillet` | `WithRadiusAt`; spine points `c` with `dist(c, T_i) = r(λ(c))`, `λ` the chord projection between the edge's end vertices; canal blend tag |
| 10 | `FilletFaces` full round | two faces separated by one face strip that the ball removes whole |
| 11 | Boolean refresh | a restating operand whose faces lift is re-meshed from its graph at the pair tolerance instead of restating its held mesh |
| 12 | Lift of analytic payloads | prism, brep, revolve, loft and sweep receivers lift from their own tessellation, as the trim-graph fallback after their routes decline |

### Public API by increment

| Inc | Change | Docs to update in the same PR |
|---|---|---|
| 1–3b | none; `Shell` builds on more receivers | `shell.go` doc comment, `docs/api.md` Shell entry |
| 4 | `Surface` variant `Pipe{Radius units.Value}` (`KindPipe`): a rolling-ball surface of constant radius whose spine stays private | `agent-docs/api-design.md` §6.1, `docs/api.md`, `docs/data-structures.md`; STEP writes such a body faceted |
| 5 | `Curve` variant `Intersection3{Bound units.Value}`: the edge lies on both faces' surfaces; `Bound` is the held chain's proven distance from it | same |
| 6 | none; `Draft` builds on more receivers | `draft.go` doc, `docs/api.md` |
| 7 | `Surface` variant `Ruled{}` (`KindRuled`) | same as 4 |
| 9 | `func WithRadiusAt(p r3.Vec, r units.Value) FilletOption`: radius `r` at the selected edges' end vertex whose stored position equals `p` component-wise (as `EndpointAt`); `Surface` variant `Canal{MinRadius, MaxRadius units.Value}` | same |
| 10 | `func FilletFaces(ctx context.Context, b *Body, first, second FaceSelector, r units.Value) (*Body, error)`, `OperationFilletFaces` | same, plus `operation_kind.go` |

### Table TG — refusals

Each message ends with its code in parentheses, so a test asserts its own gate fired. `%w` is the sentinel.

| TG | Condition | Sentinel and text |
|---|---|---|
| TG1 | receiver does not lift, or selection outside the increment | none: the route declines |
| TG2 | a vertex outside the erosion-piece table (valence above 3, two concave edges and one convex) | `ErrUnsupported`: "a trim-graph shell has no piece for the vertex at %v (trim TG2)" |
| TG3 | three tags with no single common point (`ErrSingular`), or an enclosure too wide to decide a sign | `ErrUnsupported`: "the offset faces at %v do not meet in one point (trim TG3)" |
| TG4 | an offset removes a feature: radius `≤ 0`, a cavity face's loop orientation flips | `ErrUnsupported` (modify S11a's meaning): "the shell offset drops face %d of the receiver (trim TG4)" |
| TG5 | a new facet fails its orientation margin, or the held mesh folds | `ErrUnsupported`: "a rebuilt facet of face %d fails its orientation margin (trim TG5)" / "the rebuilt held mesh folds at facets %d and %d (trim TG5)" |
| TG6 | gate C1 or a Claim T gate fails | `ErrUnsupported`: "the cavity vertex at %v leaves the offset half-space of face %d (trim TG6)"; Claim T gates name the gate: "… fails T-far (trim TG6)" |
| TG7 | a size that does not convert exactly to millimetres | `ErrUnsupported`: "a trim-graph shell needs a thickness that converts exactly to millimetres (trim TG7)" |
| TG8 | `facetedrecord.Build`'s held audit fails | `ErrUnsupported`: "the rebuilt mesh fails its held audit (trim TG8): %w" |
| TG9 | reserved | — |
| TG10 | a Krawczyk test fails (increment 5) | `ErrUnsupported` |
| TG11 | a tube does not cover its held chain (increment 5) | `ErrUnsupported` |
| TG12 | a blend that the bounds prove consumes its face (increment 4) | `ErrDegenerate` |

## Soundness rules

- **S1 Topology is read, never decided here.** Faces, loops, incidences and convexity come from the receiver's
  topology. A tag-derived convexity that disagrees with the held flag rejects (L6). A failed certificate refuses; it
  never re-topologises.
- **S2 Tags are admitted by record, never by fit.** A face enters only by L2. A `Faceted` face, a `NURBSSurface`, a
  denoted face whose tag is not exactly its record, and a tag with a stated departure decline.
- **S3 Residuals only reject.** A held vertex far from a closed-form curve, or a held chain outside its tube, rejects.
  A near one proves nothing; the curve is trusted because Claims R and E hold.
- **S4 Enclosures are proofs, not residuals.** A Krawczyk certificate evaluates `F` at the centre and the Jacobian
  over the whole box in exact rational intervals, so it proves existence and uniqueness. A float Newton point is a
  seed only.
- **S5 Derived tags carry intervals.** An offset, rotation or blend parameter is an interval that contains its exact
  value. A stored float tag records its distance from that interval in `TagDeparture`.
- **S6 Tangency is admitted only by construction.** A blend's foot curve is tangent because the construction made it
  so. Two receiver tags that merely look tangent have no certificate (singular Jacobian) and refuse.
- **S7 No repair.** A fold, a flipped loop or a margin failure refuses. The only vertex moves are the existing
  float-box and gap-tolerance embedding searches (evaluator §9), whose displacement is charged.
- **S8 Topology-preserving only.** A feature an offset drops, merges or splits refuses (TG4, TG6).
- **S9 Every bound covers every later operation.** Every raw float operation after an enclosure is charged through
  `proofbound`; no measurement site computes a bound inline.
- **S10 Specific routes first.** This route never replaces an answer an existing route gives.
- **S11 Measurements stay `Approximate`.** No trim-graph result reports a zero volume, area or centroid bound.
- **S12 No 2D question is asked here.** Loops come from topology; triangulating decad's own held loops is mesh
  construction. A future rule that needs a region or arrangement asks `sketch`.

## Performance

- Increment 1 adds one Lift (linear in topology), `O(V·F)` exact half-space tests for gate C1, and one held-fold scan
  over the new facets (box tree, `internal/meshbool/held_embedding_tree.go`). Expected cost is below the receiver's
  own Boolean.
- Claim T's T-far is box-pruned to pairs within `t + δ` of each other.
- Krawczyk work is per vertex and per chord; widen intervals with `proofbound.RatFloorGrid` to bound rational growth,
  and poll `ctx` per box (evaluator §9's polling rule).
- No timing gate is added; CI shard times are read from GitHub job logs.

## Rollout

Each increment is one PR carrying its tests and the doc changes. Increment 1's PR also carries
`agent-docs/trim-graph-design.md`.

| Inc | Lands | `missing-features.md` rows | Gate to start the next |
|---|---|---|---|
| 1 | Lift, planar `anasurf`, planar `facemesh`, `trimrebuild`, closed inward Shell of a convex planar receiver | "Closed shell (`WithNoOpenings`) outside …" narrows | `TestTrimGraphClosedShell` green on CI |
| 2 | Shell openings on convex planar receivers | "`Shell` of a mesh Boolean result …" narrows | Task 9 test green |
| 3a | Cylinder and sphere pieces and charts, Claim T, outward convex shell | both Shell rows narrow | a `design-doc-reviewer` pass on trim §5 with no blocking issue before merge; Task 13 tests green |
| 3b | Curved receivers with closed-form curves; cone and torus charts | both Shell rows narrow | its tests green |
| 4 | General constant-radius Fillet on closed-form spines; `Pipe` | "Fillet/chamfer of most faceted boolean results", "Retained edge blend between crossing cylinders …" (oblique and cone parts) | its tests green |
| 5 | Certified curves; `Intersection3` | crossing-cylinder blend, Shell of crossing results | its tests green |
| 6 | Draft | "`Draft` of a mesh Boolean result" | its tests green |
| 7 | General Chamfer; `Ruled` | chamfer halves of the blend rows | its tests green |
| 8 | Blend trimming, vertex blends | "Retained edge blend or tangent chain whose new strip would cross …", SR2 | its tests green |
| 9 | Variable radius; `Canal` | "Variable-radius fillet …" (variable half) | its tests green |
| 10 | `FilletFaces` | "Variable-radius fillet, face-to-face fillet" closes | its tests green |
| 11 | Boolean refresh | "Held mesh operand coarser than the pair tolerance …" narrows | its tests green |
| 12 | Analytic payload lift | the RC5 rows narrow | — |

Rollback: each increment's dispatch arm is one `if` before an existing refusal; reverting the PR restores the
refusal.

## Decisions

- **Pipe, Ruled, Canal surface variants.** A non-analytic blend surface is published as the new sealed variants
  `Pipe`, `Ruled` and `Canal` (increments 4, 7, 9), because `NURBSSurface{}` promises a source-derived control net and
  `Faceted{Bound}` loses the face identity. Increment 1 adds none.
- **Ball erosion, with the orthogonal split documented.** A trim-graph shell rounds a reflex corner with radius `t`
  (ball erosion). The orthogonal shell (`shell_orthogonal.go`) runs first and keeps its mitred cube erosion for an
  axis-square faceted solid with no vertex bound, so no result that builds today changes. Increments 1 and 2 are
  unaffected: convex receivers have no concave edges.

## Staged tasks

Increment 1 has landed. Increment 2 is Tasks 8-9. Increment 3a is Tasks 10-13. Later increments are the outline rows
of Component 6; the PR that implements one adds its tasks here first.

### Task 8: `trimshell.Open` for convex planar receivers

- **Files:** create `internal/trimshell/open.go`, `open_test.go`; modify `internal/brepmodify/shell.go` (call
  `trimshell.Open` after the `fp.PrismUnionFillet` arm, before `Receiver(...)`).
- **What:** Component 6 increment 2: removed planes unshifted, Claim C1 over `C`, one lip face per removed face (outer
  loop the face's, one hole the cavity's face on that plane, reversed), groups `lip(k)` keeping the removed face's tag,
  `Reversed` and `CapRecord`. Adjacent removed faces and "all faces removed" decline.
- **Depends on:** 7.
- **Tests:** hand-built pentagonal prism with the top removed, `t = 1`: lip face has 2 loops; cavity vertices on the
  top plane are exact meets of the top plane with two offset planes.
- **Done when:** `go test ./internal/trimshell/` passes.

### Task 9: Public test and docs for increment 2

- **Files:** modify `apitest/trim_shell_test.go` (test `TestTrimGraphOpenShell`), `.github/test-shards-apitest.txt`
  (add `10\t0.00\tTestTrimGraphOpenShell`), `shell.go` doc, `docs/api.md`, `agent-docs/missing-features.md` (row
  "`Shell` of a mesh Boolean result …"), `agent-docs/trim-graph-design.md` (increment 2 lands).
- **What:** end-to-end test and doc rows.
- **Depends on:** 8.
- **Tests:** `Shell(ctx, beveled, Faces(Facing(r3.NewVec(0, 0, 1))), 1 mm)`: the cavity section in `(y, z)` is
  `(1, 1), (19, 1), (19, 20), (2 + √2, 20), (1, 19 − √2)` over `x ∈ [1, 39]`, so the volume is
  `15920 − 38·(342 − (1 + √2)²/2)` within the published bound; the top lip face has two loops; removing the bevel
  face and the top together (adjacent) returns `ErrUnsupported` with the existing refusal text (declined).
- **Done when:** `go test ./apitest/ -run '^TestTrimGraphOpenShell$'` passes; `go test . ./apitest/ -run '^TestCI'`
  passes.

### Task 10: `anasurf` cylinder and sphere; closed-form piece curves

- **Files:** create `internal/anasurf/cylinder.go`, `sphere.go`, `curve.go` and tests.
- **What:** interval `Cylinder` (`Origin`, `Axis`, `R` intervals) and `Sphere`; pipe and sphere pieces of the erosion
  table; closed-form curves: foot line, cylinder ∩ plane (`Circle3`/`Arc3` when the axis is normal, `Ellipse3` per
  retention §10.5 when oblique with `k ≥ 10⁻³`, else TG3), sphere–cylinder tangency great circle; each returns
  enclosed sample points at a requested angular step.
- **Depends on:** 7.
- **Tests:** an ellipse arc's enclosed points lie on both surfaces within their bounds and match the retention §10.5
  closed form; `k = 5·10⁻⁴` refuses TG3.
- **Done when:** `go test ./internal/anasurf/` passes.

### Task 11: Claim T gate and trim §5

- **Files:** create `internal/trimrebuild/claim_t.go`, `claim_t_test.go`; modify `internal/trimrebuild/build.go`
  (`NewFace` gains `Generator` (the receiver face, edge or vertex index the piece is generated by) and `Contact`
  (the receiver faces its balls touch); `Splice` gains `Reverse []ReversedFace`, each a receiver face index whose
  facets are kept with flipped winding under a new `Group`, for an outward shell's void; `Build` adds the sagitta
  terms `τ·A_up` to `VolSymDiff` and the chart area slack to `AreaSlack` for curved new faces, and runs Claim T
  when any `NewFace` has a generator), `build_test.go`; modify `agent-docs/trim-graph-design.md` (§5: the full proof
  of Claim T (ii)–(iii) and Table TL).
- **What:** T-far, T-local and T-pieces of Component 6 increment 3a; the `Reverse` splice; the curved-face bound
  terms.
- **Depends on:** 10.
- **Tests:** a U-channel narrower than `2t` refuses with "(trim TG6)" or "(trim TG5)"; a grooved-bar cavity passes; a
  far face within `t` of a piece refuses with the T-far text; a `Reverse` splice of a cube builds a void shell of 6
  faces whose `NormalAt` points into the cube.
- **Done when:** `go test ./internal/trimrebuild/` passes. Before increment 3a merges, a `design-doc-reviewer` pass
  on trim §5 returns no blocking issue (Rollout gate).

### Task 12: `facemesh` cylinder and sphere charts; `trimshell` erosion pieces

- **Files:** create `internal/facemesh/cylinder.go`, `sphere.go` and tests; modify `internal/trimshell/closed.go`,
  `open.go` (pieces, outward convex through `Splice.Reverse`), `closed_test.go`; modify `agent-docs/trim-graph-design.md` (§3 sphere-piece proof, §4 chart
  proofs).
- **What:** the two charts of Component 4 with their sagitta bounds; the erosion table rows of increment 3a; the
  dilation table for an outward shell of a convex receiver.
- **Depends on:** 11.
- **Tests:** the grooved bar's closed inward shell has one cylinder piece of radius `t` about each concave groove-floor
  edge; a
  rectangular pocket's cavity corners are sphere octants; the outward closed shell of the bevelled plate has cylinder
  pieces on its 15 edges and sphere pieces at its 10 vertices, volume `V_dilated − 15920` with `V_dilated` from the
  Steiner formula `V + A·t + M·t² + (4/3)πt³`, `M` the mean-width term `Σ ℓ_e·(π − φ_e)/2`.
- **Done when:** `go test ./internal/facemesh/ ./internal/trimshell/` passes.

### Task 13: Public tests and docs for increment 3a

- **Files:** modify `apitest/trim_shell_test.go` (test `TestTrimGraphErosionShell`), `.github/test-shards-apitest.txt`,
  `shell.go` doc, `docs/api.md`, `agent-docs/missing-features.md` (both Shell rows), `agent-docs/trim-graph-design.md`
  (increment 3a lands).
- **What:** end-to-end tests through a real mesh `Cut` (the grooved bar `wedgedPlate(t, sc.w.YZ(), [][2]float64{{8, 14}, {10, 12}, {46, 48}, {44, 50}}, 100, r3.Vec{X: 0.5})`,
  a groove whose walls run at 45° in the YZ plane, so its vertices carry nonzero bounds and `shellOrthogonal` does not
  take it; a rotated tool's tags state a departure, retention §3.5, and the lift refuses them at L2) and real `Shell`.
- **Depends on:** 12.
- **Tests:** the grooved bar is an extrusion along x of its `(y, z)` section `A`, and the distance to the boundary of
  a product is the smaller of the two factors' distances, so its erosion is `(A ⊖ D_t) × [t, 40 − t]`. The closed
  shell's volume equals `V − (40 − 2t)·area(A ⊖ D_t)` within the bound, with `area(A ⊖ D_t)` computed in the test
  from the offset lines and a circular sector of radius `t` at each reflex corner (modify §8 offset table);
  `VerifyAll` passes; the Steiner outward test of Task 12 through the public API.
- **Done when:** `go test ./apitest/ -run '^TestTrimGraphErosionShell$'` passes; `go test . ./apitest/ -run '^TestCI'`
  passes; `go vet ./...` and `golangci-lint run` pass.
