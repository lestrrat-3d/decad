# Coplanar Contact Design

Exact classification of coplanar facet contacts in the mesh boolean, and exact planar faces in its results: root
cause (§1), required behavior (§2), the classification rule (§3), composition with face bounds and records (§4), the
manifold audit (§5), the proximity gates' obligation (§6), planar face simplification (§7), records of results (§8),
soundness (§9), performance (§10), tests (§11), risks (§12), user decisions (§13), code map (§14), open questions
(§15), seats in an Intersect (§16), placed seats (§17), flush planes that agree only within a bound (§18),
separated cap pairs (§19) and separated caps on mesh Boolean results (§20).

Implements `agent-docs/interference-design.md` §5.2 and §11 row 4. Companion to `agent-docs/evaluator-design.md` §9
(mesh boolean), `agent-docs/gap-tolerance-design.md` §5.2 (shifted retry),
`agent-docs/faceted-vertex-bounds-design.md` §3.4 (weld) and `agent-docs/faceted-source-retention-design.md`
(retained tags). Adds no public API and no option.

## Router

Navigation, not authority.

| Question | Section |
|---|---|
| Why does a flush pair fail without the exact classification? | §1 |
| What must hold? | §2 |
| Which coplanar facet pairs are classified, and how? | §3 |
| What happens to a face with a positive bound? | §4 |
| What stops a non-manifold result, and what does the retry see? | §5 |
| Why do the proximity gates stay sound? | §6 |
| How does a `Cut` or `Union` result get exact planar faces? | §7 |
| Why does a flush `Union` keep its `Plane` faces and history? | §8 |
| Why can a real overlap never read as contact? | §9 |
| What does it cost? | §10 |
| Which tests prove it? | §11 |
| Which code and tests implement each part? | §14 |
| Why can a seat on a bored pocket floor be admitted in an `Intersect`? | §16 |
| How is a placed body's true plane read? | §17 |
| Why do planes that agree only within a bound go to the gap-tolerance retry? | §18 |
| How are two bodies on opposite sides of a shared cap, such as a frustum and a cone on one circle, decided? | §19 |
| Why does the shifted retry never read a separation certificate? | §19.7 |
| How is a mesh `Cut` or `Union` result decided across a shared cap? | §20 |

## Assumptions

- "Exact face" reuses the proximity gate's per-face reading: `topo.Mesh.FaceBound[f]` narrowed by
  `BoolMesh.FaceSurface[f]` (`facesOfMesh`, `internal/meshbool/tangency_gate.go`) is `<= 0`. No new exactness notion.
- Non-coplanar grazes (both apexes strictly on one side) and isolated point touches not produced by an admitted
  coplanar pair keep their refusals.
- A shifted, grown or fixed-tool attempt (`hasShifts || fixed != nil` in `booleaneval.evaluate`) admits no coplanar
  pair: its `BoolMesh.Exact` stays nil. The retry ladder therefore sees the refusal a coplanar pair gives without §3.
- The bounded replay path classifies coplanar contacts as the live pass does, on the replay meter: it reads exactness
  for its operands, admits the same pairs, inserts the same cuts, keeps or drops covered regions by the same table,
  classifies an unanchored region by exact parity of its probe, and runs the manifold audit. A region it cannot
  classify, and a simplified mesh the live stitch would flip, are declines. §7 runs on the replay stitch as well, so
  live and replay held meshes are equal for every boolean both paths build.
- Planar face simplification (§7) leaves any face region with a hole untouched in this design.
- decad tests use minimal box and prism fixtures and name no consumer model (`agent-docs/testing.md` "Test scope").
  The consumer dovetail check runs outside this repository (§11.4).
- The behavior has no switch: the exact coplanar pass and planar face simplification run on every mesh boolean.

## 1. Problem and root cause

Two solids that meet face to face (zero gap, no interior overlap) must verify as `Sound`, and `Union`, `Intersect`
and `Cut` must build them exactly. An analytic path admits some pairs (prism class A, class B with axis-aligned walls,
history certificates). Every other pair reaches the mesh boolean, where, without §3 and §7, it fails as follows. Steps
1 and 3 still describe the path a pair takes when a face has a positive bound (§4):

1. **Coplanar refusal.** `TriTriClassify` returns `ContactRegion` for any coplanar facet pair sharing positive area or
   a positive-length boundary segment (`CoplanarOverlap`, `internal/meshbool/triangle_contact.go`). `pairCuts` refuses
   the boolean with `TouchContactError{What: TouchCoplanar}` (`internal/meshbool/boolean_pair_cuts.go`).
2. **Coplanar-bounded edges read as grazes.** An in-plane facet edge is a crossing only when its two apexes straddle
   the partner's plane (`EdgeSide`, `internal/meshbool/boolean_mesh.go`). When the adjacent facet lies IN that plane
   (apex sign 0), a two-outcome test calls the edge a graze, although it is the rim of a flush patch.
   `EdgeCoplanarBounded` (§3.3) is the third outcome.
3. **The gap retry replaces the exact answer.** `booleaneval.Evaluate` catches the refusal and runs `retryShifted`
   (`internal/booleaneval/shift.go`). A flush `Union` first shrinks the second operand by `reach/4096` (2.44e-6 mm at
   the 0.01 mm default), then tries translations, then growth. The result:
   - every face of the moved operand reads `Faceted`, because `MovedGroupWithShift` keeps no tag for a piece with
     `Grow != 0` (`internal/boolshift/moved_tag.go`). The bound size is not the reason: a retained `Plane` is admitted
     at any face bound (`facetedrecord.HeldNormalTerm`). The first operand's faces keep their tags;
   - the moved faces carry the move in their vertex and face bounds (`internal/boolshift/shift.go`);
   - `ev.Shift` appends `CodeGapShiftApplied` to `OperationWarnings` (`booleandispatch.meshBody`), so
     `boolchain.Expandable` refuses the body and no history certificate or regroup expands it;
   - rims between a kept `Plane` face and a moved face fall back to `FacetedCurve`.
4. **Mesh results hold rounded artifact vertices.** A mesh `Cut` or `Union` of exact prisms has exact B-rep corners,
   but its held facets also hold vertices where one operand's triangulation diagonal crosses a plane of the other.
   Those exact points are not floats; the weld rounds them and charges about 1e-15 mm
   (`internal/meshbool/mesh_stitch.go`). A face's `FaceBound` is its largest facet bound
   (`internal/facetedrecord/restate.go`), so every face holding such a vertex is inexact.
5. **The proximity gate refuses the next pair.** `RefuseUndecidableProximity` refuses a face pair whose facets come
   within the summed face bounds without a crossing proven deeper than that sum. Step 3's moved faces and step 4's
   rounded faces meet their flush partners inside the bound, so `Verify` reports `DiagUnsupportedPairContact` and
   `Union`/`Intersect` return `BooleanUnsupportedContact`. The refusal is correct for the data held. The fix is to
   classify exact flush contact (§3) and stop producing inexact faces where the true face is exact (§7).

`TestCoplanarContactMadeByCut` (R1 to R4, §11.2) pins the made-body pairs that hit steps 4 and 5.

**Out of scope:** curved coplanar or tangent contact; non-coplanar grazes and isolated point touches; contact between
faces with a positive bound that are no flush pair (they keep the refusal and retry; flush pairs are
`agent-docs/flush-contact-design.md`); `ErrCrossContainer`; changes to the gap-retry ladder; the analytic (prism,
class B, brep) paths; merging coplanar faces of different operands into one face.

## 2. Correct behavior and success criteria

- **C1.** A pair whose contacts are coplanar patches of exact faces (§3.1), plus the edges and points that bound those
  patches, is classified by exact predicates. When the result is manifold, no `TouchContactError` is raised, so the
  gap retry is never reached.
- **C2.** Opposite-facing coincident patches are boundary-only contact: `Union` drops both copies, `Intersect` drops
  both, `Cut` keeps the target's copy. Same-facing coincident patches keep one copy (operand A's) for `Union` and
  `Intersect`, and none for `Cut` (§3.4).
- **C3.** An unshifted empty `Intersect` proves disjoint interiors, so `Verify` reports no row and, with no other
  finding, `Sound`. `interference-design.md` §6.1's premise P2 and Step 1 state this for admitted pairs.
- **C4.** A flush result keeps every retained `Plane` tag of both operands, carries no `CodeGapShiftApplied` warning,
  and stays `boolchain.Expandable` (§8).
- **C5.** A face of a mesh result whose held facets are coplanar, hole-free, with every vertex claim zero and float
  corners, is held exactly: its stitched vertices carry zero bound and the face's `FaceBound` is 0 (§7).
- **C6.** A pair whose true solids overlap is never reported touching or disjoint (§9).
- **C7.** R1 to R4 (§11.2) and the six consumer dovetail parameter sets read `Sound`, `Intersect` returns
  `BooleanEmpty`, and `Union` volume equals the sum of the halves within its bound (§11.4).
- **C8.** A flush `Union` of exact boxes under the default gap tolerance runs no slower than the retry path, and
  non-coplanar booleans stay within benchmark noise (§10).

## 3. The exact coplanar classification rule

decad computes the 2D answers of this section (clipping, cover tests) and of §7 (re-triangulation) itself instead of
asking `sketch`: they are exact rational predicates over decad's own held facets, run per facet pair on the boolean's
hot path, and a private `sketch` scene per facet pair would cost a scene build per pair for no gain in exactness. The
same reason backs the exact coplanar clip of `agent-docs/multibody-dynamics-design.md` §9.4.

### 3.1 Admission

A facet `t` of a prepared operand is **exact** when both hold:

- `FacetBound[t] == 0`;
- the gate reading of its source face is zero, computed by `meshbool.FaceGateDelta`, the helper `facesOfMesh` uses.

A coplanar facet pair `(i, j)` (all six plane-side signs zero, the `CountZero == 3` branch) is **admitted** when
`pairExact(i, j)` holds: facet `i` of A and facet `j` of B are both exact, or their faces are a flush pair
(`flushPair`, `agent-docs/flush-contact-design.md` §2, §4.5). Otherwise a coplanar pair with `CoplanarOverlap` true is
**refused**: a `coplanarContactOf` contact, `TouchCoplanar`, Slack 0, Decisive when facing opposite. The refused path
leads to the gap-retry ladder. `Exact` and every reader of it alone (`seat.go` wall exactness) are unchanged.

Face-level exactness, not carrier coplanarity, is the admission test because `interference-design.md` §5.2 forbids
settling a coplanar pair on carrier coplanarity alone: a planar face with a chorded rim holds an inscribed outline. A
zero face reading is the tessellation's claim that every held polygon of the face IS the true trimmed patch. It is the
predicate the gate already skips on, which makes §6 hold by construction. A flush pair settles the pair on carrier
coplanarity together with outlines that keep their arrangement under every move their bounds allow (flush-contact
§4.4), which covers the inscribed outline.

### 3.2 Pair pass (`pairCuts`)

A **patch-boundary edge** of a facet `t` is an edge whose twin facet (`TwinFacet`) is not coplanar with `t`. An edge
whose twin is coplanar is internal to the operand's planar patch (a triangulation diagonal) and is never inserted.
Inserting internal edges would give a partner corner inside a facet three incident segments, which `BuildChains`
(`internal/meshbool/boolean_cut.go`) refuses as a branch.

For each admitted pair `(i, j)` with `CoplanarOverlap` true:

1. Clip every patch-boundary edge of `tb` to closed `ta`, exactly, in the dominant-axis projection of `na`
   (`ProjAxes`), with `CoplanarClip`. Keep each clipped piece of positive length that does not lie along an
   edge of `ta`. Append it to `cutsA[i]` as `Xseg{ViaParity: true, Partner: tb}`, with `AOnEdge`/`BOnEdge` from the
   clip. Do the same with `ta`'s patch-boundary edges into `cutsB[j]`. `ViaParity` means "never an anchor"
   (`RegionOf`). That is required: a region beside a coplanar segment lies IN the partner's plane, so the
   partner-plane side test reads 0.
2. Record `coplanarA[i] += j` and `coplanarB[j] += i`.
3. Add every clipped endpoint key to the `owned` key set.
4. Append `coplanarContactOf(ma, mb, i, j)` to `legacyCoplanar` (capped at `maxTouchContacts`). The pair pass never
   raises it; §5 may.

An admitted pair whose closed triangles share only a point (`CoplanarTouch`) adds that point to `owned`, appends
`touchOf(ma, i, j)` to `legacyPoints`, and does nothing else. `TriContact` and `TriContactSpan` carry `Coplanar bool`,
set only by the coplanar branch of `triTriSpan`, so the pass tells coplanar points from crossing points.

**Remaining refusals.** A segment set that still branches inside a facet keeps `BuildChains`' refusal, an expected
contact (`ErrUnclassifiableContact`). That needs a crossing chain of the other operand ending on a patch-boundary edge
at a point strictly inside the facet, which happens only where the other operand's surface passes through the plane at
its own patch rim.

### 3.3 In-plane edges

`EdgeSide` returns one of three outcomes.

| Apex signs `s0`, `s1` | Outcome | Action in `pairCuts` |
|---|---|---|
| `s0·s1 < 0` | `EdgeCrossing` | block the edge, add a `ViaParity` cut to the partner |
| both nonzero, same sign | `EdgeGraze` | collect a `touchGraze` contact and refuse |
| exactly one is 0 | `EdgeCoplanarBounded` | the adjacent facet with sign 0 lies in the partner's plane |
| both 0 | impossible | `ErrBooleanFailed` (the facet itself would be coplanar with the partner) |

`EdgeSide` also returns the index of the adjacent facet that lies in the plane. A coplanar-bounded edge is accepted
when the edge's own facet and the partner facet are both exact, or the in-plane facet and the partner facet lie on a
flush pair (flush-contact §4.6: the edge's own facet is a wall whose rim is an outline edge of the flush face), and the
in-plane facet and the partner facet form an admitted pair recorded by §3.2 step 2. It then adds its two segment
endpoints to `owned`, appends `touchOf(ma, p.I, p.J)` to `legacyGrazes`, and does nothing else: no cut (the segment
runs on the facet's own edge, and the admitted pair already cut the partner), no block (the coplanar facet takes the
cut path, §3.4, so no flood fill crosses the edge). Two other cases:

- The pair is refused (an inexact facet off a flush pair): §3.1's refusal returns before the in-plane loop runs.
- No such pair was recorded: `ErrBooleanFailed`. The two facets share a positive-length segment in one plane, so box
  enumeration visited them.

A contact with both `EdgeA >= 0` and `EdgeB >= 0` applies each edge's own action. The isolated-point rule reads
`rims ∪ owned` instead of `rims`.

### 3.4 Side pass

`keepSideWith(ctx, m, other, cuts, opts keepOptions)` carries the coplanar map, the op and which operand `m`
is. `keepSide` delegates with zero `keepOptions` and keeps its signature and behavior.

Every facet with a non-empty `coplanar[i]` takes the cut path and counts as cut for the uncut-component flood fill, so
no component seed's verdict spreads onto it. With no surviving segment it is one region, the facet itself, with its
centroid as probe, and `CutTriangle` is not called. For each region `R` with probe `p` (strictly inside `R`,
`RegionOf`):

1. Cover set `C = { j in coplanar[i] : p in closed tb }`, by exact 2D orientation in the projection of §3.2. A probe on
   a patch-boundary edge of some `tb` is `ErrBooleanFailed`: those edges were inserted. A probe on an internal edge of
   the partner's patch lies in both facets sharing it, and both are in `C` (both are coplanar partners of `i`).
2. `C` empty: classify by anchor, else by parity ( parity `onBoundary` is the expected contact refusal).
3. `C` non-empty: read the facing `s = sign(na · nb)` over the exact normals for every `j` in `C`. Mixed signs give
   `ErrUnclassifiableContact("mixed coplanar side evidence")`. Otherwise keep or drop by this table, never by parity or
   anchor (`coplanarKeep`):

| Op | same facing: A copy | same facing: B copy | opposite facing: A copy | opposite facing: B copy |
|---|---|---|---|---|
| `Union` | keep | drop | drop | drop |
| `Intersect` | keep | drop | drop | drop |
| `Cut` (A − B) | drop | drop | keep | drop |

The table is read before `Cut` flips B's kept facets. Derivation: at a covered point both boundaries pass, and each
operand's material lies opposite its outward normal. Same facing puts both materials on one side; opposite facing puts
one on each side. A boundary point of the result is one with result material on exactly one side: `A ∨ B` for
`Union`, `A ∧ B` for `Intersect`, `A ∧ ¬B` for `Cut`. Keeping A's copy, never B's, makes the choice independent of
facet order.

The covered part of facet `i` is `∪ (ta ∩ tb)` over `coplanar[i]`, the same point set as the covered parts of those B
facets, so keeping one copy leaves neither a hole nor a duplicate.

Before the exact cover predicate in the ordinary `keepSideWith` pass, a partner may be rejected when the nearest-float
probe lies strictly outside the closed axis-aligned interval of one coordinate of its triangle. Build each interval
from the three exact vertices converted with `Xpt.Vec`, and cache it lazily by partner facet for one `keepSideWith`
call. Round each exact probe with the same conversion. Reject only when the probe and all three converted vertices
are finite, and a coordinate is strictly below the rounded minimum or above the rounded maximum. Otherwise run the
existing exact predicate. Round-to-nearest is monotone, so a point inside or on the exact triangle cannot fall outside
any such rounded interval. Keep partner order, exact boundary checks, and replay work accounting unchanged.

### 3.5 Bounds of new vertices

A clipped endpoint of an exact pair lies on two exact facets. `CutCornerBound` gives a point that is neither a facet
corner nor a rim key the facet's `FacetBound`, which is 0 for an exact facet. No `rims` entry is added for coplanar
points, so `RimBound` and `MaxRim` are unchanged. A clipped endpoint of a flush pair `(ta, tb)` records
`max(FacetBound[ta], FacetBound[tb], Cross)`, the largest over such pairs, and the side pass gives a kept corner that
is not one of its facet's corners at least that record (flush-contact §4.8). An exact pair records nothing.

## 4. Composition with face bounds, `Plane` records and the `Faceted` fallback

- **Positive face bound.** A face whose gate reading is positive enters §3 only through a flush pair
  (`agent-docs/flush-contact-design.md`): two planar carrier faces in one exact plane whose outlines keep their
  arrangement. Its other coplanar contacts are refused with `TouchCoplanar` and the gap retry runs as documented. This
  covers curved faces, planar faces whose chorded rims come near the other outline, every shifted or grown attempt
  (Assumptions), and mesh results whose vertices round near the other outline.
- **Surface kind is not read.** Admission reads bounds only. A `Faceted` face with zero bound is admitted. A `Plane`
  face with a positive bound is refused. The retained tag matters for what the result keeps (§8), not for admission.
- **Rounding-only bounds.** §7 removes the bound where it comes only from artifact vertices, so a face whose true face
  is an exact plane with float corners becomes exact and enters §3 in the next boolean.

## 5. Manifold audit

Classifying edge-only and point-only coplanar contacts can produce a `Union` or `Cut` result that meets itself along an
edge or at a vertex. `stitchEdgesPaired` reports such an edge as `ErrBooleanFailed`, an invariant failure, and does not
detect a pinched vertex. Both must be expected contact refusals carrying the contacts of the refused-pair path, so `Verify` reads `Suspect`
instead of returning an error, and the gap retry for such a pair (translation and, for `TouchCoplanar`, growth) runs.

`MeshBooleanStats` returns `Stats.Coplanar *ContactKeys`, non-nil if and only if `owned` is non-empty:

```go
// ContactKeys is what the stitch needs to tell a contact pinch from a broken mesh.
type ContactKeys struct {
	Owned  map[string]struct{} // exact keys of every point §3.2–§3.3 accepted
	Legacy error              // the refusal the pair gives with no exact facet
}
```

`Legacy` is built from the lists of §3.2–§3.3 in this order: `legacyCoplanar` non-empty gives
`ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: TouchCoplanar, Contacts: legacyCoplanar})`; else
`legacyGrazes` gives `touchGraze`; else `legacyPoints` gives `touchPoint`. These are the contacts the refused-pair path builds, with the same `NormalA`, `Decisive`, `FacetsB` and Slack 0, so the
retry reads an identical error.

`booleaneval` passes `Stats.Coplanar` to `StitchFacetsWith`. After the conforming passes, when it is non-nil:

1. Edge audit: every undirected edge is used by exactly two facets, in opposite directions.
2. Vertex audit: the facets around every vertex form one cycle through shared edges.

A failing edge or vertex whose exact keys are all in `Owned` returns `Legacy`. Any other failure is `ErrBooleanFailed`.

The retry reads the same refusal wherever the exact classification stops. A graze or isolated point found beside a
coplanar pair, and an expected-contact refusal of the side pass for a pair whose points the pass accepted, return the
coplanar refusal or `Legacy` the pair gives without the classification, because the retry ladder starts from it. The
audit reads only stitched vertex keys, so it needs no operand normals and no map back to kept facets.
`StitchFacetsContext(ctx, kept)` delegates to `StitchFacetsWith` with zero options and keeps its behavior.

## 6. Proximity gate obligations

`interference-design.md` §5.2 requires every coplanar deferral of a gate (`gatherNearContacts` returns `deferred` on
the first `ContactRegion`; the caller skips the face pair) to be backed by a refusal. Two gate calls exist.

**The pre-pass** (`booleaneval.evaluate` → `RefuseUndecidableProximity`):

- A face pair with summed reading `<= 0` is skipped before any gather. Both faces are exact, every coplanar
  pair between them is admitted and classified exactly, and nothing a chord could hide exists.
- A face pair with a positive sum has at least one inexact face. Every coplanar facet pair in it holds a facet of that
  face, so §3.1 refuses it with `TouchCoplanar` in the mesh pass, unless the two faces are a flush pair, whose
  admission classifies their contact on held facets that keep the true arrangement (flush-contact §4.7, F5). The
  deferral stays backed either way.

**A separated cap pair** (§19) skips the pre-pass as a whole on its unshifted attempt: its operands meet only over the
caps' common region, which has interior (§19.3).

**The occupancy re-run** (`composeOccupancy`, `internal/booleaneval/eval.go`) runs the gate again over widened face
bounds after the mesh pass. An admitted exact pair can get positive slack there, and no mesh refusal backs that
deferral. `composeOccupancy` returns nil (no occupancy record; the boolean result is unchanged) whenever
`Stats.Coplanar != nil`. This also covers `localDeepTerm`, which reads `Stats.Contacts` and so never sees admitted
coplanar contacts.

The doc comments of `FacesNearMiss`, `GatherNearContacts` and `RefuseUndecidableProximity` state this. Test G1 pins the
pre-pass and G2 the `composeOccupancy` guard (§11.1).

## 7. Planar face simplification

### 7.1 Rule

A mesh boolean result face whose held facets are exactly coplanar, hole-free, carry only zero vertex claims, and whose
B-rep corners are floats is re-triangulated on its corners alone before rounding, so it holds no artifact vertex and
rounds with no charge.

`SimplifyPlanarFaces` (`internal/meshbool/planar_simplify.go`) runs inside `StitchFacetsWith` when
`opts.SimplifyPlanar`, after the conforming passes and before `flipNeedlesHeld`, on the exact stitched mesh:

1. **Candidate faces.** A source id `s` is a candidate when all hold:
   - every facet with `src == s` lies exactly in one plane (exact orientation of every corner against the first
     facet's plane is 0);
   - every vertex of those facets has `xbeta == 0`, so every facet of `s` has `facetBound == 0`;
   - each connected region of those facets has a boundary that is one loop (no hole).
2. **Removable vertices.** A vertex `v` of candidate `s` is removable when either
   - every facet around `v` has `src == s` (face-interior), or
   - the facets around `v` carry exactly two source ids `{s, s'}`, both candidates, `v` has exactly two boundary edges
     of `s` (and so of `s'`), and `v` lies strictly between their other ends, collinear with them (exact). This covers
     two faces on one plane, where the boundary may turn at `v`.
   Every other vertex of `s` is a corner.
3. **Float corners.** `s` stays a candidate only when every corner's exact point is a float (exact `Vec()` round
   trip). Otherwise `s` is dropped, which can turn a removable vertex of a neighbour into a corner. Repeat steps 2–3
   until the candidate set stops shrinking (it only shrinks).
4. **Re-triangulate.** For each region of each surviving candidate, walk its boundary loop over corners only, orient it
   counter-clockwise in the face's dominant-axis projection (`ProjAxes` of the plane normal), and triangulate it with
   `EarClipX`. Each new facet keeps `src = s` and `facetBound = 0`.
5. **Compact.** Drop every removed vertex from `xverts`, `approx` and `xbeta`, and remap `tris`. `roundStitched`
   measures a gap for every entry of `xverts`, so an orphan would keep a positive `Round` and a stray held vertex.
6. **Audit and fall back.** Re-run the closure audit. Keep the unsimplified `xverts`, `approx`, `xbeta`, `tris`,
   `src` and `facetBound` (the flips edit them in place). If the closure audit, `flipNeedlesHeld`, or the rounding
   (`roundStitched` with its fold flips) fails on the simplified mesh with a closure, fold or embedding refusal, run the
   rest of the stitch on the unsimplified arrays. `ctx.Err()` and every other error return as they are. The choice is
   deterministic, so equal inputs give equal held meshes.

Step 2's two-face rule makes removal symmetric: an edge-interior vertex leaves both faces' loops or neither, so the
shared chain keeps the same vertices on both sides and the mesh stays watertight.

### 7.2 Why it is sound

The exact held surface before rounding is unchanged: each region of `s` is the planar polygon bounded by its loop;
face-interior vertices lie inside it and edge-interior vertices lie on straight chains of it; `EarClipX` covers the
same polygon with no new point. The exact volume integral, the symmetric-difference terms and every retained tag are
unchanged. Every vertex of `s` claimed zero before simplification (step 1), so the held polygon already was the true
patch, and dropping vertices drops no claim. Every remaining vertex is a float, so its measured weld gap is 0 and the
face's `FaceBound` is 0. Faces outside the candidate set keep their facets, vertices and bounds.

### 7.3 Effect on the cases

In R3 and R4 (§11.2) the socket's B-rep corners are exact (every vertex bound 0) and its 1.03e-15 bound comes only from
artifact vertices, so every socket face becomes exact. A flush `Union` built by §3 has the same structure, so the tail
half's faces become exact too. The exact planar arm of `Verify` reads the global `MeshBound`
(`internal/planarsnapshot/faceted.go`), which becomes 0, so that arm can decide such pairs directly as well.

## 8. Records of a flush result

With no refusal, `Evaluate` returns the first attempt:

- `ev.Shift` stays nil, so no `CodeGapShiftApplied` warning is appended and `boolchain.Expandable` admits the body.
- No piece is grown, so every source face of both operands keeps its group, retained tag, `CapRecord` and `AxialDelta`
  (`facetedrecord.SourceTag`).
- `RetainedEdgeCurve` publishes `Line3` for every rim between two retained non-parallel planes (E2, E6).
- Two coplanar faces of different operands that share an edge (flush walls of a `Union`) become one face in the
  mesh Boolean result (`agent-docs/union-face-merge-design.md`), so no rim lies between them. A face stays separate
  when neither a facet with zero bound at all three corners nor an exact retained plane tag fixes its plane.
- Vertex bounds are the measured weld gaps: zero for every float vertex, and §7 leaves no artifact vertex on a
  simplified face. A flush pair's clipped points carry §3.5's flush record.

## 9. Soundness

Claim: for every admitted pair the held result equals the exact boolean of the held operands, and the held operands
equal the true ones on every admitted facet, so a true overlap is reported by the same paths as any crossing pair and
is never hidden as contact.

1. **Held equals true on admitted facets.** §3.1 admits facets whose true piece equals the held facet: zero facet
   bound and zero face reading. Every predicate in §3 is an exact sign over those held values. A flush pair's held
   facets keep the true arrangement instead, and its points carry bounds that cover both outlines (flush-contact §7).
2. **Coincident points.** At a point of `ta ∩ tb` in a region's interior both boundaries coincide, and the material
   sides come from the exact outward normals of oriented, audited closed meshes. §3.4's table is the point-set
   definition of each op there. Skipping internal patch edges changes no cover: a region crossing such an edge lies in
   the union of the two partner facets sharing it, both with the same facing.
3. **Every other region.** A region outside every cover has an interior probe on no facet of the other operand:
   coplanar facets are excluded by the cover test; every non-coplanar facet meeting this facet was inserted as a
   crossing cut or is a coplanar-bounded edge on the facet's own boundary. Parity at that probe is exact, and a probe
   on the boundary still refuses.
4. **Overlap is never read as contact.** If the true interiors overlap, either some boundary point of one operand lies
   strictly inside the other, or the operands share an open set bounded by same-facing coincident patches. In the first
   case a region with an interior probe strictly inside the other solid exists and is classified inside, as for any crossing pair. In
   the second case §3.4 keeps one same-facing copy for `Intersect`. Either way `Intersect` is non-empty with positive
   exact volume, and the `V − E > 0` gate (`interference-design.md` §6) decides the row unchanged.
5. **Overlap just beyond zero.** An overlap of depth `d > 0` between exact faces is a non-coplanar crossing with exact
   rims, classified exactly for every `d`. With a positive bound the gate still refuses unless the crossing is proven
   deeper than the summed bound. No pair moves from refused to touching except through §3.1, flush pairs included.
6. **Bounded faces and gates.** §4 and §6 keep every refusal for faces with a positive bound that are no flush pair,
   and the occupancy re-run cannot defer an admitted pair (§6).
7. **Simplification** changes no exact surface and drops no nonzero claim (§7.2), so it cannot turn an overlap into a
   contact or the reverse.

## 10. Performance

- **Non-coplanar pairs.** Added work: one bool test on the coplanar branch, one map lookup per in-plane edge, and §7's
  per-result candidate scan (linear in result facets, one exact plane-side sign per distinct vertex against a normal
  formed once per face). §7's `EarClipX` is
  superlinear in a loop's corner count and runs only on candidate faces; the fallback's second rounding runs only when
  the first fails. `BenchmarkMeshBooleanCrossing` (`internal/meshbool/coplanar_bench_test.go`) times two crossing
  boxes through `MeshBooleanStats` and `StitchFacetsWith` with `SimplifyPlanar`.
- **Flush pairs.** Without §3 a flush `Union` under the default reach runs one exact pass, up to 6 shrink steps
  (`shiftSteps`), then up to 4 rounds (`shiftRounds`) of 6 steps × 3 directions of translation, each a full re-mesh and
  boolean. With §3 it runs one pass, plus clipping (up to 6 edge clips per admitted pair), forced `CutTriangle` on
  coplanar facets, and the manifold audit (linear, only when `Stats.Coplanar != nil`).
- **Worst case.** The legacy refusal lists stop at 64 contacts (`maxTouchContacts`), but exact classification visits
  every admitted pair, so two finely triangulated coplanar faces of `n` and `m` facets cost `O(n·m)` clips and one facet
  can collect many segments. `cutBatchSize` bounds live memory. `BenchmarkCoplanarCaps` times a `Union` of two 64-gon
  prisms stacked cap to cap (§13 D2).
- **Coplanar coverage.** A rounded axis-aligned interval reject in §3.4 can avoid exact `PointOnTri` work when a
  rounded probe lies strictly outside a triangle's rounded vertex range. Bounds come from exact vertices, and every
  nonfinite or inconclusive case reaches the exact predicate. Keep partner order and boundary refusal unchanged.
  Measure the rejected share and net consumer time before retaining the filter.

## 11. Test plan

Every test asserts computed geometry: volumes, face counts, surface kinds, bounds, diagnostics. Fixtures are boxes and
prisms with float coordinates. "Exact" means `Volume.Bound == 0` and `Exactness` exact. A is the 10×10×10 box at the
origin. B fixtures are offset 0.5 mm in x where listed, as `apitest/boolean_gap_tolerance_test.go` does, so the
analytic prism path does not take the pair.

### 11.1 Unit tests (`internal/meshbool`, hand-built `BoolMesh` from box meshes, unless noted)

| ID | Test function | Case | Expectation |
|---|---|---|---|
| U1 | `TestCoplanarClip` | inside, crossing, shared edge, shared vertex, disjoint, collinear overlap | exact pieces; pieces along `ta`'s edges dropped |
| U2 | `TestEdgeSide` | one fixture per outcome | crossing, graze, coplanar-bounded with the in-plane neighbour's index |
| U3 | `TestKeepSideCoplanarEqualFootprints` | face to face, equal footprints, three ops | kept area and exact volume integral equal hand values |
| U4 | `TestKeepSideCoplanarPartial` | partial footprint overlap | as U3; no two kept facets with the same key triple |
| U5 | `TestKeepSideCoplanarSameFacing` | pocket flush with the top | `Cut` drops both copies; `Union` keeps A's only |
| U6 | `TestKeepSideCoplanarMixedSides` | synthetic mixed facing | expected contact refusal |
| U7 | `TestKeepSideCoplanarNoSegment` | facet with coplanar partners and no surviving segment | one region, classified by cover |
| U8 | `TestKeepSideCoplanarInteriorCorner` | partner patch corner strictly inside a facet, with a partner diagonal ending there | no branch refusal; kept area equals hand value |
| U9 | `TestRoundedCoverBoxRejectStrictFiniteOnly`, `TestRoundedCoverBoxRejectFallsThroughForNonfiniteValues` | inside, edge and vertex probes, strict outside, nonfinite rounded values | only finite strict outside probes reject; all other cases retain exact coverage and errors |
| G1 | `TestCoplanarGateInexactFace` | coplanar pair where B's face has `FaceBound > 0` and its facet `FacetBound == 0` | `TouchCoplanar`; red leg: admit on `FacetBound` alone |
| G2 | `TestComposeOccupancyDropsCoplanar` (`internal/booleaneval`) | tracked `Union` with an admitted pair | no occupancy record; same held result as with tracking off |
| M1 | `TestStitchManifoldContact` | edge-only and vertex-only `Union` | returns `Legacy` (`TouchCoplanar` / `touchPoint`, contacts equal to the refused-pair refusal); red leg: no audit gives `ErrBooleanFailed` or a pinched mesh |
| M2 | `TestStitchFacetsWithZeroOptionsUnchanged` | existing crossing fixture | held mesh identical to `StitchFacetsContext` |
| S1 | `TestSimplifyPlanarTrapezoidCut` | box `Cut` by a 10° trapezoid prism | `StitchedMesh.VertexBound` all 0 on simplified faces, `Round == 0`, exact volume integral unchanged |
| S2 | `TestSimplifyPlanarNonFloatCorner` | a corner that is not a float | that face and its neighbours' shared chain untouched |
| S3 | `TestSimplifyPlanarHoledFace` | holed face | untouched |
| S4 | `TestSimplifyPlanarFallback` | forced fold on the simplified mesh | result equals the unsimplified stitch |
| S5 | `TestSimplifyPlanarFan` | face with one interior vertex with `xbeta > 0` | untouched; red leg: a corners-only check simplifies it |

### 11.2 API tests (`apitest/coplanar_contact_api_test.go`)

Each case runs in a default document and in an `ExactOnly` one, and asserts the same outcome in both unless stated.

| ID | Case | `Union` | `Intersect` | `Cut` | `Verify` |
|---|---|---|---|---|---|
| A1 | B = 10×10×10 at (0.5, 0, 10) | 2000 exact, 1 lump, no diagnostic | `BooleanEmpty`, no shift diagnostic | 1000 exact | `Sound` |
| A2 | B = 10×4×10 at (5.5, 0, 10), so B overhangs A | 1400 exact | `BooleanEmpty` | 1000 exact | `Sound` |
| A3 | edge-only: B = 10×10×10 at (10, 0.5, 10) | default: no error, 1 lump, `CodeGapShiftApplied`, volume within bound of 2000; `ExactOnly`: `BooleanUnsupportedContact` | `BooleanEmpty` | 1000 exact | `Sound` |
| A4 | vertex-only: B = 10×10×10 at (10, 10, 10) | as A3 | `BooleanEmpty` | 1000 exact | `Sound` |
| A5 | same-facing: B = 6×6×5 at (2, 2, 5), top flush | 1000 exact | 180 exact | 820, top face holed (an analytic blind pocket, so a rounding bound) | `Interfering`, one row, `Value − Bound > 0` |
| A6 | overlap just beyond zero: B = 10×10×10 at (0.5, 0, 10 − 2^-30) | exact | positive, `Value − Bound > 0` | exact | `Interfering` |
| A7 | gap just beyond zero: B at (0.5, 0, 10 + 2^-30) | 2 lumps | `BooleanEmpty` | 1000 exact | `Sound` |
| R1 | exact tenon in a slot made by two `Cut`s | exact | `BooleanEmpty` | target | `Sound` |
| R2 | exact box under a box made by `Cut`, flush walls | exact | `BooleanEmpty` | target | `Sound` |
| R3 | exact trapezoid prism in a socket made by `Cut` | exact | `BooleanEmpty` | target | `Sound` |
| R4 | trapezoid made by flush `Union` vs socket made by `Cut` | exact | `BooleanEmpty` | target | `Sound` |
| P1 | records of a flush `Union` | every face of both operands `Plane`, no warning, a later `Cut` expands through history (`boolchain.Expandable`) | | | |
| P2 | 200 seeded random box pairs on an integer grid (sizes 1–4, offsets −4–4, plus 0.5 in x for half of them) | by contact class, below | | | |

"Exact" for R1–R4 `Union` means volume equal to the sum of the operands' volumes within the sum of their bounds.

The tests are in `apitest/coplanar_contact_api_test.go`: `TestCoplanarContactFaceToFace` (A1, A2, A5, A7),
`TestCoplanarContactOverlapBeyondZero` (A6), `TestCoplanarContactEdgeAndVertexOnly` (A3, A4),
`TestCoplanarContactMadeByCut` (R1 to R4), `TestCoplanarContactKeepsRecords` (P1) and
`TestCoplanarContactRandomBoxPairs` (P2). A8 (§11.3) is in `apitest/coplanar_contact_test.go`.

P2 classifies each pair by the intersection box's extents `(ex, ey, ez)`: any negative → disjoint; all positive →
overlap; one zero, two positive → face; two zero → edge; three zero → vertex.

| Class | `Union` | `Intersect` | `Cut` | `Verify` |
|---|---|---|---|---|
| disjoint | 2 lumps, exact sum | `BooleanEmpty` | target exact | `Sound` |
| face | 1 lump, exact sum | `BooleanEmpty` | target exact | `Sound` |
| edge, vertex | A3's outcome | `BooleanEmpty` | target exact | `Sound` |
| overlap | exact inclusion–exclusion volume | exact box volume | exact difference | `Interfering` |

### 11.3 Existing tests this change flips

A test that asserts the gap retry (`BooleanUnsupportedContact`, `TouchCoplanar`, a `CodeGapShiftApplied` diagnostic, a
shifted volume bound) or a positive rounding bound needs a fixture whose contact face has a positive bound. A8,
`TestCoplanarContactRoundedFixtureKeepsRetry` (`apitest/coplanar_contact_test.go`), pins that A1's box turned by
`roundTrip` keeps the retry and `CodeGapShiftApplied`. The retry assertions of `apitest.TestBooleanGapTolerance`,
`apitest.TestVerifyGapToleranceDecidesAFlushPair`, `apitest.TestBooleanGapToleranceEdgeAndPointTouches`,
`apitest.TestBooleanGapToleranceRestingBossJoinsPlate` and `apitest.TestBooleanGapShiftKeepsTranslatedTags` use such
fixtures. A boss resting on a plain plate is a separated cap pair (§19) and takes no retry, so the boss tests rest it
on a plate that is a mesh `Union` result, whose payload states no cap side. Rules:

- Build the fixture with `roundTrip`: B turned by an `r3` rotation of 20° about the diagonal axis `(1, 1, 1)`, then by
  −20°, so its float vertices round. A turn about a cardinal axis keeps the corners exact. Before relying on a fixture,
  assert in the test that its contact face reads a positive `FaceBound` in its tessellation; if it reads zero, pick
  another rounding transform. NEVER delete a retry assertion without moving it onto such a fixture.
- A test that asserted a positive rounding bound on a face §7 simplifies asserts the exact value it reads.

### 11.4 End-to-end acceptance (consumer; not committed here)

Run, in `.tmp/repro`,
`go test -run 'TestDovetailPair|TestExp' -v -count=1 .` (its `go.mod` replaces decad with that worktree and the
consumer with `.tmp/gallery-snap`). Required for all six dovetail parameter sets: `Verify` status `Sound` with no
diagnostic; `Intersect` returns `BooleanEmpty`; `Union` succeeds with volume equal to `first + second` within the
reported bound; neither half carries `CodeGapShiftApplied`. Every `TestExp` case reads `Verify` `Sound`. Record wall
time before and after for §10. The consumer's own test that works around the refusal changes in the consumer
repository after a decad release.

## 12. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| Wide test churn: §7 turns many mesh results exact and §3 removes retries | large test diff | rules in §11.3 |
| A cover or flood case touches the other boundary unseen | wrong keep/drop | parity `onBoundary` still refuses; U3–U8, P2 |
| Branching segments at reflex patch rims | expected refusal where exact classification was possible | stated in §3.2; §15 Q3 |
| Non-manifold results at coplanar edges or points | invariant error instead of refusal | §5; M1, A3, A4 |
| Worst-case `O(n·m)` clipping on fine coplanar faces | slower booleans | `BenchmarkCoplanarCaps`; §13 D2 |
| Replay diverges from live on coplanar pairs | a re-tessellation or certificate declines on a coplanar-built body | `TestReplayCoplanarMatchesLive`; a decline is sound |
| Simplified faces change facet counts | tests or tools that count facets | counts are not public contract; tests flipped per §11.3 |
| Two faces on one plane after a flush `Union` | later face selection sees two faces | merged by the union face merge; §8, §15 Q1 |

## 13. Decisions for the user

- **D1. Scope of planar simplification (§7).** Decided: option A, every mesh boolean result. It gains exact faces
  wherever a face's corners are floats, which later booleans and `Verify` need; it costs test churn and a per-result
  candidate scan. Option B, only results of a boolean that admitted a coplanar pair, would give smaller churn, but the
  socket's `Cut` admits no coplanar pair, so C5 and C7 would fail.
- **D2. Clipping cap (§10).** Decided: option A, no cap. Every exact coplanar pair is classified, at `O(n·m)` on fine
  coplanar faces. Option B, refusing with `TouchCoplanar` above a cap of admitted pairs per pass, would bound time but
  send large coplanar faces to the retry. The code has no cap.

## 14. Tasks

Every task below has shipped. Each entry names the code and tests that implement it. The switch that gated the work
(`internal/coplanarcontact`) is removed: §3, §5 and §7 run on every mesh boolean.

### Task 1: Land the design doc

This document and its row in `agent-docs/layout/design-docs.md`.

### Task 2: Switch and exact-facet flags

- `BoolMesh.Exact` and `(*BoolMesh).SetExactFacets(m *topo.Mesh) error` (`internal/meshbool/face_exact.go`) set
  `Exact[t] = FacetBound[t] == 0 && FaceGateDelta(...) <= 0`. A facet whose face states no bound is
  `ErrBooleanFailed`, as in `facesOfMesh`.
- `FaceGateDelta(m *topo.Mesh, surface map[*topo.Face]float64, f *topo.Face) (float64, bool)` is the helper
  `facesOfMesh` calls; `false` means no stated bound.
- `booleaneval.evaluate` calls `SetExactFacets` on both operands, on the live and the replay path, when `!hasShifts`
  and no fixed operand is set. Otherwise `Exact` stays nil, and nil reads as all false.
- Tests: `TestSetExactFacets` (`internal/meshbool/face_exact_test.go`).

### Task 3: Coplanar flag and exact clip

- `TriContact.Coplanar` and `TriContactSpan.Coplanar` are true only from the coplanar branch of `triTriSpan`
  (`internal/meshbool/triangle_contact.go`).
- `CoplanarClip(edge [2]proof.Xpt, xta [3]proof.Xpt, n proof.Xpt) (ClipPiece, bool, error)` with
  `type ClipPiece struct { P0, P1 proof.Xpt; P0OnA, P1OnA bool }` clips one edge to closed `ta` in exact rationals
  (`internal/meshbool/coplanar_clip.go`). It returns `false` when the piece is empty, a point, or lies along an edge of
  `ta`.
- Tests: U1 and `TestTriContactCoplanarFlag` (`coplanar_clip_test.go`).

### Task 4: Pair pass admission

- §3.2 and §3.3 in `internal/meshbool/boolean_pair_cuts.go`. `EdgeSide(m *BoolMesh, f, k int, partner [3]r3.Vec)
  (key [2]int, side EdgeOutcome, inPlane int, err error)` and the constants `EdgeCrossing`, `EdgeGraze`,
  `EdgeCoplanarBounded` are in `internal/meshbool/boolean_mesh.go`.
- `pairCutsResult` carries `CoplanarA`, `CoplanarB map[int][]int`, `Owned map[string]struct{}` and `Legacy error` (§5).
- Tests: U2, `TestPairCutsCoplanarAdmitted`, `TestPairCutsCoplanarRefusedWhenInexact`
  (`boolean_pair_cuts_coplanar_test.go`).

### Task 5: Side pass cover rule and gate contract

- §3.4 in `internal/meshbool/coplanar_cover.go` and `boolean_mesh.go`: `keepSideWith` and
  `keepOptions{Coplanar map[int][]int; Op OperationKind; Second bool}`; `keepSide` delegates with zero options.
  `meshBooleanStats` fills `Stats.Coplanar` (`ContactKeys`, §5). `coplanarKeep(op OperationKind, second, sameFacing
  bool) bool` and `coverOf(...)` are in `coplanar_cover.go`.
- The three gate doc comments state §6.
- Tests: U3 to U8 (`keep_side_coplanar_test.go`), G1 (`tangency_gate_coplanar_test.go`).

### Task 6: Manifold audit and occupancy guard

- §5 in `internal/meshbool/stitch_manifold.go`: `type StitchOptions struct { Coplanar *ContactKeys; SimplifyPlanar bool }`
  and `StitchFacetsWith(ctx, kept []KeptFacet, opts StitchOptions) (*StitchedMesh, error)`; `StitchFacetsContext`
  delegates with `StitchOptions{}`. `booleaneval` passes `stats.Coplanar` on the live path.
- §6: `composeOccupancy` takes a `coplanar` flag and returns nil when it is set (`internal/booleaneval/eval.go`).
- Tests: M1, M2 (`stitch_manifold_test.go`), G2 (`internal/booleaneval/occupancy_coplanar_test.go`).

### Task 7: Planar face simplification

- §7 in `internal/meshbool/planar_simplify.go`: `SimplifyPlanarFaces(ctx context.Context, in PlanarMesh) (PlanarMesh,
  bool, error)` with `type PlanarMesh struct { Xverts []proof.Xpt; Approx []r3.Vec; Xbeta []float64; Tris [][3]int;
  Src []int; FacetBound []float64 }`. It returns a new compacted mesh and whether it changed anything, and never
  modifies its input.
- `StitchFacetsWith` calls it when `opts.SimplifyPlanar` and applies §7.1 step 6's fallback. `booleaneval` sets
  `SimplifyPlanar: true` on the live path.
- Tests: S1 to S5 (`planar_simplify_test.go`).

### Task 8: Benchmarks

`BenchmarkCoplanarCaps` and `BenchmarkMeshBooleanCrossing` in `internal/meshbool/coplanar_bench_test.go` (§10).

### Task 9: Planar simplification on the replay stitch

`StitchFacetsReplayWithKeys` (`internal/meshbool/brake_stitch_replay.go`) runs `SimplifyPlanarFacesBudget` and the
same fallback, charging the replay budget per facet and vertex visited and per `EarClipX` step. Live and replay give
the same held mesh for every boolean both build. Test: `TestReplaySimplifyMatchesLive`
(`replay_simplify_test.go`).

### Task 10: Turn it on

The switch is removed, so §3, §5 and §7 always run on the live path and §7 on the replay stitch. A8 pins that a rounded
fixture keeps the retry (§11.3). The documents that cited the coplanar refusal (`interference-design.md`,
`evaluator-design.md`, `gap-tolerance-design.md`, `faceted-vertex-bounds-design.md`, `missing-features.md`) state the
current behavior.

### Task 11: API coverage

A1 to A7, R1 to R4, P1 and P2 (§11.2) as the `TestCoplanarContact*` functions in
`apitest/coplanar_contact_api_test.go`.

### Task 12: Bounded replay parity for coplanar contacts

§3 and §5 run in the bounded collector, cutter, conform and stitch with the surrounding replay code's budget charges
(`internal/meshbool/pair_cuts.go`: `boundedPairCuts`, `PairCutsBudget`; `contact_replay_coplanar.go`; the `brake_*`
replay files). A replayed coplanar boolean gives the same held mesh as the live one. Tests in
`internal/meshbool/replay_coplanar_test.go`: `TestReplayCoplanarMatchesLive`, `TestReplayCoplanarBudget`,
`TestReplayCoplanarDeclinesWhatLiveClassifies`, `TestReplayCoplanarLegacyRefusal`.

## 15. Open questions

- **Q1.** Resolved: the mesh Boolean merges two coplanar faces of different operands into one face
  (`agent-docs/union-face-merge-design.md`).
- **Q2.** Should §7 handle holed faces? Decide between leaving them untouched (current design) and an exact hole bridge
  before `EarClipX`, because a plate with a through hole cut by a mesh boolean stays inexact until then.
- **Q3.** Should the branch refusal of §3.2 ("Remaining refusals") be lifted with a planar-arrangement cutter? Decide
  when P2 or the acceptance run (§11.4) finds a real case that hits it, because the cutter is a separate, larger change
  to `boolean_cut.go`.

## 16. Seats in an Intersect

A bolt head or nut that rests flush on a bored pocket floor meets the floor in a plane, but neither face is exact
(§3.1): the floor's rim round the bore is chorded, and so is the bearing face's rim round its own hole or chamfer.
§3.1 then refuses the pair, and the rim edges that touch the other face's plane reach the held-mesh gates. Hardware
interference checks run `Intersect` on exactly these pairs. This section admits them for `Intersect` only.

Code: `internal/meshbool/seat.go`; callers in `boolean_pair_cuts.go`, `tangency_gate.go` and
`internal/booleaneval/eval.go` (an unshifted `Intersect` with no replay or fixed operand calls `SetSeats`).

### 16.1 Carrier-exact facets and seat pairs

A facet is **carrier-exact** when its true piece lies in the facet's own plane, whatever its outline: its slab bound is
zero (a retained plane whose tag is the facet's plane, `SetChargedSlabBounds`), or its face is an analytic `Plane` of an
unplaced body with no retained tag, no stated departure and zero displacement, whose plane slab (`planeSlab`) at the
facet is zero, or §17 reads the face's exact true plane through its corners. A placed analytic body denotes its record
carried through the placement read as exact rationals, which its float tag only approximates, so the analytic reading
applies to an unplaced body only (§17.2).

A **seat pair** is two coplanar facets, one of each operand, both carrier-exact and facing opposite ways
(`proof.XdotSign` of their normals is negative). §3's pass admits a seat pair as it admits an exact pair.

**Claim S1.** In an `Intersect`, the cover table's answer for a seat pair is right even though the outlines are chorded.

Both true pieces lie in the common plane `P`, one operand's material on each side near them. Opposite-facing pairs
drop both copies (§3.3's table), and so does every point of either outline's neighbourhood that the pass may misplace
by a chord: near `P` the operands lie on opposite sides, so no point near the outlines is inside both. The `Intersect`
result has no surface there whatever the outlines are. In a `Union` or `Cut` the kept copy's outline is the result's
boundary, so a chorded outline would make the result's face wrong; seats are not read there.

### 16.2 Walls that leave a seat plane

A **wall** `W` is a face beside a carrier-exact face `F` of its own operand, or all the faces of one operand that share
one `Cylinder` or `Cone` tag (the two arcs of a drawn bore). Its **rim** is the set of edges it shares with `F`.
`W` earns a seat on `F`'s plane `P`, and its rim edges are marked, when:

1. every corner of `F` lies in `P` exactly and every facet of `F` is carrier-exact;
2. every facet of `W` has a vertex off the rim, and every such vertex lies strictly on `F`'s material side of `P`
   (`proof.OrientSign` negative);
3. either every facet of `W` is exact (§3.1), or all of the following hold:
   - the rim edges form closed loops;
   - every ruling of `W`'s true surface meets `P` at a sine above `seatSinMin = 10⁻³` (below);
   - every vertex off the rim lies farther from `P` than `W`'s largest facet bound plus 64 ulps of the coordinate scale.

**Claim S2.** Then `W`'s true piece lies on `F`'s material side of `P` and meets `P` only along the rim.

- *Exact wall.* `W` is its facets. Each facet is the convex hull of its corners, which lie in `P` (rim) or strictly
  below it, and no facet has all three on the rim, so every point lies below `P` except on rim edges.
- *Held wall.* Let `C` be the curve where `W`'s true surface meets `P`. Every ruling meets `P` transversally, so it
  crosses `P` once, and `C` is one closed curve on a cylinder (or on the cone's nappe). The true rim lies in `P`,
  since it is the boundary `W` shares with `F`, whose true piece lies in `P`. The held rim is closed, so the true rim
  is a closed curve inside `C`, which is all of `C`. `W`'s true interior is connected and misses `C`, which is its
  boundary, so it lies on one side of `P`. A vertex off the rim lies below `P` by more than its facet's bound, so the
  true point near it is below `P` too, and the whole true interior is.

An open rim fails the argument. A side edge where the true face meets another wall may rise above `P` by less than
the facet bound and still round to held vertices below it.

**Transversal rulings.** A face that records the surface its revolve denotes (`topo.Face.Denoted`) is read from that
record (`denotedTransversal`). A revolve cone's tag is a float re-expression with no stated normal bound (its
`NormalBound` is 2, retained-modify-reach §9.5), so the record's rational leaves are its only exact reading. For a
meridian segment from `(z₀, ρ₀)` to `(z₁, ρ₁)` a ruling runs along `d(φ) = Δz·W + Δρ·(cos φ·E₀ + sin φ·E₁)`,
with the axis `W` and basis `E₀`, `E₁` known only inside their intervals. With `N` the exact cross product of the facet's
edges,

    |d·N| ≥ |Δz|·|W·N| − |Δρ|·(|E₀·N| + |E₁·N|),   |d|·|N| ≤ (|Δz|·|W|₁ + |Δρ|·(|E₀|₁ + |E₁|₁))·|N|₁,

each absolute value taken at its least or greatest over the intervals, in exact rationals. The ruling's sine against
`P` is `|d·N| / (|d|·|N|)`; the lower bound over the upper one must exceed `seatSinMin`. A circular meridian (a sphere
or torus) has no rulings and refuses. A face without a record is read from its `Cylinder` or `Cone` tag
(`transversal`), whose unit normal departs from the true one by at most `NormalBound` `d < 2`, an angle of at most
`2·asin(d/2)`; the margin `seatSinMin` dwarfs the float trigonometry's rounding, padded by a relative `10⁻⁹`.

### 16.3 What the seat decides

**Claim S3.** For a carrier-exact face `X` of the other operand lying exactly in `P` and facing opposite `F`, the true
surfaces of `X` and `W` meet only on `W`'s rim, and no point near that contact lies inside both operands.

By S2, `W` lies on `F`'s side of `P` and touches `P` only on its rim; `X` lies in `P` with its operand on the far side.
Near the rim, `W`'s operand lies on `F`'s side (it is bounded there by `F` and `W`) and `X`'s operand on the other.

The pass reads this in three places:

- **Proximity gate.** `RefuseUndecidableProximity` skips the face pair (`seatSkip`): every facet of `X` is
  carrier-exact, lies on the seat plane exactly and faces opposite the seat facet. S3 leaves nothing for a chord to
  hide.
- **Held-mesh gate.** A contact along a marked rim edge, or a single-point contact at a vertex of a marked rim edge
  (`rimPointOnCarrier`), against a carrier-exact facet does not enter the touched lists the chain-depth gate
  (`RefuseCoarseHeldContact`) reads. By S3 the contact lies in `P` and adds no volume. A wall held coarser than the
  pair's chord tolerance then meets the seat without tripping the gate; every other contact of its facets still
  enters the lists.
- **In-plane edges.** An `EdgeCoplanarBounded` rim edge (§3.3) is accepted when the facet across it is admitted as
  coplanar with the facet it lies in: a seat pair. A point contact may be owned by the coplanar pass when both facets
  are exact, or one is carrier-exact and the other carrier-exact or on a marked rim (`seatPoint`).

### 16.4 Rejected variants and tests

`apitest/intersect_seat_test.go` (`TestIntersectSeat`) and `internal/meshbool/seat_internal_test.go`
(`TestDenotedTransversal`):

| Case | Expect | Guards |
|---|---|---|
| a bored block on a bored pocket floor, both orders | `BooleanEmpty` | S1–S3 |
| a bored round nut on the floor, both orders | `BooleanEmpty` | S2 for a held cylinder wall |
| the block sunk 0.05 into the floor | the overlap's volume | seats admit no false empty |
| the block flush with a solid plate's top, same facing | the block less the bore | same-facing pairs are no seat |
| a seated `Union` | `ErrUnsupported` | S1 holds for `Intersect` only; fails red if `SetSeats` runs for every op |
| both bodies turned by 31° | `ErrUnsupported` | planes with a stated departure are not carrier-exact |
| a meridian in the plane, or flatter than `seatSinMin`, or a plane across the axis | refuse | transversality |

Kept refusals:

- **Turned or placed planes.** A plane whose tag states a departure, however small (a placed body's planes carry the
  rounding of the moved frame, often 10⁻¹⁵), is not carrier-exact. Its true piece lies only within the departure of
  the held plane, so two such faces may be apart or overlap by that much. Admitting them needs the two true planes
  compared exactly from their records, which this section does not do.
- **Open rims** of held walls (S2).
- **Shifted operands, replays and fixed operands** read no seats.

## 17. Placed seats

Hardware and parts in an assembly are placed bodies, and a placement rounds their tags: a placed retained plane states
a departure (about 10⁻¹⁵ in the gallery's clamp arm), so §16.1 reads none of its facets as carrier-exact. Yet a turn
about a plane's normal, or any placement whose matrix row along that normal is exact, carries the plane onto its float
image exactly. This section reads a placed face's true plane exactly when the record allows it, and §16 then applies
unchanged.

### 17.1 Faceted records

A faceted payload holds its mesh in its current frame, and its true solid is the earlier one carried through each
placement's delta read as exact rationals (`facetedrecord.Payload.Placed`). Only the float tags survive a placement, so
the claim is recorded when the placement happens.

- **Claim.** `Group.PlacedPlane` (and `topo.Face.PlacedPlane`, copied in `AssembleBody`) names a retained `Plane` tag
  whose point set is exactly the true plane. `placedPlane` sets it when the group's tag before the placement was exact
  (retained, not denoted, no stated departure and zero displacement, or an earlier claim still equal to the tag) and
  `PlaneImageIsTag` proves the moved float tag is, as a point set, the exact image: its normal `U′×V′` is parallel to
  `(B·U)×(B·V)` and its origin lies in the image plane through `B·O + t`, all in exact rationals. The frames may differ;
  only the plane is claimed.
- **Why it holds.** The tag before the placement held the true piece. The true piece after it is that piece's exact
  image, which lies in the exact image plane, and the claim proves that plane equals the moved tag.
- **Voiding.** Every placement that moves the record clears the claim before it proves it again, since a move too
  small to change the tag's floats still moves the true plane. Every other path that moves a tag leaves a claim naming
  the old tag, and a reader (`trueFacePlane`) accepts the claim only while the face's tag equals it. A new record that
  `SourceTag` restates from a face starts without a claim.
- **Denoted revolve planes.** A plane face with a revolve record (`topo.Face.Denoted`) whose meridian segments all run
  at one exact level `z₀` lies in the plane through `O + z₀·W` spanned by `E₀` and `E₁`, all exact point leaves of the
  record (`denotedPlane`). A placement transforms the record exactly (`Revolved.Transformed`), so this holds for a
  placed bolt's bearing face whatever its tag states.

A facet is carrier-exact when its three held corners lie in the face's true plane, decided in exact rationals
(`exactPlane.holds`).

### 17.2 Placed analytic bodies

A body built by an analytic payload denotes its record at the identity placement carried through `Payload.Transform`
read as exact rationals. `placedCarriers` (`internal/booleaneval/placed_seat.go`) rebuilds the record at the identity,
pairs its faces with the placed body's by position (as `placedOperandDepartures` does), reads each paired plane face
whose identity tag states no departure, no displacement and no denoted patch as an exact plane `P₀`, and maps it
exactly: the image passes through `L·o + t` with normal `cof(L)·n`, the direction of `L⁻ᵀ·n`. The held corners must lie
in it exactly. Faceted payloads use §17.1 instead.

A placed analytic body's float tag is not its true plane. A plate moved by `10⁻³⁰` along its normal keeps every float,
so its held floor and tag are unchanged, but its true floor is `10⁻³⁰` higher: a block resting on the held floor
overlaps it, and reading the tag as exact would make `Intersect` return `BooleanEmpty`. The analytic reading of §16.1
therefore applies to unplaced bodies only, and the moved plate refuses.

### 17.3 Why the hardware-first order passed

Some gallery hardware pairs refuse with the part as the first operand and prove empty with the hardware first. Both
orders refuse on the first attempt. A document's gap tolerance is 0.01 mm unless set otherwise
(`WithGapTolerance`, api-design.md), and the refusal is a touch contact, so `Evaluate` retried with the second operand moved by up to that tolerance
(gap-tolerance-design.md §5.2). `shiftDirections` looks for one direction that opens every contact, read along operand
A's facet normal at each contact. With the hardware first, the contacts carry the bolt's or nut's normals: its bearing
face and chamfer cone all lean the same way along the axis, so moving the part along the axis opens them, and the moved
part proves empty. With the part first, the contacts carry the part's normals, among them the bore wall's radial normals
round its whole rim, which no single translation opens; the retry tries nothing and the refusal stands.

So the hardware-first answers were the gap tolerance's, not an exact proof, and the retry is order-dependent: one
contact reads as openable or not depending on which operand's normal it records. With §17 both orders decide those pairs
on the first attempt. The order dependence of the retry itself is left as it is: it only ever answers within the
document's stated tolerance.

### 17.4 Kept refusals

- **Placements whose image plane holds no float.** A turn whose matrix row along the plane's normal is inexact tilts the
  true plane off every held float. The gallery's adapter and stud pairs share a basis with an off-axis entry of
  `5.55×10⁻¹⁷`, and the raised nut J4 pair a basis vector of length `1 + 2⁻⁵²`. Both operands' true planes may still
  coincide, but no held corner lies in them, so no exact predicate on the held facets decides the contact.
- **Held planar walls.** §16.2 admits a planar wall only when it is exact. A placed planar wall is held within a bound,
  and its rim is an open segment, which S2's closed-rim argument does not cover.

### 17.5 The J2 bolt and swivel

Bolt J2 refuses against the swivel in both orders on the chain-depth gate without the point rule below. The swivel's J2 hole
is cut while the operands are swivel-sized, so its wall is held to about 0.0105 mm, while the bolt and swivel pair's
chord tolerance is 0.0019 mm. The wall earns a seat on the pocket floor (§16.2), but its long facets fan from the rim,
and many touch the bolt's bearing face only at one rim vertex. Those single-point contacts carried no rim edge, so they
entered the touched lists and the gate refused the coarse wall. A point at a rim vertex is a rim contact by S3, so
`rimPointOnCarrier` leaves it out (§16.3), and the pair proves empty in both orders. `TestRimPointOnCarrier` covers
the predicate, including a point off the rim, a carrier that is not exact and a segment contact, which it rejects.

### 17.6 Tests

| Test | Case | Expect |
|---|---|---|
| `TestIntersectPlacedSeat` | a round nut and the plate, analytic, turned about the bore's axis by 70° and 25°, both orders | `BooleanEmpty` (fails red without `placedCarriers`) |
| `TestIntersectPlacedSeat` | the same bodies built on the mesh path (faceted records) | `BooleanEmpty` (fails red without `placedPlane`) |
| `TestIntersectPlacedSeat` | the plate moved `10⁻³⁰` along its normal, both orders | `ErrUnsupported` |
| `TestIntersectPlacedSeat` | both turned 31° about a horizontal axis | `ErrUnsupported` |
| `TestPlacedPlane` | a turn about the normal, twice | a claim |
| `TestPlacedPlane` | a tilting turn, a sub-resolution move, a stated departure, a stale claim, a displacement, an unretained tag | no claim |
| `TestTrueFacePlane` | a proved claim, a level revolve record | a plane |
| `TestTrueFacePlane` | a stale claim, a climbing meridian | none |
| `TestRatPlaneImage` | the cofactor image against three mapped points | exact |

## 18. Flush planes that agree only within a bound

After §17 the gallery's clamp arm still refused some seats with the part first: the adapter nut and the adapter, and the
hardware of the tilted poses (pan, shoulder-max, elbow-min, and the raised nut J4 and knob J4). This section records why
no exact rule admits them, and what decides them instead.

### 18.1 What the true planes are

- **Translation sums.** The adapter nut is placed at the float sum `t = I + 4.7`, which rounds to
  `110.68069729522874`, while the adapter's pocket floor is its local plane `z = 4.7` carried by `I`, the exact sum
  `I + 4.7`. The float sum is `2.66×10⁻¹⁵` above the exact one. Both held faces are exactly at the float value, so the
  held facets coincide; the nut's true bottom is its held one, and the adapter's true floor lies `2.66×10⁻¹⁵` below its
  held floor, into the adapter. The true bodies stand apart by that gap, so the empty answer is true.
- **Tilted poses.** A 40° turn has no exact matrix. Each body's true plane is the exact image of its local plane under
  its own float matrix read as exact rationals, and two bodies whose matrices differ in a last place have true planes
  that cross at an angle near `10⁻¹⁶`. Across a face several millimetres wide they then overlap on one side of the
  crossing line by up to about `10⁻¹⁵` mm: the exact Intersect is a sliver, not empty.

### 18.2 Why a bounded rule is not the route

A rule that admits planes agreeing within a bound `ε` answers "empty" where the true bodies may overlap by `ε`, as the
tilted poses do. That is a tolerance answer, not an exact one, and decad already has one with a stated reach: the gap
tolerance's shifted retry (gap-tolerance-design.md §5.2), 0.01 mm unless set otherwise, whose empty results Verify
reads as shifted (`AppliedShift`). Its hardware-first answers were already empty for every one of these pairs (§17.3);
only its order dependence and one decline kept the part-first order refused. Fixing those costs two small changes and
no new proof, against a new bounded-coplanarity rule that the mesh pass would also have to follow when it cuts facets
that are not exactly coplanar. So the route taken is the retry:

- **Contact direction** (gap-tolerance §5.2 item 8). In an Intersect, a contact between a curved face of the first
  operand and a flat face of the second reads along the flat face's normal. A part's bore wall fanned its normals round
  the rim, and no move opened them all.
- **Swapped retry** (gap-tolerance §5.2 item 9). A second operand whose held mesh folds under any move declines every
  shift. The adapter nut is one: its slivers flip under rounding. An Intersect the retry still refuses is retried with
  the operands swapped, and only an empty result is taken.

The translation-sum case could be proved exactly: a placement could record the true plane's exact offset from the held
one, and §16's seat would admit a pair whose offsets each recede into their own body. It is not built, because the
retry decides the pair and the exact proof would serve only the translation-only placements.

### 18.3 Kept refusal

The J2 bolt and the swivel refuse in both orders when the arm pans. The pan turns the swivel about the vertical axis,
so its pocket floor's true plane holds no float and its bore wall earns no seat (§17.4). The wall's rim contacts then
reach the chain-depth gate, and the wall is held to 0.0105 mm, coarser than the pair's 0.0019 mm chord tolerance. The
gap tolerance's retry cannot help: a move clears a coarse facet only past the facet's own bound, and 0.0105 mm is beyond
the 0.01 mm reach. Holding the swivel's bore finer where it is built would decide it.

### 18.4 Tests

| Test | Case | Expect |
|---|---|---|
| `TestIntersectSeatWithinGapTolerance` | a bored block and a round nut on the floor, both turned 31° about a horizontal axis, both orders | `BooleanEmpty` (fails red without §5.2 item 8, and for the block without item 9) |
| `TestIntersectSeatWithinGapTolerance` | the turned block sunk 0.05, deeper than the reach, both orders | its overlap's volume |

`TestIntersectSeat` and `TestIntersectPlacedSeat` run in an `ExactOnly` document, so they keep testing the exact
path alone.

## 19. Separated cap pairs

Two bodies that meet across one planar cap each, such as a lofted frustum and a revolved cone on one circle or a boss
standing on a plate, lie on opposite sides of the caps' plane. Their caps' outlines are chorded, so §3.1 reads the caps
inexact, and flush admission (`agent-docs/flush-contact-design.md` §4.4) refuses them wherever one outline comes within
its reach of the other, as two chordings of one circle do all round it. Without this section the proximity gate then
refuses the walls that leave the rims: a chorded loft wall facet comes within the summed face bounds of the other
body's cap and wall and proves no crossing deeper than that. The shifted retry cannot open those contacts for an
oblique frustum: its wall normals round the rim run from +0.9 to about −0.6 along the axis, so no one translation within
the reach clears them all. This section decides such a pair on the unshifted attempt, with no shift.

Code: `internal/meshbool/separated_caps.go`, `internal/flushface/deep.go`, `internal/booleaneval/cap_sides.go`; the
callers in `internal/booleaneval/eval.go`, `internal/meshbool/flush_faces.go`, `boolean_pair_cuts.go` and
`boolean_mesh.go`.

### 19.1 The rule

Operands A and B form a **separated cap pair** on an exact plane P when all hold:

1. A states a cap side (§19.2) whose face X lies in P, and B one whose face Y lies in P with the opposite outward
   normal. So A lies in one closed half-space of P and meets P only in the closure of X, and B in the other one and Y.
2. Every held corner of X and of Y lies in P exactly, and the first facet of each faces out of its own operand.
3. Every held vertex of A lies in A's closed half-space, no facet of A outside X lies in P, and a facet of A outside X
   that touches P does so at an edge or a vertex of X's facets. The same holds for B and Y.
4. Some point of P lies inside both held faces, farther from each held outline than that face's outline reach (§19.4).

All signs are exact rationals over held floats and recorded leaves. `SeparatedOperands` looks for such a pair among the
two operands' cap sides; `separatedPair` decides one face pair, memoised.

### 19.2 Cap sides

A `meshbool.CapSide` is a payload's claim: its true solid lies in the closed half-space `{x : N·(x − O) ≤ 0}`, the
true face `Face` lies in the plane `N·(x − O) = 0`, and the solid meets that plane only in the closure of `Face`.
`booleaneval.capSides` reads three payloads and the sides a mesh `Union` recorded on its result (§20.1); every other
payload states none.

| Payload | Read when | Why the claim holds |
|---|---|---|
| straight prism (`prismrecord.Payload`) | not a sheet; exactly two `CapRecord` faces whose record planes (`capRecordPlane`) are exact, parallel and distinct (`SlabCapSides`) | the walls run along the frame normal from one cap level to the other |
| plain loft (`loftrecord.Payload`) | not a sheet; no fillet connector, bore, construction or join displacement, or join gap; the same two caps | the walls are ruled between the two sections, and a segment from one of two parallel planes to the other lies between them and meets each only at its end |
| revolve (`revolveplan.Payload`) | not a sheet; zero `SectionDelta`; every face but a partial sweep's end cap records straight meridian segments with exact ends on one exact axis frame (`RevolveCapSides`) | see below |

A revolve record places meridian point `(z, ρ)` at `O + B₀·ρ·cos φ + B₁·ρ·sin φ + B₂·z`. With `n = B₀ × B₁`,
`n·(x − O − z₀·B₂) = (n·B₂)·(z − z₀)`, so the solid lies between the planes of its lowest and highest meridian levels,
which straight segments reach at their ends. A level gives a side only when exactly one face lies wholly at it and
every other meridian end at that level has a radius that face's ends have; otherwise the solid meets the plane in a
circle beside the cap. Arcs reach their extreme levels between their ends, so a meridian with an arc gives no side.

### 19.3 What the pair decides

**Claim P1.** Under §19.1 items 1 and 4, the true solids meet exactly in `cl(X) ∩ cl(Y)`, and that region holds a disc
of positive radius.

A lies in its closed half-space and B in the other, whose intersection is P, so `A ∩ B ⊆ (A ∩ P) ∩ (B ∩ P)`, which is
`cl(X) ∩ cl(Y)` by item 1. Neither interior reaches P, so the interiors are disjoint. §19.4 gives the disc.

The pair is read in three places:

- **Proximity gate.** `booleaneval.evaluate` skips `RefuseUndecidableProximity` for the attempt. The gate asks whether
  two true surfaces touch or cross where only the chords could tell. By P1 they never cross and touch over the caps'
  common region, which has interior, so no answer depends on where a chord falls. Item 3 puts the held meshes in the
  same half-spaces, so every held contact lies in P, and the mesh pass classifies it exactly or refuses.
- **Coplanar pass.** `flushPair` answers true for the facets of X and Y, so §3.2 admits their coplanar pairs and §3.4's
  table keeps or drops each region. The table is the point-set definition of each operation for the held operands,
  exact or not; §19.5 charges how far the held result lies from the true one.
- **Occupancy.** The attempt tracks no occupancy record (`composeOccupancy` would re-run the gate it skipped).

### 19.4 Overlap with a margin

`flushface.Deep` accepts a point p of P for face F with reach `D(F)`, the largest outline vertex radius `ρ` of
`agent-docs/flush-contact-design.md` §4.3, when p's squared distance from every held outline edge exceeds `D(F)²` and a
ray from p crosses the outline an odd number of times. Both read the plane that drops the normal's largest axis, which
shortens no distance, in exact rationals; a float distance below the margin rejects first. The candidates are the mean
of each face's outline corners, their midpoint, and up to 64 facet centroids of each face.

**Claim P2.** Such a p lies inside the true face, together with the disc of radius `d − D(F)` about it, `d` its
distance from the held outline.

By the end correspondence of `agent-docs/flush-contact-design.md` §4.2, every held outline vertex lies within its `ρ` of
its true point, and the true outline arc between two such points lies within the larger radius of the chord. Each
chord, its arc and the two short segments to the true points lie in the capsule of radius `D(F)` about the chord, a
convex set, so the held and the true outline are homotopic inside the union of those capsules. p lies outside that
union, so both outlines wind about p the same number of times, and p lies inside the held face.

A tangent contact, two caps that touch at a point or along an edge or overlap by less than their reaches, has no such
point. Such a pair keeps the gate's refusal and the retry, as before this section.

### 19.5 Bounds of the result

Let R be the true `Union`. A held point x of P that a face of the pair keeps lies within `max(D(X), D(Y))` of R's
boundary:

- x in X but not in Y, both true: x is on R's boundary.
- x outside true X: x lies between a held chord of X and its arc, within `D(X)` of the true outline of X, whose points
  are on R's boundary or are limits of points of P that are.
- x inside true Y but outside held Y: likewise within `D(Y)` of the true outline of Y.

So a kept point needs the reach only inside the strip of half-width `max(D(X), D(Y))` about the two held outlines
(`separatedBand`). The pass records each admitted facet's band (`addBand`); the side pass lifts the corners of a kept
piece near the band to `reach = D(X) + D(Y)`, at the corners near it, or at all three when only the piece's interior
comes near (`separatedBand.raise`). The piece's facet bound is its largest corner bound, so every point of it is
charged, while a corner far from both outlines keeps its own bound for the facets that share it. A contact segment
whose ends both lie in P is a touch of the two outlines, not a crossing, so its rim bound is the reach, not the slab
tube `(δA + δB)/sin θ` of `agent-docs/faceted-vertex-bounds-design.md` §3.2, which reads the walls as crossing where
they only meet in P. The slider's frustum and cone junction holds about 0.0059 mm instead of 0.015 to 0.020 mm, below
the next `Union`'s chord tolerance.

### 19.6 Edges and points in the plane

With the pair found, `pairCuts` accepts an edge graze whose two edges lie in P (`separatedEdge`, `acceptSeparated`) and
owns a point contact in P (`separatedPoint`). Every facet off P lies strictly on its own operand's side except its
boundary, so a probe inside it is outside the other operand, and exact parity classifies it without the graze's side
test. The manifold audit (§5) still refuses a result that meets itself along an edge or at a point.

### 19.7 The retry

Cap sides reach only the unshifted live attempt: `AttachCapSides` runs beside `AttachFlushFaces`, which no shifted,
grown, fixed-tool or replayed attempt calls, and `SeparatedOperands` runs only on that attempt. `separatedEdge` and
`separatedPoint` read only the plane that call found. Each claim is about the operands as given.

A moved operand no longer lies in its half-space: a `Union` sinks B into A by up to the reach. The retry's attempts
carry no cap side, so the gate, the coplanar admission, the in-plane rules and the rim bounds run on them exactly as
without this section, and no moved operand is judged by a separation it lost. When the certified attempt succeeds, no
retry runs and the result states `A∘B`, with no `CodeGapShiftApplied`. When it refuses for another reason (§19.8), the
retry runs from that refusal as it did before, under the full gate.

### 19.8 Kept refusals

| Case | Why |
|---|---|
| Caps that touch, or overlap by less than their reaches | no deep point (§19.4) |
| A revolve with an arc or a section displacement, a filleted, bored or joined loft, a sheet | no cap side (§19.2) |
| A mesh `Boolean` result | no cap side of its own, unless its `Union` carried one (§20.1) or it reads a local side (§20.2) |
| A placed body whose cap plane holds no held corner exactly | §19.1 item 2 |
| A meridian or held wall that touches the plane beside the cap | §19.2, §19.1 item 3 |
| An extruded circle and a revolve of the same radius | near-coincident chords (below) |
| The bounded replay | it attaches no flush faces (flush-contact §2), so it refuses the inexact caps as before |

An extruded circle and a revolve of the same radius chord the circle at the same angles, but their vertices differ by
an ulp. The coplanar pass then leaves slivers narrower than the float grid between each pair of near-coincident chords,
and the stitch's rounding folds them (`HeldFoldError`). A default document takes the growth retry
(`agent-docs/gap-tolerance-design.md` §5.2 item 7); `ExactOnly` refuses.

### 19.9 Tests

Root tests in `union_shared_cap_test.go`. Each `TestUnionSharedCap*` test that succeeds asserts one lump, no
`CodeGapShiftApplied`, a volume equal to the operands' sum within the three bounds, and a box spanning both operands'.

| Test | Case | Expect |
|---|---|---|
| `TestUnionSharedCapObliqueLoftAndCone` | Ø4 base 1.9 off the axis at 235°, lofted to a 0.9922 circle 1.25 above on the axis; a cone on it | `Union` builds; `Intersect` empty; `Cut` keeps the frustum (red: gate refuses) |
| `TestUnionSharedCapCylinderAndCone` | a lofted cylinder of radius 1 and a cone on its cap, `ExactOnly` | `Union` builds (fails red: the gate refuses) |
| `TestUnionSharedCapPartialOverlap` | a cone whose cap overlaps a frustum's top over a lens, `ExactOnly` | `Union` builds (fails red: the gate refuses) |
| `TestUnionSharedCapBossOnPlate` | an extruded boss standing inside a plate's top, default and `ExactOnly` | `Union` builds with no shift (fails red: the retry shifts, `ExactOnly` refuses) |
| `TestUnionSharedCapTangentCapsRefused` | a cone whose cap touches a frustum's top at one point, `ExactOnly` | `BooleanUnsupportedContact` (passes before; fails red when §19.4 accepts every pair) |
| `TestRevolveCapSidesNeedTheCapAloneOnItsPlane` | a cone, and a revolve whose meridian returns to its lowest level at radius 1 | two sides, then only the top side |

## 20. Separated caps on mesh Boolean results

§19 reads cap sides from three analytic payloads only, so a separated pair whose operand is itself a mesh Boolean
result reaches the proximity gate. A slider tongue cut to length by a box (a mesh `Cut` result) and a drive boss lofted
on its end face with a cone on top (a mesh `Union` result) meet that way: the boss's base is the tongue's end region and
their walls continue each other's along the shared rim. The gate refuses the rim contacts, and the shifted retry cannot
open the walls that continue each other. This section gives such operands sides of their own.

Code: `internal/booleaneval/cap_sides.go` (§20.1), `internal/meshbool/local_cap_side.go` (§20.2), `SeparatedOperands`
in `internal/meshbool/separated_caps.go`, `facetedrecord.Payload.CapSides`.

### 20.1 Sides a Union carries

An unshifted live mesh `Union` of A and B with no replay and no fixed operand carries a cap side `s` of A, on plane `P`
with face `X`, to its result when B states a side `t` whose normal is a positive multiple of `s`'s and whose plane lies
strictly inside `s`'s half-space: `N_s·(O_t − O_s) < 0`. The same holds with A and B swapped.

**Claim U.** The true result `R = A ∪ B` lies in `s`'s closed half-space and meets `P` only in the closure of `X`'s
true patch, which is `R`'s face there.

B lies in `t`'s half-space, which lies in the open half-space of `s`, so B misses `P`, and `R ∩ P = A ∩ P = cl(X)`.
Every point of `X` lies outside B, so `X` stays on `R`'s boundary.

The result records the side against `X`'s source group (`facetedrecord.CapSideRecord`), bound to its held mesh with an
`occupancy.Binding` (`CapSideRecords`). Placement and scaling clear it, and every other rewrite allocates new held
arrays, so a copied record is never read. `booleaneval.capSides` reads a record only when every facet of the group
lies in one face of the result, which becomes the side's face; a later `Union` carries it again.

**Kept outline.** A faceted face's outline corners read with the weak radius of `agent-docs/flush-contact-design.md`
§4.3, `√3·(w₁ + w₂)/sin φ`. A lofted cap's outline borders one ruled wall facet per chord, and consecutive chords meet
at nearly straight angles, so that reading gives the boss's base above 1.6 mm, which the band of §19.5 then charges to
every kept piece near the outlines. A record sets `KeptOutline` when the result group holds exactly the operand face's
facets, compared by their exact corners, every corner's bound in the result is no smaller than in the operand, and the
operand read its outline with radius `β` (an analytic payload, or a face that already kept its outline). Each outline
vertex then stands for the same true point within its own bound, as it did in the operand, so `readOutline` gives it
`ρ = β`. The boss's base then reads 0.0056 mm.

### 20.2 Local sides

A mesh `Cut` result states no side, and the tongue need not lie below its end plane everywhere: the end cutter leaves
the tongue's root rounds standing past it, far from the boss. When no pair of payload sides is separated,
`SeparatedOperands` pairs each payload side `t` of one operand B with a **local side** of the other operand A: plane
`P` of `t`, outward normal `−N_t`, and region `K`, the box of B's held vertices widened by the largest bound any face of
B states, padded by a relative `10⁻⁹` and 64 ulps of the coordinate scale. B's true solid lies inside its boundary's
box, so it lies in `K`.

A face `X` of A is the side's face when its held corners lie in `P`, its first facet faces `−N_t` and every facet is
carrier-exact (§16.1). The side holds when every other face F of A whose held facets come within F's bound of `K`
(its gate bound or largest facet bound, padded by a relative `10⁻⁹`) passes one of:

- **Clear below.** Every held vertex of F lies inside the half-space farther from `P` than that bound, exactly. F's true
  patch, within the bound of its facets, lies in the open half-space.
- **Leaving wall.** F carries a retained `Plane` or `Cylinder` tag with no `TagDeparture`, denoted patch,
  `AxialDelta`, `ExtentDelta`, view charge or flat patch, so its true patch lies on the tag
  (`agent-docs/faceted-source-retention-design.md` Claim R). The tag crosses `P`: a cylinder's axis is not parallel to
  `P`, a plane is not parallel to it. Every held vertex lies in the closed half-space, and every edge of F is shared
  with `X`, with a face that is clear below, or with another such tag along a seam. A seam needs a direction `d` with
  `d·N_t ≠ 0` along which both tags are invariant: two planes' crossing direction, a cylinder's axis held by the plane,
  or the axis of two parallel cylinders that are not one surface. One end vertex of the seam lies on `X`'s loops and the
  other end, or both, lies below `P` farther than its vertex bound.

**Claim L.** Then A's true solid has no point inside `K` strictly beyond `P`.

- A leaving wall's true seam is a connected piece of the two tags' intersection (faceted-source-retention Claim E),
  which is a union of lines along `d`, so the seam is the segment between its true end vertices. One lies in `P` (on
  `X`'s boundary) or below it, the other below it, so the seam lies in the closed half-space. Edges shared with `X` lie
  in `P`, and edges shared with a face clear below lie in the open half-space.
- F's true patch is a compact piece of its tag whose boundary lies in the closed half-space. The tag's part strictly
  beyond `P` is connected and unbounded, since every ruling crosses `P`. F meets that part in a set that is closed in it,
  and open in it because no boundary point of F lies there, so F misses it.
- Every true face of A that reaches `K` is `X`, clear below or a leaving wall, so A's true boundary misses the open
  convex set of points of `K` beyond `P`. The point just beyond `X` above §19.4's deep point lies outside A, since `X`
  faces out of A there, so the whole set lies outside A.

So `A ∩ B` lies in `P`, and `A ∩ P ∩ K` lies in `cl(X)`: a point of A in `P` inside `K` lies on A's boundary, the
walls meet `P` only along their edges with `X` (an interior point of a crossing tag would have neighbours beyond `P`),
and the other faces miss `P`. Claim P1 of §19.3 then holds for the pair, and §19.3–§19.6 apply unchanged. The held checks
of §19.1 item 3 read only the facets whose boxes meet `K` (`CapSide.Region`): a facet outside `K` meets no facet of B.

A local side joins A's sides only when the pair it forms is separated; `SeparatedOperands` then clears the separated
verdicts read before it joined.

### 20.3 Kept refusals

| Case | Why |
|---|---|
| A face of A near `K` that crosses `P`, such as a post standing beside the boss | the held check of §19.1 item 3, and no claim of §20.2 |
| A curved wall near `K` other than a retained cylinder, a wall whose tag departs or carries a charge, or a cylinder whose axis lies along `P` | no leaving wall |
| Two operands that both state only local sides | §20.2 needs one payload side for `K` |
| A carried side of a shifted, replayed or fixed-tool `Union` | §20.1 reads the unshifted live attempt only |
| A tangent or shallow cap overlap | no deep point (§19.4) |

A local side reads A's own outline with §4.3's corner radius of `agent-docs/flush-contact-design.md`. Without its
seam radius the cut prism's top reads its tangent corners at about 0.07 mm, which the band charges to every kept piece
near the outlines; with it, `TestUnionFlushCapCutPrismAndLoft`'s result holds a 0.011 mm boundary bound.

### 20.4 Tests

`union_faceted_cap_test.go`:

| Test | Case | Expect |
|---|---|---|
| `TestUnionFlushCapCutPrismAndLoft` | a hull prism cut back to its top plane by a box, and a boss lofted from that hull; both orders, default and `ExactOnly` | `Union` builds with no shift, boundary bound under 0.03 mm (fails red: the gate refuses; about 0.08 mm without the seam radius) |
| `TestUnionFlushCapFarPostAcrossPlane` | the cut prism with a post standing through the cap plane outside `K` | `Union` builds, two lumps, boundary bound under 0.03 mm (fails red: the gate refuses) |
| `TestUnionFlushCapNearPostRefused` | the post inside `K`, clear of the boss | `BooleanUnsupportedContact` |
| `TestUnionFlushCapTangentBossRefused` | a cylinder boss whose floor touches the cut prism's top at one point | `BooleanUnsupportedContact` (fails red when §19.4 accepts every pair) |
| `TestUnionFlushCapCarriedByDrive` | an analytic hull prism and the boss with a cone on top | `Union` builds, boundary bound under 0.03 mm (fails red: the gate refuses; about 3 mm without `KeptOutline`) |
| `TestUnionFlushCapCutPrismAndDrive` | the cut prism and the boss with a cone on top | `Union` builds, boundary bound under 0.03 mm (fails red: the gate refuses) |

`internal/meshbool/local_cap_side_internal_test.go` covers the seam directions and the tags `exactRuledTag` refuses;
`internal/facetedrecord/cap_side_record_test.go` covers the binding; `TestFlushOutline` covers the seam radius and
its fallback to §4.3's corner radius.

## 21. Exact loft segment containment

`loftrecord.DirectSectionLineWithinEdge` compares a specified whole straight segment of an authenticated, unchanged direct
loft section with a retained straight Boolean edge whose endpoints have zero bound. It lifts the recorded section
endpoints through their frame in exact rational arithmetic. Both lifted points must lie on the finite target edge.
The direct-section seal must match the loft body and its source producer. Non-finite, degenerate, partial, displaced,
or unauthenticated inputs are refused.

This answers only segment containment. The caller must establish that the edge belongs to its target, bind each mesh
contact to the source edges, and prove the surrounding contact arrangement before admitting any Boolean. No Boolean
route is changed by this comparison.

`loftrecord.DirectSectionLineContainsPoints` checks two distinct held points against the named authenticated whole
section line using exact rational collinearity and interval comparisons. It identifies no mesh edge and does not
classify contact; its caller must bind both points to the original loft facets separately.

`loftrecord.DirectSectionLineSupportsBoundary` checks an authenticated section's entire line-and-arc boundary
against a named whole line. Every other line must stay on one side and meet it only at the named line's endpoints.
Every arc's full circle must stay on that side, with any tangent point pinned to a named endpoint, and its two pins
must have exactly the same radius. Exact rational determinants and squared distances decide the comparisons. This
proves a boundary support line only; a consumer must separately establish the filled-region and face relationships.

`loftrecord.DirectCellSourceSegment` reads a private copy of the original loft builder's outer-loop cell-to-segment
map. The builder binds it to the original body and its authenticated direct sections after assembling the loft. The
map holds at most 8192 cells, and every entry names an existing first-section segment. A copied or changed body,
changed section, unsealed map, or out-of-range cell is refused. Mutating the payload's public station map cannot
change the private copy. A caller must separately prove that a mesh wall facet is the named original body cell;
this lookup alone does not classify contact.

The original builder also seals its two cap face identities and ordered wall face identities, after checking the
assembly's outer-loop cell and side indices. `DirectWallFaceSourceSegment` requires the named original face, cell,
and triangle side to match that seal before reading the cell's source segment. `DirectCapFaceMatches` identifies
the original cap by pointer. Both checks refuse rewritten or unsealed bodies. They identify source faces only; a
consumer must still bind a mesh facet to one and prove the contact before changing a Boolean.

The face seal also copies each wall triangle's three held coordinates after checking them against the face's loop.
`DirectWallFacetSourceSegment` requires the verified mesh's source face and ordered triangle coordinates, and the
prepared triangle's coordinates, to match that original wall. This binds that held facet to the authenticated cell;
it does not prove the facet's contact with another operand or admit a Boolean.

The seal also copies the original ordered cap triangles from the authenticated builder. `DirectCapFacetMatches`
requires the verified mesh's cap face, original facet index and ordered coordinates, and the prepared coordinates
to match that record. The two cap ranges each hold no more triangles than the bounded outer-loop cell count. This
identifies a held cap facet; it does not establish its contact or prove that the facet's plane is the true cap plane.

`meshbool.RetainedEdgeAtMeshEdge` reads a verified operand mesh and its prepared copy. It finds the two source faces
beside a mesh edge and requires them to share exactly one retained `Line3` with zero-bound endpoints. Exact rational
collinearity and interval tests must place both mesh-edge vertices on that finite topological edge. The mesh and its
prepared copy must agree on the two facets and their edge vertices. The result identifies a true edge containing the
held segment; it does not establish any contact with the other operand or permit a Boolean by itself.

`booleaneval.directLoftSharedEdge` combines those checks for a target mesh edge and a loft wall edge. It also requires
the wall edge's twin to be an authenticated original start-cap facet, verifies that the prepared wall and cap match
their source facets, and checks that the two held edge intervals overlap by a positive length. The common target edge
alone is insufficient: two disjoint held segments can lie on it. This comparison proves only the shared source-edge
relationship. It does not prove the adjacent face arrangement or admit a Boolean by itself.

`booleaneval.directLoftTargetFaceMeetsCapAtEdge` adds an exact target-face check: the verified target mesh must give
the face zero source-bound, and every prepared facet of that face must still match the source mesh. Exact plane tests
require all its vertices to lie on one side of the loft's original first-section plane; every vertex on the plane must
lie on the shared retained edge. Every triangle is convex, so the exact face meets that plane only on the edge. The
function does not establish the loft cap's filled region or admit a Boolean by itself.

`booleaneval.directLoftCapEdgeSupport` additionally requires the held wall edge to equal the entire source segment.
It checks every original start-cap facet, including its source and its unshifted prepared coordinates. The cap's held
vertices must lie in the source plane and on one side of the line, and vertices on the line must lie in the segment.
The authenticated source boundary also meets the support line only on that segment. A bounded filled profile with a
boundary wholly on one side cannot enter the open opposite half-plane: a path from an interior point there to infinity
would cross the boundary in that open half-plane. Its points on the support line are therefore boundary points, and
they lie on the named segment. The held cap's triangles give the same conclusion by convexity. The true and held
target faces meet the cap's plane only on the retained edge, which contains the entire source segment. Both cap rims
contain the segment. Hence the exact target face and the loft cap meet along exactly that segment in both the true and
held models. This face-pair statement does not classify the neighboring wall faces or make a Boolean result manifold.

For an unshifted ordinary `Cut`, `directLoftCapContactEvidence` searches for that certificate when the proximity gate
would otherwise refuse a face pair. A shared strict work and byte budget bounds the search. A successful certificate
defers this one pair to the mesh classifier; all other pairs retain the original gate. Replay, moved operands, fixed
tools and the cone-trim route do not use it. The mesh classifier still refuses an unsupported edge graze.

## 22. Shared straight edges of a direct loft

For an unshifted ordinary cut, a shared-edge classifier may use an authenticated direct loft's start-cap edge only
when it proves the two held edge intervals lie on the same retained target edge and overlap. The target's two
incident facets must match their source mesh, and each must lie in a proven exact source plane. The original cap and
its source boundary must have the one-sided support established in §21.

The incident loft wall must be a line-to-line cell's original first triangle. Its three held vertices must equal,
in exact lifted source coordinates, the first section line's two endpoints and the paired second section line's
end. Both source lines must be whole and their recorded following segments must start at their respective endpoints.
The builder's cell and face seals bind that triangle to its source cell. These conditions prove that the true and
held wall triangle coincide; a bound on distance to a plane alone does not prove its angle at the shared edge.

The mesh classifier reads both triangles beside each shared edge and refuses a flat, tangent, or near-tangent wedge.
It classifies the strict local half-planes using exact arithmetic. The Boolean refuses a local result with any number
of kept facets other than zero or two. Every other contact retains the existing checks. This route does not admit a
shifted operand, an unmatched source triangle, a curved cell, or a result with an unproved local arrangement.
