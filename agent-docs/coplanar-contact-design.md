# Coplanar Contact Design

Exact classification of coplanar facet contacts in the mesh boolean, and exact planar faces in its results: root
cause (§1), required behavior (§2), the classification rule (§3), composition with face bounds and records (§4), the
manifold audit (§5), the proximity gates' obligation (§6), planar face simplification (§7), records of results (§8),
soundness (§9), performance (§10), tests (§11), risks (§12), user decisions (§13), code map (§14), open questions
(§15), seats in an Intersect (§16), placed seats (§17) and flush planes that agree only within a bound (§18).

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
faces with a positive bound (they keep the refusal and retry); `ErrCrossContainer`; changes to the gap-retry
ladder; the analytic (prism, class B, brep) paths; merging coplanar faces of different operands into one face.

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
facet `i` of A and facet `j` of B are both exact. Otherwise a coplanar pair with `CoplanarOverlap` true is **refused**: a
`coplanarContactOf` contact, `TouchCoplanar`, Slack 0, Decisive when facing opposite. The refused path leads to the
gap-retry ladder.

Face-level exactness, not carrier coplanarity, is the admission test because `interference-design.md` §5.2 forbids
settling a coplanar pair on carrier coplanarity alone: a planar face with a chorded rim holds an inscribed outline. A
zero face reading is the tessellation's claim that every held polygon of the face IS the true trimmed patch. It is the
predicate the gate already skips on, which makes §6 hold by construction.

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
when that facet and the partner facet form an admitted pair recorded by §3.2 step 2. It then adds its two segment
endpoints to `owned`, appends `touchOf(ma, p.I, p.J)` to `legacyGrazes`, and does nothing else: no cut (the segment
runs on the facet's own edge, and the admitted pair already cut the partner), no block (the coplanar facet takes the
cut path, §3.4, so no flood fill crosses the edge). Two other cases:

- The pair is refused (an inexact facet): §3.1's refusal returns before the in-plane loop runs.
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

A clipped endpoint lies on two exact facets. `CutCornerBound` gives a point that is neither a facet corner nor a rim key
the facet's `FacetBound`, which is 0 for an exact facet. No `rims` entry is added for coplanar points, so `RimBound`
and `MaxRim` are unchanged.

## 4. Composition with face bounds, `Plane` records and the `Faceted` fallback

- **Positive face bound.** A face whose gate reading is positive never enters §3. Its coplanar contacts are refused
  with `TouchCoplanar` and the gap retry runs as documented. This covers curved faces, planar faces with chorded rims,
  every shifted or grown attempt (Assumptions), and mesh results whose vertices round.
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
  face, so §3.1 refuses it with `TouchCoplanar` in the mesh pass. The deferral stays backed.

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
  when no facet with zero bound at all three corners fixes its plane.
- Vertex bounds are the measured weld gaps: zero for every float vertex, and §7 leaves no artifact vertex on a
  simplified face.

## 9. Soundness

Claim: for every admitted pair the held result equals the exact boolean of the held operands, and the held operands
equal the true ones on every admitted facet, so a true overlap is reported by the same paths as any crossing pair and
is never hidden as contact.

1. **Held equals true on admitted facets.** §3.1 admits only facets whose true piece equals the held facet: zero facet
   bound and zero face reading. Every predicate in §3 is an exact sign over those held values.
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
   deeper than the summed bound. No pair moves from refused to touching except through §3.1.
6. **Bounded faces and gates.** §4 and §6 keep every refusal for faces with a positive bound, and the occupancy re-run
   cannot defer an admitted pair (§6).
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
fixtures. Rules:

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
