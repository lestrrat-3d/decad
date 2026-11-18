# Coplanar Contact Design

Exact classification of coplanar facet contacts in the mesh boolean, and exact planar faces in its results: root
cause (§1), required behavior (§2), the classification rule (§3), composition with face bounds and records (§4), the
manifold audit (§5), the proximity gates' obligation (§6), planar face simplification (§7), records of results (§8),
soundness (§9), performance (§10), tests (§11), risks (§12), user decisions (§13), tasks (§14), open questions (§15).

Implements `agent-docs/interference-design.md` §5.2 and §11 row 4. Companion to `agent-docs/evaluator-design.md` §9
(mesh boolean), `agent-docs/gap-tolerance-design.md` §5.2 (shifted retry),
`agent-docs/faceted-vertex-bounds-design.md` §3.4 (weld) and `agent-docs/faceted-source-retention-design.md`
(retained tags). Adds no public API and no option.

## Router

Navigation, not authority.

| Question | Section |
|---|---|
| Why do flush solids fail today? | §1 |
| What must hold after this lands? | §2 |
| Which coplanar facet pairs are classified, and how? | §3 |
| What happens to a face with a positive bound? | §4 |
| What stops a non-manifold result, and what does the retry see? | §5 |
| Why do the proximity gates stay sound? | §6 |
| How does a `Cut` or `Union` result get exact planar faces? | §7 |
| Why does a flush `Union` keep its `Plane` faces and history? | §8 |
| Why can a real overlap never read as contact? | §9 |
| What does it cost? | §10 |
| Which tests prove it? | §11 |
| What does the implementer build, in what order? | §14 |

## Assumptions

- "Exact face" reuses the proximity gate's per-face reading: `topo.Mesh.FaceBound[f]` narrowed by
  `BoolMesh.FaceSurface[f]` (`facesOfMesh`, `internal/meshbool/tangency_gate.go`) is `<= 0`. No new exactness notion.
- Non-coplanar grazes (both apexes strictly on one side) and isolated point touches not produced by an admitted
  coplanar pair keep today's refusals.
- A shifted, grown or fixed-tool attempt (`hasShifts || fixed != nil` in `booleaneval.evaluate`) admits no coplanar
  pair: its `BoolMesh.Exact` stays nil. The retry ladder therefore behaves exactly as today.
- The bounded replay path refuses coplanar contacts until Task 12 ports §3 and §5. A replay refusal is a decline. §7
  runs on the replay stitch as well, so live and replay held meshes are equal for every boolean both paths build.
- Planar face simplification (§7) leaves any face region with a hole untouched in this design.
- decad tests use minimal box and prism fixtures and name no consumer model (`agent-docs/testing.md` "Test scope").
  The consumer dovetail check runs outside this repository (§11.4).
- The behavior has no switch: the exact coplanar pass and planar face simplification run on every mesh boolean.

## 1. Problem and root cause

Two solids that meet face to face (zero gap, no interior overlap) must verify as `Sound`, and `Union`, `Intersect`
and `Cut` must build them exactly. Today that holds only when an analytic path admits the pair (prism class A, class B
with axis-aligned walls, history certificates). Every pair that reaches the mesh boolean fails:

1. **Coplanar refusal.** `TriTriClassify` returns `ContactRegion` for any coplanar facet pair sharing positive area or
   a positive-length boundary segment (`CoplanarOverlap`, `internal/meshbool/triangle_contact.go`). `pairCuts` refuses
   the boolean with `TouchContactError{What: TouchCoplanar}` (`internal/meshbool/boolean_pair_cuts.go`).
2. **Coplanar-bounded edges read as grazes.** `EdgeCrosses` (`internal/meshbool/boolean_mesh.go`) calls an in-plane
   facet edge a crossing only when its two apexes straddle the partner's plane. When the adjacent facet lies IN that
   plane (apex sign 0), the edge is called a graze, although it is the rim of a flush patch.
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

Measured on the reproductions (`.tmp/repro/exp_test.go`;
dated illustration, 2026-11-18):

| Pair | `Verify` | `Intersect` | `Union` |
|---|---|---|---|
| exact prism vs exact prism (trapezoid in socket) | `Sound` | `0.00023 ± 0.0041` | `7680 ± 0` |
| exact trapezoid vs socket made by `Cut` (`FaceBound` 1.03e-15) | `Suspect`, `unsupported_pair_contact` | refused (gate) | `7679.9997 ± 0.004` |
| trapezoid made by flush `Union` vs either socket | `Suspect` | refused (gate) | refused (gate) |
| exact tenon vs slot made by two `Cut`s | `Suspect` | refused (gate) | `990.3967 ± 0.001` |
| exact box under a box made by `Cut`, flush walls | `Sound` (history walk) | refused (gate) | `1935.3076 ± 0.0015` |

**Out of scope:** curved coplanar or tangent contact; non-coplanar grazes and isolated point touches; contact between
faces with a positive bound (they keep today's refusal and retry); `ErrCrossContainer`; changes to the gap-retry
ladder; the analytic (prism, class B, brep) paths; merging coplanar faces of different operands into one face.

## 2. Correct behavior and success criteria

- **C1.** A pair whose contacts are coplanar patches of exact faces (§3.1), plus the edges and points that bound those
  patches, is classified by exact predicates. When the result is manifold, no `TouchContactError` is raised, so the
  gap retry is never reached.
- **C2.** Opposite-facing coincident patches are boundary-only contact: `Union` drops both copies, `Intersect` drops
  both, `Cut` keeps the target's copy. Same-facing coincident patches keep one copy (operand A's) for `Union` and
  `Intersect`, and none for `Cut` (§3.4).
- **C3.** An unshifted empty `Intersect` proves disjoint interiors, so `Verify` reports no row and, with no other
  finding, `Sound`. `interference-design.md` §6.1's premise P2 and Step 1 are restated for admitted pairs in Task 10;
  the claim is unchanged.
- **C4.** A flush result keeps every retained `Plane` tag of both operands, carries no `CodeGapShiftApplied` warning,
  and stays `boolchain.Expandable` (§8).
- **C5.** A face of a mesh result whose held facets are coplanar, hole-free, with every vertex claim zero and float
  corners, is held exactly: its stitched vertices carry zero bound and the face's `FaceBound` is 0 (§7).
- **C6.** A pair whose true solids overlap is never reported touching or disjoint (§9).
- **C7.** The reproductions and the six consumer dovetail parameter sets read `Sound`, `Intersect` returns
  `BooleanEmpty`, and `Union` volume equals the sum of the halves within its bound (§11.4).
- **C8.** A flush `Union` of exact boxes under the default gap tolerance runs no slower than today's retry path, and
  non-coplanar booleans stay within benchmark noise (§10).

## 3. The exact coplanar classification rule

decad computes the 2D answers of this section (clipping, cover tests) and of §7 (re-triangulation) itself instead of
asking `sketch`: they are exact rational predicates over decad's own held facets, run per facet pair on the boolean's
hot path, and a private `sketch` scene per facet pair would cost a scene build per pair for no gain in exactness. The
same reason backs the exact coplanar clip of `agent-docs/multibody-dynamics-design.md` §9.4.

### 3.1 Admission

A facet `t` of a prepared operand is **exact** when both hold:

- `FacetBound[t] == 0`;
- the gate reading of its source face is zero, computed by the helper `facesOfMesh` uses (Task 2 extracts it as
  `meshbool.FaceGateDelta`).

A coplanar facet pair `(i, j)` (all six plane-side signs zero, today's `CountZero == 3` branch) is **admitted** when
facet `i` of A and facet `j` of B are both exact. Otherwise a coplanar pair with `CoplanarOverlap` true is **refused**
exactly as today: a `coplanarContactOf` contact, `TouchCoplanar`, Slack 0, Decisive when facing opposite. The refused
path, and with it the whole gap-retry ladder, is unchanged.

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
   (`ProjAxes`), with `CoplanarClip` (Task 3). Keep each clipped piece of positive length that does not lie along an
   edge of `ta`. Append it to `cutsA[i]` as `Xseg{ViaParity: true, Partner: tb}`, with `AOnEdge`/`BOnEdge` from the
   clip. Do the same with `ta`'s patch-boundary edges into `cutsB[j]`. `ViaParity` means "never an anchor"
   (`RegionOf`). That is required: a region beside a coplanar segment lies IN the partner's plane, so the
   partner-plane side test reads 0.
2. Record `coplanarA[i] += j` and `coplanarB[j] += i`.
3. Add every clipped endpoint key to the `owned` key set.
4. Append `coplanarContactOf(ma, mb, i, j)` to `legacyCoplanar` (capped at `maxTouchContacts`). The pair pass never
   raises it; §5 may.

An admitted pair whose closed triangles share only a point (`CoplanarTouch`) adds that point to `owned`, appends
`touchOf(ma, i, j)` to `legacyPoints`, and does nothing else. `TriContact` and `TriContactSpan` gain `Coplanar bool`,
set only by the coplanar branch of `triTriSpan`, so the pass tells coplanar points from crossing points.

**Remaining refusals.** A segment set that still branches inside a facet keeps `BuildChains`' refusal, an expected
contact (`ErrUnclassifiableContact`). That needs a crossing chain of the other operand ending on a patch-boundary edge
at a point strictly inside the facet, which happens only where the other operand's surface passes through the plane at
its own patch rim.

### 3.3 In-plane edges

`EdgeSide` (Task 4) replaces the bool of `EdgeCrosses` with three outcomes. `EdgeCrosses` stays as a wrapper that maps
`EdgeCoplanarBounded` to `false`, so the bounded replay collector keeps today's behavior.

| Apex signs `s0`, `s1` | Outcome | Action in `pairCuts` |
|---|---|---|
| `s0·s1 < 0` | `EdgeCrossing` | today: block the edge, add a `ViaParity` cut to the partner |
| both nonzero, same sign | `EdgeGraze` | today: collect a `touchGraze` contact and refuse |
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

`keepSideWith(ctx, m, other, cuts, opts keepOptions)` (Task 5) carries the coplanar map, the op and which operand `m`
is. `keepSide` delegates with zero `keepOptions` and keeps its signature and behavior.

Every facet with a non-empty `coplanar[i]` takes the cut path and counts as cut for the uncut-component flood fill, so
no component seed's verdict spreads onto it. With no surviving segment it is one region, the facet itself, with its
centroid as probe, and `CutTriangle` is not called. For each region `R` with probe `p` (strictly inside `R`,
`RegionOf`):

1. Cover set `C = { j in coplanar[i] : p in closed tb }`, by exact 2D orientation in the projection of §3.2. A probe on
   a patch-boundary edge of some `tb` is `ErrBooleanFailed`: those edges were inserted. A probe on an internal edge of
   the partner's patch lies in both facets sharing it, and both are in `C` (both are coplanar partners of `i`).
2. `C` empty: classify as today (anchor, else parity; parity `onBoundary` stays the expected contact refusal).
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
detect a pinched vertex. Both must stay expected contact refusals carrying exactly today's contacts, so `Verify` reads
`Suspect` instead of returning an error, and the gap retry for such a pair (translation and, for `TouchCoplanar`,
growth) behaves as it does today.

`MeshBooleanStats` returns `Stats.Coplanar *ContactKeys`, non-nil if and only if `owned` is non-empty:

```go
// ContactKeys is what the stitch needs to tell a contact pinch from a broken mesh.
type ContactKeys struct {
	Owned  map[string]struct{} // exact keys of every point §3.2–§3.3 accepted
	Legacy error              // the refusal the pair gives with no exact facet
}
```

`Legacy` is built from the lists of §3.2–§3.3 in today's order: `legacyCoplanar` non-empty gives
`ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: TouchCoplanar, Contacts: legacyCoplanar})`; else
`legacyGrazes` gives `touchGraze`; else `legacyPoints` gives `touchPoint`. These are the contacts today's pass builds,
with the same `NormalA`, `Decisive`, `FacetsB` and Slack 0, so the retry reads an identical error.

`booleaneval` passes `Stats.Coplanar` to `StitchFacetsWith` (Task 6). After the conforming passes, when it is non-nil:

1. Edge audit: every undirected edge is used by exactly two facets, in opposite directions.
2. Vertex audit: the facets around every vertex form one cycle through shared edges.

A failing edge or vertex whose exact keys are all in `Owned` returns `Legacy`. Any other failure keeps today's
`ErrBooleanFailed`.

The retry reads the same refusal wherever the exact classification stops. A graze or isolated point found beside a coplanar
pair, and an expected-contact refusal of the side pass for a pair whose points the pass accepted, return the coplanar
refusal or `Legacy` the pair gave before the classification existed, because the retry ladder starts from it. The audit reads only stitched vertex keys, so it needs no operand normals and no map back to kept
facets. `StitchFacetsContext(ctx, kept)` delegates to `StitchFacetsWith` with zero options and keeps its behavior.

## 6. Proximity gate obligations

`interference-design.md` §5.2 requires every coplanar deferral of a gate (`gatherNearContacts` returns `deferred` on
the first `ContactRegion`; the caller skips the face pair) to be backed by a refusal. Two gate calls exist.

**The pre-pass** (`booleaneval.evaluate` → `RefuseUndecidableProximity`):

- A face pair with summed reading `<= 0` is skipped before any gather, as today. Both faces are exact, every coplanar
  pair between them is admitted and classified exactly, and nothing a chord could hide exists.
- A face pair with a positive sum has at least one inexact face. Every coplanar facet pair in it holds a facet of that
  face, so §3.1 refuses it with `TouchCoplanar` in the mesh pass. The deferral stays backed.

**The occupancy re-run** (`composeOccupancy`, `internal/booleaneval/eval.go`) runs the gate again over widened face
bounds after the mesh pass. An admitted exact pair can get positive slack there, and no mesh refusal backs that
deferral. `composeOccupancy` returns nil (no occupancy record; the boolean result is unchanged) whenever
`Stats.Coplanar != nil`. This also covers `localDeepTerm`, which reads `Stats.Contacts` and so never sees admitted
coplanar contacts.

Task 5 rewrites the doc comments of `FacesNearMiss`, `GatherNearContacts` and `RefuseUndecidableProximity` to state
this and adds test G1. Task 6 adds the `composeOccupancy` guard and test G2 (§11.1).

## 7. Planar face simplification

### 7.1 Rule

A mesh boolean result face whose held facets are exactly coplanar, hole-free, carry only zero vertex claims, and whose
B-rep corners are floats is re-triangulated on its corners alone before rounding, so it holds no artifact vertex and
rounds with no charge.

`SimplifyPlanarFaces` (new, `internal/meshbool/planar_simplify.go`) runs inside `StitchFacetsWith` when
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

In the reproductions the socket's B-rep corners are exact (every vertex bound 0) and its 1.03e-15 bound comes only from
artifact vertices, so every socket face becomes exact. A flush `Union` built by §3 has the same structure, so the tail
half's faces become exact too. The exact planar arm of `Verify` reads the global `MeshBound`
(`internal/planarsnapshot/faceted.go`), which becomes 0, so that arm can decide such pairs directly as well.

## 8. Records of a flush result

With no refusal, `Evaluate` returns the first attempt:

- `ev.Shift` stays nil, so no `CodeGapShiftApplied` warning is appended and `boolchain.Expandable` admits the body.
- No piece is grown, so every source face of both operands keeps its group, retained tag, `CapRecord` and `AxialDelta`
  (`facetedrecord.SourceTag`).
- `RetainedEdgeCurve` publishes `Line3` for every rim between two retained non-parallel planes (E2, E6).
- A rim between two coplanar faces of different operands (flush walls of a `Union`) stays `FacetedCurve`, because E2
  needs non-parallel normals. The two faces stay separate faces on one plane.
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
   case a region with an interior probe strictly inside the other solid exists and is classified inside, as today. In
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
  per-result candidate scan (linear in result facets, one exact orientation per corner). §7's `EarClipX` is
  superlinear in a loop's corner count and runs only on candidate faces; the fallback's second rounding runs only when
  the first fails. Task 8 measures `BenchmarkMeshBooleanCrossing` (two crossing boxes, full `MeshBooleanStats` plus
  `StitchFacetsWith` with `SimplifyPlanar`) with the switch off and on; the target is no significant change.
- **Flush pairs.** Today a flush `Union` under the default reach runs one exact pass, up to 6 shrink steps, then up to
  4 rounds of 6 steps × 3 directions of translation, each a full re-mesh and boolean (about 10–20 ms per box pair in
  `apitest.TestBooleanGapTolerance`; dated illustration). After this design it runs one pass, plus clipping (up to 6
  edge clips per admitted pair), forced `CutTriangle` on coplanar facets, and the manifold audit (linear, only when
  `Stats.Coplanar != nil`).
- **Worst case.** Today the pair pass stops at 64 coplanar contacts. Exact classification visits every admitted pair,
  so two finely triangulated coplanar faces of `n` and `m` facets cost `O(n·m)` clips and one facet can collect many
  segments. `cutBatchSize` already bounds live memory. Task 8 measures it (§13 D2).
- **Consumer CI.** The six dovetail cases take 0.06–0.11 s each today (dated illustration). Record their wall time
  before and after (§11.4); the retry ladder no longer runs for those pairs.

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
| G1 | `TestCoplanarGateInexactFace` | coplanar pair where B's face has `FaceBound > 0` and its facet `FacetBound == 0` | `TouchCoplanar`; red leg: admit on `FacetBound` alone |
| G2 | `TestComposeOccupancyDropsCoplanar` (`internal/booleaneval`) | tracked `Union` with an admitted pair | no occupancy record; same held result as with tracking off |
| M1 | `TestStitchManifoldContact` | edge-only and vertex-only `Union` | returns `Legacy` (`TouchCoplanar` / `touchPoint`, contacts equal to the switch-off refusal); red leg: no audit gives `ErrBooleanFailed` or a pinched mesh |
| M2 | `TestStitchFacetsWithZeroOptionsUnchanged` | existing crossing fixture | held mesh identical to `StitchFacetsContext` |
| S1 | `TestSimplifyPlanarTrapezoidCut` | box `Cut` by a 10° trapezoid prism | `StitchedMesh.VertexBound` all 0 on simplified faces, `Round == 0`, exact volume integral unchanged |
| S2 | `TestSimplifyPlanarNonFloatCorner` | a corner that is not a float | that face and its neighbours' shared chain untouched |
| S3 | `TestSimplifyPlanarHoledFace` | holed face | untouched |
| S4 | `TestSimplifyPlanarFallback` | forced fold on the simplified mesh | result equals the unsimplified stitch |
| S5 | `TestSimplifyPlanarNonzeroClaim` | face with one interior vertex with `xbeta > 0` | untouched; red leg: a corners-only check simplifies it |

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

A3's default outcome is today's: before Task 10 the implementer runs A3 and A4 with the switch off and confirms it. If
it differs, stop and report. "Exact" for R1–R4 `Union` means volume equal to the sum of the operands' volumes within the
sum of their bounds. R1–R4 restate `.tmp/repro/exp_test.go`'s cases with neutral names and minimal sizes.

P2 classifies each pair by the intersection box's extents `(ex, ey, ez)`: any negative → disjoint; all positive →
overlap; one zero, two positive → face; two zero → edge; three zero → vertex.

| Class | `Union` | `Intersect` | `Cut` | `Verify` |
|---|---|---|---|---|
| disjoint | 2 lumps, exact sum | `BooleanEmpty` | target exact | `Sound` |
| face | 1 lump, exact sum | `BooleanEmpty` | target exact | `Sound` |
| edge, vertex | A3's outcome | `BooleanEmpty` | target exact | `Sound` |
| overlap | exact inclusion–exclusion volume | exact box volume | exact difference | `Interfering` |

### 11.3 Existing tests this change flips

Any test whose contact faces are all exact and that asserts `BooleanUnsupportedContact`, `TouchCoplanar`, a
`CodeGapShiftApplied` diagnostic, a shifted volume bound, or a positive rounding bound on a face §7 simplifies. Known:
`apitest.TestBooleanGapTolerance`, `apitest.TestVerifyGapToleranceDecidesAFlushPair`, the face-shared cases of
`apitest.TestBooleanGapToleranceEdgeAndPointTouches`, `apitest.TestBooleanGapToleranceRestingBossJoinsPlate`,
`apitest.TestBooleanGapShiftKeepsTranslatedTags`, `examples/boolean_errors_example_test.go`,
`internal/meshbool/boolean_pair_cuts_test.go`. Find the rest with `go test ./... 2>&1 | grep -E '^(--- FAIL|FAIL)'`
after Task 10's switch removal. Rules:

- Move each gap-retry assertion, in its own file, onto a fixture whose contact face has a positive bound: B turned by an
  `r3` rotation of 30° about z, then by −30°, so its float vertices round. Before moving, assert in the test that this
  fixture's contact face reads a positive `FaceBound` in its tessellation; if it reads zero, pick another rounding
  transform and record it. NEVER delete a retry assertion without moving it.
- A test that asserted a positive rounding bound now asserts the exact value it reads.

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
| Wide test churn: §7 turns many mesh results exact and §3 removes retries | large Task 10 diff | Task 10 alone flips; rules in §11.3 |
| A cover or flood case touches the other boundary unseen | wrong keep/drop | parity `onBoundary` still refuses; U3–U8, P2 |
| Branching segments at reflex patch rims | expected refusal where exact classification was possible | stated in §3.2; §15 Q3 |
| Non-manifold results at coplanar edges or points | invariant error instead of refusal | §5; M1, A3, A4 |
| Worst-case `O(n·m)` clipping on fine coplanar faces | slower booleans | Task 8; §13 D2 |
| Replay diverges from live on coplanar pairs | a re-tessellation or certificate declines on a coplanar-built body | Task 12; the decline is sound meanwhile |
| Simplified faces change facet counts | tests or tools that count facets | counts are not public contract; flips in Task 10 |
| Two faces on one plane after a flush `Union` | later face selection sees two faces | stated in §8; §15 Q1 |

## 13. Decisions for the user

- **D1. Scope of planar simplification (§7).** Option A, every mesh boolean result (designed): gains exact faces
  wherever a face's corners are floats, which later booleans and `Verify` need; costs test churn in Task 10 and a
  per-result candidate scan. Option B, only results of a boolean that admitted a coplanar pair: smaller churn; but the
  socket's `Cut` admits no coplanar pair, so C5 and C7 fail. Recommendation: A, because B misses the acceptance case.
- **D2. Clipping cap (§10), decided from Task 8's numbers.** Option A, no cap: every exact coplanar pair is classified;
  costs `O(n·m)` on fine coplanar faces. Option B, refuse with `TouchCoplanar` above a cap of admitted pairs per pass:
  bounds time; costs falling back to the retry for large coplanar faces. Recommendation: A unless
  `BenchmarkCoplanarCaps/on` exceeds 10× `BenchmarkFlushUnion/on`.

## 14. Tasks

Each task merges on its own with `go test ./...`, `go vet ./...` and `golangci-lint run` passing. Tasks 2–9 change no
public behavior: their code runs only when `coplanarcontact.Of(ctx)` is true. Every production PR carries the Module
boundary gate evidence (`agent-docs/module-boundary.md`): owning section of this doc, input class "coplanar contact
between exact planar faces of mesh boolean operands" (Tasks 2–6, 12) or "planar faces of mesh boolean results"
(Tasks 7, 9), the focused tests, and no new root adapter.

### Task 1: Land the design doc

- **Files:** `agent-docs/coplanar-contact-design.md`, `agent-docs/layout/design-docs.md`.
- **What:** Commit this doc and its layout row.
- **Depends on:** none.
- **Done when:** `go test . -run '^TestCLAUDEMDLayoutStaysCompact$' -count=1` passes.

### Task 2: Switch and exact-facet flags

- **Files:** `internal/coplanarcontact/coplanarcontact.go` (new), `internal/coplanarcontact/coplanarcontact_test.go`
  (new), `internal/meshbool/boolean_mesh.go`, `internal/meshbool/tangency_gate.go`, `internal/meshbool/face_exact.go`
  (new), `internal/meshbool/face_exact_test.go` (new), `internal/booleaneval/eval.go`,
  `agent-docs/layout/internal-contact.md`.
- **What:**
  - `coplanarcontact.With(ctx context.Context, on bool) context.Context` and `coplanarcontact.Of(ctx) bool`, false
    when unset.
  - `meshbool.FaceGateDelta(m *topo.Mesh, surface map[*topo.Face]float64, f *topo.Face) (float64, bool)`, extracted
    from `facesOfMesh`, which then calls it; `false` means no stated bound.
  - `BoolMesh.Exact []bool` (doc comment points to §3.1) and `(*BoolMesh).SetExactFacets(m *topo.Mesh) error` setting
    `Exact[t] = FacetBound[t] == 0 && FaceGateDelta(...) <= 0`; a facet whose face has no stated bound is an
    `ErrBooleanFailed`, as in `facesOfMesh`.
  - `booleaneval.evaluate` calls `SetExactFacets` on both operands after `surfaceSlabs`, only on the non-replay path
    and only when `!hasShifts && fixed == nil`. Otherwise `Exact` stays nil; nil reads as all false.
- **Depends on:** 1.
- **Pattern refs:** `internal/rimsplit/rimsplit.go`; `facesOfMesh` in `internal/meshbool/tangency_gate.go`.
- **Tests:** `TestSetExactFacets`: exact box all true; a facet with `FacetBound > 0` false; a zero-bound facet on a
  face with positive `FaceBound` false; `FaceSurface` narrowing to 0 makes it true. `TestOf` as `rimsplit`'s.
- **Done when:** `go test ./internal/coplanarcontact/ ./internal/meshbool/ -run 'TestOf|TestSetExactFacets|TestRefuseUndecidableProximity' -count=1` passes.

### Task 3: Coplanar flag and exact clip

- **Files:** `internal/meshbool/triangle_contact.go`, `internal/meshbool/coplanar_clip.go` (new),
  `internal/meshbool/coplanar_clip_test.go` (new).
- **What:**
  - `TriContact.Coplanar` and `TriContactSpan.Coplanar`, true only from the coplanar branch of `triTriSpan`.
  - `CoplanarClip(edge [2]proof.Xpt, xta [3]proof.Xpt, n proof.Xpt) (ClipPiece, bool, error)` with
    `type ClipPiece struct { P0, P1 proof.Xpt; P0OnA, P1OnA bool }`: one edge clipped to closed `ta`; `false` when the
    piece is empty, a point, or lies along an edge of `ta`. Exact rationals only.
- **Depends on:** 2.
- **Pattern refs:** `SegTriOverlap2` (`internal/meshbool/triangle_contact.go`), `EdgeCross2Fracs`
  (`internal/meshbool/polygon_exact.go`).
- **Tests:** U1. `TestTriContactCoplanarFlag`: a coplanar point and region set it, a crossing segment does not.
- **Done when:** `go test ./internal/meshbool/ -run 'TestCoplanarClip|TestTriContactCoplanarFlag|TestTriTri' -count=1` passes.

### Task 4: Pair pass admission

- **Files:** `internal/meshbool/boolean_pair_cuts.go`, `internal/meshbool/boolean_mesh.go`,
  `internal/meshbool/boolean_pair_cuts_coplanar_test.go` (new).
- **What:** When `coplanarcontact.Of(ctx)`: §3.2 and §3.3. Add `EdgeSide(m *BoolMesh, f, k int, partner [3]r3.Vec)
  (key [2]int, side EdgeOutcome, inPlane int, err error)` with constants `EdgeCrossing`, `EdgeGraze`,
  `EdgeCoplanarBounded`; `EdgeCrosses` wraps it. `pairCutsResult` gains `CoplanarA, CoplanarB map[int][]int`,
  `Owned map[string]struct{}` and `Legacy error` (§5). Switch off: identical behavior.
- **Depends on:** 3.
- **Tests:** U2; `TestPairCutsCoplanarAdmitted` (box on box: no refusal, expected `CoplanarA/B`, only patch-boundary
  segments inserted, `Legacy` equal to the switch-off refusal); `TestPairCutsCoplanarRefusedWhenInexact`
  (`TouchCoplanar` unchanged); `TestPairCutsCoplanarSwitchOff` (identical result to today on the same fixtures).
- **Done when:** `go test ./internal/meshbool/ -run 'TestPairCuts|TestEdgeSide' -count=1` passes.

### Task 5: Side pass cover rule and gate contract

- **Files:** `internal/meshbool/boolean_mesh.go`, `internal/meshbool/coplanar_cover.go` (new),
  `internal/meshbool/keep_side_coplanar_test.go` (new), `internal/meshbool/depth_witness.go`,
  `internal/meshbool/boolean.go`, `internal/meshbool/tangency_gate.go`,
  `internal/meshbool/tangency_gate_coplanar_test.go` (new).
- **What:** §3.4: `keepSideWith` and `keepOptions{Coplanar map[int][]int; Op OperationKind; Second bool}`; `keepSide`
  delegates with zero options, so its callers (including `boolean_cut_batch_test.go` and
  `boolean_pair_cuts_isolated_test.go`) stay unchanged; `meshBooleanStats` calls `keepSideWith` and fills
  `Stats.Coplanar` (type from §5, declared here). `coplanarKeep(op OperationKind, second, sameFacing bool) bool` and
  `coverOf(...)` live in `coplanar_cover.go`. Rewrite the three gate doc comments per §6; no gate logic change.
- **Depends on:** 4.
- **Tests:** U3–U8, G1 (full `MeshBooleanStats` with the switch on).
- **Done when:** `go test ./internal/meshbool/ -run 'TestKeepSideCoplanar|TestCoplanarGate|TestMeshBoolean' -count=1` passes.

### Task 6: Manifold audit and occupancy guard

- **Files:** `internal/meshbool/mesh_stitch.go`, `internal/meshbool/stitch_manifold.go` (new),
  `internal/meshbool/stitch_manifold_test.go` (new), `internal/booleaneval/eval.go`,
  `internal/booleaneval/occupancy_coplanar_test.go` (new).
- **What:** §5: `type StitchOptions struct { Coplanar *ContactKeys; SimplifyPlanar bool }`;
  `StitchFacetsWith(ctx, kept []KeptFacet, opts StitchOptions) (*StitchedMesh, error)`; `StitchFacetsContext`
  delegates with `StitchOptions{}`; `booleaneval` passes `stats.Coplanar` on the non-replay path. §6: `composeOccupancy`
  returns nil when `Stats.Coplanar != nil` (pass the flag in).
- **Depends on:** 5.
- **Tests:** M1, M2, G2.
- **Done when:** `go test ./internal/meshbool/ ./internal/booleaneval/ -run 'TestStitch|TestComposeOccupancy' -count=1` passes.

### Task 7: Planar face simplification

- **Files:** `internal/meshbool/planar_simplify.go` (new), `internal/meshbool/planar_simplify_test.go` (new),
  `internal/meshbool/mesh_stitch.go`, `internal/booleaneval/eval.go`.
- **What:** §7. `SimplifyPlanarFaces(ctx context.Context, in PlanarMesh) (PlanarMesh, bool, error)` with
  `type PlanarMesh struct { Xverts []proof.Xpt; Approx []r3.Vec; Xbeta []float64; Tris [][3]int; Src []int;
  FacetBound []float64 }`. It returns a new compacted mesh and whether it changed anything, and never modifies its
  input. `StitchFacetsWith` calls it when `opts.SimplifyPlanar` and applies §7.1 step 6's fallback. `booleaneval` sets
  `SimplifyPlanar = coplanarcontact.Of(ctx)` on the non-replay path.
- **Depends on:** 6.
- **Pattern refs:** `EarClipX` (`internal/meshbool/polygon_exact.go`); `ProjAxes`; `FacetAdjacencyContext`.
- **Tests:** S1–S5.
- **Done when:** `go test ./internal/meshbool/ -run 'TestSimplifyPlanar' -count=1` passes.

### Task 8: Benchmarks with the switch off and on

- **Files:** `internal/meshbool/coplanar_bench_test.go` (new), `internal/booleaneval/coplanar_bench_test.go` (new).
- **What:**
  - `internal/booleaneval`: `BenchmarkFlushUnion` (A1's pair through `Evaluate`, context carrying the default gap reach
    through `gaptol` so the retry ladder runs when off) and `BenchmarkCoplanarCaps` (two coplanar 64-gon caps,
    fan-triangulated, one rotated by one vertex), each with sub-benchmarks `off` and `on`.
  - `internal/meshbool`: `BenchmarkMeshBooleanCrossing` (§10), `off` and `on`; numbers only.
  - Report `benchstat` output in the PR description; apply §13 D2 if triggered.
- **Depends on:** 7.
- **Done when:** `go test ./internal/booleaneval/ -run '^$' -bench 'FlushUnion' -benchtime 5x -count=1` shows `on`
  faster than `off`, and `go test ./internal/meshbool/ ./internal/booleaneval/ -run '^$' -bench 'CoplanarCaps|MeshBooleanCrossing' -benchtime 5x -count=1` runs.

### Task 9: Planar simplification on the replay stitch

- **Files:** `internal/meshbool/brake_stitch_replay.go`, `internal/meshbool/planar_simplify.go`,
  `internal/meshbool/replay_simplify_test.go` (new), `internal/booleaneval/eval.go`.
- **What:** `StitchFacetsReplayWithKeys` runs `SimplifyPlanarFaces` and its fallback under the same switch, charging
  the replay budget one work unit per facet and vertex visited and per `EarClipX` step. Live and replay give the same
  held mesh for every boolean both build.
- **Depends on:** 8.
- **Pattern refs:** budget charges in `internal/meshbool/brake_stitch_replay.go`.
- **Tests:** `TestReplaySimplifyMatchesLive`: S1's fixture and a crossing box pair, replayed with the switch on, equal
  the live held mesh vertex for vertex; budget exhaustion returns the budget error.
- **Done when:** `go test ./internal/meshbool/ -run 'TestReplaySimplify|TestSimplifyPlanar' -count=1` passes.

### Task 10: Turn it on

- **Files:** `internal/coplanarcontact/` (delete), `internal/meshbool/boolean_pair_cuts.go`,
  `internal/meshbool/brake_stitch_replay.go`, `internal/booleaneval/eval.go`, `agent-docs/layout/internal-contact.md`;
  every test from Tasks 2–9 that calls `coplanarcontact.With`: `boolean_pair_cuts_coplanar_test.go` (drop
  `TestPairCutsCoplanarSwitchOff`), `keep_side_coplanar_test.go`, `tangency_gate_coplanar_test.go`,
  `stitch_manifold_test.go`, `occupancy_coplanar_test.go`, `planar_simplify_test.go`, `replay_simplify_test.go`, and
  both `coplanar_bench_test.go` files (keep the `on` path only); every test flipped per §11.3;
  `apitest/coplanar_contact_test.go` (new: the rounding-rotation fixture helper and A8 below);
  `.github/test-shards.txt`; `.github/test-shards-apitest.txt`; `agent-docs/interference-design.md` (§5.2, §6.1 premise
  P2 and Step 1, §9 table, §10.2 coplanar bullet, §11 row 4); `agent-docs/evaluator-design.md` (§9 coplanar
  paragraph); `agent-docs/gap-tolerance-design.md` (§5.2 lead: the retry is reached only for contacts on faces with a
  positive bound or for non-manifold contact; §6; §7 rows moved); `agent-docs/faceted-vertex-bounds-design.md` (§3.4:
  pointer to §7 of this doc); `agent-docs/missing-features.md` (coplanar refusal row); `docs/api.md` (only where
  `grep -n -i 'coplanar\|flush' docs/api.md` finds the refusal stated).
- **What:** Remove the switch and its reads, so §3, §5 and §7 always run on the non-replay path and §7 on the replay
  stitch. Flip tests per §11.3. Add A8: B = A1's box turned by §11.3's rounding rotation; it keeps today's retry and
  `CodeGapShiftApplied`. Update owning docs to current state.
- **Depends on:** 9.
- **Tests:** §11.3 flips; A8.
- **Done when:** `go test ./... -count=1`, `go vet ./...`, `golangci-lint run` and `go test . ./apitest/ -run '^TestCI' -count=1` pass, and §11.4's acceptance run passes.

### Task 11: API coverage

- **Files:** `apitest/coplanar_contact_test.go`, `.github/test-shards-apitest.txt`.
- **What:** A1–A7, R1–R4, P1, P2 (§11.2), as `TestCoplanarContact*` functions.
- **Depends on:** 10.
- **Done when:** `go test ./apitest/ -run 'TestCoplanarContact' -count=1` and `go test . ./apitest/ -run '^TestCI' -count=1` pass.

### Task 12: Bounded replay parity for coplanar contacts

- **Files:** `internal/meshbool/pair_cuts.go`, `internal/meshbool/brake_boolean_replay.go`,
  `internal/meshbool/brake_cutter_replay.go`, `internal/meshbool/brake_conform_replay.go`,
  `internal/meshbool/brake_memo_replay.go`, `internal/meshbool/brake_stitch_replay.go`,
  `internal/meshbool/brake_static_refusals.go`, `internal/meshbool/contact_replay_coplanar.go`,
  `internal/booleaneval/eval.go`, `internal/meshbool/replay_coplanar_test.go` (new).
- **What:** Port §3 and §5 to the bounded collector, cutter, conform and stitch with the surrounding replay code's
  budget charges; set `Exact` on the replay path under Task 2's condition; remove the `EdgeCrosses` wrapper once nothing
  calls it. A replayed coplanar boolean gives the same held mesh as the live one.
- **Depends on:** 11.
- **Pattern refs:** `boundedPairCuts` and `PairCutsBudget` in `internal/meshbool/pair_cuts.go`.
- **Tests:** `TestReplayCoplanarMatchesLive`: A1, A5 and R3 replayed equal the live held mesh key for key; budget
  exhaustion on a coplanar pair returns the budget error.
- **Done when:** `go test ./internal/meshbool/ ./internal/booleaneval/ -run 'TestReplay' -count=1` and `go test . -run 'Tessellate|BooleanGroup' -count=1` pass.

## 15. Open questions

- **Q1.** Should a flush `Union` merge two coplanar faces of different operands into one face? Decide between leaving
  two faces (current design; edge `FacetedCurve`) and a merge pass in `facetedrecord` (one face, but a new topology
  rule and retained-tag merge), because later face selection and fillets see the difference.
- **Q2.** Should §7 handle holed faces? Decide between leaving them untouched (current design) and an exact hole bridge
  before `EarClipX`, because a plate with a through hole cut by a mesh boolean stays inexact until then.
- **Q3.** Should the branch refusal of §3.2 ("Remaining refusals") be lifted with a planar-arrangement cutter? Decide
  after Task 11's P2 and the acceptance run show whether any real case hits it, because the cutter is a separate,
  larger change to `boolean_cut.go`.
