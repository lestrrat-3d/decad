# Stacked Prism Design

The `stackedPrismPayload`: an axial stack of recorded sections over one sketch
plane, and the one construction that builds it today — a blind `Cut` of a
straight prism by a same-plane prism tool whose sweep ends inside the target.
`docs/modify-reach-design.md` §9.1 names this payload as the record a general
prism shell will migrate to; this document owns the payload itself (its record,
its invariants, how its body is built and measured, how it tessellates, and
what every consumer does with it), and `docs/prism-boolean-design.md` owns the
boolean admission that produces one. References of the form "core §N" are to
`docs/api-design.md`, "prism-boolean §N" to `docs/prism-boolean-design.md`,
"modify-reach §N" to `docs/modify-reach-design.md`, "tessellation §N" to
`docs/tessellation-design.md`.

The blind cut reads prism-boolean §3.1's G3 with its shared-axis offset-plane
arm and its exact rational G5 shift: a tool sketched on the target's datum
plane, or on a `CreateOffsetPlane` of it, is compared over `big.Rat`, and the
one level the tool contributes to the result (its inner end) is that exact
rational rounded to a float once, charged with `rationalFloatError`.

## 1. Problem

A `Cut` whose tool does not span the target misses prism-boolean §3.1's G5 and
takes the mesh path. Its result is a `facetedPayload`, so every later `Cut` on
the body misses G1 and takes the mesh path too, and the mesh path's chain depth
(core §8) refuses the second through hole after a blind bore:

```
decad: not supported by the current evaluator: requested tolerance 0.00268 mm
is below the faceted body's minimum mesh bound 0.00509 mm; ...
```

A plate with a blind hole is an ordinary part. The two natural ways to draw its
tool — sketched on the plate's top face pointing down, or on the plate's own
plane pointing up to the depth — both end in a plate face, which the mesh path
refuses outright as a coplanar contact. A blind hole therefore needs an analytic
result, and the result is not a prism: it has two different sections at two
different heights.

## 2. The payload

### 2.1 Record

```go
type prismSlab struct {
    regions          []ProfileRecord // one region today (§2.2)
    z0, z1           float64         // evaluator coordinates on the frame's normal
    z0Delta, z1Delta float64         // each level's proven axial displacement
}

type prismSlabInterface struct {
    lowerExposed []ProfileRecord // material below the plane only: a floor, outward +N
    upperExposed []ProfileRecord // material above the plane only: a ceiling, outward -N
}

type stackedPrismPayload struct {
    slabs        []prismSlab
    interfaces   []prismSlabInterface // exactly len(slabs)-1; interfaces[i] lies between slabs[i] and slabs[i+1]
    frame        r3.Frame
    xform        r3.Transform
    sectionDelta float64
}
```

Material is the union of every slab's regions swept over that slab's interval.
The payload is evaluator-private, re-evaluates under `Body.Placed` through the
same `featurePayload` interface every other payload implements, and carries the
accumulated placement in `xform` exactly as `prismPayload` does.

`sectionDelta` is one bound for the whole payload: the largest section
displacement any slab's region carries (prism-boolean §7). Every reading that
charges a section displacement charges this one.

Modify-reach §9.1 wrote a `Shared` field on the interface for the material on
both sides of the plane. It is not stored: that material is the narrower side's
own region, nothing emits a face for it, and storing it would add an invariant
the audit below would have to re-check without any consumer reading it.

### 2.2 Invariants

`falsifyStackedPayload` checks every one of these before a body is built and
before a mesh is chorded. A record that fails one is refused — `ErrDegenerate`
for a record no stacked body matches, `ErrUnsupported` for a shape this
evaluator does not build — never repaired.

| # | Invariant | Sentinel |
|---|---|---|
| I1 | At least two slabs. A one-slab stack is a prism and is recorded as `prismPayload`. | `ErrUnsupported` |
| I2 | Each slab holds exactly one region. Multi-region slabs are modify-reach §9.1's shell migration and are not built. (A region enclosing no area is refused by the build's own integrals, `ErrDegenerate`, as `evalCup` refuses one.) | `ErrUnsupported` |
| I3 | Each slab has `z0 < z1`. | `ErrDegenerate` |
| I4 | Consecutive slabs meet: `slabs[i].z1 == slabs[i+1].z0` and `slabs[i].z1Delta == slabs[i+1].z0Delta`, both as stored floats. One plane, one displacement. | `ErrDegenerate` |
| I5 | Every slab's outer loop is the same record (`loopRecordsEqual`). A clean-nesting cut never touches the outer loop, so the outer wall runs the whole height. | `ErrUnsupported` |
| I6 | Each interface is **monotone**: every hole of the lower region either equals (`loopRecordsEqual`) a hole of the upper region or is lower-only; every hole of the upper region either equals a hole of the lower region or is upper-only; and lower-only and upper-only holes do not both exist at one interface. | `ErrUnsupported` |
| I7 | `interfaces[i].lowerExposed` holds exactly one record per upper-only hole — `{Outer: reverse(hole)}` — and `upperExposed` one per lower-only hole, the same way. The audit re-derives both lists from the two regions and compares them record for record. | `ErrDegenerate` |

I6 is what lets the interface be recorded without a planar boolean. The material
on both sides of the plane is the narrower region; the exposed material is each
exclusive hole's own interior, which `reverse(hole)` states as a region with
material inside. An interface where both sides have exclusive holes needs the
two hole sets proven disjoint before the exposed regions can be stated, and
nothing in this evaluator proves that yet (§7, stage 2).

### 2.3 Loop columns

A **column** is one loop record over a maximal run of consecutive slabs that
all carry it (I5 and I6 make "carry it" a `loopRecordsEqual` question). The
outer loop is one column over every slab. A through hole is one column over
every slab. A pocket's wall is a column over the slabs the pocket reaches, and
a hole that vanishes and reappears is two columns. `stackedColumns` derives the
column list from the slabs alone, and the body build and the tessellator both
read it, so a wall is one face however many slabs it crosses — evaluator §3's
canonicalization rule — and both consumers name the same face by the same role.

A column records its loop, the slab it starts in, that loop's index in that
slab's region, its sweep interval `[z0, z1]` (the first slab's `z0` to the last
slab's `z1`) and the two levels' displacements.

## 3. Topology and roles

`evalStackedContext` builds the body under the boolean's own private producer
identity, after `falsifyStackedPayload`:

| Face | Built from | Surface | Role |
|---|---|---|---|
| wall | one column, through `buildLoopSidesAs` over a `prismPayload` view of that column (the payload's frame, placement and `sectionDelta`; the column's interval and level displacements) | `Plane` or `Cylinder` per segment | `slab(k).region(0).side(i,j)` — `k` the slab the column starts in, `i` the loop's index in that slab's region, `j` the segment |
| bottom cap | the first slab's region | `Plane` at `slabs[0].z0`, outward `-N` | `capStart` |
| top cap | the last slab's region | `Plane` at the last `z1`, outward `+N` | `capEnd` |
| floor | `interfaces[i].lowerExposed[e]` | `Plane` at the interface level, outward `+N` | `floor(i,e)` |
| ceiling | `interfaces[i].upperExposed[e]` | `Plane` at the interface level, outward `-N` | `ceiling(i,e)` |

Every planar face's loops are the coedges the column walls already placed at
that level, so every edge bounds exactly two faces: a column that starts in
slab 0 gives its bottom coedges to `capStart`, one that starts at an interface
gives them to that interface's floor; a column that ends in the last slab gives
its top coedges to `capEnd`, one that ends at an interface gives them to that
interface's ceiling. The cup build (`shell_cup.go`) already pairs a reversed
cavity wall's floor coedges with its pocket floor this way. `sheetLumps` derives
the lump set from face adjacency, as `evalPrism` does; a stage-1 body is one
lump with one shell.

`capStart` and `capEnd` keep their prism names so `CapStart(body)` and
`CapEnd(body)` select them. Roles are fresh under the boolean's step and no
operand `FeatureRef` is carried forward (prism-boolean §11).

## 4. Measurements

Every quantity is a bounded sum of per-slab or per-column terms through
`internal/proofbound/bounded.go`'s arithmetic, so every float operation between a proven term and
the published value charges its own rounding. No new `internal/proofbound/bounds.go` helper is
needed.

| Quantity | Composition |
|---|---|
| `Volume` | `Σ_k A_k · h_k` — each slab's region area (with `sectionDisplacementArea(sectionDelta, walks_k, perimeter_k)` folded into its bound, as `evalPrism` does) times its bounded height |
| `Area` | the first region's area (bottom cap) + the last region's area (top cap) + each exposed record's area (`loopEnclosedAreaContext` of its outer loop) + `Σ_columns perimeter · height` |
| `Centroid` | `Σ_k m_k · c_k / Σ_k m_k`, with `m_k = A_k · h_k` and `c_k` the region's centroid lifted to the slab's bounded midpoint through `prismPayload.point`; taken component by component in bounded arithmetic, each `c_k` component carrying `prismPointBound`'s radius as its bound, and the three component bounds folded into one radius through `radius3D`. `prismCentroidGeometryBound` over the outer-only prism on the full interval, plus `sectionDelta`, caps the result as `evalPrism`'s does |
| `Bounds` | `prismBoundsContext` over the outer-only prism (`{Outer: slabs[0].regions[0].Outer}`) on the full interval with the end slabs' own level displacements and the payload's `sectionDelta` — every slab's region lies inside its outer loop (I5) |
| `extentAlong` | the outer-only prism's reading, as `cupPayload.extentAlong` reads its outer prism |
| `axialDelta` | the largest level displacement over every slab |

`Exactness` follows `exactnessOf` on each composed bound: a rectangular pocket in
a rectangular plate drawn on one plane reports `Exact` volume with a zero bound;
any circular wall, any nonzero level displacement or any nonzero `sectionDelta`
reports `Approximate`.

## 5. Tessellation

`tessellateStacked` (tessellation §5's prism contract, applied per column and
per planar patch):

- Reserve `sectionDelta` from the requested tolerance exactly as the prism
  arm does, through the same helper, and refuse a tolerance it exhausts.
- Chord each column's loop once (`chordLoop` over the column's height). Its
  samples get one vertex at the column's `z0` and one at its `z1`; its walls
  are the prism's outward quads. Chording per column is modify-reach §9.1's
  "once per shared carrier".
- Prove loop clearance (`requireLoopClearance`) **per slab**, over the rings of
  the columns that slab carries. Two pockets from opposite faces may overlap in
  plan while never sharing a slab; a whole-body clearance proof would refuse a
  valid body.
- Triangulate `capStart` from slab 0's rings at their `z0` (reversed, `-N`),
  `capEnd` from the last slab's rings at their `z1` (`+N`), each floor from its
  upper-only column's bottom ring with the hole's winding reversed (`+N`), and
  each ceiling from its lower-only column's top ring, reversed, emitted
  reversed (`-N`). Every planar patch indexes the already allocated ring
  vertices, so the mesh closes by construction and `internal/tessellation.RequireClosedMesh` proves it.
- Face bounds: a wall's largest sagitta plus the larger of its column's two
  level displacements; a planar patch's largest bounding-loop sagitta plus its
  own level's displacement; `composeFaceBounds` adds every vertex's store term
  and `sectionDelta`.
- `areaSlack`: per column `wallSlack + 2 · capSlack` (every column ring bounds
  exactly two planar patches, one at each end); when `sectionDelta > 0`, one
  `sectionDisplacementArea` term per planar patch over that patch's own loops
  and one `sectionDisplacementLength · height` term per column; then
  `meshStoreAreaAllow`.
- Occupied volume (`publishSymDiff`): `Σ_k h_k · E_k`, with `E_k` the summed
  circular-segment area of the columns slab `k` carries; `Σ_k h_k ·
  sectionDisplacementArea(sectionDelta, walks_k, perimeter_k)`; each planar
  patch's level displacement times `meshFaceAreaUpper` of that patch; and
  `sweptVolumeAllow` over the store maximum. The body is the disjoint union of
  its slab prisms and the mesh solid is the disjoint union of the chorded slab
  prisms over the same intervals, so the symmetric difference is at most the
  sum of the per-slab symmetric differences, each of which is tessellation §5's
  own prism bound.

The mesh carries the occupied-volume proof, so a stacked body is an ordinary
mesh-path boolean operand and an ordinary export input. `pairChordTolerance`
reads its `sectionDelta` through `sectionDisplacementOf`, as it reads a prism's.

## 6. Consumers

| Consumer | Behaviour |
|---|---|
| `Body.Placed` / `Duplicate` / `PlacedCopy` | re-evaluates the payload under the composed motion |
| later `Cut` with a prism tool | prism-boolean §3.2's stacked-target row: a tool spanning the whole stack and clean-nesting in every slab's region builds a stacked result (stage 1); a tool ending inside the stack, or one touching any slab's boundary, takes the mesh path |
| `Union` / `Intersect` with a stacked operand | mesh path, over this payload's own tessellation |
| `Tessellate`, `export.STL` / `OBJ` / `STEP` | §5; one shell, so STEP's one-shell rule is met |
| `ThroughAll` / `ThroughAllSide` stops | `extentAlong` (§4), `ErrUnsupported` at `sectionDelta > 0` as for a prism |
| `ToFace` stops | read the selected face's own `axialDelta` |
| `Verify` structural audit | every edge bounds two faces by construction (§3) |
| `Verify` tolerance gate | `gateWitnessPrism` reads the outer-only prism on the full interval, a shape containing the whole body, and shrinks its witness maximum by `sectionDelta + axialDelta` (`docs/verification-design.md` §3) |
| `Verify` wall survey | staged: `DiagUnsupportedSurveyPayload`, `Suspect` (modify-reach Table DX, DX9); a pocket floor is a wall the 2D spanning-disk proof does not read |
| `Verify` undercut and minimum-radius surveys | staged: `DiagUnsupportedSurveyPayload`, `Suspect` (stage 3 lifts both: DX7's exact per-face normals over the columns and planar patches, DX8's `prismMinRadius` over the outer loop plus every column's hole loop) |
| `Verify` clearance | `newBodyGeomBudget` has no arm, so a pair the boxes do not separate reads `Suspect`; a box-disjoint pair is proven (stage 3 adds the exposed-face model, DX6) |
| `Verify` interference | `analyticBodiesEqual` answers undecided; the read-only mesh intersection reads §5's proof |
| `Fillet` / `Chamfer` / `Shell` | `ErrUnsupported` (modify-reach RX3 / SX10) |
| `Thicken` / `Offset` / `Patch` / `Stitch` | not reachable: the body is a solid |

## 7. Stages

Each stage ships its implementation and tests together; no stage is a design
change alone.

1. **Blind cut and through cuts on the result.** The payload, `evalStacked`,
   `tessellateStacked` with its occupied-volume proof, prism-boolean §3.2's two
   new `Cut` rows (a blind tool on a prism target, a spanning tool on a stacked
   target), the `Verify` gate arm, `sectionDisplacementOf`'s arm, the explicit
   stagings in §6, and an executable example. The gallery's landing clip cuts
   one blind hole per frame and renders the body, so the tessellator ships in
   the same stage as the cut that produces the payload.
2. **Blind cut on a stacked target, and the enclosing tool.** A tool ending
   inside a stacked target splits the slab its end falls in (or lands on an
   existing interface plane) and must clean-nest in every slab it reaches. An
   interface whose two sides would both hold exclusive holes is admitted only
   when a private scene of the two hole loops arranges them into two separate
   whole cells, neither carrying the other as a hole — sketch's own answer, read
   structurally — and otherwise falls back. A tool whose section encloses one or
   more of the target's holes (a counterbore over a through hole) resolves
   through a second structural match: the target's cell keeps the holes outside
   the tool plus the tool, and one further cell reproduces the tool's outer with
   the enclosed holes as its own, so the exposed floor is `{Outer:
   reverse(tool), Holes: enclosed holes}` and I7 generalizes to carry them.
3. **Surveys and clearance.** DX7, DX8, DX6 and `analyticBodiesEqual` for the
   stacked payload.

Not planned here: a blind tool crossing the target's boundary (a side notch),
whose per-slab walls split at the crossing and need column-wise edge splitting;
`Fillet`/`Chamfer` on a stacked body; the cup migration of modify-reach §9.1.

## 8. Required tests

Every test asserts on computed geometry. Bounds are asserted as relations and
ceilings, never as literals, because FMA rounding differs between amd64 and
arm64.

- A rectangular blind pocket in a rectangular plate, both drawn on one plane,
  from each end: the result is analytic (no `Faceted` face), has one lump and
  eleven faces, every edge bounds two faces, and its `Exact` volume equals
  the rational answer. Its centroid bound contains the rational answer;
  the Z coordinate is `Approximate` when that answer is not a float.
- A round blind bore: `Approximate` volume within the published bound of
  `A·h − π r² d`, and the bound below `1e-9` of the value.
- Blind bore then two through bolt holes: all three build analytically, the
  volume lands within its bound of the closed form, and the face count is ten.
- A tool whose cap lies exactly in the target's far face builds (the mesh path
  refused it as a coplanar contact).
- A tool touching the target only at a cap (zero depth), a tool strictly inside
  both ends (an enclosed void), and a blind tool crossing the outer boundary
  each still take the mesh path with the mesh path's own result.
- A blind tool on a stacked target takes the mesh path (stage 2 lifts it).
- Level displacement, incoming: a tool whose inner end is a converted magnitude
  (`units.Inches`) publishes a positive axial delta on that end, asserted first
  on the tool so the fixture cannot silently stop exercising it, and the
  result's volume bound is at least that delta times the floor's area.
- Level displacement, rounding: a tool sketched on `CreateOffsetPlane(XY,
  0.1)` and extruded `Symmetric` 0.3 mm into a plate on XY puts its inner end at
  the exact rational `fl(0.1) + fl(0.3)`, which is no float; the result's
  ceiling level is that rational rounded once and its delta equals
  `rationalFloatError` of the rounding, asserted positive. A tool extruded
  `Distance` from an offset plane puts its inner end exactly on that plane
  (`0 + s`, a float), so its delta is exactly zero.
- Section displacement: a tool placed in its plane (nonidentity re-expression)
  yields `sectionDelta > 0`, an `Approximate` volume even for a rectangular
  pocket, and a bound containing the exact rational residual.
- Payload audits I1–I7, each on a hand-built payload, each refusing with the
  stated sentinel.
- Tessellation at `VerifyAll`: the mesh closes, `Bound` is at or below the
  requested tolerance plus the axial maximum, and a mesh-path `Cut` by a
  non-coplanar tool on the pocketed plate builds, so the occupied-volume proof
  is published.
- Export: STEP of the pocketed plate writes one shell.
- `Verify` on the pocketed plate is `Sound` with no survey asked; with the wall
  survey asked it reports `DiagUnsupportedSurveyPayload` and `Suspect`.
- `Fillet` on the pocketed plate is `ErrUnsupported`.
- `Placed` by a rotation keeps the volume, exactness and face count.
- Cancellation during the second slab's arrangement returns `ctx.Err()` and
  leaves both operands live.
