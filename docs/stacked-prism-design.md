# Stacked Prism Design

The `stackedPrismPayload`: an axial stack of recorded sections over one sketch
plane, and the two constructions that build it — a blind `Cut` of a straight
prism by a same-plane prism tool whose sweep ends inside the target, and a
`Union` of two co-directional prisms or stacked prisms whose sweep intervals
overlap or touch without being equal (`docs/general-boolean-design.md` §3 A1).
`docs/modify-reach-design.md` §9.1 records the prism shells on it — the cup
and the both-caps shell of a holed section (§2.4); this document owns the
payload itself (its record,
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

`internal/stackedrecord` owns `Slab` and `Interface`. A slab records its regions,
its axial interval `Z0`–`Z1`, and the two levels' displacement bounds. An
interface records `LowerExposed` and `UpperExposed`, the material exposed on
one side of its plane. A slab holds one region, or several under §2.2's group
and lining readings. The root payload holds `[]stackedrecord.Slab` and
`[]stackedrecord.Interface` beside its frame, placement and section bound.
There is exactly one interface between each pair of adjacent slabs.

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

`stackedrecord.Falsify` checks every one of these before a body is built and
before a mesh is chorded. A record that fails one is refused — `ErrDegenerate`
for a record no stacked body matches, `ErrUnsupported` for a shape this
evaluator does not build — never repaired.

| # | Invariant | Sentinel |
|---|---|---|
| I1 | At least two slabs, or one slab holding two or more regions (a prism group, below). A one-slab stack with one region is a prism and is recorded as `prismPayload`. | `ErrUnsupported` |
| I2 | Each slab holds exactly one region, except a prism group's one slab, whose regions are proven pairwise disjoint when the group is built, and the narrow side of a lining interface (below). (A region enclosing no area is refused by the build's own integrals, `ErrDegenerate`.) | `ErrUnsupported` |
| I3 | Each slab has `z0 < z1`. | `ErrDegenerate` |
| I4 | Consecutive slabs meet: `slabs[i].z1 == slabs[i+1].z0` and `slabs[i].z1Delta == slabs[i+1].z0Delta`, both as stored floats. One plane, one displacement. | `ErrDegenerate` |
| I5 | Consecutive slabs carry one outer loop record (`loopRecordsEqual`), or their interface meets the union reading below. A clean-nesting cut never touches the outer loop, and a mirror join rewrites every slab's outer the same way, so a cut-built stack's outer wall runs the whole height. | `ErrUnsupported` |
| I6 | Hole sets are monotone, or one new hole replaces enclosed old holes through §7's two-cell sketch proof. | `ErrUnsupported` |
| I7 | A monotone interface exposes each exclusive hole's interior; an enclosing interface exposes one patch with the new hole's reversed loop as outer and the enclosed old holes as holes. | `ErrDegenerate` |

For a monotone interface, every hole of either region equals a hole on the
other side or occurs on that side alone. Only one side has exclusive holes.
Each exposed patch reverses one exclusive hole into an outer loop with no
holes. Reversal rebuilds segments from their walks; the audit accepts either
`outer == reverse(hole)` or `reverse(outer) == hole` for a shell offset loop.

For an enclosing interface, both sides have exclusive holes. One side has
exactly one new hole, and the other has one or more old holes it encloses.
The cut's two whole-loop sketch cells prove the nesting. The patch on the
old-hole side reverses the new hole for its outer and carries the old holes
inside it. Other opposed-hole interfaces remain `ErrUnsupported`.

A mirror join (`docs/mirror-pattern-design.md` §5) builds a stacked payload by
rewriting every slab's region with one splice. The splice is a function of the
loop record alone, so equal loops in two slabs rewrite to equal loops: the
outer stays one record (I5) and every interface stays monotone (I6). The
join re-derives each interface's exposed records from the rewritten regions
(I7), and this audit checks all three again before the body is built. A
union-built stack, whose outer loop changes between slabs, is not joined.

A union-built stack reads I5–I7 per interface. Where the two outer loops are
one record, the rows above apply unchanged. Where they differ:

| # | Union reading | Sentinel |
|---|---|---|
| I5 | Both regions are hole-free, and the narrower outer is proven nested in the wider region by prism-boolean §4.2's clean-nesting match when the union is built: the wider region's cell carries the narrower outer, whole, as its one hole | `ErrUnsupported` |
| I6 | Exactly one side records exposed material: `lowerExposed` when the lower region is the wider one, `upperExposed` when the upper is | `ErrDegenerate` |
| I7 | That side holds exactly one record, `{Outer: wider.Outer, Holes: [reverse(narrower.Outer)]}`. The audit re-derives it from the two regions and compares it record for record | `ErrDegenerate` |

The audit checks the records the union reading names. The nesting itself is
proven once, by the match that built the union, as I6's monotone holes are
proven by the cut that built them.

An interface where one side holds several regions takes the **lining
reading**. It is what a shell's floor and wall slabs meet at (§2.4): the wide
side holds one region `W` with `k` holes, and the narrow side holds the
`1 + k` bands that line `W`'s loops.

| # | Lining reading | Sentinel |
|---|---|---|
| I5 | Exactly one side holds one region `W`; the other holds `1 + k` regions, `k` the number of `W`'s holes. The first narrow region's outer is `W`'s outer and it has exactly one hole; narrow region `m >= 1` has exactly one hole, `W`'s hole `m - 1` | `ErrUnsupported` |
| I6 | Only `W`'s side records exposed material | `ErrDegenerate` |
| I7 | That side holds exactly one record: its outer is the first narrow region's hole reversed and its holes, in order, are every other narrow region's outer reversed, each reversal in either spelling | `ErrDegenerate` |

That every narrow region lies in `W` and that the narrow regions are pairwise
disjoint is proven when the shell builds them, by the offset section's audit
(`docs/modify-design.md` §5); the audit here compares records. Each narrow
region's own loop — the first one's hole, every other one's outer — has no
equal loop on the wide side, so its column starts or ends at the interface,
and the one exposed patch reads those columns' rings there.

A **prism group** (`docs/mirror-pattern-design.md` §6.3) is one slab holding
two or more regions, each a separate lump over the slab's interval. It has
no interface, so I3 is its only level rule and I4–I7 do not apply. The audit
checks the slab, the empty interface list and that every region has an outer
loop; that the regions are pairwise disjoint is proven when the group is
built, by `provePrismRegionsDisjoint`: a private scene of every region's outer
must return one valid cell per region, each reproducing that outer with
every edge whole and no hole. A `Union` whose survivors close into several
such loops builds one (`docs/general-boolean-design.md` §3 A5).

### 2.3 Loop columns

A **column** is one loop record over a maximal run of consecutive slabs that
all carry it, in whichever region of each slab holds it (I5 and I6 make
"carry it" a `loopRecordsEqual` question). In a
cut-built stack the outer loop is one column over every slab. A union-built
stack has one outer column per run of slabs that share an outer record: a
boss's wall is one column over the slabs the boss alone reaches, and a plate's
wall is one column over the slabs the plate spans. A through hole is one
column over every slab. A pocket's wall is a column over the slabs the pocket
reaches, and a hole that vanishes and reappears is two columns.
`stackedColumns` derives the column list from the slabs alone, and the body
build and the tessellator both read it, so a wall is one face however many
slabs it crosses — evaluator §3's canonicalization rule — and both consumers
name the same face by the same role.

A column records its loop, the slab it starts in, the region holding the loop
there and the loop's index in that region, its sweep interval `[z0, z1]` (the first slab's `z0` to the last
slab's `z1`) and the two levels' displacements.

### 2.4 Shell records

`Shell` (`docs/modify-reach-design.md` §9) records two results on this
payload, with `O` the shell's outer region and `C` its cavity region: `P` and
its offset `Q` inward, `Q` and `P` outward. Its wall bands are the band between
`O`'s outer and `C`'s outer (`C`'s outer reversed as its one hole), then, per
hole `i`, the band between `C`'s hole `i` reversed (an outer) and `O`'s hole
`i`.

| Result | Record |
|---|---|
| both caps removed, `k >= 1` holes (reach BX8) | a prism group: one slab over the receiver's sweep holding the `1 + k` bands; `sectionDelta` is the offset's proven displacement |
| no opening, `k = 0` (reach BX5, `WithNoOpenings`) | three slabs: the cap region (`P` inward, `Q` outward) under the band `{O.Outer, reverse(C.Outer)}` under the cap region, both interfaces monotone, each exposing `C`; `sectionDelta` is the offset's proven displacement; the cavity is a void shell (§3) |
| one cap removed, any `k` (a cup, modify B5/B6) | a floor slab over `O` and a wall slab over the bands, meeting at a lining interface (`k >= 1`) or a monotone one (`k = 0`) whose one exposed record is `C` itself; `sectionDelta` is zero |

A cup is not handed over as a bare `stackedPrismPayload`: `cupPayload` holds
the record beside its shell morphology, and its consumers read the cup's own
view of it (reach §9.1). Its offset loops carry the offset displacement column
by column (§4).

## 3. Topology and roles

`evalStackedContext` builds the body under the boolean's own private producer
identity, after `stackedrecord.Falsify`. It builds through the payload's own
naming plan; a cup builds the same record through its own plan
(`stackedPlan`), which mints the cup's roles (modify Table B) and states each
column's displacement:

| Face | Built from | Surface | Role |
|---|---|---|---|
| wall | one column, through `buildLoopSidesAs` over a `prismPayload` view of that column (the payload's frame, placement and `sectionDelta`; the column's interval and level displacements) | `Plane` or `Cylinder` per segment | `slab(k).region(r).side(i,j)` — `k` the slab the column starts in, `r` the region (0 outside a prism group), `i` the loop's index in that region, `j` the segment |
| bottom cap | the first slab's region | `Plane` at `slabs[0].z0`, outward `-N` | `capStart` |
| top cap | the last slab's region | `Plane` at the last `z1`, outward `+N` | `capEnd` |
| floor | `interfaces[i].lowerExposed[e]` | `Plane` at the interface level, outward `+N` | `floor(i,e)` |
| ceiling | `interfaces[i].upperExposed[e]` | `Plane` at the interface level, outward `-N` | `ceiling(i,e)` |

A union-built interface's one patch has two loops: its outer is the wider
side's outer column, read at that column's end on the interface, and its hole
is the narrower side's outer column, read at that column's start or end there.
`stackedInterfacePatches` lists every patch with its column rings, and the
body build and the tessellator both read that list.

Every planar face's loops are the coedges the column walls already placed at
that level, so every edge bounds exactly two faces: a column that starts in
slab 0 gives its bottom coedges to `capStart`, one that starts at an interface
gives them to that interface's floor; a column that ends in the last slab gives
its top coedges to `capEnd`, one that ends at an interface gives them to that
interface's ceiling. `stackedLumps` derives the lump set from face adjacency:
each connected face set holding a region outer's wall or an end cap is one
lump's outer shell, and a connected set holding neither — hole walls closed
by a floor below and a ceiling above, a closed shell's cavity (§2.4) — is a
void shell of the one outer lump. Several outer lumps beside a cavity would
need a nesting proof and are `ErrUnsupported`. A stage-1 body is one lump
with one shell.

A prism group carries one bottom cap and one top cap per region, every one
under `capStart` or `capEnd`, so `CapStart(body)` selects one face per lump,
and `Lumps()` reports one lump per region. Columns, measurements and
tessellation run per region; loop clearance is proven over every ring of the
one slab, all regions together.

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
| `Centroid` | `Σ_k m_k · c_k / Σ_k m_k`, with `m_k = A_k · h_k` and `c_k` the region's centroid lifted to the slab's bounded midpoint through `prismPayload.point`; taken component by component in bounded arithmetic, each `c_k` component carrying `prismPointBound`'s radius as its bound, and the three component bounds folded into one radius through `radius3D`. `prismCentroidGeometryBound`, the largest over the outer runs, plus `sectionDelta`, caps the result as `evalPrism`'s does |
| `Bounds` | the union of `prismBoundsContext` over each outer run: one outer-only prism per run of slabs sharing an outer record, over that run's interval with its end levels' displacements and the payload's `sectionDelta`, the largest run bound covering the union. A cut-built stack is one run, the outer-only prism on the full interval |
| `extentAlong` | the extreme over every outer run's reading |
| `axialDelta` | the largest level displacement over every slab |

A column whose plan states a displacement (a cup's offset loop, §2.4) builds
its walls with that displacement added to `sectionDelta`, so its vertices,
lengths and face areas carry it; each region holding the loop adds
`sectionDisplacementArea` over that loop alone to its area and widens its
first moments by that area times the farthest coordinate the displaced
boundary reaches, so the centroid quotient covers it; each planar patch the
loop bounds adds the same area term; and every outer run whose outer loop is
displaced reads its box with that displacement. The geometric centroid cap is
then a ceiling (`math.Min`) on the covered formula answer. A zero column
displacement adds no term, so a boolean-built stack's readings are unchanged.

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
  reversed (`-N`). A union-built patch triangulates the wider outer column's
  ring as its outer and the narrower outer column's ring, reversed, as its
  hole. Its two rings come from two slabs, so `requireLoopClearance` runs over
  the patch's own rings before it is triangulated. Every planar patch indexes the already allocated ring
  vertices, so the mesh closes by construction and `internal/tessellation.RequireClosedMesh` proves it.
- Face bounds: a wall's largest sagitta plus the larger of its column's two
  level displacements; a planar patch's largest bounding-loop sagitta plus its
  own level's displacement; `composeFaceBounds` adds every vertex's store term
  and `sectionDelta`.
- `areaSlack`: per column `wallSlack + 2 · capSlack` (every column ring bounds
  exactly two planar patches, one at each end); when `sectionDelta > 0`, one
  `sectionDisplacementArea` term per planar patch over that patch's own loops
  and one `sectionDisplacementLength · height` term per column; then
  `tessellation.StoreAreaAllow`.
- Occupied volume (`publishSymDiff`): `Σ_k h_k · E_k`, with `E_k` the summed
  circular-segment area of the columns slab `k` carries; `Σ_k h_k ·
  sectionDisplacementArea(sectionDelta, walks_k, perimeter_k)`; each planar
  patch's level displacement times `tessellation.FaceAreaUpper` of that patch; and
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
| later `Cut` with a prism tool | A spanning tool builds when each slab proves a clean cut or no change inside a hole; other tools take the mesh path (§7). |
| `Union` with a stacked operand | `docs/general-boolean-design.md` §3 A1: every slab region hole-free, the stack splits at every level of both operands; a prism-group operand over the partner's interval is A5's |
| `Cut` by a prism-group tool | `docs/general-boolean-design.md` §3 A5 on a prism target: one arrangement for every lump |
| `Intersect` with a stacked operand | mesh path, over this payload's own tessellation |
| `Tessellate`, `export.STL` / `OBJ` / `STEP` | §5; a body with one shell meets STEP's one-shell rule, and a closed shell's void shell is STEP's refusal |
| `ThroughAll` / `ThroughAllSide` stops | `extentAlong` (§4), `ErrUnsupported` at `sectionDelta > 0` as for a prism |
| `ToFace` stops | read the selected face's own `axialDelta` |
| `Verify` structural audit | every edge bounds two faces by construction (§3) |
| `Verify` tolerance gate | `gateWitnessPrisms` reads every outer run's prism over its own interval, whose wall stations are all points of the body, and shrinks their maximum by `sectionDelta + axialDelta` plus the widest station gap (`docs/verification-design.md` §3) |
| `Verify` wall survey | staged: `DiagUnsupportedSurveyPayload`, `Suspect` (modify-reach Table DX, DX9); a pocket floor is a wall the 2D spanning-disk proof does not read |
| `Verify` undercut and minimum-radius surveys | staged: `DiagUnsupportedSurveyPayload`, `Suspect` (stage 4 adds DX7 face normals and DX8 `radiussurvey.Prism` over each loop) |
| `Verify` clearance | `newBodyGeomBudget` has no arm, so a pair the boxes do not separate reads `Suspect`; a box-disjoint pair is proven (stage 4 adds the exposed-face model, DX6) |
| `Verify` interference | `analyticBodiesEqual` answers undecided; the read-only mesh intersection reads §5's proof |
| `Fillet` / `Chamfer` / `Shell` | through the brep face view (modify-reach RX3); a prism group and a stack enclosing a cavity have none (brep-modify SB2) |
| `Thicken` / `Offset` / `Patch` / `Stitch` | not reachable: the body is a solid |

## 7. Stages

Each admitted arm ships its implementation and tests together.

1. **Blind cut and through cuts on the result.** The payload, `evalStacked`,
   `tessellateStacked` with its occupied-volume proof, prism-boolean §3.2's two
   new `Cut` rows (a blind tool on a prism target, a spanning tool on a stacked
   target), the `Verify` gate arm, `sectionDisplacementOf`'s arm, the explicit
   stagings in §6, and an executable example. The gallery's landing clip cuts
   one blind hole per frame and renders the body, so the tessellator ships in
   the same stage as the cut that produces the payload.
2. **Enclosing counterbore.** A blind tool around existing holes builds when
   sketch returns the target's outside cell with the tool as one hole and a
   second cell inside the tool with precisely the enclosed holes. The exposed
   floor or ceiling is `{Outer: reverse(tool), Holes: enclosed holes}`. A
   spanning tool inside a pre-existing blind hole leaves that slab unchanged
   when a private scene returns both the unchanged material cell and the
   annulus between the existing hole and the tool. A spanning cut on the other
   slabs then builds the same counterbore in the opposite construction order.
3. **Blind cut on a stacked target (staged).** A tool ending inside a stacked
   target must split the slab its end falls in and clean-nest in every slab it
   reaches. An interface with unrelated exclusive holes on both sides needs a
   private scene proving that the two hole sets occupy separate whole cells.
   Until that proof and the slab split are built, the pair takes the mesh path.
4. **Surveys and clearance.** DX7, DX8, DX6 and `analyticBodiesEqual` for the
   stacked payload.

Not planned here: a blind tool crossing the target's boundary (a side notch),
whose per-slab walls split at the crossing and need column-wise edge splitting;
`Fillet`/`Chamfer` on a stacked body (`docs/brep-modify-design.md` takes it
through the face view).

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
- A blind tool on a stacked target takes the mesh path (stage 3 lifts it).
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
