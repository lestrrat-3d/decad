# Twisted-Sweep Cut: Stationed Refinement and Chained Cuts

`Cut`, `Union` and `Intersect` of a twisted `Sweep` against a prism or
cylinder. §1–§6 cover one twisted tool and its stationed refinement; §7 covers
a body cut by several twisted tools and prism windows in a chain, which also
needs a finer chord for the partner. Companion to `agent-docs/sweep-design.md`
§17 (the twisted Sweep), `agent-docs/prism-boolean-design.md` Part G (the mesh
path), `agent-docs/tessellation-design.md` §11 step 4 (the tangency gate),
`agent-docs/faceted-vertex-bounds-design.md` §3 and §5 (the bound composition
and the chain-depth gate) and `agent-docs/api-design.md` §8 "The chain depth".

## 1. The problem

A twisted rectangular `Sweep` tool bores a tube wall. The
repro is a 15.4 x 4.75 mm centred rectangle swept 6 mm along +Z with a 30
degree twist, cut from a 4 mm tall, radius 18.5 mm cylinder whose caps the tool
passes through. Without the refinement of §3, `Cut` returns
`BooleanUnsupportedContact`: the operands' held facets come within the chord
tolerance without provably interpenetrating deeper than it.

## 2. Cause

The tangency gate (`internal/meshbool/tangency_gate.go`) decides a face pair
through its slack `b = deltaA + deltaB`, the sum of the two faces' proven
displacements from their true surfaces. It admits a meet only when a witness
point lies inside the other operand deeper than `b`. The tool, not the target,
sets `b` here, in two steps.

1. A twisted `Sweep` along a line with an angle of at most one radian and a
   straight profile is the one-cell build (`twistsweep.Eval`). Each wall is two
   flat triangles. The true wall is a ruled, twisting quad, and the proven
   departure is `R*a*a/8 + E*a/4 + ...` (sweep §17.1). For this tool,
   `R = 8.07`, `E = 15.4`, `a = 0.5236`: the `E*a/4` warp term is 2.0 mm and
   `Delta` is 2.36 mm. The wall face bound is 2.36 mm.
2. `booloperand.heldPrimitiveFloor` raises the pair's chord tolerance to the
   operand's `Delta`, because a restated held mesh cannot be requested finer.
   The tolerance is then 2.36 mm, which chords the target's radius 18.5 mm
   circle to a 1.86 mm sagitta. The face-pair slack is 4.22 mm, more than the
   plate is thick (4 mm) and the bore is wide (4.75 mm). No interior point
   lies 4.22 mm from every boundary, so no witness exists and the gate
   refuses.

The refusal is correct for the held boundary it was given. Nothing in the gate
or the Cut is wrong; the held boundary is too coarse for the question.

The same section at 60 or 87 degrees already Cuts without refinement, because
an angle above one radian takes the stationed build (sweep §17.3), whose cells
keep the departure near four percent of the profile radius, or 0.08 mm when the
optional finer count fits.

## 3. Stationed refinement

The refinement restates a twisted sweep's true section as a stationed
build's held mesh, for a consumer that asks for a boundary closer than the
sweep's own `Delta`. A one-cell sweep and a sweep that already holds a
stationed mesh both refine, to the target of §7.5. The sweep is not rebuilt,
republished or re-measured; only its held mesh is.

- `Payload.RefinedDelta` computes the stationed build's per-cell departure
  `R*a*a/8 + E*a/4 + e` at the cell count `refinedCellCount` chooses,
  plus `refineSlack` (1e-9 relative and absolute) for each cell Loft's
  rounding-sized facet departure. It answers false for a sweep that has an
  arc, has no refinement under the cell and work ceilings, or whose
  refinement is no closer than `Delta`.
- `Payload.RefinedMesh` runs `evalStationedCells` over the same record
  (profile, path, angle, placement), takes the resulting mesh, and rebinds each
  facet and face bound to the live sweep's own face with the same source role
  (`cap start`, `cap end`, `side(0,0,j)`). The stationed build merges its cell
  walls into one face per source edge with the same roles, so the rebinding is
  one-to-one. It refuses when the refinement's own proven `Delta` exceeds the
  `RefinedDelta` it was selected on, or a role has no live face.
- `tessdispatch` uses it when a tessellation asks for a chord below `Delta` and
  at or above `RefinedDelta`. A request below `RefinedDelta` still refuses. A
  request at or above `Delta` still restates the one-cell mesh unchanged.
- `booloperand.HeldFloor` and `heldPrimitiveFloor` answer `RefinedDelta` when
  it exists. The pair tolerance is then set by the operands' extents, not by
  the sweep's warp term.

Soundness. The refined mesh is the stationed build's own output with its own
boundary, occupied-volume and area proofs; the refinement adds no new proof.
The rebinding changes which face handle a facet points at, not any coordinate
or bound. `RefinedDelta` only chooses the request; the mesh publishes the bound
it proved, and `RefinedMesh` refuses when that is larger than the request. An
understated `RefinedDelta` therefore refuses and never admits.

The gate is unchanged. It still admits a meet only on a witness deeper than
the slack, and still refuses a near miss closer than the slack.

## 4. Result for the repro

The refinement takes 6 cells for the 30 degree tool. `Delta` falls from 2.36
mm to 0.35 mm. The pair slack falls from 4.22 mm to under 1 mm, below the
plate's half thickness, and the Cut returns a solid that passes
`Tessellate(VerifyAll)`.

## 5. What is admitted and refused

Admitted: a straight-profile twisted `Sweep` along one +Z line (any admitted
angle, placed by any rigid `Transform`) as an operand of `Cut`, `Union` or
`Intersect` against a prism, cylinder or other operand the mesh path accepts,
when the true surfaces cross, or stay apart, by more than the slack. The
bore crossing a tube wall obliquely is this case. A tilted placement works
because `Placed` replays the record under the composed rigid motion.

Refused, soundly:

- A true gap or crossing depth smaller than the slack (about 0.7 mm for the
  §1 sizes). A 60 degree tool at 15.5 mm from the tube axis ends 0.49
  mm from the 18.5 mm wall; the refusal is the contact gate working, not a
  limit of the refinement. Moving the tool 1 mm admits it.
- An arc profile, which is already stationed (sweep §17.4) and keeps its own
  `Delta`.
- A tool whose refinement exceeds the station or facet-pair ceilings.

Bound quality. The Cut's volume bound is the mesh path's occupied-volume proof
for the whole pair: each operand's displacement times its held area. The
target's share falls with its chord request (§7.4), and the tool's with its
refinement (§7.5).

## 6. A near miss the refinement still refuses

A true gap or crossing under about 0.7 mm needs a closer pair tolerance for the
target (the chorded cylinder contributes about half of the slack) or an
analytic Cut of a cylinder by a twisted wall. §7.4 closes most of that gap for a
partner that is chorded at its own tolerance; the contacts left are in §7.7.

## 7. Chained cuts: fine partner chord and fine twist refinement

A tube bored by several twisted `Sweep` tools, then cut by prism windows,
chained on one body.

### 7.1 The problem

The case is a tube (outer radius 18.5 mm, inner 12.5 mm, 37.5 mm
tall) bored by twisted rectangular tools (15.4 x 4.15 mm, 7.27 degrees of twist
per mm, 10.9 mm of axis, 79 degrees) along two axes 80 degrees apart, then cut
by two slanted prism windows. Without §7.4 and §7.5, every `Cut` after the
first is refused:

> cut's target holds its mesh, where this pair meets, only within a bound of
> 1.33 mm, coarser than this pair's chord tolerance 0.385 mm

A first cut that reaches its inner end 0.9 mm from the tube's inner wall is
refused as well, by the contact gate.

### 7.2 Cause

Three separate terms stack.

1. **The tool sets the pair tolerance.** A twisted tool that is already
   stationed (any angle above one radian) holds its mesh within `Delta` =
   0.39 to 0.43 mm: the stationed build aims at 4% of the profile radius.
   `booloperand.heldPrimitiveFloor` raises the pair's chord tolerance to that
   floor, 0.385 mm against a diameter-derived 0.0012 mm.
2. **The tolerance chords the partner.** Both operands are meshed at the pair
   tolerance, so the tube's two cylinders are chorded to a 0.3 mm sagitta.
   The contact gate's slack (`deltaA + deltaB`, tessellation §11 step 4) is
   then 0.7 to 0.9 mm, wider than the clearance of the tool's inner end. The
   cylinders' volume allowance (`delta * area`) is 200 mm3, larger than all
   four bores together.
3. **A rim vertex amplifies, and a facet takes its worst corner.** A vertex the
   cut creates carries `(delta_target + delta_tool) / sin theta`
   (vertex-bounds §3.2), 0.7 to 1.33 mm here. A facet's bound is the largest of
   its three corners (vertex-bounds §2). A chorded cylinder wall is a strip as
   tall as the tube and 0.17 mm wide, so the strip a rim vertex lands in
   becomes triangles that run the whole 37.5 mm and carry the rim's bound along
   all of it. The next tool's pair touches those strips wherever it crosses the
   same angular range, even 14 mm higher, and the chain-depth gate
   (vertex-bounds §5) refuses: the strip's bound exceeds the pair's tolerance,
   which only the next tool's floor sets.

Windows after bores meet the same strips against a tolerance of
`2e-5 * diameter` (1.4e-3 mm), so a window anywhere over a rimmed strip is
refused.

### 7.3 Choice

The gate is policy, not a proof: a facet's bound is proven wherever it is
admitted, and `WithChainTolerance` already admits a larger one with every bound
still carried. The design therefore shrinks the terms that make the bounds
large and the touched set wide, and leaves the gate as it is.

| Option | Verdict |
|---|---|
| Restate the tube finer (§7.4) | Taken. Slack falls from 0.9 mm to 0.05 mm, the cylinders' volume allowance from 200 mm3 to about 10 mm3, and a tool that clears the wall by less than 0.9 mm is admitted. |
| Refine the tool finer (§7.5) | Taken. The tool's own `Delta` is the largest term left, and it sets the tool's volume allowance and the rim bounds. |
| Keep the cylinder analytic through the cut | Not taken. The result of a mesh Boolean is a `Faceted` body of flat triangles by design (vertex-bounds §2.1); an analytic wall is a different payload class and a different route. |
| A multi-tool `Cut(target, tools...)` | Not taken. It needs a new public signature and a proof that the tools are pairwise apart, and it removes only the tool-on-tool rim contacts. With §7.4 and §7.5 the four bores no longer touch one another's rims, so it buys nothing here. |
| Subdivide a chorded cylinder's strips along the axis | Not taken. It would keep a rim's bound local to its neighbourhood and admit a window over a rimmed strip. It changes every prism and revolve tessellation and multiplies their facet counts; §7.7 states what is left without it. |

### 7.4 A partner is chorded at its own tolerance

`meshbool.OperandChord(pairTol, diameter, floor, restating)` is the chord
request for one operand of a pair. An operand that restates a held mesh keeps
the pair tolerance. Every other operand is requested at
`max(diameter * ChordFactor, its own floor)`, never above the pair tolerance.

`booleaneval.evaluate` and `Evaluate`'s shifted retry use it for both operands,
and `verifyrun.newVerifyMeshCache` uses it for each shared mesh, so `Verify`
meshes a body as the construction did. The pair tolerance itself, the gate and the
volume proofs do not depend on it. The request is finer only when
another operand's held floor raised the pair tolerance, so a pair of two
analytic operands is meshed at the pair tolerance.

A public Cut by a tool with a face that is not a plane, whose operand boxes
overlap in at most 1/8 of the pair's diameter (`meshbool.OverlapChordRatio`),
first passes the overlap's diameter (`meshbool.OverlapDiameter`) in place of
`diameter`. Only the common part of the boxes can meet, so a short curved tool
at one end of a long bar chords the pair as finely as a short bar would, and
the result's held facets near the cut keep a smaller bound. For a prism tool
the overlap is read tighter (`booleaneval.clipOverlapDiameter`): the target's
held facets, or its box's faces when it holds no mesh, are clipped in floats to
the tool's section box over its levels in the tool's frame, and the clipped
points' box is measured. Only there can the two boundaries meet, so a long
straight tool crossing a long diagonal bar reads the short crossing rather
than the boxes' common part along the bar. The smaller of the two readings is
the overlap. Held operands, the pair tolerance and the chain gate read the
whole pair's diameter. When that attempt refuses at any stage,
`booleandispatch.BodyScaled` runs the Cut again at the pair's diameter, so the
finer chord never refuses a Cut the pair's chord admits. A tool of planes keeps
the pair's chord: its rim lies on its own exact planes. Unions keep it too;
the same rule on Unions asks some revolves for more facets than their mesh
ceiling allows. The ratio keeps ordinary pairs on the pair's chord. The same
soundness argument covers the finer request.

Soundness. A mesh publishes the sagitta it took, and every proof reads that
published figure, never the request (tessellation §2). A finer request makes
the published figure smaller. The tangency gate's slack is a sum of published
displacements, so it falls with them, and the gate still admits a meet only on
a witness deeper than the slack. No gate changes.

Cost. A coil or mitred-sweep cut of a larger part meshes its partner finer by
the same ratio, up to `MaxFacetPairTestsPerCall`, which still refuses an excess.

### 7.5 A twisted sweep refines to a finer, fixed target

`twistsweep.Payload.RefinedDelta` and `RefinedMesh` (§3)

- apply to a sweep that already holds a stationed mesh, not only a one-cell
  sweep, so a twist of more than one radian refines too;
- aim at `refineTargetScale` = 0.15% of the profile's largest L1 radius, about
  a twenty-seventh of the stationed build's 4% target;
- use `refinedCellCount`: the fewest cells that reach the target, else the
  most the ceilings allow (`maxRefinedTwistCells` = 512 and the facet-pair
  ceiling). A refinement that is no closer than the sweep's own `Delta` is
  not offered.

The proofs are the stationed build's own (§3, Soundness): `RefinedMesh`
refuses when the mesh proves more than `RefinedDelta` promised.

### 7.6 What is admitted

The sleeve test (apitest `TestCutChainedTwistedBoresAndWindows`) shows:

- Four one-sided bore cuts (stations 8.6 to 19.5 mm and -19.5 to -8.6 mm on
  each axis) are admitted by default, then two narrow slanted prism windows
  placed in the angular gaps the bores leave (centred on the +Y and -Y
  sides).
- The result is a solid, `Tessellate(VerifyAll)` passes, and the volume lies
  within its certified bound of the tube minus an independently integrated
  overlap.
- A window wide enough to reach a bore's rim is refused by default with the
  chain-depth message (it meets rimmed strips, §7.2 item 3) and admitted with
  `WithChainTolerance(0.1 mm)`, with the same volume check
  (`TestCutChainedWindowOnBoreRims`).

Refused, soundly:

- A prism window over a rimmed strip, as above, without
  `WithChainTolerance`.
- One through-sweep per bore axis (285 degrees, 39 mm): the refinement reaches only
  the cell ceiling (`Delta` = 0.054 mm) and the second cut still touches the
  first one's rim at 0.088 mm.
- A true crossing or gap under the new slack (about 0.05 mm).

### 7.7 What still needs WithChainTolerance

Bore cuts that follow a window cut, and windows that follow bore cuts and sit
over a bored strip, still need `WithChainTolerance`. Cutting the windows
first avoids it: their rims carry about 0.004 mm, far under a bore pair's
tolerance. Keeping a rim's bound local to a facet neighbourhood, by axial
subdivision of a chorded wall or by a bound that weights corners at the
touched point (vertex-bounds §2), would remove the need; neither is built.
