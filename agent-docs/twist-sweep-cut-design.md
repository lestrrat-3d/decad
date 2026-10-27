# Twisted-Sweep Cut: Stationed Refinement of a Twist

`Cut`, `Union` and `Intersect` of a twisted `Sweep` against a prism or
cylinder. `agent-docs/chained-cut-design.md` §4.2 sets the refinement target
and cell ceilings and applies it to stationed sweeps; §3 states the rule. Companion to `agent-docs/sweep-design.md` §17 (the twisted Sweep),
`agent-docs/general-boolean-design.md` (the mesh path) and
`agent-docs/tessellation-design.md` §11 step 4 (the tangency gate).

## 1. The problem

A screw gear bores a tube wall with a twisted rectangular `Sweep` tool. The
repro is a 15.4 x 4.75 mm centred rectangle swept 6 mm along +Z with a 30
degree twist, cut from a 4 mm tall, radius 18.5 mm cylinder whose caps the tool
passes through. `Cut` returned `BooleanUnsupportedContact`: the operands' held
facets come within the chord tolerance without provably interpenetrating deeper
than it.

## 2. Cause

The tangency gate (`internal/meshbool/tangency_gate.go`) decides a face pair
through its slack `b = deltaA + deltaB`, the sum of the two faces' proven
displacements from their true surfaces. It admits a meet only when a witness
point lies inside the other operand deeper than `b`. The tool, not the target,
sets `b` here, in two steps.

1. A twisted `Sweep` along a line with an angle of at most one radian and a
   straight profile is the one-cell build (`twistsweep.Eval`). Each wall is two
   flat triangles. The true wall is a ruled, twisting quad, and the proven
   departure is `R*a*a/8 + E*a/4 + ...` (§17.1). For this tool,
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

The same section at 60 or 87 degrees already sweeps and already Cuts, because
an angle above one radian takes the stationed build (§17.3), whose cells keep
the departure near four percent of the profile radius, or 0.08 mm when the optional finer count fits.

## 3. Stationed refinement

The refinement restates a twisted sweep's true section as a stationed
build's held mesh, for a consumer that asks for a boundary closer than the
sweep's own `Delta`. A one-cell sweep and a sweep that already holds a
stationed mesh both refine, to the finer target of chained-cut §4.2. The sweep is not rebuilt, republished or
re-measured; only its held mesh is.

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
boundary, occupied-volume and area proofs; this change adds no new proof. The
rebinding changes which face handle a facet points at, not any coordinate or
bound. `RefinedDelta` only chooses the request; the mesh publishes the bound it
proved, and `RefinedMesh` refuses when that is larger than the request. An
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
  screw gear's sizes). A 60 degree tool at 15.5 mm from the tube axis ends 0.49
  mm from the 18.5 mm wall; the refusal is the contact gate working, not a
  limit of the refinement. Moving the tool 1 mm admits it.
- An arc profile, which is already stationed (§17.4) and keeps its own `Delta`.
- A sweep that is stationed already: it has no coarser mesh to refine.
- A tool whose refinement exceeds the station or facet-pair ceilings.

Bound quality. The Cut's volume bound is the mesh path's occupied-volume proof
for the whole pair: each operand's displacement times its held area. The
target's share falls with its chord request, and the tool's with its
refinement (chained-cut §4). For the 37.5 mm tube bored four times the bound
is 104 mm3, 26 mm3 for each bore.

## 6. What else would close the screw gear gap

- The 43.6 degree twist. It sweeps only once the parallel angle work lands
  (35 to 55 degrees); this change then covers its Cut with no further edit,
  since the refinement reads the same record.
- A near miss under 0.7 mm. This needs a closer pair tolerance for the target
  (the chorded cylinder contributes about half of the slack) or an analytic
  Cut of a cylinder by a twisted wall. Neither is part of this change.

## 7. Size

About 150 lines of Go in `internal/twistsweep/refine.go`, 8 in
`internal/tessdispatch/dispatch.go`, 12 in `internal/booloperand/chord.go`,
and `apitest/twist_sweep_cut_test.go`. The sweep construction and the tangency
gate are untouched.
