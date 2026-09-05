package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/loft-design.md PR 1a: the evaluator half of Loft — the
// loftPayload, Table S gates S1-S5, S7's STRUCTURAL arm and S13's
// coordinate-range gate (S9-S11 are the public entry point's job,
// docs/loft-design.md §2/§4), the wiring of the already-landed §6 audit
// (loft_audit.go) and §8 mass kernel (loft_moments.go), and the four
// measurements. Document.Loft/LoftContext are PR 1b; nothing here is called
// from outside this file's own tests, the same shape #114 (loft_audit.go/
// loft_moments.go) already shipped.
//
// Three sibling files carry the construction evalLoft drives, each with its
// own doc comment: loft_pairing.go decides Table P's correspondence,
// loft_stations.go places the chord stations and proves their departure, and
// loft_topology.go assembles the flat-triangle solid (§5) and its topology.
//
// loftPayload.placed and its delta field are §12 PR 2a (Table D, D7): a
// placement re-lifts both records under the composed motion and re-runs
// §5-§8 from scratch — every record-only Table S gate (S1-S8) and S13's
// coordinate-range gate, plus the placement-only S12, while S9-S11 and S4's
// arity half judge the original
// call's own arguments and never re-run (§4) — so delta is the ONE new term
// this PR adds, composed into every vertex, edge length, face area and body
// measurement §8 already derives.

// loftPayload is the evaluator's own record of a lofted body: the two
// authenticated sections, their planes and frames, the per-loop alignment,
// the accumulated rigid placement, and the assembled triangle set the
// construction produced (docs/loft-design.md §5/§7). verts/tris/walls are the
// same globally oriented triangle set §6's audit classified and §8's
// accumulator integrated — tris[:walls] the wall triangles (side(i,j,k)), the
// rest the two caps' own triangulations — kept on the payload so a later
// Tessellate (PR 2) restates it rather than rebuilding it.
type loftPayload struct {
	profile0, profile1 ProfileRecord
	plane0, plane1     PlaneRecord
	frame0, frame1     r3.Frame
	alignment          []int
	xform              r3.Transform

	// delta is the proven displacement of every held vertex from the exact
	// point the record denotes for it (docs/loft-design.md §5, §12 PR 2a,
	// a10-plan.md Part 3 PR 6): absSumUpper(stationRound, placeAllow).
	// placeAllow is zero for an unplaced body — pl.xform == r3.Identity(), an
	// exact struct comparison, never a tolerance — and otherwise
	// bounds.go's rigidRoundAllow, read at the pre-transform lifted point's
	// own magnitude and the composed translation's magnitude. stationRound is
	// each station's own displacement from the point the record denotes for
	// it — an exact-rational trig enclosure rounded once into a Point2 for a
	// circular station, lerp2's own gap from ratLerp for a LineSeg station
	// sitting at a TRIMMED parameter, and the arc-end radial residual
	// (arcNaturalEndRadialUpper) at an untrimmed ArcSeg's t == 1 end, whose
	// recorded coordinate the record states while denoting another point
	// there. It is zero exactly where every station publishes a zero
	// (docs/loft-design.md §5.2's guaranteed-zero list), which no segment KIND
	// grants on its own beyond an untrimmed LineSeg: a trimmed LineSeg
	// pairing does not reach that zero, and a curved pairing is positive
	// whenever a cell has an interior station or an arc end off its own
	// recorded radius. So delta is zero exactly when
	// BOTH terms are, no longer merely when the body is unplaced. Every
	// measurement this payload publishes composes it.
	delta float64

	// sectionDelta is the proven upper bound on how far any BUILT CHORD point
	// of a wall cell sits from the recorded curve it chords, AS A SET — the
	// curve's own sagitta, taken as a MAXIMUM over cells rather than a sum
	// (docs/loft-design.md §5, a10-plan.md Part 3 PR 5/PR 6). It is zero for
	// a LineSeg pairing — a straight wall's own chord IS the recorded
	// segment, so there is no curve for it to depart from — and positive for
	// a same-kind circular pairing, to the sagitta its station chording
	// commits.
	//
	// It is NEVER delta and never stands in for it, the identical
	// independence prismPayload's own sectionDelta/z0Delta pair states one
	// mechanism over (extrude.go): delta bounds a HELD VERTEX's own
	// displacement from the exact point the record denotes for it, while
	// sectionDelta bounds a BUILT CHORD's own displacement, in the section
	// plane, from the curve it chords. A reading that needs both sums them
	// into its own bound; neither is ever substituted for the other.
	//
	// It is ALSO never bounds.go's cellChordCurveAreaUpper's own
	// matchedDeltaUpper obligation, a STRONGER, DIFFERENT quantity that
	// helper's own doc comment defines. §5.2's table owns both terms and the
	// relation between them: its matchedDelta row states what that
	// parameter-matched quantity is, what it is composed from and what it
	// refuses on. A consumer that needs it reads that row and composes the
	// terms the row names, never this field on its own.
	//
	// Every matched-delta obligation is discharged by a SEPARATE quantity,
	// the matchedDelta evalLoft composes: loftPairings accumulates each
	// cell's own chord-to-curve departure into a MAX beside sectionDelta,
	// and evalLoft sums that MAX with the delta above through
	// chordCellDeltaUpper before passing it — never this field — to
	// newLoftMassAccumulator and computeLoftChordedAllow (loft_moments.go),
	// which is where every chordedBoundaryVolumeAllow,
	// chordedBoundaryMomentAllow, chordedBoundarySeamAllow and cap-area
	// matched argument comes from; cellChordCurveAreaUpper reads the same
	// composition per cell, over the cell's own chord-to-curve half.
	// The raw matched quantity itself is a PER-BUILD LOCAL of evalLoft and is
	// never a field here. What the payload stores instead is the proof
	// COMPOSED from it — the three terms of the proof field below, which the
	// tessellation restates (docs/tessellation-design.md §2's loftPayload
	// row). That keeps the property the local was protecting: a stored term
	// can disagree with the records only if it can outlive them, and none
	// here can, because placed nils the triangle fields and re-runs evalLoft,
	// which recomputes the records' every quantity and rewrites the proof
	// beside the triangle set it speaks for. The two helpers above still have
	// exactly one production call site each — evalLoft's own — so no caller
	// can reach them with the other quantity either.
	//
	// This field's OWN remaining spend is Bounds.Bound, a SET-distance
	// reading, which is what makes sectionDelta the term it correctly reads.
	// The cap-area tube is NOT one of them: §5.2's capAreaAllow row names the
	// matched term there, because a held cap polygon's own vertices are
	// displaced as well as chorded.
	sectionDelta float64

	verts []r3.Vec
	tris  [][3]int
	walls int
	// capStartCount is how many of tris[walls:] belong to capStart; the rest
	// belong to capEnd. cell/side parallel tris[:walls] — cell[k] is that wall
	// triangle's {loop index i, cell index j} and side[k] its 0/1 half — so a
	// consumer can name the side(i,j,k) role of every wall triangle without
	// re-deriving the assembly's own split. All three are copied verbatim from
	// loftAssembly, whose own doc comment owns the convention.
	capStartCount int
	cell          [][2]int
	side          []uint8

	// proof is the mesh proof record docs/tessellation-design.md §2 states for
	// this payload, composed by evalLoft from the same build the triangle set
	// above came out of (docs/tessellation-reach-design.md §4).
	proof loftMeshProof
}

// loftMeshProof is docs/tessellation-design.md §2's loftPayload row: the three
// private proofs a Mesh restating this payload's triangle set publishes.
// evalLoft composes it once, from the terms docs/loft-design.md §5.2 and §8
// already derive for the SAME triangle set, and no consumer recomposes it.
//
// It is deliberately not any of the payload's other published displacements.
// facetDeparture is §5.2's facet-departure row — absSumUpper(matchedDelta,
// maxTwistOffsetUpper) — and never Bounds.Bound's absSumUpper(delta,
// sectionDelta), which answers the different, SET-distance question §5.2's own
// Bounds.Bound row states. A zero in any field is a published proof of
// exactness, so each is composed from the terms' own values and never assumed
// from a segment kind or from the build being unplaced.
type loftMeshProof struct {
	// facetDeparture bounds how far one point of a held facet sits from the
	// true boundary surface that facet stands for: the mesh's own
	// sourceBound(face) for every face, and its Bound.
	facetDeparture float64
	// areaSlack bounds how far the held triangle areas sit from the areas of
	// the surfaces they stand for, without cancellation: the per-triangle
	// perturbation sum, the wall's held-to-bilinear, ruled and station-shift
	// legs, and the two caps' own capAreaAllow.
	areaSlack float64
	// volSymDiff bounds volume(TrueBody △ MeshSolid) — occupied volume, so no
	// term of it may cancel another. It composes the vertex-displacement swept
	// allowance with the FOUR-leg chorded boundary allowance, the twist leg
	// included, because the mesh holds the UNCORRECTED triangles rather than
	// Volume's twist-corrected value.
	volSymDiff float64
}

// transform is the accumulated rigid placement.
func (pl loftPayload) transform() r3.Transform { return pl.xform }

// axialDelta reports the displacement of every held loft vertex. Its planar
// caps are built from those vertices, so a ToFace stop against either cap
// inherits this bound.
func (pl loftPayload) axialDelta() float64 { return pl.delta }

// placed re-evaluates the same two records under the composed motion
// (docs/loft-design.md §7, §12 PR 2a): it re-lifts every vertex from the
// record under the FULL composed transform rather than moving the held mesh
// incrementally, so delta does not accumulate across repeated placements —
// one rounding, not one per placement, an advantage over
// facetedPayload.placed's move-the-held-mesh path (boolean_body.go). It is a
// re-evaluation path: no moments preflight has run on either record within
// this call, so the build opens two fresh counters, one per record, exactly
// as prismPayload.placed opens its own (extrude.go). §5's whole-shell
// orientation step re-decides the sign from the placed triangle set on its
// own, so a mirror flips `reversed` with no separate winding-flip case
// needed here.
func (pl loftPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	next := pl
	next.xform = composed
	next.verts, next.tris, next.walls = nil, nil, 0
	// The assembly-derived fields and the proof composed from them are cleared
	// beside the triangle set they speak for: evalLoft rewrites all of them
	// from the re-lifted records, and a stale copy surviving the re-evaluation
	// is exactly the disagreement the payload's own doc comment forbids.
	next.capStartCount, next.cell, next.side = 0, nil, nil
	next.proof = loftMeshProof{}
	return evalLoft(ctx, d, ref, next, newWorkBudget(ctx), newFreeformWork(), newFreeformWork())
}

// validateLoftBodyMeasurements is evalLoft's own finiteness gate (design O2).
// Volume, Centroid and Bounds must be fully finite, exactly as every other
// analytic payload's validateAnalyticBodyMeasurements requires — but Area's
// Bound is deliberately NOT checked: §8 requires a saturated Area bound to
// publish +Inf as a proof term (a wall set whose areas approach float64's own
// ceiling), and checking it here would refuse the very body that fixture
// constructs. Only Area's Value is checked for finiteness.
func validateLoftBodyMeasurements(body *Body) error {
	if !finiteMeasurementValues(body.volume.Value.Base(), body.volume.Bound.Base()) {
		return fmt.Errorf(`%w: the loft's volume measurement is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(body.area.Value.Base()) {
		return fmt.Errorf(`%w: the loft's area value is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(body.centroid.Value.X, body.centroid.Value.Y, body.centroid.Value.Z, body.centroid.Bound.Base()) {
		return fmt.Errorf(`%w: the loft's centroid measurement is not finite`, ErrNotFinite)
	}
	if !finiteMeasurementValues(
		body.bounds.Min.X, body.bounds.Min.Y, body.bounds.Min.Z,
		body.bounds.Max.X, body.bounds.Max.Y, body.bounds.Max.Z,
		body.bounds.Bound.Base(),
	) {
		return fmt.Errorf(`%w: the loft's bounds measurement is not finite`, ErrNotFinite)
	}
	return nil
}

// evalLoft builds the lofted body from the payload's own records
// (docs/loft-design.md §5-§8): pairing, assembly, the §6 audit, topology, and
// the four measurements — all four published at build, never staged (§12).
// budget is shared with the rest of the pre-commit cancellation path exactly
// as modify §5's audits already share one; the caller (LoftContext, PR 1b)
// mints it once for the whole build.
//
// work0/work1 are the per-profile free-form work counters (spline design
// §5.2): the R7 ceiling is one record's across a whole OPERATION, and
// LoftContext also runs falsifyRecordedArea on both records before evalLoft
// is called, so those counters — not two fresh ones minted here — must be
// the ones every walkOf call site in this build spends against. S3 admits
// only same-kind LineSeg or circular pairs, neither of which is a free-form
// kind, so nothing here charges them yet — but the counters are still
// threaded through so a future free-form correspondence does not silently
// open a second ceiling per record.
func evalLoft(ctx context.Context, d *Document, ref StepRef, pl loftPayload, budget *workBudget, work0, work1 *freeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	offsets, walks0, walks1, err := validateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, work0, work1)
	if err != nil {
		return nil, err
	}

	target, err := loftChordTarget(pl.profile0, pl.profile1, walks0, walks1)
	if err != nil {
		return nil, err
	}

	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftPairings(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	if err != nil {
		return nil, err
	}

	a, err := assembleLoft(ctx, pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform, stationRound)
	if err != nil {
		return nil, err
	}

	if err := loftCrossingAudit(budget, a.verts, a.tris); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cap0Rat := capPolygonAreaRat(a.pts0, a.loopIdx0)
	cap1Rat := capPolygonAreaRat(a.pts1, a.loopIdx1)

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: roleBody}, solid: true}

	capStart, capEnd, walls, err := buildLoftTopology(ctx, body, ref, a, cap0Rat, cap1Rat)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	faces := append([]*Face{capStart, capEnd}, walls...)
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}

	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	anchor := pl.xform.Apply(pl.plane0.Origin)
	// docs/loft-design.md §5.2's matchedDelta row, composed here and nowhere
	// else: absSumUpper of the build's own MAX-over-cells chord-to-curve
	// departure (loftPairings' sectionMatchedDelta) and the held vertex
	// displacement a.delta. The two halves are accumulated apart — a chord's
	// departure from the curve it chords, and a station's departure from the
	// point the record and the motion denote for it — and every chorded leg
	// charges their SUM, since the chord the build DREW joins two displaced
	// stations. The sagitta alone would leave the computed station's own
	// displacement uncharged (§5.2's matchedDelta paragraph). Left at exactly
	// 0 on a build with no chorded cell, which is what keeps a LineSeg-only
	// pairing's published measurements free of every chorded term: its held
	// triangle pair IS the boundary §5 gives it, and a.delta already reaches
	// its measurements through the accumulator's own delta-keyed legs.
	matchedDelta := 0.0
	if sectionDelta > 0 || sectionMatchedDelta > 0 {
		matchedDelta = chordCellDeltaUpper(sectionMatchedDelta, a.delta)
	}
	mass := newLoftMassAccumulator(anchor, a.delta, sectionDelta, matchedDelta)
	for k, t := range a.tris {
		mass.add(a.verts[t[0]], a.verts[t[1]], a.verts[t[2]], k < a.walls)
	}
	// The chorded correction terms (docs/loft-design.md §5/§8, a10-plan.md
	// Part 3 PR 6) read the mass accumulator's own coordUpper, which is only
	// complete once every triangle has folded into it above — so this runs
	// after the add loop, gated on EITHER sectionDelta or sectionMatchedDelta
	// being positive rather than on sectionDelta alone: a free-form cell can
	// carry a positive matchedDelta at an exactly-zero sagitta
	// (spline_sagitta.go's own counterexample), and skipping the computation
	// there would silently drop a genuine chord-to-curve area/volume
	// obligation. Left at its zero value (every field of loftChordedAllow)
	// for a LineSeg-only build, where both are zero.
	//
	// It is also where S14's CONSTRUCTION arm decides the cap
	// planeOffsetUpper term §5.2's table lists: an assembly stating no proven
	// distance from the anchor to a held cap1 vertex refuses here
	// (errLoftCapOffsetUnderivable, loft_moments.go) instead of measuring on,
	// so no measurement below is ever composed from a substituted value.
	if sectionDelta > 0 || sectionMatchedDelta > 0 {
		chorded, err := computeLoftChordedAllow(
			pairs, a.vIdx, a.wIdx, a.verts, anchor, matchedDelta, a.delta, mass.distUpper, a.reversed,
		)
		if err != nil {
			return nil, err
		}
		mass.chorded = chorded
	}
	body.volume = mass.volume(a.verts, a.tris)
	centroid, err := mass.centroid(a.verts, a.tris)
	if err != nil {
		return nil, err
	}
	body.centroid = centroid
	bounds, ok := mass.bounds()
	if !ok {
		return nil, fmt.Errorf(`%w: the loft has no vertices to bound`, ErrDegenerate)
	}
	body.bounds = bounds
	body.area = mass.area(cap0Rat, cap1Rat)

	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}

	pl.verts, pl.tris, pl.walls = a.verts, a.tris, a.walls
	pl.capStartCount, pl.cell, pl.side = a.capStartCount, a.cell, a.side
	pl.proof = loftMeshProofOf(a, mass, sectionMatchedDelta, matchedDelta)
	pl.delta = a.delta
	// sectionDelta is loftPairings' own accumulated MAX over cells
	// (loftPayload's own doc comment): zero for a LineSeg-only pairing,
	// positive for a same-kind circular one, to the sagitta its station
	// chording commits.
	pl.sectionDelta = sectionDelta
	body.payload = pl
	return body, nil
}

// loftMeshProofOf composes docs/tessellation-design.md §2's loftPayload row
// from the build evalLoft has just finished (docs/tessellation-reach-design.md
// §4). Every input is a term docs/loft-design.md §5.2 or §8 already derives for
// the SAME triangle set the payload keeps, so the mesh restating that set
// publishes the payload's own proofs and states nothing new of its own.
//
// sectionMatchedDelta is loftPairings' own MAX-over-cells chord-to-curve
// departure and matchedDelta is evalLoft's composed §5.2 matchedDelta — 0 on a
// build that reached no chorded cell, where the accumulator's delta-keyed legs
// carry the whole displacement instead.
//
// The three terms and why each reads what it does:
//
//   - facetDeparture is §5.2's own facet-departure row, absSumUpper of the
//     parameter-matched chord departure and the wall's facet twist. Its first
//     leg is composed UNCONDITIONALLY rather than through evalLoft's gated
//     matchedDelta, because a LineSeg-only build reaches that gate's zero while
//     its facets still sit delta from the boundary they stand for: §5.2's row
//     states matchedDelta reduces to delta there, and a published 0 would be
//     the claim that the mesh IS the true boundary. It is NOT Bounds.Bound's
//     absSumUpper(delta, sectionDelta), which answers a SET-distance question.
//   - areaSlack sums the per-triangle perturbation allowance the accumulator
//     already holds with the wall's held-to-bilinear leg, its ruled and
//     station-shift legs, and the two caps' own capAreaAllow. The
//     held-to-bilinear leg appears here and not in Area's own bound because
//     Area.Value has been MOVED onto the bilinear patches by areaCorrection
//     while the mesh keeps the uncorrected triangles.
//   - volSymDiff composes the vertex-displacement swept allowance with the
//     FOUR-leg chorded boundary allowance — the twist leg included, for the
//     same reason: Volume.Value applies the exact twist correction and the mesh
//     does not, so the residual three-leg form Volume spends would understate
//     the occupied volume this triangle set differs by.
//
// Each sum rounds up at every step (docs/tessellation-design.md §2), so a term
// this build could not state saturates rather than vanishing, and the caller
// refuses on it instead of publishing it.
func loftMeshProofOf(a loftAssembly, m *loftMassAccumulator, sectionMatchedDelta, matchedDelta float64) loftMeshProof {
	return loftMeshProof{
		facetDeparture: absSumUpper(
			chordCellDeltaUpper(sectionMatchedDelta, a.delta),
			m.chorded.maxTwistOffsetUpper,
		),
		areaSlack: absSumUpper(
			m.perturbAreaSum,
			m.chorded.twistAreaAllow,
			m.chorded.areaExcess,
			m.chorded.capAreaExcess,
		),
		volSymDiff: absSumUpper(
			sweptVolumeAllow(a.delta, perturbedAreaUpper(a.verts, a.tris, a.delta)),
			chordedBoundaryVolumeAllow(
				matchedDelta,
				m.chorded.wallAreaUpper,
				m.chorded.twistVolumeUpper,
				m.chorded.capVolumeUpper,
				m.chorded.seamAllow,
			),
		),
	}
}
