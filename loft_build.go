package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/twistsweep"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/loft-design.md PR 1a: the evaluator half of Loft — the
// loftPayload, Table S gates S1-S5, S7's STRUCTURAL arm and S13's
// coordinate-range gate (S9-S11 are the public entry point's job,
// docs/loft-design.md §2/§4), the wiring of the already-landed §6 audit
// (internal/loftmesh/loft_audit.go) and §8 mass kernel (loft_moments.go), and the four
// measurements. Document.Loft is PR 1b; nothing here is called
// from outside this file's own tests, the same shape #114 (internal/loftmesh/loft_audit.go/
// loft_moments.go) already shipped.
//
// Three sibling files carry the construction evalLoft drives, each with its
// own doc comment: loft_pairing.go decides Table P's correspondence,
// loft_stations.go places the chord stations and proves their departure, and
// loft_topology.go assembles the flat-triangle solid (§5) and its topology.
//
// loftPayload.Placed and its delta field are §12 PR 2a (Table D, D7): a
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
type loftSourceIdentity struct{ origin producerID }

type loftPayload struct {
	// sourceIdentity is shared only by the original Loft and its own placed
	// copies. It lets a structural consumer require one authenticated source.
	sourceIdentity     *loftSourceIdentity
	profile0, profile1 profileRecord
	plane0, plane1     planeRecord
	frame0, frame1     r3.Frame
	alignment          []int
	xform              r3.Transform
	// authenticatedReconstruction is set only after the public Loft seam has
	// admitted both Sketch profiles. Certified fillet and bore rewrites retain
	// that provenance when they re-evaluate their private loft records.
	authenticatedReconstruction bool
	// blendSegs names connector segments inserted by Fillet. Every
	// triangle of a connector's loft cell also carries fillet(loop,segment).
	blendSegs []map[int]struct{}
	// A fitted-spline fillet records float trim parameters and a float arc.
	// These fields enclose the ideal radius-r tangent rewrite certified at
	// construction. The first is the maximum matched plane displacement over
	// all rewritten carriers and connector arcs. The second bounds each ideal
	// connector's complete arc length, keyed by the outer-loop segment index.
	constructionDelta    float64
	constructionArcUpper map[int]float64
	// A trimmed carrier can gain length when its ideal tangent parameter
	// replaces the recorded float trim. Each value bounds that increase for
	// every cell of the named outer-loop segment.
	constructionLengthExtra map[int]float64
	// throughBoreCut marks a profile pair rewritten by Cut. The added hole
	// did not come from the two authenticated source Sketch profiles.
	throughBoreCut bool
	// analyticBore is set only by the certified whole-circle through Cut.
	// Its loft mesh still supplies mass and tessellation proofs, while the
	// public topology uses one cylindrical wall for the new hole.
	analyticBore bool

	// surfaceResult is WithSurfaceResult's own flag (docs/surface-design.md
	// §4): true when the build must omit both section caps and publish a
	// sheet instead of a solid. It is part of the re-evaluable record for the
	// same reason prismPayload's and revolvePayload's own copies are
	// (internal/prismrecord, revolve_build.go), so Transform and Copy
	// reproduce the sheet with no further code — a plain solid
	// loft leaves it false.
	surfaceResult bool

	// recordArea holds each record's own exact region integral, the value
	// falsifyRecordedArea computed and compared against sketch's area in
	// Loft. The chord target reads it (loftmesh.ChordTarget,
	// docs/loft-gear-bounds-design.md §5), so no evaluation integrates a
	// record a second time to size its chords. A rigid placement moves
	// neither record, so placed carries it over unchanged.
	recordArea [2]float64

	// delta is the proven displacement of every held vertex from the exact
	// point the record denotes for it (docs/loft-design.md §5, §12 PR 2a,
	// a10-plan.md Part 3 PR 6): stationRound, liftAllow and placeAllow summed
	// through proofbound.AbsSumUpper (loftmesh.Assemble). liftAllow is the
	// rounding each station's lift through its own section frame commits
	// (loftmesh.FrameLiftRoundAllow), zero only on an XY-aligned frame whose
	// origin lies on the Z axis. placeAllow is zero for an unplaced body —
	// pl.xform == r3.Identity(), an exact struct comparison, never a
	// tolerance — and otherwise
	// internal/proofbound/bounds.go's proofbound.RigidRoundAllow, read at the pre-transform lifted point's
	// own magnitude and the composed translation's magnitude. stationRound is
	// each station's own displacement from the point the record denotes for
	// it — an exact-rational trig enclosure rounded once into a Point2 for a
	// circular station, lerp2's own gap from dyLerp for a LineSeg station
	// sitting at a TRIMMED parameter, and the arc-end radial residual
	// (loftmesh.ArcNaturalEndRadialUpper) at an untrimmed ArcSeg's t == 1 end, whose
	// recorded coordinate the record states while denoting another point
	// there. It is zero exactly where every station publishes a zero
	// (docs/loft-design.md §5.2's guaranteed-zero list), which no segment KIND
	// grants on its own beyond an untrimmed LineSeg: a trimmed LineSeg
	// pairing does not reach that zero, and a curved pairing is positive
	// whenever a cell has an interior station or an arc end off its own
	// recorded radius. So delta is zero exactly when
	// all three terms are, never merely when the body is unplaced. Every
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
	// It is ALSO never internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper's own
	// matchedDeltaUpper obligation, a STRONGER, DIFFERENT quantity that
	// helper's own doc comment defines. §5.2's table owns both terms and the
	// relation between them: its matchedDelta row states what that
	// parameter-matched quantity is, what it is composed from and what it
	// refuses on. A consumer that needs it reads that row and composes the
	// terms the row names, never this field on its own.
	//
	// Every matched-delta obligation is discharged by a SEPARATE quantity,
	// the matchedDelta evalLoft composes: loftmesh.PairRecords accumulates each
	// cell's own chord-to-curve departure into a MAX beside sectionDelta,
	// and buildLoftMass sums that MAX with the delta above through
	// loftmesh.ChordCellDeltaUpper before passing it — never this field — to
	// newLoftMassAccumulator and loftmesh.ComputeLoftChordedAllow (loft_moments.go),
	// which is where the skirt leg's, the centroid radius' and every other
	// build-wide matched argument comes from;
	// proofbound.CellChordCurveAreaUpper, the wall leg and the cap tube read the
	// same composition per cell, over the cell's own chord-to-curve half.
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

	// chorded reports that the build holds a cell whose held triangle pair is
	// NOT the boundary it denotes: a circular or free-form cell, which stands
	// for a ruled patch (loftmesh.LoopPair.Faceted). It is true on a degree-1
	// free-form build whose sectionDelta is zero, so a consumer asking whether
	// the held mesh IS the surface reads this field beside sectionDelta.
	chorded bool

	verts []r3.Vec
	tris  [][3]int
	walls int
	// capStartCount is how many of tris[walls:] belong to capStart; the rest
	// belong to capEnd. cell/side parallel tris[:walls] — cell[k] is that wall
	// triangle's {loop index i, cell index j} and side[k] its 0/1 half — so a
	// consumer can name the side(i,j,k) role of every wall triangle without
	// re-deriving the assembly's own split. All three are copied verbatim from
	// loftmesh.Assembly, whose own doc comment owns the convention.
	capStartCount int
	cell          [][2]int
	side          []uint8
	// segment names the authenticated source segment of each wall cell.
	// It stays parallel to the section stations and is retained for structural
	// consumers that must omit a source wall without inferring its identity
	// from its sampled coordinates.
	segment [][]int
	// twistAreaCells retains only the certified per-cell readings needed by a
	// private twisted Sweep comparison Loft. Ordinary Loft payloads omit them.
	twistAreaCells       [][]twistsweep.TwistAreaCellClaims
	retainTwistAreaCells bool

	// proof is the mesh proof record docs/tessellation-design.md §2 states for
	// this payload, composed by evalLoft from the same build the triangle set
	// above came out of (docs/tessellation-reach-design.md §4).
	proof loftmesh.MeshProof
}

func (pl loftPayload) raiseReconstructionLimit(work ...*freeform.FreeformWork) {
	if !pl.authenticatedReconstruction {
		return
	}
	for _, counter := range work {
		counter.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	}
}

// rewriteWorkLimit is private to the certified fillet and centered-bore
// rewrites. Their extra section proof precedes the ordinary area and station
// work on the same counter. An untrusted payload keeps the ordinary ceiling.
func (pl loftPayload) rewriteWorkLimit(p uint64) uint64 {
	limit := loftmesh.StationWorkLimit(0, p)
	if pl.authenticatedReconstruction {
		limit *= 2
	}
	return limit
}

// Transform is the accumulated rigid placement.
func (pl loftPayload) Transform() r3.Transform { return pl.xform }

// AxialDelta reports the displacement of every held loft vertex. Its planar
// caps are built from those vertices, so a ToFace stop against either cap
// inherits this bound.
func (pl loftPayload) AxialDelta() float64 { return pl.delta }

// Placed re-evaluates the same two records under the composed motion
// (docs/loft-design.md §7, §12 PR 2a): it re-lifts every vertex from the
// record under the FULL composed transform rather than moving the held mesh
// incrementally, so delta does not accumulate across repeated placements —
// one rounding, not one per placement, an advantage over
// facetedPayload.Placed's move-the-held-mesh path (boolean_body.go). It is a
// re-evaluation path: no moments preflight has run on either record within
// this call, so the build opens two fresh counters, one per record, exactly
// as prismPayload.Placed opens its own (extrude.go). §5's whole-shell
// orientation step re-decides the sign from the placed triangle set on its
// own, so a mirror flips `reversed` with no separate winding-flip case
// needed here. next := pl carries surfaceResult over unchanged — only the
// triangle set and the proof composed from it are cleared below — so a
// placed, duplicated or placed-copied sheet reproduces the sheet with no
// further code (docs/surface-design.md §4.2).
func (pl loftPayload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	d, ref := documentHandle(env.Doc), env.Ref
	next := pl
	next.xform = composed
	next.verts, next.tris, next.walls = nil, nil, 0
	// The assembly-derived fields and the proof composed from them are cleared
	// beside the triangle set they speak for: evalLoft rewrites all of them
	// from the re-lifted records, and a stale copy surviving the re-evaluation
	// is exactly the disagreement the payload's own doc comment forbids.
	next.capStartCount, next.cell, next.side, next.segment = 0, nil, nil, nil
	next.twistAreaCells = nil
	next.proof = loftmesh.MeshProof{}
	return placedNode(evalLoft(ctx, d, ref, next, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork()))
}

// validateLoftBodyMeasurements is evalLoft's own finiteness gate (design O2);
// loftmesh.ValidateBodyReadings owns the rule.
func validateLoftBodyMeasurements(body *Body) error {
	return loftmesh.ValidateBodyReadings(loftmesh.BodyReadings{
		Volume: body.n.Volume.Value.Base(), VolumeBound: body.n.Volume.Bound.Base(),
		Area:     body.n.Area.Value.Base(),
		Centroid: body.n.Centroid.Value, CentroidBound: body.n.Centroid.Bound.Base(),
		BoundsMin: body.n.Bounds.Min, BoundsMax: body.n.Bounds.Max,
		BoundsBound: body.n.Bounds.Bound.Base(),
	})
}

// evalLoft builds the lofted body from the payload's own records
// (docs/loft-design.md §5-§8): pairing, assembly, the §6 audit, topology, and
// the four measurements — all four published at build, never staged (§12).
// budget is shared with the rest of the pre-commit cancellation path exactly
// as modify §5's audits already share one; the caller (Loft, PR 1b)
// mints it once for the whole build.
//
// work0/work1 are the per-profile free-form work counters (spline design
// §5.2): the R7 ceiling is one record's across a whole OPERATION, and
// Loft also runs falsifyRecordedArea on both records before evalLoft
// is called, so those counters — not two fresh ones minted here — must be
// the ones every walkOf call site in this build spends against, and the ones
// a same-kind free-form pair's station generator charges
// (docs/loft-design.md §5.1), so a build never opens a second ceiling per
// record.
func evalLoft(ctx context.Context, d *Document, ref producerID, pl loftPayload, budget *proofbound.WorkBudget, work0, work1 *freeform.FreeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	pl.raiseReconstructionLimit(work0, work1)

	offsets, walks0, walks1, target, err := loftmesh.ValidateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, pl.recordArea, work0, work1)
	if err != nil {
		return nil, err
	}

	pairs, sectionDelta, sectionMatchedDelta, stationRound, err := loftmesh.PairRecords(pl.profile0, pl.profile1, offsets, walks0, walks1, target, work0, work1)
	if err != nil {
		return nil, err
	}
	if pl.constructionDelta > 0 {
		if proofbound.IsNonFinite(pl.constructionDelta) {
			return nil, fmt.Errorf(`%w: the loft fillet construction displacement is not finite`, ErrUnsupported)
		}
		stationRound = proofbound.AbsSumUpper(stationRound, pl.constructionDelta)
		if proofbound.IsNonFinite(stationRound) {
			return nil, fmt.Errorf(`%w: the loft fillet station displacement is not finite`, ErrUnsupported)
		}
	}
	if len(pl.constructionArcUpper) != 0 {
		if len(pairs) == 0 {
			return nil, fmt.Errorf(`%w: the loft fillet has no outer loop`, ErrUnsupported)
		}
		counts := make(map[int]int, len(pl.constructionArcUpper))
		for _, segment := range pairs[0].Segment {
			if _, exists := pl.constructionArcUpper[segment]; exists {
				counts[segment]++
			}
		}
		for segment := range pl.constructionArcUpper {
			if counts[segment] == 0 {
				return nil, fmt.Errorf(`%w: the loft fillet arc-length key has no cell`, ErrUnsupported)
			}
		}
		for cell, segment := range pairs[0].Segment {
			idealLength, exists := pl.constructionArcUpper[segment]
			if !exists {
				continue
			}
			if counts[segment] == 0 || idealLength <= 0 || proofbound.IsNonFinite(idealLength) {
				return nil, fmt.Errorf(`%w: the loft fillet arc-length bound is invalid`, ErrUnsupported)
			}
			perCell := proofbound.UpRound(idealLength / float64(counts[segment]))
			pairs[0].ArcUpperV[cell] = max(pairs[0].ArcUpperV[cell], perCell)
			pairs[0].ArcUpperW[cell] = max(pairs[0].ArcUpperW[cell], perCell)
			energy := proofbound.UniformSpeedTangentEnergyUpper(perCell, 0)
			pairs[0].TangentEnergyV[cell] = max(pairs[0].TangentEnergyV[cell], energy)
			pairs[0].TangentEnergyW[cell] = max(pairs[0].TangentEnergyW[cell], energy)
		}
	}
	if len(pl.constructionLengthExtra) != 0 {
		if len(pairs) == 0 {
			return nil, fmt.Errorf(`%w: the loft fillet has no outer loop`, ErrUnsupported)
		}
		seen := make(map[int]bool, len(pl.constructionLengthExtra))
		for cell, segment := range pairs[0].Segment {
			extra, exists := pl.constructionLengthExtra[segment]
			if !exists {
				continue
			}
			if extra < 0 || proofbound.IsNonFinite(extra) {
				return nil, fmt.Errorf(`%w: the loft fillet carrier-length bound is invalid`, ErrUnsupported)
			}
			seen[segment] = true
			pairs[0].ArcUpperV[cell] = proofbound.AbsSumUpper(pairs[0].ArcUpperV[cell], extra)
			pairs[0].ArcUpperW[cell] = proofbound.AbsSumUpper(pairs[0].ArcUpperW[cell], extra)
			// The ideal carrier uses a slightly different normalized trim. Its
			// tangent energy is not the recorded carrier's energy, so use the
			// premise-free area leg until an exact ideal energy is derived.
			pairs[0].TangentEnergyV[cell] = math.Inf(1)
			pairs[0].TangentEnergyW[cell] = math.Inf(1)
		}
		for segment := range pl.constructionLengthExtra {
			if !seen[segment] {
				return nil, fmt.Errorf(`%w: the loft fillet carrier-length key has no cell`, ErrUnsupported)
			}
		}
	}

	a, err := loftmesh.AssembleTriangulated(ctx, pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform, stationRound)
	if err != nil {
		return nil, err
	}

	if err := loftmesh.LoftCrossingAuditStructured(budget, a.Verts, a.Tris, a.Walls, a.CapStartCount, a.VIdx, a.WIdx); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	cap0Rat := loftmesh.CapPolygonAreaRat(a.Pts0, a.LoopIdx0)
	cap1Rat := loftmesh.CapPolygonAreaRat(a.Pts1, a.LoopIdx1)

	// A surface result publishes a sheet, never a solid: solid false is what
	// makes Volume() and Centroid() answer ErrNotSolid through their existing
	// guards, and kind BodySheet is what Kind() reports
	// (docs/surface-design.md §4.3), the same rule prismPayload's and
	// revolvePayload's own builds already follow (internal/prismrecord/build.go,
	// revolve_build.go).
	kind := BodySolid
	if pl.surfaceResult {
		kind = BodySheet
	}
	body := bornBody(&Body{n: topo.Body{Solid: !pl.surfaceResult, Origin: provenance.Ref{Producer: ref, Role: provenance.RoleBody}, Kind: topo.BodyKind(kind), Doc: documentNode(d)}})

	capStart, capEnd, walls, wallAreas, err := buildLoftTopology(ctx, body, ref, a, cap0Rat, cap1Rat)
	if err != nil {
		return nil, err
	}
	// Section 0 lifts through frame0 and section 1 through frame1
	// (loftmesh.Assemble), so each cap denotes its own section plane.
	capStart.n.CapRecord = &topo.CapRecord{Frame: pl.frame0, Xform: pl.xform}
	capEnd.n.CapRecord = &topo.CapRecord{Frame: pl.frame1, Xform: pl.xform}
	for k, wall := range walls {
		li, cell := a.Cell[k][0], a.Cell[k][1]
		if li >= len(pl.blendSegs) {
			continue // A through-bore adds an unblended hole wall.
		}
		if cell >= len(pairs[li].Segment) {
			return nil, fmt.Errorf(`%w: a loft fillet lost its segment correspondence`, ErrUnsupported)
		}
		seg := pairs[li].Segment[cell]
		if _, blend := pl.blendSegs[li][seg]; blend {
			wall.n.Origins = append(wall.n.Origins, provenance.Ref{Producer: ref, Role: fmt.Sprintf("fillet(%d,%d)", li, seg)})
		}
	}
	directCircle := directLoftCircularHole(pl)
	var circularWall *Face
	if pl.analyticBore || directCircle == loftCircleWallCylinder {
		walls, err = buildLoftAnalyticBoreTopology(ctx, body, ref, pl, a, capStart, capEnd, walls)
		if err != nil {
			return nil, err
		}
	} else if directCircle != loftCircleWallNone {
		walls, circularWall, err = buildLoftVaryingCircularHoleTopology(ctx,
			body, pl, directCircle, a, capStart, capEnd, walls)
		if err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// A surface result omits both caps from the shell (Table W,
	// docs/surface-design.md §4.1-§4.2): capStart/capEnd stay constructed above
	// so the area accumulation below can still read their area fields, but
	// neither joins the attached or published face set, which is what leaves
	// each section-plane rim edge with only its wall face free. topobuild.SheetLumps
	// splits the walls by connectivity — a holed loft's outer and hole wall
	// tubes touch nowhere once their shared cap is gone — and MUST run after
	// topobuild.AttachFaceLoops has registered every wall face on its own edges
	// (topobuild.ShellIsOpen's and topobuild.SheetLumps's doc comments); a solid loft keeps
	// the single-shell, single-lump body PR 1 already published, unchanged.
	if pl.surfaceResult {
		if err := topobuild.AttachFaceLoops(ctx, faceNodes(walls)); err != nil {
			return nil, err
		}
		body.n.Lumps = topobuild.SheetLumps(handleAlloc{}, faceNodes(walls))
	} else {
		faces := append([]*Face{capStart, capEnd}, walls...)
		if err := topobuild.AttachFaceLoops(ctx, faceNodes(faces)); err != nil {
			return nil, err
		}
		body.n.Lumps = []*topo.Lump{lumpNode(bornLump(&Lump{n: topo.Lump{Shells: []*topo.FaceShell{faceShellNode(bornFaceShell(&FaceShell{n: topo.FaceShell{Faces: faceNodes(faces)}}))}}}))}
	}

	mass := buildLoftMassWithWallAreas(pl, a, pairs, sectionDelta, sectionMatchedDelta, wallAreas)
	body.n.Volume = measurementToInternal(mass.volume(a.Verts, a.Tris))
	centroid, err := mass.centroid(a.Verts, a.Tris)
	if err != nil {
		return nil, err
	}
	body.n.Centroid = vecMeasurementToInternal(centroid)
	bounds, ok := mass.bounds()
	if !ok {
		return nil, fmt.Errorf(`%w: the loft has no vertices to bound`, ErrDegenerate)
	}
	body.n.Bounds = boxToInternal(bounds)
	body.n.Area = measurementToInternal(mass.area(cap0Rat, cap1Rat))
	if circularWall != nil {
		var wallArea proofbound.BoundedScalar
		if directCircle != loftCircleWallCylinder {
			bottom, bottomOK := pl.profile0.Holes[0].Segments[0].(circleSeg)
			top, topOK := pl.profile1.Holes[0].Segments[0].(circleSeg)
			if !bottomOK || !topOK {
				return nil, fmt.Errorf(`%w: a circular loft wall lost its circle pair`, ErrUnsupported)
			}
			var ok bool
			switch directCircle {
			case loftCircleWallCone:
				wallArea, ok = loftConeWallArea(pl, bottom, top)
			case loftCircleWallRuled:
				wallArea, ok = loftMovingCircleWallArea(pl, bottom, top)
			case loftCircleWallQuarterTurn:
				wallArea, ok = loftQuarterTurnWallArea(pl, bottom, top)
			}
			if !ok {
				return nil, fmt.Errorf(`%w: the circular loft wall area has no finite bound`, ErrUnsupported)
			}
		} else {
			// The Loft mass proof bounds the complete true surface. The two
			// circular-hole caps and every retained straight outer wall each
			// bound their own true areas, so outward subtraction isolates the
			// single curved hole wall without treating another face as exact.
			wallArea = proofbound.BoundedSub(
				proofbound.BoundedSub(measurementScalar(measurementFromInternal(body.n.Area)),
					proofbound.MeasuredScalar(capStart.n.Area, capStart.n.AreaBound)),
				proofbound.MeasuredScalar(capEnd.n.Area, capEnd.n.AreaBound))
			for _, face := range walls {
				if face == circularWall {
					continue
				}
				wallArea = proofbound.BoundedSub(wallArea,
					proofbound.MeasuredScalar(face.n.Area, face.n.AreaBound))
			}
		}
		if proofbound.IsNonFinite(wallArea.Value) || proofbound.IsNonFinite(wallArea.Bound) {
			return nil, fmt.Errorf(`%w: the circular loft wall area has no finite bound`, ErrUnsupported)
		}
		circularWall.n.Area, circularWall.n.AreaBound = wallArea.Value, wallArea.Bound
	}
	if pl.surfaceResult {
		// §4.3: a surface result's area is the solid's area minus the two
		// omitted caps'. Each cap's area and bound are read back from the
		// Face fields buildLoftTopology stamped, which already compose the
		// cap's own placement allowance (loftmesh.CapTriangleAreaAllow) — never
		// re-derived here — and the subtraction runs LAST, after the mass
		// accumulator's own area (walls plus both caps) has been read, so
		// nothing after this point still widens capStart's or capEnd's own
		// bound out from under a value already composed from them.
		// Collapsing this to the wall triangles' own sum instead (mass.area()
		// called with no cap terms) would hold the same VALUE — the two caps
		// cancel exactly, since they are the identical terms mass.area()
		// added in — but a SMALLER, uncomposed bound: the wall sum alone
		// carries none of the two caps' own rounding, which §4.3 requires
		// the sheet's bound to include.
		areaScalar := proofbound.BoundedSub(
			proofbound.BoundedSub(measurementScalar(measurementFromInternal(body.n.Area)), proofbound.MeasuredScalar(capStart.n.Area, capStart.n.AreaBound)),
			proofbound.MeasuredScalar(capEnd.n.Area, capEnd.n.AreaBound),
		)
		// Published Approximate EXPLICITLY, never derived from the composed
		// bound: the solid loft's own mass.area() always publishes
		// Approximate (loft_moments.go — a triangle's area is a square root,
		// never exactly representable), and a sheet built from the SAME wall
		// triangles must never claim more exactness than the body it came
		// from, whatever its own composed bound happens to round to.
		body.n.Area = measurement.Measurement{
			Value:     units.SquareMillimeters(areaScalar.Value),
			Exactness: measurement.Approximate,
			Bound:     units.SquareMillimeters(areaScalar.Bound),
		}
	}

	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}

	pl.verts, pl.tris, pl.walls = a.Verts, a.Tris, a.Walls
	if pl.sourceIdentity == nil {
		pl.sourceIdentity = &loftSourceIdentity{origin: ref}
	}
	pl.capStartCount, pl.cell, pl.side = a.CapStartCount, a.Cell, a.Side
	pl.segment = make([][]int, len(pairs))
	for i := range pairs {
		pl.segment[i] = append([]int(nil), pairs[i].Segment...)
	}
	if pl.retainTwistAreaCells {
		pl.twistAreaCells = make([][]twistsweep.TwistAreaCellClaims, len(pairs))
		for i, pair := range pairs {
			pl.twistAreaCells[i] = make([]twistsweep.TwistAreaCellClaims, len(pair.Segment))
			for j := range pair.Segment {
				pl.twistAreaCells[i][j] = twistsweep.TwistAreaCellClaims{
					ArcLength: max(pair.ArcUpperV[j], pair.ArcUpperW[j]),
					Energy:    max(pair.TangentEnergyV[j], pair.TangentEnergyW[j]),
					Matched:   pair.MatchedDelta[j],
				}
			}
		}
	}
	pl.proof = loftmesh.MeshProofOf(a, mass.MassAccumulator, sectionMatchedDelta)
	pl.delta = a.Delta
	// sectionDelta is loftmesh.PairRecords' own accumulated MAX over cells
	// (loftPayload's own doc comment): zero for a LineSeg-only pairing,
	// positive for a same-kind circular one, to the sagitta its station
	// chording commits.
	pl.sectionDelta = sectionDelta
	pl.chorded = loftmesh.IsChorded(pairs, sectionDelta, sectionMatchedDelta)
	body.n.Payload = pl
	return body, nil
}

// buildLoftMass folds the assembled triangle set into the mass accumulator
// and, on a chorded build, attaches the chorded proof terms. evalLoft calls it
// once per build; it is its own function so the volume and area tests can read
// the same accumulator at a chord target of their choosing.
//
// docs/loft-design.md §5.2's matchedDelta row is composed here and nowhere
// else: proofbound.AbsSumUpper of the build's own MAX-over-cells chord-to-curve
// departure (loftmesh.PairRecords' sectionMatchedDelta) and the held vertex
// displacement a.Delta. The two halves are accumulated apart — a chord's
// departure from the curve it chords, and a station's departure from the
// point the record and the motion denote for it — and every chorded leg
// charges their SUM, since the chord the build DREW joins two displaced
// stations. The sagitta alone would leave the computed station's own
// displacement uncharged (§5.2's matchedDelta paragraph). Left at exactly 0 on
// a build with no chorded cell, which is what keeps a LineSeg-only pairing's
// published measurements free of every chorded term: its held triangle pair IS
// the boundary §5 gives it, and a.Delta already reaches its measurements
// through the accumulator's own delta-keyed legs.
//
// A build is chorded if any cell is not faceted, including a degree-1
// free-form cell with zero departure. Its bilinear patch still needs the
// exact correction and twist legs. The chorded terms remain zero for a
// LineSeg-only build, whose cells are all faceted.
func buildLoftMass(pl loftPayload, a loftmesh.Assembly, pairs []loftmesh.LoopPair, sectionDelta, sectionMatchedDelta float64) *loftMassAccumulator {
	return buildLoftMassWithWallAreas(pl, a, pairs, sectionDelta, sectionMatchedDelta, nil)
}

func buildLoftMassWithWallAreas(pl loftPayload, a loftmesh.Assembly, pairs []loftmesh.LoopPair, sectionDelta, sectionMatchedDelta float64, wallAreas []loftmesh.WallArea) *loftMassAccumulator {
	anchor := pl.xform.Apply(pl.plane0.Origin)
	chorded := loftmesh.IsChorded(pairs, sectionDelta, sectionMatchedDelta)
	matchedDelta := 0.0
	if chorded {
		matchedDelta = loftmesh.ChordCellDeltaUpper(sectionMatchedDelta, a.Delta)
	}
	mass := newLoftMassAccumulator(anchor, a.Delta, sectionDelta, matchedDelta)
	if wallAreas == nil {
		mass.AddMesh(a.Verts, a.Tris, a.Walls)
	} else {
		mass.AddMeshWithWallAreas(a.Verts, a.Tris, wallAreas)
	}
	if chorded {
		if wallAreas == nil {
			mass.Chorded = loftmesh.ComputeLoftChordedAllow(pairs, a.VIdx, a.WIdx, a.Verts, anchor, matchedDelta, a.Delta, a.Reversed)
		} else {
			mass.Chorded = loftmesh.ComputeLoftChordedAllowWithWallAreas(pairs, a.VIdx, a.WIdx, a.Verts, anchor, matchedDelta, a.Delta, a.Reversed, wallAreas)
		}
	}
	return mass
}

// tessellateLoft restates a lofted body's held triangle set as a Mesh
// (docs/tessellation-design.md §2's "loftPayload exact restatement", §4's
// source-face table; docs/tessellation-reach-design.md §4).
// internal/loftmesh.RestateLoft copies the set, names each triangle's
// face by its provenance role, publishes the payload's own proof record and
// audits the result; this adapter numbers the live faces for it and maps the
// numbers back to *Face.
//
// It takes no chord tolerance, and that is the design's own reading rather
// than an omission: the path adds no chording of its own, so the whole
// published Bound is inherited payload displacement, which §1's Tolerance row
// lets ride above tol exactly as a prism's per-end axial displacement does.
// Orientation needs no work either: docs/loft-design.md §5's whole-shell step
// already turned every triangle outward, and placed re-runs that step, so
// repeating it would reverse a mirrored shell twice. A solid runs closure and
// signed-volume audits; a sheet runs the manifold-with-boundary and
// vertex-link audits instead, because the signed sum is anchor-dependent
// without caps. A missing source role is ErrDegenerate because the live
// topology contradicts its payload.
func tessellateLoft(ctx context.Context, b *Body, lp loftPayload, privateRoles ...map[string]*Face) (*Mesh, error) {
	faces := b.Faces()
	number := make(map[*Face]int, len(faces))
	faceOfRole := map[string]int{}
	for i, f := range faces {
		number[f] = i
		for _, o := range f.Origins() {
			faceOfRole[o.Role] = i
		}
	}
	if len(privateRoles) != 0 {
		faceOfRole = make(map[string]int, len(privateRoles[0]))
		for role, face := range privateRoles[0] {
			i, ok := number[face]
			if !ok {
				return nil, fmt.Errorf(`%w: a held loft source has no live face`, ErrDegenerate)
			}
			faceOfRole[role] = i
		}
	}
	sheet := b.Kind() == BodySheet
	in := loftmesh.LoftInput{
		Vertices: lp.verts, Triangles: lp.tris,
		WallCount: lp.walls, StartCapCount: lp.capStartCount,
		WallCell: lp.cell, WallSide: lp.side,
		Sheet:      sheet,
		FaceOfRole: faceOfRole, StartCapRole: provenance.RoleCapStart, EndCapRole: provenance.RoleCapEnd,
		FacetDepartureMM: lp.proof.FacetDeparture,
		AreaSlackMM2:     lp.proof.AreaSlack,
		VolumeSymDiffMM3: lp.proof.VolSymDiff,
	}
	if sheet {
		counts := freeChainCountsByFace(b)
		in.FreeChainCounts = make(map[int]int, len(counts))
		for f, n := range counts {
			in.FreeChainCounts[number[f]] = n
		}
	} else {
		// The same anchor evalLoft used (docs/loft-design.md §5's whole-shell
		// orientation rule).
		in.Anchor = lp.xform.Apply(lp.plane0.Origin)
	}
	out, err := loftmesh.RestateLoft(ctx, in)
	if err != nil {
		return nil, liftTessellationError(err)
	}
	src := make([]*Face, len(out.SourceFaces))
	for k, i := range out.SourceFaces {
		src[k] = faces[i]
	}
	mesh := &Mesh{n: topo.Mesh{Vertices: out.Vertices, Triangles: out.Triangles}, source: src}
	for _, f := range src {
		mesh.setFaceBound(f, out.BoundMM)
	}
	mesh.n.Bound = out.BoundMM
	mesh.n.AreaSlack = out.AreaSlackMM2
	if out.VolumeProof {
		mesh.n.VolSymDiff = out.VolumeSymDiffMM3
		mesh.n.SymDiffOK = true
	}
	return mesh, nil
}
