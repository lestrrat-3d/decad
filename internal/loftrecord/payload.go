package loftrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file is the loft payload docs/loft-design.md PR 1a states: the record
// Build evaluates over topology nodes and a placement re-evaluates. It
// implements topo.Payload (docs/evaluator-design.md §8).

// TwistAreaCellClaims comes from the same authenticated Loft pairing that
// emitted a private Sweep cell's two wall triangles. The circular arm has a
// uniform source parameter on each chord cell; the line arm is affine.
type TwistAreaCellClaims struct {
	ArcLength float64 // upper bound on integral |source derivative| over s in [0,1]
	Energy    float64 // upper bound on integral |source derivative - exact chord|^2
	Matched   float64 // upper bound on source curve - exact chord at the same s
}

// SourceIdentity names the producer of the original Loft. It is shared only
// by that Loft and its own placed copies.
type SourceIdentity struct{ Origin provenance.Producer }

// Payload is the evaluator's own record of a lofted body: the two
// authenticated sections, their planes and frames, the per-loop alignment,
// the accumulated rigid placement, and the assembled triangle set the
// construction produced (docs/loft-design.md §5/§7). Verts/Tris/Walls are the
// same globally oriented triangle set §6's audit classified and §8's
// accumulator integrated — Tris[:Walls] the wall triangles (side(i,j,k)), the
// rest the two caps' own triangulations — kept on the payload so a later
// Tessellate (PR 2) restates it rather than rebuilding it.
type Payload struct {
	// SourceIdentity is shared only by the original Loft and its own placed
	// copies. It lets a structural consumer require one authenticated source.
	SourceIdentity     *SourceIdentity
	Profile0, Profile1 momentinput.Profile
	Plane0, Plane1     sectionrecord.PlaneRecord
	Frame0, Frame1     r3.Frame
	Alignment          []int
	Xform              r3.Transform
	// AuthenticatedReconstruction is set only after the public Loft seam has
	// admitted both Sketch profiles. Certified fillet and bore rewrites retain
	// that provenance when they re-evaluate their private loft records.
	AuthenticatedReconstruction bool
	// BlendSegs names connector segments inserted by Fillet. Every
	// triangle of a connector's loft cell also carries fillet(loop,segment).
	BlendSegs []map[int]struct{}
	// A fitted-spline fillet records float trim parameters and a float arc.
	// These fields enclose the ideal radius-r tangent rewrite certified at
	// construction. The first is the maximum matched plane displacement over
	// all rewritten carriers and connector arcs. The second bounds each ideal
	// connector's complete arc length, keyed by the outer-loop segment index.
	ConstructionDelta    float64
	ConstructionArcUpper map[int]float64
	// A trimmed carrier can gain length when its ideal tangent parameter
	// replaces the recorded float trim. Each value bounds that increase for
	// every cell of the named outer-loop segment.
	ConstructionLengthExtra map[int]float64
	// ThroughBoreCut marks a profile pair rewritten by Cut. The added hole
	// did not come from the two authenticated source Sketch profiles.
	ThroughBoreCut bool
	// AnalyticBore is set only by the certified whole-circle through Cut.
	// Its loft mesh still supplies mass and tessellation proofs, while the
	// public topology uses one cylindrical wall for the new hole.
	AnalyticBore bool

	// SurfaceResult is WithSurfaceResult's own flag (docs/surface-design.md
	// §4): true when the build must omit both section caps and publish a
	// sheet instead of a solid. It is part of the re-evaluable record for the
	// same reason prismPayload's and revolvePayload's own copies are
	// (internal/prismrecord, internal/revolveplan/build.go), so Transform and Copy
	// reproduce the sheet with no further code — a plain solid
	// loft leaves it false.
	SurfaceResult bool

	// RecordArea holds each record's own exact region integral, the value
	// falsifyRecordedArea computed and compared against sketch's area in
	// Loft. The chord target reads it (loftmesh.ChordTarget,
	// docs/loft-gear-bounds-design.md §5), so no evaluation integrates a
	// record a second time to size its chords. A rigid placement moves
	// neither record, so placed carries it over unchanged.
	RecordArea [2]float64

	// Delta is the proven displacement of every held vertex from the exact
	// point the record denotes for it (docs/loft-design.md §5, §12 PR 2a,
	// a10-plan.md Part 3 PR 6): stationRound, liftAllow and placeAllow summed
	// through proofbound.AbsSumUpper (loftmesh.Assemble). liftAllow is the
	// rounding each station's lift through its own section frame commits
	// (loftmesh.FrameLiftRoundAllow), zero only on an XY-aligned frame whose
	// origin lies on the Z axis. placeAllow is zero for an unplaced body —
	// pl.Xform == r3.Identity(), an exact struct comparison, never a
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
	Delta float64

	// SectionDelta is the proven upper bound on how far any BUILT CHORD point
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
	// the matchedDelta Build composes: loftmesh.PairRecords accumulates each
	// cell's own chord-to-curve departure into a MAX beside sectionDelta,
	// and BuildMass sums that MAX with the delta above through
	// loftmesh.ChordCellDeltaUpper before passing it — never this field — to
	// newLoftMassAccumulator and loftmesh.ComputeLoftChordedAllow (loft_moments.go),
	// which is where the skirt leg's, the centroid radius' and every other
	// build-wide matched argument comes from;
	// proofbound.CellChordCurveAreaUpper, the wall leg and the cap tube read the
	// same composition per cell, over the cell's own chord-to-curve half.
	// The raw matched quantity itself is a PER-BUILD LOCAL of Build and is
	// never a field here. What the payload stores instead is the proof
	// COMPOSED from it — the three terms of the proof field below, which the
	// tessellation restates (docs/tessellation-design.md §2's Payload
	// row). That keeps the property the local was protecting: a stored term
	// can disagree with the records only if it can outlive them, and none
	// here can, because placed nils the triangle fields and re-runs Build,
	// which recomputes the records' every quantity and rewrites the proof
	// beside the triangle set it speaks for. The two helpers above still have
	// exactly one production call site each — Build's own — so no caller
	// can reach them with the other quantity either.
	//
	// This field's OWN remaining spend is Bounds.Bound, a SET-distance
	// reading, which is what makes sectionDelta the term it correctly reads.
	// The cap-area tube is NOT one of them: §5.2's capAreaAllow row names the
	// matched term there, because a held cap polygon's own vertices are
	// displaced as well as chorded.
	SectionDelta float64

	// Chorded reports that the build holds a cell whose held triangle pair is
	// NOT the boundary it denotes: a circular or free-form cell, which stands
	// for a ruled patch (loftmesh.LoopPair.Faceted). It is true on a degree-1
	// free-form build whose sectionDelta is zero, so a consumer asking whether
	// the held mesh IS the surface reads this field beside sectionDelta.
	Chorded bool

	Verts []r3.Vec
	Tris  [][3]int
	Walls int
	// CapStartCount is how many of tris[walls:] belong to capStart; the rest
	// belong to capEnd. cell/side parallel tris[:walls] — cell[k] is that wall
	// triangle's {loop index i, cell index j} and side[k] its 0/1 half — so a
	// consumer can name the side(i,j,k) role of every wall triangle without
	// re-deriving the assembly's own split. All three are copied verbatim from
	// loftmesh.Assembly, whose own doc comment owns the convention.
	CapStartCount int
	Cell          [][2]int
	Side          []uint8
	// segment names the authenticated source segment of each wall cell.
	// It stays parallel to the section stations and is retained for structural
	// consumers that must omit a source wall without inferring its identity
	// from its sampled coordinates.
	Segment [][]int
	// twistAreaCells retains only the certified per-cell readings needed by a
	// private twisted Sweep comparison Loft. Ordinary Loft payloads omit them.
	TwistAreaCells       [][]TwistAreaCellClaims
	RetainTwistAreaCells bool

	// Proof is the mesh proof record docs/tessellation-design.md §2 states for
	// this payload, composed by Build from the same build the triangle set
	// above came out of (docs/tessellation-reach-design.md §4).
	Proof loftmesh.MeshProof
}

// RaiseReconstructionLimit lifts each counter's free-form reconstruction
// ceiling to the loft limit when the public Loft seam admitted both profiles.
// An untrusted payload keeps the ordinary ceiling.
func (pl Payload) RaiseReconstructionLimit(work ...*freeform.FreeformWork) {
	if !pl.AuthenticatedReconstruction {
		return
	}
	for _, counter := range work {
		counter.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	}
}

// RewriteWorkLimit is private to the certified fillet and centered-bore
// rewrites. Their extra section proof precedes the ordinary area and station
// work on the same counter. An untrusted payload keeps the ordinary ceiling.
func (pl Payload) RewriteWorkLimit(p uint64) uint64 {
	limit := loftmesh.StationWorkLimit(0, p)
	if pl.AuthenticatedReconstruction {
		limit *= 2
	}
	return limit
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/reportvocab), the name the evaluator has always published for it.
func (Payload) SurveyClass() string { return "loftPayload" }

// Transform is the accumulated rigid placement.
func (pl Payload) Transform() r3.Transform { return pl.Xform }

// ProvesSimple implements topo.SimpleProver. A payload with SurfaceResult true
// and SectionDelta zero admits leg 4 on an argument STRONGER than the prism's:
// the evaluator cannot return a loft body at ALL unless docs/loft-design.md
// §6's crossing audit already passed over the complete held triangle set —
// walls and both caps together — and non-self-intersection of a SUBSET (the
// walls alone, once the caps are omitted) follows from non-self-intersection
// of that superset with no further proof needed. A positive SectionDelta means
// the body denotes a curved surface the held chords are only WITHIN that
// displacement of, so simplicity of the chord mesh does not transfer to the
// curved surface it stands for, and the answer is undecided rather than
// violated — the identical reading a nonzero SectionDelta gives a prism,
// restated here because a loft's own displacement is section-plane rather than
// axial. A chorded loft at zero SectionDelta (a degree-1 free-form pair) is
// undecided for the same reason: each wall cell denotes the bilinear ruled
// patch through its four corners, and the audit cleared the triangle pair
// instead.
func (pl Payload) ProvesSimple(context.Context) bool {
	return pl.SurfaceResult && pl.SectionDelta == 0 && !pl.Chorded
}

// AxialDelta reports the displacement of every held loft vertex. Its planar
// caps are built from those vertices, so a ToFace stop against either cap
// inherits this bound.
func (pl Payload) AxialDelta() float64 { return pl.Delta }

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
func (pl Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	next := pl
	next.Xform = composed
	next.Verts, next.Tris, next.Walls = nil, nil, 0
	// The assembly-derived fields and the proof composed from them are cleared
	// beside the triangle set they speak for: Build rewrites all of them
	// from the re-lifted records, and a stale copy surviving the re-evaluation
	// is exactly the disagreement the payload's own doc comment forbids.
	next.CapStartCount, next.Cell, next.Side, next.Segment = 0, nil, nil, nil
	next.TwistAreaCells = nil
	next.Proof = loftmesh.MeshProof{}
	return Build(ctx, env, next, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork())
}
