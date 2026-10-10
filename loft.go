package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/loftmesh"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// This file is docs/loft-design.md PR 1b: the public entry point over PR 1a's
// evaluator (loft_build.go). It owns the LoftOption surface, gates S9-S10,
// and the atomic record->evaluate->commit tail (§10). internal/featureoption
// owns S11 and S4's arity half. The other gates — S1-S8 and S13's coordinate-range
// gate — is loftmesh.ValidateLoftRecords', assembleLoft's and loftmesh.LoftCrossingAuditStructured's, run
// inside evalLoft in §4's stated order.

// LoftOption configures Loft.
type LoftOption = featureoption.LoftOption

// WithLoftAlignment records, per loop, which recorded segment index of the
// SECOND profile pairs with segment index 0 of the FIRST profile's
// corresponding loop. offsets[0] is the outer loop; offsets[1+h] is
// p1.Holes[h]. Omitting the option means every offset is 0 — the natural
// case where both profiles were authored with matching segment order
// (docs/loft-design.md §2/§3 Table P row P4).
//
// The option is accepted at most once: two payloads are two different
// correspondences, so a repeat is [ErrDegenerate] rather than last-wins
// (Table S row S4).
func WithLoftAlignment(offsets ...int) LoftOption {
	return featureoption.WithLoftAlignment(offsets...)
}

// Loft builds a solid ruled between two profiles recorded on distinct
// geometric planes (docs/loft-design.md), registers it, and returns the new
// body. s0/p0 is the FROM section (capStart); s1/p1 is the TO section
// (capEnd) — the same naming Extrude already uses for its two caps. Both
// profiles pass the unmodified core §7 seam gates independently, in argument
// order (p0 of s0, then p1 of s1): a foreign, stale, invalid, or
// unrecordable profile is the seam's own sentinel.
//
// Every corresponding segment pair must be same-kind, over the recorded
// segment type: both LineSeg, both ArcSeg, both CircleSeg, or both the same
// Tier A free-form type (SplineSeg, ClosedSplineSeg, a NURBSSeg with equal
// weights, or FitSplineSeg). An ArcSeg paired against a CircleSeg, or a
// SplineSeg against a FitSplineSeg, is a mixed-kind pairing like any other,
// and a mixed-kind pairing is [ErrUnsupported] (§1, P5, S3). A free-form pair
// whose two curves convert to different Bézier span counts is
// [ErrUnsupported] too (S17). A circular or free-form pair's walls are chorded
// (§5.1), which carries three refusals of its own, each [ErrUnsupported]: a
// pair the fixed station cap cannot chord to its chord target (S15), a build
// whose certified sagitta or station displacement has no derivation from the
// two records (S14), and a chord cell that collapses to one point on exactly
// one of the two sections (S16).
// The two profiles must lie on distinct geometric planes;
// coplanar sections are [ErrDegenerate] (§4, S5), since every wall vertex
// would then lie in one plane and the solid has zero volume by construction.
// A hole-count or per-loop segment-count mismatch has no positional or
// one-to-one pairing and is [ErrUnsupported] (S1/S2). A build-time audit
// proves the ruled walls do not cross or self-touch anywhere but their
// recorded shared edges and vertices (§6); a proven crossing is
// [ErrDegenerate] (S7's audit arm). A circular pair whose two sides walk in
// opposite senses is that same crossing — the correspondence walls each side
// against the other's reversed walk — and is the same [ErrDegenerate] (S7's
// structural arm), decided from the two records before any triangle is built.
// Exhausting the audit's fixed pair-test budget is
// [ErrUnsupported] (S8). A section point whose world coordinate runs past the
// representable float64 range is [ErrUnsupported] (S13) — the body exists,
// and this evaluator cannot hold its vertex table. WithSurfaceResult() omits
// both section caps and publishes a sheet body instead of a solid
// (docs/surface-design.md §4).
//
// A failed call leaves the document untouched.
func (d *Document) Loft(ctx context.Context, s0 *sketch.Sketch, p0 *sketch.Profile, s1 *sketch.Sketch, p1 *sketch.Profile, opts ...LoftOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a loft`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	// S10: an explicit nil check ahead of the seam, so the message names
	// which of the four arguments was nil (RecordProfile would answer the
	// same sentinel, but less specifically).
	if s0 == nil || p0 == nil || s1 == nil || p1 == nil {
		return nil, fmt.Errorf(`%w: Loft requires two non-nil sketches and two non-nil profiles`, ErrDegenerate)
	}

	// S11 (option ownership) and S4's ARITY half (a repeated
	// WithLoftAlignment), both pre-gates decidable with no record at all
	// (docs/loft-design.md §4, amended). decad owns the option vocabulary, so
	// a foreign concrete type — including one that embeds LoftOption to
	// promote the sealed marker — is rejected before its Ident() ever runs.
	cfg, err := featureoption.DecodeLoft(opts)
	if err != nil {
		return nil, err
	}
	alignment, surfaceResult := cfg.Alignment, cfg.SurfaceResult

	// S9: both profiles through the unmodified seam gates, in argument order.
	profile0, plane0, area0, err := recordProfile(s0, p0)
	if err != nil {
		return nil, err
	}
	profile1, plane1, area1, err := recordProfile(s1, p1)
	if err != nil {
		return nil, err
	}

	// evaluator §1's one live-profile read, reject-only (not a new gate —
	// docs/loft-design.md §2 states no new gate here; both Extrude and
	// Revolve run the identical falsifier on their own recorded profile). One
	// counter per profile, threaded through to evalLoft so a single loft
	// operation opens exactly two R7 ceilings — one per record — rather than
	// four (docs/spline-design.md §5.2).
	// The two integrals the falsifier computes are also the records' own
	// areas the chord target reads (docs/loft-gear-bounds-design.md §5), so
	// they ride on the payload rather than being integrated again in evalLoft.
	work0 := freeform.NewFreeformWork()
	work1 := freeform.NewFreeformWork()
	recordArea0, err := falsifyRecordedArea(profile0, area0, work0)
	if err != nil {
		return nil, err
	}
	recordArea1, err := falsifyRecordedArea(profile1, area1, work1)
	if err != nil {
		return nil, err
	}

	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the first recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the second recorded plane is degenerate: %s`, ErrDegenerate, err)
	}

	ref := d.nextProducerID()

	body, err := evalLoft(ctx, d, ref, loftPayload{
		profile0: profile0, profile1: profile1,
		plane0: plane0, plane1: plane1,
		frame0: frame0, frame1: frame1,
		alignment:     alignment,
		xform:         r3.Identity(),
		surfaceResult: surfaceResult,
		recordArea:    [2]float64{recordArea0, recordArea1},
	}, proofbound.NewWorkBudget(ctx), work0, work1)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// This section is LoftChain of docs/loft-design.md §16: the ruled ribbon
// between two OPEN sketch chains. §16.1 states how Table P reads over two open
// walks — P1 and P2 collapse onto one walk with no holes, P3 and P5 survive
// verbatim, P4 loses its modular wrap so segment j pairs with segment j, and
// P6's intrinsic winding is replaced by the walk direction sketch itself
// publishes. §16.2 states the one thing a closed shell supplied that two open
// walks do not — the positive side — and the exactly-parallel plane gate that
// replaces it.

// ChainLoftOption configures LoftChain. It is its own sealed tier rather than
// [LoftOption]: WithSurfaceResult() does not implement it, so the compiler
// refuses that option outright rather than accepting it as a no-op — a
// chain-fed loft always returns a sheet, so there is no cap for the option to
// omit (docs/surface-design.md §13.2, docs/loft-design.md §16.3).
// WithLoftAlignment is not a member either: Table P's own P4 row forces a
// chain pair's offset to 0, so the option would name a correspondence that
// does not exist. This placeholder tier has no member yet; it reserves the
// call for a later chain-only option.
type ChainLoftOption interface {
	chainLoftOption()
}

// chainLoftPayload is LoftChain's own record of a ribbon body: the two
// recorded open walks, the two planes and frames they lift through, and the
// accumulated rigid placement. It stays DISTINCT from loftPayload because a
// chain-fed build mints no cap, carries no volume and runs no whole-shell
// orientation step, so a consumer dispatching on loftPayload would read three
// facts about it that are not true.
type chainLoftPayload struct {
	chain0, chain1 chainRecord
	plane0, plane1 planeRecord
	frame0, frame1 r3.Frame
	xform          r3.Transform
}

// transform is the accumulated rigid placement.
func (lp chainLoftPayload) transform() r3.Transform { return lp.xform }

// placed re-evaluates the same two records under the composed motion (core
// §8), re-running every gate decided from the records and rebuilding from
// scratch, exactly as docs/loft-design.md §4 states a loft placement does.
func (lp chainLoftPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	lp.xform = composed
	return evalChainLoftContext(ctx, d, ref, lp, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork())
}

// LoftChain builds a sheet ruled between the open chains c0 (of s0) and c1 (of
// s1), registers it, and returns the new body. c0 is the FROM walk and c1 the
// TO walk, the same argument order Loft uses for its two sections and the
// order §16.2's plane gate reads its positive normal from. Both chains pass
// the identical seam gates ExtrudeChain runs (RecordChain,
// docs/surface-design.md §13.3), in argument order, and the seam's own
// sentinel wins before any shape gate.
//
// The two walks MUST carry the same recorded segment count (ErrUnsupported),
// and segment j of c0 pairs with segment j of c1 — there is no alignment
// offset for an open walk, since a nonzero one would pair a segment across a
// free end. Every paired segment MUST be a LineSeg pair in this increment: a
// mixed-kind pairing and a same-kind circular pairing are each ErrUnsupported
// (docs/loft-design.md Table SL rows SL4 and SL7). The two recorded planes
// MUST be exactly parallel with the to-plane strictly on the from-plane's
// positive side, which is what states the ribbon's positive side
// (ErrUnsupported, SL5); coplanar sections are ErrDegenerate (S5). A build-time
// audit proves the ruled walls cross nowhere but their own shared edges and
// vertices, and a proven crossing is ErrDegenerate (S7) — which is what
// refuses a pair whose two published walks run opposite ways.
//
// The result is always a sheet — Kind() == BodySheet — two flat triangles per
// chord cell with no cap and no closing face, so WithSurfaceResult() does not
// compile against this call. A failed call leaves the document untouched.
func (d *Document) LoftChain(ctx context.Context, s0 *sketch.Sketch, c0 *sketch.Chain, s1 *sketch.Sketch, c1 *sketch.Chain, opts ...ChainLoftOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a loft`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if s0 == nil || c0 == nil || s1 == nil || c1 == nil {
		return nil, fmt.Errorf(`%w: LoftChain requires two non-nil sketches and two non-nil chains`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// SL2 before every shape gate, in argument order: a seam refusal names a
	// repair the caller makes in the sketch, and a shape refusal reported
	// first would hide it (docs/loft-design.md Table SL).
	chain0, plane0, err := recordChain(s0, c0)
	if err != nil {
		return nil, err
	}
	chain1, plane1, err := recordChain(s1, c1)
	if err != nil {
		return nil, err
	}

	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the first recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the second recorded plane is degenerate: %s`, ErrDegenerate, err)
	}

	ref := d.nextProducerID()
	body, err := evalChainLoftContext(ctx, d, ref, chainLoftPayload{
		chain0: chain0, chain1: chain1,
		plane0: plane0, plane1: plane1,
		frame0: frame0, frame1: frame1,
		xform: r3.Identity(),
	}, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}

// evalChainLoftContext builds the ribbon body: the two station chains, the two
// wall triangles per chord cell, the crossing audit over that set, the B-rep
// topology with no cap, and the two readings a sheet publishes
// (docs/loft-design.md §16.5).
//
// It never runs §5's whole-shell orientation step. That step reads the signed
// tetrahedron sum over a CLOSED triangle set, which this build does not
// assemble; loftmesh.ChainLoftPlaneSideGate is what states the side instead, and the
// local winding below is emitted unflipped because that gate has already
// proven it correct.
func evalChainLoftContext(ctx context.Context, d *Document, ref producerID, lp chainLoftPayload, budget *proofbound.WorkBudget, work0, work1 *freeform.FreeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	walks0, walks1, err := loftmesh.ValidateChainLoftRecords(lp.chain0, lp.chain1, lp.plane0, lp.plane1, work0, work1)
	if err != nil {
		return nil, err
	}
	v, w, stationRound, err := loftmesh.ChainLoftStations(walks0, walks1)
	if err != nil {
		return nil, err
	}

	// S13, decided before the first coordinate is lifted into an exact dyadic,
	// exactly as assembleLoft decides it: tessellation.OrientationSign is not run here,
	// but the mass accumulator and the audit both lift every vertex through
	// dyVec, whose mustDyOf PANICS on a non-finite float.
	anchor := lp.xform.Apply(lp.plane0.Origin)
	if !proofbound.FiniteVec(anchor) {
		return nil, errLoftPointUnrepresentable("placed plane origin")
	}

	verts := make([]r3.Vec, 0, 2*len(v))
	maxInputAbs := 0.0
	for _, side := range []struct {
		pts   []Point2
		frame r3.Frame
		what  string
	}{{v, lp.frame0, "first"}, {w, lp.frame1, "second"}} {
		for j, pt := range side.pts {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			lifted := side.frame.ToWorldUV(pt.U, pt.V)
			maxInputAbs = math.Max(maxInputAbs, proofbound.VecMaxAbs(lifted))
			placed := lp.xform.Apply(lifted)
			if !proofbound.FiniteVec(placed) {
				return nil, errLoftPointUnrepresentable(fmt.Sprintf("placed station %d on the %s walk", j, side.what))
			}
			verts = append(verts, placed)
		}
	}

	// placeAllow is zero exactly when xform is the identity transform — an
	// exact struct comparison, never a tolerance — and stationRound is the
	// independent leg beside it, exactly as assembleLoft composes the two.
	placeAllow := 0.0
	if lp.xform != r3.Identity() {
		placeAllow = proofbound.RigidRoundAllow(maxInputAbs, proofbound.VecMaxAbs(lp.xform.Translation()))
	}
	delta := proofbound.AbsSumUpper(stationRound, placeAllow)

	n := len(v) - 1
	vIdx := make([]int, len(v))
	wIdx := make([]int, len(w))
	for j := range v {
		vIdx[j] = j
		wIdx[j] = len(v) + j
	}
	tris := make([][3]int, 0, 2*n)
	for j := range n {
		tris = append(tris,
			[3]int{vIdx[j], vIdx[j+1], wIdx[j+1]},
			[3]int{vIdx[j], wIdx[j+1], wIdx[j]},
		)
	}

	if err := loftmesh.LoftCrossingAudit(budget, verts, tris); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	walls, err := buildChainLoftTopology(ctx, body, ref, verts, tris, vIdx, wIdx, delta)
	if err != nil {
		return nil, err
	}
	if err := attachFaceLoopsContext(ctx, walls); err != nil {
		return nil, err
	}
	body.lumps = sheetLumps(walls)

	mass := newLoftMassAccumulator(anchor, delta, 0, 0)
	for _, t := range tris {
		mass.add(verts[t[0]], verts[t[1]], verts[t[2]], true)
	}
	// No cap rational is passed: this build mints no cap, so the reading is
	// the wall sum alone rather than a sum with the two caps subtracted back
	// off it the way a surface-result Loft's is (docs/loft-design.md §16.5).
	body.area = mass.area()
	bounds, ok := mass.bounds()
	if !ok {
		return nil, fmt.Errorf(`%w: the chain loft has no vertices to bound`, ErrDegenerate)
	}
	body.bounds = bounds
	// volume and centroid stay at their zero value: finite, so the validation
	// below passes, and neither is reachable through Body.Volume/Body.Centroid
	// while solid is false, exactly as evalChainExtrudeContext leaves them.
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = lp
	return body, nil
}

// buildChainLoftTopology builds the ribbon's B-rep over the assembled triangle
// set: the same four edge families buildLoftTopology builds for a closed loft
// — bottom rim, top rim, diagonal and rung — with the wrap dropped everywhere.
//
// An n-cell walk carries n bottom rims, n top rims, n diagonals and n+1 rungs.
// Every diagonal and every INTERIOR rung bounds two triangles; the n rims of
// each walk and the two END rungs bound one each, which is the 2n+2 free edges
// docs/loft-design.md §16.5 states. No loop is closed back to its first
// station and no cap loop is built, so nothing here reads a wrap index.
//
// A free end rung takes the concave default rather than a measured sense: only
// an interior rung has two incident triangles to cross, so a free end carries
// no turn of its own — the identical rule buildChainSides applies to a chain
// prism's own free-end sweep edges. Each walk's rims take the outer-loop
// convexity convention, since a chain walk is loop 0 throughout
// (docs/surface-design.md §13.4).
func buildChainLoftTopology(ctx context.Context, body *Body, ref producerID, verts []r3.Vec, tris [][3]int, vIdx, wIdx []int, delta float64) ([]*Face, error) {
	n := len(vIdx) - 1
	vertexObjs := make([]*Vertex, len(verts))
	for i, p := range verts {
		vertexObjs[i] = loftVertex(p, delta)
	}

	lowerTri := make([][3]int, n)
	upperTri := make([][3]int, n)
	for j := range n {
		lowerTri[j] = tris[2*j]
		upperTri[j] = tris[2*j+1]
	}

	rimBottom := make([]*Edge, n)
	rimTop := make([]*Edge, n)
	diagE := make([]*Edge, n)
	rungE := make([]*Edge, n+1)
	for j := range n {
		rimBottom[j] = loftEdge(vertexObjs, verts, vIdx[j], vIdx[j+1], true, delta)
		rimTop[j] = loftEdge(vertexObjs, verts, wIdx[j], wIdx[j+1], true, delta)
		diagE[j] = loftEdge(vertexObjs, verts, vIdx[j], wIdx[j+1],
			loftmesh.JunctionConvex(verts, lowerTri[j], upperTri[j], vIdx[j], wIdx[j+1]), delta)
	}
	for j := 0; j <= n; j++ {
		convex := false
		if j > 0 && j < n {
			convex = loftmesh.JunctionConvex(verts, lowerTri[j-1], upperTri[j], vIdx[j], wIdx[j])
		}
		rungE[j] = loftEdge(vertexObjs, verts, vIdx[j], wIdx[j], convex, delta)
	}

	walls := make([]*Face, 0, 2*n)
	for j := range n {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		lowerFace, err := buildLoftWallFace(body, ref, verts, lowerTri[j], 0, j, 0, delta)
		if err != nil {
			return nil, err
		}
		lowerFace.loops = []*Loop{{outer: true, coedges: []coedge{
			{edge: rimBottom[j], forward: true},
			{edge: rungE[j+1], forward: true},
			{edge: diagE[j], forward: false},
		}}}
		walls = append(walls, lowerFace)

		upperFace, err := buildLoftWallFace(body, ref, verts, upperTri[j], 0, j, 1, delta)
		if err != nil {
			return nil, err
		}
		upperFace.loops = []*Loop{{outer: true, coedges: []coedge{
			{edge: diagE[j], forward: true},
			{edge: rimTop[j], forward: false},
			{edge: rungE[j], forward: false},
		}}}
		walls = append(walls, upperFace)
	}
	return walls, nil
}
