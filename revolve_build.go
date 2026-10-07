package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds a revolve's body from its revolvePayload: the wall surface
// each recorded segment sweeps, the two cap faces a partial sweep closes with,
// the poles and seams a full sweep instead joins, and the measurements the
// finished body publishes.
//
// evalRevolveContextWork is the whole build and states the order.
// buildRevolveLoop is the per-loop walk, and revLoopParts is what it hands
// back so a full sweep and a partial one assemble the same parts differently
// rather than through two builds. Every face carries the bound its own
// surface was built from. See docs/evaluator-design.md §6.

// revolvePayload is the evaluator's own record of a revolved body: the
// recorded region, the plane frame, the oriented plane-local axis, the sweep
// interval (right-handed about the axis, phi1 > phi0; exactly 2π apart when
// full), and the accumulated rigid placement. Every measurement and the
// whole topology derive from it, which is what makes Placed exact: it
// re-evaluates the same payload under the composed motion
// (docs/evaluator-design.md §8).
//
// den is the ANGULAR twin of prismPayload's z0Delta/z1Delta: the exact angle
// each end of phi0/phi1 denotes, per revolve_denotation.go. phi0/phi1 are
// what the resolver HELD — a float64 the extent's own resolution rounded to
// (docs/evaluator-design.md §6's "held sweep angle") — and den is what the
// recorded AngularExtent itself DENOTES, exactly, wherever this evaluator can
// state it. A reading that folds phi0 or phi1 into a published measurement
// takes the per-end displacement den proves (phi0Delta/phi1Delta,
// angularDelta) rather than reading the held float as the truth; den's own
// zero value is the invalid denotation, so a payload literal built without it
// (a test fixture, or an extent this evaluator cannot yet denote exactly)
// falls back to the same magnitude envelope every consumer published before
// this field existed. Being a payload field it re-evaluates with the
// payload, so Placed and Duplicate carry it unchanged.
//
// surfaceResult is WithSurfaceResult's own flag (docs/surface-design.md §4):
// true when the build must publish a sheet instead of a solid. A full
// revolution mints no closing face at all, so the flag changes the face set
// only when full is false — a partial sweep omits its two caps — while it
// changes Kind() and every measurement derived from solid vs. sheet in every
// case, full sweep included (Table W). It is part of the re-evaluable record
// for the same reason prismPayload.surfaceResult is, so Placed, Duplicate and
// PlacedCopy reproduce the sheet with no further code; a plain revolve leaves
// it false. sweep_arc.go and sweep_composite_measure.go also build a
// revolvePayload literal and leave this field at its zero value.
type revolvePayload struct {
	profile       ProfileRecord
	frame         r3.Frame
	ax            axisFrame
	phi0, phi1    float64
	full          bool
	den           sweepDenotation
	xform         r3.Transform
	surfaceResult bool
	// sectionDelta is prismPayload's own §7 term over the MERIDIAN: the proven
	// upper bound on how far any recorded meridian coordinate sits from the
	// meridian its construction denotes. Only the SHEET readings charge it
	// today — the wall areas, through docs/surface-intersection-design.md
	// §7.1's fold into the axis-coordinate walk, and the box, through
	// extentBoundedAlong's fifth mechanism. The Pappus VOLUME and CENTROID do
	// not, so requireExactRevolveSection refuses a nonzero value at the solid
	// build rather than integrating a region over a section it cannot charge
	// (§3.4; §6's RS13). A revolve a caller draws directly leaves it zero and
	// every reading takes the path it takes today, bit for bit.
	sectionDelta float64
	// radialProof belongs to this exact profile and resolved axis. A path
	// replacing either must clear it; placement alone preserves both.
	radialProof bool
}

// requireExactRevolveSection is RS13's reject-only guard: the solid build
// integrates a volume and a centroid over the recorded meridian, and
// docs/surface-intersection-design.md §7.1 derives the section displacement's
// reach into the AREA and the BOX alone. The field is either zero or it is
// not, so the guard needs no tolerance and can only refuse.
//
// It is also what keeps auditAxisContact's own exact-leaf reading of a
// plane-local coordinate sound. That audit runs ONCE, at axis resolution, over
// the caller's own undisplaced profile, and its four regionSnapAllow figures
// are read in exactly one place — evalRevolveContextWork's region integrals
// below, every one of them past this guard. A trimmed body reuses the
// receiver's already-resolved axis and never re-runs the audit, so no
// displaced meridian reaches it by either route.
func requireExactRevolveSection(rp revolvePayload, what string) error {
	if rp.sectionDelta == 0 {
		return nil
	}
	return fmt.Errorf(
		`%w: %s cannot integrate a region over a meridian carrying its own section displacement of %v mm`,
		ErrUnsupported, what, rp.sectionDelta)
}

// meridian is a *view* of rp as a prismPayload carrying the recorded MERIDIAN
// and the frame and placement it is expressed in — never a body this evaluator
// builds from. It is what docs/surface-intersection-design.md §3.1 hands
// buildPrismScene, newPrismReexpression, prismcells.Classify and the rest of
// §3's resolution, every one of which reads a profile, a frame and a placement
// and nothing else. The sweep fields are deliberately absent: the revolve's own
// angular interval is S6's business, never the private 2D scene's, and the
// levels a prismPayload would carry have no meaning for a meridian.
func (rp revolvePayload) meridian() prismPayload {
	return prismPayload{profile: rp.profile, frame: rp.frame, xform: rp.xform}
}

// transform is the accumulated rigid placement.
func (rp revolvePayload) transform() r3.Transform { return rp.xform }

// placed re-evaluates the same record under the composed motion.
func (rp revolvePayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	rp.xform = composed
	return evalRevolveContext(ctx, d, ref, rp)
}

// basis derives the sweep basis from the plane frame and the axis frame.
func (rp revolvePayload) basis() revolvemesh.RevolveBasis {
	a3 := rp.frame.ToWorldUV(rp.ax.aU, rp.ax.aV)
	w := rp.frame.U().Scale(rp.ax.dU).Add(rp.frame.V().Scale(rp.ax.dV))
	e0 := rp.frame.U().Scale(-rp.ax.dV).Add(rp.frame.V().Scale(rp.ax.dU))
	return revolvemesh.RevolveBasis{A3: a3, W: w, E0: e0, E1: w.Cross(e0)}
}

// revolveVertexFrameLiftAllow bounds one junction's own share of the
// payload's frame lift and accumulated placement rounding
// (internal/proofbound/bounds.go's proofbound.FrameAndPlacementRoundAllow; topology.go's Vertex.Position
// contract) — the revolve's own reading of proofbound.RigidRoundAllow's "plane-local
// coordinate" input. A swept vertex's plane-local (z, ρ) sits within
// axisRadiusUpper of the axis anchor (axisFrame.walk's own
// axisRadiusUpper field, ax.radialUpper(coordUpper) — the SAME enclosure
// that bounds |z| and ρ alike, revolve_extent.go's own frameRoundAllow
// states why), and the anchor itself sits within aUpper of the frame
// origin, so |z| and ρ are each within aUpper+axisRadiusUpper of the frame
// origin's own plane-local coordinate — folded in twice, once per axis,
// since z and ρ are independent coordinates rather than one bounding the
// other. It is exactly zero for an axis-aligned, unplaced revolve, which is
// what keeps its junction and seam vertices Exact as before.
func revolveVertexFrameLiftAllow(rp revolvePayload, axisRadiusUpper float64) float64 {
	aUpper := math.Max(math.Abs(rp.ax.aU), math.Abs(rp.ax.aV))
	maxInputAbs := proofbound.AbsSumUpper(aUpper, axisRadiusUpper, axisRadiusUpper)
	return proofbound.FrameAndPlacementRoundAllow(rp.frame, rp.xform, maxInputAbs)
}

// revolveCentroidGeometryBound bounds the centroid independently of the
// Pappus quotient. Every material point starts in the recorded profile plane,
// rotates about the resolved axis, then passes through a rigid placement. The
// L1 envelopes use three times an input L1 norm for any orthogonal map.
func revolveCentroidGeometryBound(rp revolvePayload, held r3.Vec, work *freeform.FreeformWork) (float64, error) {
	coordUpper, err := profileCoordinateUpper(rp.profile, work, nil)
	if err != nil {
		return 0, err
	}
	originUpper := vecL1(rp.frame.Origin())
	profileUpper := proofbound.AbsSumUpper(
		originUpper,
		proofbound.ProductUpper(vecL1(rp.frame.U()), coordUpper),
		proofbound.ProductUpper(vecL1(rp.frame.V()), coordUpper),
	)
	aUUpper := proofbound.AbsSumUpper(rp.ax.aU, rp.ax.aUBound)
	aVUpper := proofbound.AbsSumUpper(rp.ax.aV, rp.ax.aVBound)
	axisUpper := proofbound.AbsSumUpper(
		originUpper,
		proofbound.ProductUpper(vecL1(rp.frame.U()), aUUpper),
		proofbound.ProductUpper(vecL1(rp.frame.V()), aVUpper),
	)
	rotatedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, profileUpper), proofbound.ProductUpper(4, axisUpper))
	placedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, rotatedUpper), vecL1(rp.xform.Translation()))
	return proofbound.AbsSumUpper(vecL1(held), placedUpper), nil
}

// point places the axis-frame point (z, ρ) at sweep angle φ into placed
// world space.
func (rp revolvePayload) point(b revolvemesh.RevolveBasis, z, rho, phi float64) r3.Vec {
	sin, cos := math.Sincos(phi)
	radial := b.E0.Scale(cos).Add(b.E1.Scale(sin))
	return rp.xform.Apply(b.A3.Add(b.W.Scale(z)).Add(radial.Scale(rho)))
}

// reflected reports whether the accumulated placement flips handedness — a
// reflected solid's rotational senses invert with it, and every swept edge's
// axis carries the corrected sign.
func (rp revolvePayload) reflected() bool { return rp.xform.IsReflection() }

// axisMoments re-references the plane-origin region integrals into the axis
// frame: q = ∫ρ dA (Pappus's second theorem reads volume off it), mzr =
// ∫zρ dA (the solid centroid's axial position), and mrr = ∫ρ² dA (the
// partial sweep's in-plane centroid term) — the §4 first, second and mixed
// moments with their source and arithmetic bounds (docs/evaluator-design.md
// §6).
func axisMoments(ig regionIntegrals, ax axisFrame) (proofbound.BoundedScalar, proofbound.BoundedScalar, proofbound.BoundedScalar) {
	aU, aV := proofbound.MeasuredScalar(ax.aU, ax.aUBound), proofbound.MeasuredScalar(ax.aV, ax.aVBound)
	dU, dV := proofbound.MeasuredScalar(ax.dU, ax.dUBound), proofbound.MeasuredScalar(ax.dV, ax.dVBound)
	nU, nV := proofbound.MeasuredScalar(-ax.dV, ax.dVBound), proofbound.MeasuredScalar(ax.dU, ax.dUBound)
	area := proofbound.MeasuredScalar(ig.area, ig.areaBound)
	mu := proofbound.MeasuredScalar(ig.mu, ig.muBound)
	mv := proofbound.MeasuredScalar(ig.mv, ig.mvBound)
	muu := proofbound.MeasuredScalar(ig.muu, ig.muuBound)
	muv := proofbound.MeasuredScalar(ig.muv, ig.muvBound)
	mvv := proofbound.MeasuredScalar(ig.mvv, ig.mvvBound)

	iuu := proofbound.BoundedAdd(
		proofbound.BoundedSub(muu, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), aU), mu)),
		proofbound.BoundedMul(proofbound.BoundedMul(aU, aU), area),
	)
	iuv := proofbound.BoundedAdd(
		proofbound.BoundedSub(proofbound.BoundedSub(muv, proofbound.BoundedMul(aU, mv)), proofbound.BoundedMul(aV, mu)),
		proofbound.BoundedMul(proofbound.BoundedMul(aU, aV), area),
	)
	ivv := proofbound.BoundedAdd(
		proofbound.BoundedSub(mvv, proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), aV), mv)),
		proofbound.BoundedMul(proofbound.BoundedMul(aV, aV), area),
	)
	q := proofbound.BoundedAdd(
		proofbound.BoundedMul(nU, proofbound.BoundedSub(mu, proofbound.BoundedMul(aU, area))),
		proofbound.BoundedMul(nV, proofbound.BoundedSub(mv, proofbound.BoundedMul(aV, area))),
	)
	mzr := proofbound.BoundedAdd(
		proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.BoundedMul(dU, nU), iuu),
			proofbound.BoundedMul(proofbound.BoundedAdd(proofbound.BoundedMul(dU, nV), proofbound.BoundedMul(dV, nU)), iuv),
		),
		proofbound.BoundedMul(proofbound.BoundedMul(dV, nV), ivv),
	)
	mrr := proofbound.BoundedAdd(
		proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.BoundedMul(nU, nU), iuu),
			proofbound.BoundedMul(proofbound.BoundedMul(proofbound.ExactScalar(2), proofbound.BoundedMul(nU, nV)), iuv),
		),
		proofbound.BoundedMul(proofbound.BoundedMul(nV, nV), ivv),
	)
	return q, mzr, mrr
}

// revolveAxisAdmitBandCharge is resolveAxisSide's own tol·Lz charge for a
// LINEAR measurement (a cap's area, the centroid): tol is
// ax.radialAdmitAllow, the proven worst-case depth resolveAxisSide's gate
// admitted without proof, and Lz is ax.axialExtentUpper, the recorded
// region's own axial reach. It is exactly zero wherever that gate proved the
// region's radial minimum non-negative — every axis-aligned fixture in the
// tree, since radialAdmitAllow is then zero — and otherwise bounds how far a
// face built from the SNAPPED profile (revolve_axis.go's axisFrame.walk) can
// diverge from a measurement integrated over the UNSNAPPED one: a cap's own
// loop follows the snap, while its area is the Pappus engine's integral over
// the recorded region, and the two disagree by at most the admitted band's
// own width times the boundary's axial run.
func revolveAxisAdmitBandCharge(ax axisFrame) float64 {
	return proofbound.ProductUpper(ax.radialAdmitAllow, ax.axialExtentUpper)
}

// revolveAxisAdmitVolumeCharge is the volume's own share of the same
// admitted band, 2π·tol²·Lz: the band is swept a full turn rather than read
// once, so the material it can hide scales with tol² (a thin annulus of
// radius and width both order tol) rather than tol.
func revolveAxisAdmitVolumeCharge(ax axisFrame) float64 {
	tol := ax.radialAdmitAllow
	return proofbound.ProductUpper(proofbound.TwoPiUpper(), proofbound.ProductUpper(proofbound.ProductUpper(tol, tol), ax.axialExtentUpper))
}

// evalRevolve builds the analytic revolved body from the payload: side
// surfaces of revolution per boundary segment, caps only for a partial
// sweep, shared edges and vertices, and bounded mass measurements
// (docs/evaluator-design.md §6). The payload's segment kinds are line, circle
// and arc; anything else has already been rejected by the mass-property
// integrals it runs first.
func evalRevolve(d *Document, ref producerID, rp revolvePayload) (*Body, error) {
	return evalRevolveWork(d, ref, freeform.NewFreeformWork(), rp)
}

// evalRevolveWork is the build an operation that already holds this record's
// free-form work counter runs: the preflight below and every walkOf under it
// continue that counter rather than open a second ceiling on the same record.
func evalRevolveWork(d *Document, ref producerID, work *freeform.FreeformWork, rp revolvePayload) (*Body, error) {
	return evalRevolveContextWork(context.Background(), d, ref, rp, work)
}

func evalRevolveContext(ctx context.Context, d *Document, ref producerID, rp revolvePayload) (*Body, error) {
	return evalRevolveContextWork(ctx, d, ref, rp, freeform.NewFreeformWork())
}

func evalRevolveContextWork(ctx context.Context, d *Document, ref producerID, rp revolvePayload, work *freeform.FreeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := requireExactRevolveSection(rp, "a profile-fed revolve"); err != nil {
		return nil, err
	}
	ig, err := rp.profile.evaluatorIntegralsUncheckedContext(ctx, freeform.MomentSecondOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.area <= 0 {
		return nil, fmt.Errorf(`%w: the recorded region encloses no area`, ErrDegenerate)
	}
	// Every reading below integrates the RECORDED region while every face below
	// is built from the SNAPPED one (revolve_axis.go's axisFrame.walk), so each
	// integral owes what the snap moved its own integrand's region by
	// (regionSnapAllow). The area's share lands here, before the cap faces read
	// it; the three axis-frame moments' shares land on q, mzr and mrr below,
	// since those are the coordinates their envelopes were proven in. Every one
	// of the four is exactly zero for a profile whose on-axis endpoints already
	// sit on the axis.
	ig.areaBound = proofbound.AbsSumUpper(ig.areaBound, rp.ax.snap.area)
	sweep := rp.sweep()
	dphi := sweep.Value
	if dphi <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}
	q, mzr, mrr := axisMoments(ig, rp.ax)
	// The snap's own share of each axis-frame moment, charged where the moment
	// is read rather than back on the plane-local integrals it was composed
	// from: ρ and z are what regionSnapAllow's envelopes were proven against,
	// and a profile far down the axis has a large |z| beside a small ρ, so
	// charging the volume's ∫ρ dA at a frame-origin envelope would inflate it by
	// the whole axial offset. proofbound.BoundedMul and proofbound.BoundedDiv below then carry these
	// into the volume and the centroid through the arithmetic they already run.
	q.Bound = proofbound.AbsSumUpper(q.Bound, rp.ax.snap.first)
	mzr.Bound = proofbound.AbsSumUpper(mzr.Bound, rp.ax.snap.mixed)
	mrr.Bound = proofbound.AbsSumUpper(mrr.Bound, rp.ax.snap.second)
	if q.Value <= 0 {
		return nil, fmt.Errorf(`%w: the region has no material off the revolve axis`, ErrDegenerate)
	}

	b := rp.basis()
	// A surface result publishes a sheet, never a solid: solid false is what
	// makes Volume() and Centroid() answer ErrNotSolid through their existing
	// guards, and kind BodySheet is what Kind() reports
	// (docs/surface-design.md §4.3).
	kind := BodySolid
	if rp.surfaceResult {
		kind = BodySheet
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: !rp.surfaceResult, kind: kind}

	// Partial sweeps get two planar cap faces; a full revolution has none.
	var capStart, capEnd *Face
	if !rp.full {
		startFrame, err := rp.capFrame(b, rp.phi0, true)
		if err != nil {
			return nil, err
		}
		endFrame, err := rp.capFrame(b, rp.phi1, false)
		if err != nil {
			return nil, err
		}
		// normalBound is the cap plane's own dimensionless tilt: the frame's
		// unit normal rotates about the axis at unit rate in φ, so the angle
		// this cap's own end denotes charges its plane's normal by exactly
		// that end's proven displacement (docs/evaluator-design.md §6).
		//
		// capAdmitAllow is revolveAxisAdmitBandCharge's own term (above): each
		// cap face's LOOP is walked from the axis-snapped profile, while its
		// area here is ig.area, the Pappus engine's own integral over the
		// UNSNAPPED recorded one. The two agree exactly wherever
		// rp.ax.radialAdmitAllow is zero — every axis-aligned fixture — and
		// otherwise this is what keeps the published cap area from claiming a
		// tighter bound than the snap/unsnap mismatch can actually cost it.
		capAdmitAllow := revolveAxisAdmitBandCharge(rp.ax)
		capStart = &Face{
			surface:     Plane{Frame: startFrame},
			origins:     []FeatureRef{{producer: ref, Role: roleCapStart}},
			body:        body,
			area:        ig.area,
			areaBound:   proofbound.AbsSumUpper(ig.areaBound, capAdmitAllow),
			normalBound: rp.phi0Delta(),
		}
		capEnd = &Face{
			surface:     Plane{Frame: endFrame},
			origins:     []FeatureRef{{producer: ref, Role: roleCapEnd}},
			body:        body,
			area:        ig.area,
			areaBound:   proofbound.AbsSumUpper(ig.areaBound, capAdmitAllow),
			normalBound: rp.phi1Delta(),
		}
	}

	sideArea := proofbound.BoundedScalar{}
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	perLoop := make([][]*Face, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		parts, err := buildRevolveLoop(ctx, body, ref, rp, b, li, loop, work)
		if err != nil {
			return nil, err
		}
		perLoop[li] = parts.faces
		sideArea = proofbound.BoundedAdd(sideArea, parts.area)
		if !rp.full {
			capStart.loops = append(capStart.loops, &Loop{coedges: parts.startCo, outer: li == 0})
			capEnd.loops = append(capEnd.loops, &Loop{coedges: parts.endCo, outer: li == 0})
		}
	}

	// Shells: a partial sweep's caps connect every loop's walls into one
	// boundary, but a full revolution encloses each profile hole as its own
	// toroidal void — a separate shell, and a void one (evaluator §3). A
	// surface result never claims that void (decision B,
	// docs/surface-design.md §2.2): its hole loop's own closed surface
	// touches the outer one nowhere, so it is a wholly separate piece of the
	// body's boundary and gets its own Lump rather than a second shell of the
	// solid's one (decision A).
	var lumps []*Lump
	if rp.full {
		shells, err := fullRevolveShellsContext(ctx, perLoop, rp.surfaceResult)
		if err != nil {
			return nil, err
		}
		if rp.surfaceResult {
			lumps = make([]*Lump, len(shells))
			for i, sh := range shells {
				lumps[i] = &Lump{shells: []*Shell{sh}}
			}
		} else {
			lumps = []*Lump{{shells: shells}}
		}
	} else {
		var faces []*Face
		for _, group := range perLoop {
			faces = append(faces, group...)
		}
		// A surface result omits both caps from the shell (Table W,
		// docs/surface-design.md §4.1-§4.2): capStart/capEnd stay constructed
		// above so the area accumulation below can still read their area
		// fields, but neither joins faces nor gets its loops attached, which
		// is what leaves each rim edge with only its wall face — both
		// cap-plane rims of every wall become free edges with no
		// rim-specific code.
		if !rp.surfaceResult {
			faces = append(faces, capStart, capEnd)
			if err := attachFaceLoopsContext(ctx, []*Face{capStart, capEnd}); err != nil {
				return nil, err
			}
		}
		if rp.surfaceResult {
			// sheetLumps splits by connectivity (decision A): a holed
			// profile's loops touch nowhere once their caps are omitted, so
			// each becomes its own Lump instead of one shell regardless.
			lumps = sheetLumps(faces)
		} else {
			lumps = []*Lump{{shells: []*Shell{{faces: faces, open: shellIsOpen(faces)}}}}
		}
	}
	body.lumps = lumps

	// Measurements — Pappus with the profile and float-evaluation bounds
	// carried through (docs/evaluator-design.md §6).
	area := sideArea
	if !rp.full {
		// Both caps' own (area, areaBound) are read back rather than
		// re-derived from ig directly, so the solid's aggregate area takes the
		// SAME admitted-band charge (capAdmitAllow, above) the individual cap
		// Face fields already carry — never a narrower, uncharged bound the
		// two would then disagree with.
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(capStart.area, capStart.areaBound))
		area = proofbound.BoundedAdd(area, proofbound.MeasuredScalar(capEnd.area, capEnd.areaBound))
		if rp.surfaceResult {
			// §4.3: a surface result's area is the solid's area minus the two
			// omitted caps'. Each cap's area and bound are read back from the
			// Face fields the construction above stamped, and the
			// subtraction runs here — as the LAST thing that touches area,
			// since revolve carries no per-cap bound re-stamp the way the
			// prism's evalPrismContext does, so nothing after this point may
			// still widen capStart's or capEnd's own bound out from under a
			// value already composed from them. Collapsing this to sideArea
			// alone would hold the same value but a smaller, uncomposed
			// bound than §4.3's composed sum requires.
			area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capStart.area, capStart.areaBound))
			area = proofbound.BoundedSub(area, proofbound.MeasuredScalar(capEnd.area, capEnd.areaBound))
		}
	}
	volume := proofbound.BoundedMul(q, sweep)
	volume.Bound = proofbound.AbsSumUpper(volume.Bound, revolveAxisAdmitVolumeCharge(rp.ax))
	body.volume = Measurement{
		Value:     units.CubicMillimeters(volume.Value),
		Exactness: exactnessOf(volume.Bound),
		Bound:     units.CubicMillimeters(volume.Bound),
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: exactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}

	axial := proofbound.BoundedDiv(mzr, q)
	cen := b.A3.Add(b.W.Scale(axial.Value))
	centroidScale := proofbound.AbsSumUpper(proofbound.VecMaxAbs(b.A3), axial.Value)
	centroidBound := proofbound.AbsSumUpper(axial.Bound, proofbound.Radius3D(proofbound.AnalyticRoundBound(centroidScale)))
	if !rp.full {
		// The in-plane term is the swept radial direction integrated over
		// the interval — closed form in the sweep angle; a full turn's is
		// identically zero, which is what puts its centroid on the axis.
		sin1, cos1 := endSinCos(rp.den.phi1, rp.phi1)
		sin0, cos0 := endSinCos(rp.den.phi0, rp.phi0)
		rx := proofbound.BoundedSub(sin1, sin0)
		ry := proofbound.BoundedSub(cos0, cos1)
		radial := b.E0.Scale(rx.Value).Add(b.E1.Scale(ry.Value))
		radialBound := proofbound.Radius2D(rx.Bound, ry.Bound)
		radialScale := proofbound.BoundedDiv(mrr, proofbound.BoundedMul(sweep, q))
		cen = cen.Add(radial.Scale(radialScale.Value))
		radialUpper := vecL1(radial)
		centroidBound = proofbound.AbsSumUpper(
			centroidBound,
			proofbound.ProductUpper(radialScale.Value, radialBound),
			proofbound.ProductUpper(radialUpper, radialScale.Bound),
			proofbound.Radius3D(proofbound.AnalyticRoundBound(proofbound.ProductUpper(radialScale.Value, radialUpper))),
		)
	}
	centroidBound = proofbound.AbsSumUpper(
		centroidBound,
		proofbound.RigidRoundAllow(proofbound.VecMaxAbs(cen), proofbound.VecMaxAbs(rp.xform.Translation())),
	)
	centroidValue := rp.xform.Apply(cen)
	geometryBound, err := revolveCentroidGeometryBound(rp, centroidValue, work)
	if err != nil {
		return nil, err
	}
	centroidBound = math.Min(centroidBound, geometryBound)
	// The axis gate's own admitted-band charge is added AFTER the geometry
	// bound's independent min: revolveCentroidGeometryBound knows nothing of
	// resolveAxisSide's own uncertainty, so folding this charge in before the
	// min would let a smaller geometry bound silently discard it.
	centroidBound = proofbound.AbsSumUpper(centroidBound, revolveAxisAdmitBandCharge(rp.ax))
	body.centroid = VecMeasurement{
		Value:     centroidValue,
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	bounds, err := revolveBoundsContext(ctx, rp, work)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = rp
	return body, nil
}

// capFrame is the plane frame of a cap at sweep angle φ, under the
// placement. The start cap's outward (material-leaving) normal points
// against the sweep, the end cap's along it; the axes are ordered so the
// frame's normal IS the outward normal, and a reflected placement swaps them
// once more (a reflection flips the cross product's handedness).
func (rp revolvePayload) capFrame(b revolvemesh.RevolveBasis, phi float64, start bool) (r3.Frame, error) {
	sin, cos := math.Sincos(phi)
	radial := b.E0.Scale(cos).Add(b.E1.Scale(sin))
	// (w, radial) has w × radial = the rotated plane normal — the sweep
	// velocity direction, the END cap's outward normal.
	u3, v3 := b.W, radial
	if start {
		u3, v3 = v3, u3
	}
	if rp.reflected() {
		u3, v3 = v3, u3
	}
	f, err := r3.NewFrame(rp.xform.Apply(b.A3), rp.xform.ApplyDir(u3), rp.xform.ApplyDir(v3))
	if err != nil {
		return r3.Frame{}, fmt.Errorf(`%w: the placed cap frame is degenerate: %s`, ErrDegenerate, err)
	}
	return f, nil
}

// revLoopParts is what one recorded loop contributes to the revolved body.
type revLoopParts struct {
	faces   []*Face
	startCo []coedge                 // the loop's start-cap coedges, walk order (partial only)
	endCo   []coedge                 // the loop's end-cap coedges, walk order (partial only)
	area    proofbound.BoundedScalar // the loop's side-face area
}

// junctionRadiusInterval encloses a junction's radial coordinate ρ as
// [ρ−ρBound, ρ+ρBound], the rational-interval twin of the proven float bound
// axisFrame.walk already composed into startV/startVBound: ρ is a walk's own
// startV (a junction sits at walk i's start), and ρBound is startVBound, the
// PROVEN error between it and the axis-re-expressed value the walk's true
// (unrounded) axis direction and anchor would give. ok is false wherever
// either cannot be stated as an exact rational — a non-finite ρBound is the
// refusal shape every consumer below already turns into its own envelope.
func junctionRadiusInterval(rho, rhoBound float64) (proofbound.RatInterval, bool) {
	r, b := proofarith.FloatRat(rho), proofarith.FloatRat(math.Abs(rhoBound))
	if r == nil || b == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.IntervalWiden(proofbound.PointInterval(r), b), true
}

// revJunction is one junction between consecutive walks: the shared point in
// axis coordinates and what it sweeps to — nothing on the axis, a full
// latitude circle for a full revolution, an arc between the two caps for a
// partial sweep.
type revJunction struct {
	z, rho float64
	onAxis bool
	v0, v1 *Vertex // partial: start-/end-cap vertices (one shared on the axis)
	arc    *Edge   // partial: the swept junction arc; nil on the axis
	lat    *Edge   // full: the latitude circle; nil on the axis
}

// revolveWalks is one recorded loop resolved the way a revolve reads it: the
// coalesced walks in AXIS coordinates, what each of them sweeps, the same
// walks still in PLANE-local coordinates indexed by recorded segment, and
// whether the loop is a single whole closed curve.
//
// plane is kept beside walks because the two answer different questions. A
// build needs only the axis coordinates; a proof about how far a held sample
// sits from the point the RECORD denotes needs the recorded plane coordinates
// the axis re-expression consumed, since axisFrame.walk states no bound on the
// axial coordinate it computes and snaps a near-axis radial one to zero
// outright (docs/tessellation-design.md §8's deltaC).
type revolveWalks struct {
	walks        []survey2d.SideWalk
	kinds        []wallKind
	plane        []survey2d.SegmentWalk
	singleClosed bool
}

// revolveLoopWalks resolves one recorded loop into the walks a revolve reads,
// so the builder and the tessellator consume the SAME resolution rather than
// two copies of it (docs/tessellation-design.md §3: the mesh reads the
// evaluator's payload, never live sketch input). what names the caller in the
// free-form refusal.
func revolveLoopWalks(ctx context.Context, rp revolvePayload, loop LoopRecord, work *freeform.FreeformWork, what string) (revolveWalks, error) {
	if err := ctx.Err(); err != nil {
		return revolveWalks{}, err
	}
	if len(loop.Segments) == 0 {
		return revolveWalks{}, fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	plane := make([]survey2d.SegmentWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := ctx.Err(); err != nil {
			return revolveWalks{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return revolveWalks{}, err
		}
		if err := requireAnalyticWalk(w, what); err != nil {
			return revolveWalks{}, err
		}
		plane[i] = w
		startCharge, endCharge, err := trimRevolveSegmentCharges(seg, rp.sectionDelta)
		if err != nil {
			return revolveWalks{}, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: rp.ax.walkCharged(w, startCharge, endCharge), Segs: []int{i}}
	}
	walks, err := coalesceWalksContext(ctx, raw)
	if err != nil {
		return revolveWalks{}, err
	}
	kinds := make([]wallKind, len(walks))
	for i, w := range walks {
		kinds[i] = rp.ax.classify(w.SegmentWalk)
	}
	return revolveWalks{
		walks:        walks,
		kinds:        kinds,
		plane:        plane,
		singleClosed: len(walks) == 1 && walks[0].Closed,
	}, nil
}

// buildRevolveLoop builds one loop's side faces with shared vertices and
// edges, returning the faces, the two caps' coedges in walk order, and the
// loop's side area.
func buildRevolveLoop(ctx context.Context, body *Body, ref producerID, rp revolvePayload, b revolvemesh.RevolveBasis, li int, loop LoopRecord, work *freeform.FreeformWork) (revLoopParts, error) {
	resolved, err := revolveLoopWalks(ctx, rp, loop, work, "the revolve wall build")
	if err != nil {
		return revLoopParts{}, err
	}
	walks, kinds, singleClosed := resolved.walks, resolved.kinds, resolved.singleClosed
	n := len(walks)
	sweep := rp.sweep()
	dphi := sweep.Value
	sweepSign := 1.0
	if rp.reflected() {
		sweepSign = -1
	}
	wDir := rp.xform.ApplyDir(b.W)

	// Junction vertices and swept edges: junction i sits at walk i's start
	// (== walk i−1's end). A single whole closed curve has none; a junction
	// on the axis sweeps to a single point — no edge, and for a partial
	// sweep one vertex shared by both caps. Convexity comes from the 2D
	// turn, exactly as the prism's vertical edges: a positive cross of the
	// incoming and outgoing tangents is a convex edge on the outer loop and
	// works out identically for hole loops walked clockwise.
	var js []revJunction
	if !singleClosed {
		js = make([]revJunction, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			j := revJunction{z: w.StartU, rho: w.StartV, onAxis: w.StartV == 0}
			prev := walks[(i+n-1)%n]
			turn := prev.TanOutU*w.TanInV - prev.TanOutV*w.TanInU
			center := rp.point(b, j.z, 0, 0)
			switch {
			case rp.full && !j.onAxis:
				seam := &Vertex{position: rp.point(b, j.z, j.rho, rp.phi0), bound: units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(j.rho, rp.phi0Delta()), revolveVertexFrameLiftAllow(rp, w.AxisRadiusUpper))), denot: body.doc.mintCurve()}
				latitudeLength := 2 * math.Pi * j.rho
				latitudeBound := proofbound.ConservativeValueError(latitudeLength, proofbound.ProductUpper(w.AxisRadiusUpper, proofbound.TwoPiUpper()))
				if rhoEnc, ok := junctionRadiusInterval(j.rho, w.StartVBound); ok {
					enc := proofbound.IntervalMul(proofbound.TwoPiInterval(), rhoEnc)
					latitudeBound = math.Min(latitudeBound, proofbound.IntervalFloatError(enc, latitudeLength))
				}
				j.lat = &Edge{
					curve:       Circle3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
					start:       seam,
					end:         seam,
					convex:      turn > 0,
					length:      latitudeLength,
					lengthBound: latitudeBound,
					// The CURVE half of the shared-denotation certificate
					// (denotation.go): this junction latitude circle is
					// shared, by construction, between side faces i-1 and i
					// of THIS build alone (fullRevLoops below), exactly the
					// role a prism's own rim edge plays for its two caps.
					denot: body.doc.mintCurve(),
				}
			case !rp.full:
				j.v0 = &Vertex{position: rp.point(b, j.z, j.rho, rp.phi0), bound: units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(j.rho, rp.phi0Delta()), revolveVertexFrameLiftAllow(rp, w.AxisRadiusUpper))), denot: body.doc.mintCurve()}
				j.v1 = j.v0
				if !j.onAxis {
					j.v1 = &Vertex{position: rp.point(b, j.z, j.rho, rp.phi1), bound: units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(j.rho, rp.phi1Delta()), revolveVertexFrameLiftAllow(rp, w.AxisRadiusUpper))), denot: body.doc.mintCurve()}
					arcLength := j.rho * dphi
					dphiUpper := proofbound.AbsSumUpper(math.Abs(dphi), sweep.Bound)
					arcBound := proofbound.ConservativeValueError(arcLength, proofbound.ProductUpper(w.AxisRadiusUpper, dphiUpper))
					if rhoEnc, ok := junctionRadiusInterval(j.rho, w.StartVBound); ok {
						if widthEnc, ok := rp.den.widthInterval(); ok {
							enc := proofbound.IntervalMul(rhoEnc, widthEnc)
							arcBound = math.Min(arcBound, proofbound.IntervalFloatError(enc, arcLength))
						}
					}
					j.arc = &Edge{
						curve:       Arc3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
						start:       j.v0,
						end:         j.v1,
						convex:      turn > 0,
						length:      arcLength,
						lengthBound: arcBound,
						// The CURVE half of the shared-denotation certificate
						// (denotation.go): this junction arc is shared, by
						// construction, between side faces i-1 and i of THIS
						// build alone.
						denot: body.doc.mintCurve(),
					}
				}
			}
			js[i] = j
		}
	}

	// Cap edges (partial sweeps only): each walk's copy in each cap plane.
	// A line lying on the axis does not move when swept, so its two copies
	// coincide: ONE edge, shared by both caps — that is how the caps of a
	// solid wedge meet along the axis. Its dihedral is the sweep angle
	// itself.
	holeLoop := li != 0
	var cap0, cap1 []*Edge
	if !rp.full {
		cap0 = make([]*Edge, n)
		cap1 = make([]*Edge, n)
		for i, w := range walks {
			if err := ctx.Err(); err != nil {
				return revLoopParts{}, err
			}
			if kinds[i] == wallAxis {
				// This edge is appended only to cap0/cap1 below, never to any
				// wall face's own loop — a LineSeg on the axis emits no wall
				// face at all (evaluator §6). A solid's attached caps are
				// what give it its two faces; a surface result never attaches
				// them, so nothing else references this edge and Body.Edges()
				// never reaches it — Table W row 3's on-axis-edge omission
				// delivered by omission, not a special case to "fix" here.
				shared := &Edge{
					curve:       Line3{},
					start:       js[i].v0,
					end:         js[(i+1)%n].v0,
					convex:      dphi < math.Pi,
					length:      w.Length,
					lengthBound: w.LengthBound,
				}
				cap0[i], cap1[i] = shared, shared
				continue
			}
			var vs0, ve0, vs1, ve1 *Vertex
			if !singleClosed {
				vs0, ve0 = js[i].v0, js[(i+1)%n].v0
				vs1, ve1 = js[i].v1, js[(i+1)%n].v1
			}
			cap0[i] = rp.capEdge(b, w.SegmentWalk, singleClosed, vs0, ve0, rp.phi0, rp.phi0Delta(), holeLoop)
			cap1[i] = rp.capEdge(b, w.SegmentWalk, singleClosed, vs1, ve1, rp.phi1, rp.phi1Delta(), holeLoop)
		}
	}

	parts := revLoopParts{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return revLoopParts{}, err
		}
		if kinds[i] == wallAxis {
			if !rp.full {
				parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
				parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
			}
			continue
		}
		surf, reversed, err := rp.wallSurface(b, w.SegmentWalk, kinds[i])
		if err != nil {
			return revLoopParts{}, err
		}
		origins, err := sideOriginsContext(ctx, ref, li, w.Segs)
		if err != nil {
			return revLoopParts{}, err
		}
		segs := make([]CurveSegment, len(w.Segs))
		for j, si := range w.Segs {
			segs[j] = loop.Segments[si]
		}
		faceArea := proofbound.BoundedMul(walkAxisMoment(w.SegmentWalk, kinds[i], segs, rp.ax), sweep)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.Value,
			areaBound: faceArea.Bound,
			reversed:  reversed,
		}
		switch {
		case rp.full && singleClosed:
			// A whole closed generator swept a full turn bounds a closed
			// surface — a torus needs no boundary loop at all.
		case rp.full:
			face.loops = fullRevLoops(js[i], js[(i+1)%n], kinds[i])
		case singleClosed:
			// A closed generator's partial sweep is bounded by its two cap
			// copies — two loops, one whole closed edge each, mirroring the
			// prism's whole-circle two-loop discipline.
			face.loops = []*Loop{
				{coedges: []coedge{{edge: cap0[i], forward: true}}, outer: true},
				{coedges: []coedge{{edge: cap1[i], forward: false}}, outer: true},
			}
		default:
			co := []coedge{{edge: cap0[i], forward: true}}
			if a := js[(i+1)%n].arc; a != nil {
				co = append(co, coedge{edge: a, forward: true})
			}
			co = append(co, coedge{edge: cap1[i], forward: false})
			if a := js[i].arc; a != nil {
				co = append(co, coedge{edge: a, forward: false})
			}
			face.loops = []*Loop{{coedges: co, outer: true}}
		}
		if err := attachFaceLoopsContext(ctx, []*Face{face}); err != nil {
			return revLoopParts{}, err
		}
		parts.faces = append(parts.faces, face)
		parts.area = proofbound.BoundedAdd(parts.area, faceArea)
		if !rp.full {
			parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
			parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
		}
	}
	return parts, nil
}

// fullRevolveShellsContext builds one shell per non-empty loop group. sheet is
// rp.surfaceResult: a full revolution's wall set already closes on itself, so
// every shell built here is closed regardless of kind, but shellIsOpen still
// runs rather than assuming it — the same discipline every other shell in
// this build follows (docs/surface-design.md §2.2). void is li != 0 on a
// solid, whose hole loop bounds its own toroidal cavity (evaluator §3); a
// sheet bounds no cavity at all (decision B, §2.2), so it is always false
// there.
func fullRevolveShellsContext(ctx context.Context, perLoop [][]*Face, sheet bool) ([]*Shell, error) {
	var shells []*Shell
	for li, group := range perLoop {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if len(group) == 0 {
			continue
		}
		shells = append(shells, &Shell{faces: group, open: shellIsOpen(group), void: li != 0 && !sheet})
	}
	return shells, nil
}

// fullRevLoops assembles a full-revolution side face's boundary loops from
// its junctions' latitude circles — none on the axis (a pole or an apex
// bounds nothing), and for a planar annulus the inner circle is genuinely a
// hole; every other face's circles are both outer, like the prism's whole
// cylinder.
func fullRevLoops(j0, j1 revJunction, kind wallKind) []*Loop {
	outer0, outer1 := true, true
	if kind == wallPlane && !j0.onAxis && !j1.onAxis {
		if j0.rho < j1.rho {
			outer0 = false
		} else {
			outer1 = false
		}
	}
	var loops []*Loop
	if j0.lat != nil {
		loops = append(loops, &Loop{coedges: []coedge{{edge: j0.lat, forward: true}}, outer: outer0})
	}
	if j1.lat != nil {
		loops = append(loops, &Loop{coedges: []coedge{{edge: j1.lat, forward: false}}, outer: outer1})
	}
	if len(loops) == 2 && !loops[0].outer {
		loops[0], loops[1] = loops[1], loops[0]
	}
	return loops
}

// capEdge is one boundary walk's copy in the cap plane at sweep angle φ: a
// straight walk stays a line, a circular one an arc — or a whole circle
// with a seam vertex, closing on itself. The cap edge sits between the cap
// and the side face it bounds; the material across it is a quarter-turn wedge
// everywhere, so the material angle decides nothing — a rim reports the
// WALKED-BOUNDARY convexity instead (topology.go, Edge.IsConvex), which is the
// side the wall's material lies on. A CIRCULAR walk decides that by its own
// sense (a clockwise arc keeps the
// material outside the sphere/torus it sweeps, exactly as wallSurface reads
// it, so its cap edge is concave — a hole's arc and a concave bite in the
// outer boundary alike), while a STRAIGHT walk has no sense of its own and
// takes the loop's: outer convex, hole concave. delta is the proven angular
// displacement of THIS end (rp.phi0Delta() or rp.phi1Delta(), matching
// whichever of phi0/phi1 phi is), charged into a closed walk's own seam
// vertex the same way every other cap vertex is (docs/evaluator-design.md §6).
func (rp revolvePayload) capEdge(b revolvemesh.RevolveBasis, w survey2d.SegmentWalk, closed bool, vs, ve *Vertex, phi, delta float64, holeLoop bool) *Edge {
	convex := !holeLoop
	if w.IsCircular() {
		convex = w.Th0 < w.Th1
	}
	e := &Edge{convex: convex, length: w.Length, lengthBound: w.LengthBound}
	if !w.IsCircular() {
		e.curve = Line3{}
		e.start, e.end = vs, ve
		return e
	}
	// The cap plane at φ is the profile plane rotated about the axis, so
	// its normal is the rotated sweep-velocity direction; the walk's own
	// range order is the arc's CCW sense about it, inverted once by a
	// reflected placement.
	sin, cos := math.Sincos(phi)
	normal := b.E0.Scale(-sin).Add(b.E1.Scale(cos))
	sign := 1.0
	if w.Th1 < w.Th0 {
		sign = -1
	}
	if rp.reflected() {
		sign = -sign
	}
	axis := rp.xform.ApplyDir(normal).Scale(sign)
	center := rp.point(b, w.CU, w.CV, phi)
	radius := units.Millimeters(w.Radius)
	if closed {
		seam := &Vertex{position: rp.point(b, w.StartU, w.StartV, phi), bound: units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(w.StartV, delta), revolveVertexFrameLiftAllow(rp, w.AxisRadiusUpper)))}
		e.curve = Circle3{Center: center, Axis: axis, Radius: radius}
		e.start, e.end = seam, seam
		return e
	}
	e.curve = Arc3{Center: center, Axis: axis, Radius: radius}
	e.start, e.end = vs, ve
	return e
}

// wallSurface is the placed surface of revolution one off-axis walk sweeps,
// and whether the face's outward (material-leaving) normal is the surface's
// geometric normal negated. The walk runs with the region's material on its
// left in (z, ρ), so its outward direction is the tangent rotated a quarter
// turn right: (t_ρ, −t_z). A cylinder's and a cone's geometric normal has a
// positive ρ component, so those reverse exactly when the tangent climbs
// (t_z > 0 — a hole wall, or the inner wall of an annular section); a
// sphere's and a torus's geometric normal points away from the generating
// circle's center, which a counter-clockwise walk's outward direction is —
// so those reverse exactly when the walk runs clockwise. Radial directions
// are reflection-equivariant, so a reflected placement changes none of
// this; only the plane frames (built from cross products) correct for it.
func (rp revolvePayload) wallSurface(b revolvemesh.RevolveBasis, w survey2d.SegmentWalk, kind wallKind) (Surface, bool, error) {
	place := func(z float64) r3.Vec { return rp.xform.Apply(b.A3.Add(b.W.Scale(z))) }
	axis := rp.xform.ApplyDir(b.W)
	switch kind {
	case wallCylinder:
		return Cylinder{
			Origin: place(w.StartU),
			Axis:   axis,
			Radius: units.Millimeters((w.StartV + w.EndV) / 2),
		}, w.TanInU > 0, nil
	case wallPlane:
		// The outward normal is ±axis by the walk's radial heading; the
		// frame's axes are ordered so its normal is outward, swapped once
		// more under a reflected placement.
		u3, v3 := b.E0, b.E1
		if w.TanInV < 0 {
			u3, v3 = v3, u3
		}
		if rp.reflected() {
			u3, v3 = v3, u3
		}
		f, err := r3.NewFrame(place(w.StartU), rp.xform.ApplyDir(u3), rp.xform.ApplyDir(v3))
		if err != nil {
			return nil, false, fmt.Errorf(`%w: the placed wall frame is degenerate: %s`, ErrDegenerate, err)
		}
		return Plane{Frame: f}, false, nil
	case wallCone:
		// The apex is where the wall meets the axis; the cone's radius
		// grows along its stored Axis direction.
		dz, dr := w.EndU-w.StartU, w.EndV-w.StartV
		apex := w.StartU - w.StartV*dz/dr
		growth := 1.0
		if dz*dr < 0 {
			growth = -1
		}
		return Cone{
			Origin:    place(apex),
			Axis:      axis.Scale(growth),
			Radius:    units.Millimeters(0),
			HalfAngle: units.Radians(math.Atan2(math.Abs(dr), math.Abs(dz))),
		}, w.TanInU > 0, nil
	case wallSphere:
		return Sphere{
			Center: place(w.CU),
			Radius: units.Millimeters(w.Radius),
		}, w.Th1 < w.Th0, nil
	case wallTorus:
		return Torus{
			Center: place(w.CU),
			Axis:   axis,
			Major:  units.Millimeters(w.CV),
			Minor:  units.Millimeters(w.Radius),
		}, w.Th1 < w.Th0, nil
	default:
		return nil, false, fmt.Errorf(`%w: a wall on the axis sweeps no surface`, ErrDegenerate)
	}
}

// walkAxisMoment is the first moment ∫ρ ds of one boundary walk about the
// axis — Pappus's first theorem reads the side face's area from it:
// a straight walk's is its length times its mean radius (ρ is linear along
// it), a circular walk's is the closed-form antiderivative over its angular
// range, and an on-axis walk sweeps nothing.
//
// The straight arm's bound is composed bounded arithmetic over w's own
// proven inputs — w.length with w.lengthBound (the sqrt bracket), w.startV/
// w.endV with w.startVBound/w.endVBound (axisFrame.walk's re-expressed
// radial coordinates) — so math.Min against the magnitude envelope can only
// shrink the published bound, never widen it, following internal/proofbound/bounded.go's own
// convention.
//
// The circular arm's held value is still the axis-frame closed form (w.th0/
// w.th1, math.Atan2 results with no enclosure of their own), but its bound
// now takes math.Min against circularAxisMomentInterval's rational-interval
// closed form over the wall's own RECORDED segments (segs, moments_circular.go),
// summed additively — a coalesced wall covers exactly one recorded segment
// today (coalesceWalks never merges a circular kind), but the sum is written
// for whatever a future coalescing rule hands it. A segment
// circularAxisMomentInterval cannot bracket (a trimmed ArcSeg fragment, an
// axis whose direction carries a non-finite bound) withholds the whole sum,
// leaving the envelope as the only proof standing, exactly as before this
// change.
func walkAxisMoment(w survey2d.SegmentWalk, kind wallKind, segs []CurveSegment, ax axisFrame) proofbound.BoundedScalar {
	if kind == wallAxis {
		return proofbound.BoundedScalar{}
	}
	if !w.IsCircular() {
		meanRadius := proofbound.BoundedDiv(
			proofbound.BoundedAdd(proofbound.MeasuredScalar(w.StartV, w.StartVBound), proofbound.MeasuredScalar(w.EndV, w.EndVBound)),
			proofbound.ExactScalar(2),
		)
		result := proofbound.BoundedMul(proofbound.MeasuredScalar(w.Length, w.LengthBound), meanRadius)
		result.Bound = math.Min(result.Bound, proofbound.ConservativeValueError(result.Value, w.AxisMomentUpper))
		return result
	}
	lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
	dtheta := proofbound.BoundedSub(proofbound.ExactScalar(hi), proofbound.ExactScalar(lo))
	cosDelta := proofbound.BoundedSub(proofbound.BoundedCos(proofbound.ExactScalar(lo)), proofbound.BoundedCos(proofbound.ExactScalar(hi)))
	result := proofbound.BoundedMul(
		proofbound.ExactScalar(w.Radius),
		proofbound.BoundedAdd(
			proofbound.BoundedMul(proofbound.ExactScalar(w.CV), dtheta),
			proofbound.BoundedMul(proofbound.ExactScalar(w.Radius), cosDelta),
		),
	)
	result.Bound = proofbound.ConservativeValueError(result.Value, w.AxisMomentUpper)
	if enc, ok := circularAxisMomentTotal(segs, ax); ok {
		result.Bound = math.Min(result.Bound, proofbound.IntervalFloatError(enc, result.Value))
	}
	return result
}

// circularAxisMomentTotal sums circularAxisMomentInterval over every recorded
// segment a coalesced circular wall covers: the integral ∫ρ ds is additive
// over the walk's own segments, so the wall's total moment enclosure is their
// enclosures' sum. ok is false wherever any one segment's own bracket refuses
// — a partial sum standing in for a segment the record cannot bracket would
// publish a claim that segment never proved.
func circularAxisMomentTotal(segs []CurveSegment, ax axisFrame) (proofbound.RatInterval, bool) {
	if len(segs) == 0 {
		return proofbound.RatInterval{}, false
	}
	total, ok := circularAxisMomentInterval(segs[0], ax)
	if !ok {
		return proofbound.RatInterval{}, false
	}
	for _, seg := range segs[1:] {
		enc, ok := circularAxisMomentInterval(seg, ax)
		if !ok {
			return proofbound.RatInterval{}, false
		}
		total = proofbound.IntervalAdd(total, enc)
	}
	return total, true
}

// This section is RevolveChain's own build (docs/surface-design.md §13.4):
// the open chain's counterpart of evalRevolveContextWork/buildRevolveLoop
// above, reusing rp.wallSurface, rp.capEdge, sideOriginsContext and
// walkAxisMoment unchanged. It differs only in TOPOLOGY: n segments place
// n+1 junctions with no wraparound, and neither cap face nor cap coedge
// collection is ever built — a chain mints no cap.

// evalChainRevolveContext builds the shell body: one swept wall per recorded
// segment (Table G rows 2-3), never a cap and never a wraparound junction.
// work is the record's ONE free-form work counter (docs/spline-design.md
// §5.2): the caller opens it once and this build and the final bounds
// reading both spend from it.
func evalChainRevolveContext(ctx context.Context, d *Document, ref producerID, rp chainRevolvePayload, work *freeform.FreeformWork) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(rp.chains) == 0 {
		return nil, fmt.Errorf(`%w: a chain-revolve payload holds no walk`, ErrDegenerate)
	}
	rev := rp.revolve()
	sweep := rev.sweep()
	if sweep.Value <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: false, kind: BodySheet}
	b := rev.basis()
	var allFaces []*Face
	area := proofbound.BoundedScalar{}
	for ci := range rp.chains {
		// Each walk builds from its OWN single-walk view, so
		// sideOriginsContext's segment indices and walkAxisMoment's own
		// rp.profile.Outer read reach exactly that walk's segments, and ci
		// feeds the face role so two walks never collide on one role string
		// (docs/surface-intersection-design.md §3.4).
		view := rp.walkView(ci)
		resolved, err := chainRevolveWalks(ctx, view, rp.chains[ci], work)
		if err != nil {
			return nil, err
		}
		if len(resolved.walks) == 0 {
			return nil, fmt.Errorf(`%w: a recorded chain holds no segments`, ErrDegenerate)
		}
		if err := requireChainAxisIncidence(resolved); err != nil {
			return nil, err
		}
		faces, walkArea, err := buildChainRevolveWalls(ctx, body, ref, view, ci, b, resolved)
		if err != nil {
			return nil, err
		}
		allFaces = append(allFaces, faces...)
		area = proofbound.BoundedAdd(area, walkArea)
	}
	body.lumps = sheetLumps(allFaces)
	body.area = Measurement{
		Value:     units.SquareMillimeters(area.Value),
		Exactness: exactnessOf(area.Bound),
		Bound:     units.SquareMillimeters(area.Bound),
	}
	// volume and centroid stay at their zero value: a chain bounds no region
	// (docs/surface-design.md §13.3), so neither is ever integrated here, and
	// neither is reachable through Body.Volume/Body.Centroid while solid is
	// false (§8) — exactly as evalChainExtrudeContext leaves them.
	bounds, err := revolveBoundsContext(ctx, rev, work)
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	if err := validateAnalyticBodyMeasurements(body); err != nil {
		return nil, err
	}
	body.payload = rp
	return body, nil
}

// requireChainAxisIncidence reads an open walk without wrapping its last
// junction onto its first. A free pole has one off-axis incident wall; an
// interior axis junction still needs the profile audit's axis-line partner.
// The closed-profile revolve never calls this chain-only audit.
func requireChainAxisIncidence(resolved revolveWalks) error {
	walks, kinds := resolved.walks, resolved.kinds
	n := len(walks)
	startPole, endPole := walks[0].StartV == 0, walks[n-1].EndV == 0
	if startPole && endPole {
		return fmt.Errorf(`%w: a chain with both free ends on the revolve axis needs closed-sheet pole topology`, ErrUnsupported)
	}
	if startPole && kinds[0] == wallAxis || endPole && kinds[n-1] == wallAxis {
		return fmt.Errorf(`%w: a chain free end on the revolve axis has no incident swept wall`, ErrUnsupported)
	}

	seen := map[float64]struct{}{}
	for i, w := range walks {
		if w.StartV != 0 {
			continue
		}
		if _, duplicate := seen[w.StartU]; duplicate {
			return fmt.Errorf(`%w: two chain junctions meet the revolve axis at the same point`, ErrDegenerate)
		}
		seen[w.StartU] = struct{}{}
		if i == 0 {
			continue // the free pole has no incoming walk
		}
		if walks[i-1].EndV != 0 || (kinds[i-1] == wallAxis) == (kinds[i] == wallAxis) {
			return fmt.Errorf(`%w: a chain interior axis junction needs one swept wall and one axis line`, ErrDegenerate)
		}
	}
	if endPole {
		if _, duplicate := seen[walks[n-1].EndU]; duplicate {
			return fmt.Errorf(`%w: two chain junctions meet the revolve axis at the same point`, ErrDegenerate)
		}
	}
	return nil
}

// chainRevolveWalks resolves the chain's segments the way a revolve reads
// them, exactly as revolveLoopWalks does for a profile loop, except that
// consecutive segments never wrap: an open walk's last segment does not
// continue into its first (docs/surface-design.md §13.4). singleClosed is
// always false: RecordChain admits no whole closed segment
// (docs/sketch-seam-design.md §2.2).
func chainRevolveWalks(ctx context.Context, rp revolvePayload, chain ChainRecord, work *freeform.FreeformWork) (revolveWalks, error) {
	if len(chain.Segments) == 0 {
		return revolveWalks{}, fmt.Errorf(`%w: a recorded chain holds no segments`, ErrDegenerate)
	}
	raw := make([]survey2d.SideWalk, len(chain.Segments))
	plane := make([]survey2d.SegmentWalk, len(chain.Segments))
	for i, seg := range chain.Segments {
		if err := ctx.Err(); err != nil {
			return revolveWalks{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return revolveWalks{}, err
		}
		if err := requireAnalyticWalk(w, "the revolve wall build"); err != nil {
			return revolveWalks{}, err
		}
		plane[i] = w
		startCharge, endCharge, err := trimRevolveSegmentCharges(seg, rp.sectionDelta)
		if err != nil {
			return revolveWalks{}, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: rp.ax.walkCharged(w, startCharge, endCharge), Segs: []int{i}}
	}
	walks, err := coalesceChainWalksContext(ctx, raw)
	if err != nil {
		return revolveWalks{}, err
	}
	kinds := make([]wallKind, len(walks))
	for i, w := range walks {
		kinds[i] = rp.ax.classify(w.SegmentWalk)
	}
	return revolveWalks{walks: walks, kinds: kinds, plane: plane, singleClosed: false}, nil
}

// buildChainRevolveWalls builds an open chain's whole swept-wall set with
// shared vertices and edges: one wall per recorded segment, never a cap and
// never a wraparound junction. n segments place n+1 junctions, so the
// chain's two FREE ends (junction 0 and junction n) each carry a swept edge
// (an arc, or a full latitude circle under a full revolution) touched by
// exactly one wall — Free() resolves them (Edge.IsFree, topology.go) —
// while every INTERIOR junction (1..n-1) shares its swept edge between the
// two walls it joins, exactly as buildRevolveLoop's own junctions do for a
// closed loop. A wall's own copy of its walk at phi0/phi1 (capEdge) is never
// attached to a cap face here — a chain mints none — so it stays free
// regardless of position (docs/surface-design.md §13.4). It returns the
// faces and the walk's own total wall area, folded through proofbound.BoundedAdd.
func buildChainRevolveWalls(ctx context.Context, body *Body, ref producerID, rp revolvePayload, loopIdx int, b revolvemesh.RevolveBasis, resolved revolveWalks) ([]*Face, proofbound.BoundedScalar, error) {
	walks, kinds := resolved.walks, resolved.kinds
	n := len(walks)
	sweep := rp.sweep()
	dphi := sweep.Value
	sweepSign := 1.0
	if rp.reflected() {
		sweepSign = -1
	}
	wDir := rp.xform.ApplyDir(b.W)

	// junctionSource reads junction i's own (z, rho) and the walk whose end
	// it belongs to: junction i sits at walk i's start for i < n, and at the
	// LAST walk's own end for i == n — the chain's own two free ends.
	junctionSource := func(i int) (z, rho, rhoBound, axisRadiusUpper float64) {
		if i < n {
			w := walks[i]
			return w.StartU, w.StartV, w.StartVBound, w.AxisRadiusUpper
		}
		w := walks[n-1]
		return w.EndU, w.EndV, w.EndVBound, w.AxisRadiusUpper
	}

	js := make([]revJunction, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		z, rho, rhoBound, axisRadiusUpper := junctionSource(i)
		j := revJunction{z: z, rho: rho, onAxis: rho == 0}
		var turn float64
		if i > 0 && i < n {
			prev, w := walks[i-1], walks[i]
			turn = prev.TanOutU*w.TanInV - prev.TanOutV*w.TanInU
		}
		center := rp.point(b, j.z, 0, 0)
		switch {
		case rp.full && !j.onAxis:
			seam := &Vertex{
				position: rp.point(b, j.z, j.rho, rp.phi0),
				bound:    units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(j.rho, rp.phi0Delta()), revolveVertexFrameLiftAllow(rp, axisRadiusUpper))),
				denot:    body.doc.mintCurve(),
			}
			latitudeLength := 2 * math.Pi * j.rho
			latitudeBound := proofbound.ConservativeValueError(latitudeLength, proofbound.ProductUpper(axisRadiusUpper, proofbound.TwoPiUpper()))
			if rhoEnc, ok := junctionRadiusInterval(j.rho, rhoBound); ok {
				enc := proofbound.IntervalMul(proofbound.TwoPiInterval(), rhoEnc)
				latitudeBound = math.Min(latitudeBound, proofbound.IntervalFloatError(enc, latitudeLength))
			}
			j.lat = &Edge{
				curve:       Circle3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
				start:       seam,
				end:         seam,
				convex:      turn > 0,
				length:      latitudeLength,
				lengthBound: latitudeBound,
				denot:       body.doc.mintCurve(),
			}
		case !rp.full:
			j.v0 = &Vertex{
				position: rp.point(b, j.z, j.rho, rp.phi0),
				bound:    units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(j.rho, rp.phi0Delta()), revolveVertexFrameLiftAllow(rp, axisRadiusUpper))),
				denot:    body.doc.mintCurve(),
			}
			j.v1 = j.v0
			if !j.onAxis {
				j.v1 = &Vertex{
					position: rp.point(b, j.z, j.rho, rp.phi1),
					bound:    units.Millimeters(proofbound.AbsSumUpper(proofbound.ProductUpper(j.rho, rp.phi1Delta()), revolveVertexFrameLiftAllow(rp, axisRadiusUpper))),
					denot:    body.doc.mintCurve(),
				}
				arcLength := j.rho * dphi
				dphiUpper := proofbound.AbsSumUpper(math.Abs(dphi), sweep.Bound)
				arcBound := proofbound.ConservativeValueError(arcLength, proofbound.ProductUpper(axisRadiusUpper, dphiUpper))
				if rhoEnc, ok := junctionRadiusInterval(j.rho, rhoBound); ok {
					if widthEnc, ok := rp.den.widthInterval(); ok {
						enc := proofbound.IntervalMul(rhoEnc, widthEnc)
						arcBound = math.Min(arcBound, proofbound.IntervalFloatError(enc, arcLength))
					}
				}
				j.arc = &Edge{
					curve:       Arc3{Center: center, Axis: wDir.Scale(sweepSign), Radius: units.Millimeters(j.rho)},
					start:       j.v0,
					end:         j.v1,
					convex:      turn > 0,
					length:      arcLength,
					lengthBound: arcBound,
					denot:       body.doc.mintCurve(),
				}
			}
		}
		js[i] = j
	}

	faces := make([]*Face, 0, n)
	total := proofbound.BoundedScalar{}
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		if kinds[i] == wallAxis {
			// A LineSeg lying exactly on the axis sweeps no face — the same
			// rule a profile-fed revolve's own wallAxis segment follows
			// (docs/evaluator-design.md §6) — and, with no cap to bridge the
			// gap either side of it, a chain mints nothing at all for it.
			continue
		}
		surf, reversed, err := rp.wallSurface(b, w.SegmentWalk, kinds[i])
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		origins, err := sideOriginsContext(ctx, ref, loopIdx, w.Segs)
		if err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		segs := make([]CurveSegment, len(w.Segs))
		for oi, si := range w.Segs {
			segs[oi] = rp.profile.Outer.Segments[si]
		}
		faceArea := proofbound.BoundedMul(walkAxisMoment(w.SegmentWalk, kinds[i], segs, rp.ax), sweep)
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.Value,
			areaBound: faceArea.Bound,
			reversed:  reversed,
		}
		if rp.full {
			face.loops = fullRevLoops(js[i], js[i+1], kinds[i])
		} else {
			// holeLoop is always false: a chain has no hole, and its whole
			// walk takes the loop-0 (outer) convention
			// (docs/surface-design.md §13.4).
			cap0 := rp.capEdge(b, w.SegmentWalk, false, js[i].v0, js[i+1].v0, rp.phi0, rp.phi0Delta(), false)
			cap1 := rp.capEdge(b, w.SegmentWalk, false, js[i].v1, js[i+1].v1, rp.phi1, rp.phi1Delta(), false)
			co := []coedge{{edge: cap0, forward: true}}
			if a := js[i+1].arc; a != nil {
				co = append(co, coedge{edge: a, forward: true})
			}
			co = append(co, coedge{edge: cap1, forward: false})
			if a := js[i].arc; a != nil {
				co = append(co, coedge{edge: a, forward: false})
			}
			face.loops = []*Loop{{coedges: co, outer: true}}
		}
		if err := attachFaceLoopsContext(ctx, []*Face{face}); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		faces = append(faces, face)
		total = proofbound.BoundedAdd(total, faceArea)
	}
	return faces, total, nil
}
