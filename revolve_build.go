package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemass"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"

	"github.com/lestrrat-3d/decad/internal/surfacenormal"
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
	den           revolveangle.Sweep
	xform         r3.Transform
	surfaceResult bool
	// sectionDelta is prismPayload's own §7 term over the MERIDIAN: the proven
	// upper bound on how far any recorded meridian point sits from the
	// meridian its construction denotes, and any denoted point from the
	// recorded one, both ways. Where sectionWhole is set it also states that
	// each recorded segment pairs with a denoted one of its kind whose ends
	// (and an arc's centre) sit within it, the arc's sweep taken without a 2π
	// wrap. The wall areas take it through
	// docs/surface-intersection-design.md §7.1's fold into the axis-coordinate
	// walk, the box through extentBoundedAlong's fifth mechanism, and the
	// solid's region readings — the Pappus volume, the centroid and a cap's
	// area — through §7.2's band (revolve_section.go). A revolve a caller draws
	// directly leaves it zero and every reading takes the path it takes today,
	// bit for bit.
	sectionDelta float64
	// sectionWhole says sectionDelta reaches every recorded coordinate — each
	// segment's two ends and an arc's centre — as an offset construction's does
	// (shell_revolve.go), rather than only the cut ends a trim records, and
	// that the segment-wise pairing above holds. The per-walk readings then
	// charge every end, every vertex and every wall's denoted normal (§7.2); a
	// cut construction leaves it false and charges its own cut ends through
	// trimRevolveSegmentCharges.
	sectionWhole bool
	// radialProof belongs to this exact profile and resolved axis. A path
	// replacing either must clear it; placement alone preserves both.
	radialProof bool
	// blendSegs and blendKind are prismPayload's own blend-role descriptors,
	// read over the MERIDIAN: a revolve junction fillet or chamfer
	// (revolve_blend.go) records, per loop, the segment indices of its
	// rewritten record that are blend connectors, and blendKind is "fillet" or
	// "chamfer". The build gives each such wall a second kind(i,j) role beside
	// its side(i,j) one. They ride on the payload so Placed and Duplicate
	// re-mint the same roles; a path replacing the profile must clear them.
	blendSegs []map[int]struct{}
	blendKind string
}

// requireExactRevolveSection is the reject-only guard an operation that
// rewrites the recorded meridian takes — a junction blend, a shell: each
// offsets or cuts the RECORDED meridian, and a meridian displaced from the one
// it denotes has no proven rewrite. The field is either zero or it is not, so
// the guard needs no tolerance and can only refuse.
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
// buildPrismScene, prismcells.NewReexpression, prismcells.Classify and the rest of
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

// lift is the record rp's sweep basis is built from: the plane frame and the
// resolved axis in it.
func (rp revolvePayload) lift() revolvemesh.RevolveLift {
	return revolvemesh.RevolveLift{Frame: rp.frame, AU: rp.ax.aU, AV: rp.ax.aV, DU: rp.ax.dU, DV: rp.ax.dV}
}

// basis derives the sweep basis from the plane frame and the axis frame.
func (rp revolvePayload) basis() revolvemesh.RevolveBasis { return rp.lift().Basis() }

// axisBound is the resolved axis's own proven anchor and direction
// displacement, in the form the swept-vertex comparison reads it.
func (rp revolvePayload) axisBound() revolvemesh.AxisBound {
	return revolvemesh.AxisBound{AU: rp.ax.aUBound, AV: rp.ax.aVBound, DU: rp.ax.dUBound, DV: rp.ax.dVBound}
}

// sweptPoint is the recorded plane-local point a swept vertex denotes, with
// the walk's own proven bound on it (survey2d.SegmentWalk's StartBound/
// EndBound: zero where the record states the coordinate, the evaluation's
// own bound where the walk computed it).
type sweptPoint struct {
	u, v  float64
	bound proofbound.WalkEndBound
}

// walkStart and walkEnd read a PLANE-local walk's (revolveWalks.Plane) two
// ends, each bounded against the point its recorded segment seg denotes there
// (boundarywalk.DenotedStartBound and DenotedEndBound).
func walkStart(seg CurveSegment, w survey2d.SegmentWalk) sweptPoint {
	return sweptPoint{u: w.StartU, v: w.StartV, bound: boundarywalk.DenotedStartBound(seg, w)}
}

func walkEnd(seg CurveSegment, w survey2d.SegmentWalk) sweptPoint {
	return sweptPoint{u: w.EndU, v: w.EndV, bound: boundarywalk.DenotedEndBound(seg, w)}
}

// junctionStart reads the junction where axis walk prev ends and next starts
// as a plane-local point bounded against the points BOTH neighbours' recorded
// segments denote there (boundarywalk.JunctionVertex): at a cut junction the
// two differ (docs/evaluator-design.md §4).
func revolveJunctionStart(rw revolveWalks, prev, next survey2d.SideWalk) sweptPoint {
	pi, ni := prev.Segs[len(prev.Segs)-1], next.Segs[0]
	u, v, bound := boundarywalk.JunctionVertex(rw.Segs[pi], rw.Plane[pi], rw.Segs[ni], rw.Plane[ni])
	return sweptPoint{u: u, v: v, bound: bound}
}

// sweptEnd is one end of the sweep a vertex sits at: the held angle and the
// angle the record denotes there (rp.phi0 with rp.den.Phi0, or rp.phi1 with
// rp.den.Phi1).
type sweptEnd struct {
	phi float64
	den revolveangle.Angle
}

func (rp revolvePayload) end0() sweptEnd { return sweptEnd{phi: rp.phi0, den: rp.den.Phi0} }
func (rp revolvePayload) end1() sweptEnd { return sweptEnd{phi: rp.phi1, den: rp.den.Phi1} }

// sweptGap proves how far held sits from the point at denotes, swept to the
// angle end's record states (revolvemesh.RevolveLift.SweptPointGap;
// topology.go's Vertex.Position contract). An end with no denotation — a
// ToFaceAngular stop, or a payload literal built without one — states no
// angle to rotate to, so the gap is unbounded there, which is the +Inf the
// end's own displacement (phi0Delta/phi1Delta) answers for every reading.
func (rp revolvePayload) sweptGap(held r3.Vec, at sweptPoint, end sweptEnd) float64 {
	if !end.den.Valid() {
		return math.Inf(1)
	}
	sin, cos, ok := end.den.SinCosFor(end.phi)
	if !ok {
		return math.Inf(1)
	}
	return rp.lift().SweptPointGap(rp.axisBound(), rp.xform, at.u, at.v, at.bound, sin, cos, held)
}

// sweptVertex places one swept vertex at axial z and radius rho — the
// re-expressed, possibly snapped axis coordinates the walk carries — swept to
// the first of ends, and stamps it with denot, the zero token where the vertex
// mints no curve certificate. Its bound is ONE exact comparison per end
// against the recorded point at, rotated to the angle that end denotes
// (sweptGap): the re-expression's rounding, the snap, the axis's own anchor
// and direction error, the angular displacement and trigonometric evaluation,
// and the frame lift and placement rounding all sit inside it. A vertex one
// end alone places passes that end; the shared on-axis vertex a partial sweep
// gives both caps passes both, since it stands for the recorded point at each.
// The bound is zero wherever every comparison is exact, which keeps an
// ordinary revolve's φ = 0 cap and seam vertices Exact.
func (rp revolvePayload) sweptVertex(b revolvemesh.RevolveBasis, z, rho float64, at sweptPoint, denot curveToken, ends ...sweptEnd) *Vertex {
	held := rp.point(b, z, rho, ends[0].phi)
	gap := 0.0
	for _, end := range ends {
		gap = math.Max(gap, rp.sweptGap(held, at, end))
	}
	return &Vertex{position: held, bound: units.Millimeters(gap), denot: denot}
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
	// The denoted section's points sit within sectionDelta of the recorded
	// boundary, so the envelope that bounds the material is widened over it
	// (docs/surface-intersection-design.md §7.2).
	coordUpper = revolveaxis.SectionCoordUpper(coordUpper, rp.sectionDelta)
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

// revolveCentroidLift carries the magnitudes the centroid's own lift
// A3 + W·axial + (E0·rx + E1·ry)·scale multiplies the axis basis by: the
// axial coordinate's, and for a partial sweep the in-plane term's three
// factors, each its value plus its own proven bound. A full turn leaves the
// partial-sweep three at zero.
type revolveCentroidLift struct {
	axialUpper, rxUpper, ryUpper, scaleUpper float64
}

// charge is what that lift owes the axis basis itself, beside the float
// rounding AnalyticRoundBound charges at the centroid's own magnitude.
//
// The anchor A3 is the frame lift O + U·aU + V·aV, whose products and sums
// round at the frame origin's and the anchor's magnitudes rather than at
// A3's: a far sketch plane whose anchor lifts back near the world origin
// rounds at ulp(10⁶) while A3 itself is small. Its exact rounding is
// RevolveLift.ExactPointRound's, read at z = ρ = 0 under the identity.
//
// The axis's own proven displacement (axisInPlane's aUBound, aVBound,
// dUBound, dVBound) moves the basis the TRUE centroid is lifted through, and
// axisMoments charges it only into the axial coordinate. Read as L1 norms of
// the basis's own change: A3 moves by |U|·aUBound + |V|·aVBound, W = U·dU +
// V·dV by dW = |U|·dUBound + |V|·dVBound, E0 = −U·dV + V·dU by dE0 =
// |U|·dVBound + |V|·dUBound, and E1 = W × E0 by at most dW·|E0*| + |W|·dE0,
// since W×E0 − W*×E0* = (W − W*)×E0* + W×(E0 − E0*) and an L1 cross product
// is at most the product of its operands' L1 norms. Each moves the centroid
// by its own factor's magnitude. Every term is zero for a frame whose anchor
// lifts exactly and an axis whose anchor and direction carry no bound.
func (l revolveCentroidLift) charge(rp revolvePayload, b revolvemesh.RevolveBasis) float64 {
	a3Round := rp.lift().ExactPointRound(r3.Identity(), 0, 0, 1, 0, b.A3)
	ax := rp.ax
	lu, lv := vecL1(rp.frame.U()), vecL1(rp.frame.V())
	anchor := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, ax.aUBound), proofbound.ProductUpper(lv, ax.aVBound))
	dW := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, ax.dUBound), proofbound.ProductUpper(lv, ax.dVBound))
	terms := []float64{a3Round, anchor, proofbound.ProductUpper(l.axialUpper, dW)}
	if l.scaleUpper > 0 {
		dE0 := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, ax.dVBound), proofbound.ProductUpper(lv, ax.dUBound))
		wHeld := proofbound.AbsSumUpper(proofbound.ProductUpper(lu, math.Abs(ax.dU)), proofbound.ProductUpper(lv, math.Abs(ax.dV)))
		e0True := proofbound.AbsSumUpper(
			proofbound.ProductUpper(lu, proofbound.AbsSumUpper(ax.dV, ax.dVBound)),
			proofbound.ProductUpper(lv, proofbound.AbsSumUpper(ax.dU, ax.dUBound)),
		)
		dE1 := proofbound.AbsSumUpper(proofbound.ProductUpper(dW, e0True), proofbound.ProductUpper(wHeld, dE0))
		radial := proofbound.AbsSumUpper(proofbound.ProductUpper(l.rxUpper, dE0), proofbound.ProductUpper(l.ryUpper, dE1))
		terms = append(terms, proofbound.ProductUpper(l.scaleUpper, radial))
	}
	return proofbound.AbsSumUpper(terms...)
}

// frameCharge is massmoment.MapCharge over the map the record denotes
// through: the plane-coordinate solid of revolution carried through
// L = B·[U V U×V] (massmoment.PlaneMap), the leaves sweptGap reads. Its third
// column is the exact cross product because the sweep's own E1 = W × E0 is.
// The centroid owes nothing: an affine map carries a centroid to the centroid
// of the image, so lifting the plane-coordinate centroid through L is exact.
func (rp revolvePayload) frameCharge() (massmoment.MapCharge, error) {
	m, err := massmoment.PlaneMap(rp.frame, rp.xform)
	if err != nil {
		return massmoment.MapCharge{}, err
	}
	return massmoment.MapChargeOf(m)
}

// point places the axis-frame point (z, ρ) at sweep angle φ into placed
// world space.
func (rp revolvePayload) point(b revolvemesh.RevolveBasis, z, rho, phi float64) r3.Vec {
	return rp.lift().Point(b, rp.xform, z, rho, phi)
}

// reflected reports whether the accumulated placement flips handedness — a
// reflected solid's rotational senses invert with it, and every swept edge's
// axis carries the corrected sign.
func (rp revolvePayload) reflected() bool { return rp.xform.IsReflection() }

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
	ig, err := rp.profile.EvaluatorIntegralsUncheckedContext(ctx, freeform.MomentSecondOrder, work)
	if err != nil {
		return nil, err
	}
	if ig.Area <= 0 {
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
	ig.AreaBound = proofbound.AbsSumUpper(ig.AreaBound, rp.ax.snap.area)
	sweep := rp.sweep()
	dphi := sweep.Value
	if dphi <= 0 {
		return nil, fmt.Errorf(`%w: the sweep interval is empty`, ErrDegenerate)
	}
	frame, err := rp.frameCharge()
	if err != nil {
		return nil, err
	}
	q, mzr, mrr := revolvemass.AxisMoments(ig, rp.ax.numeric())
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
		// that end's proven displacement. denoted is the plane the record
		// states, which NormalAt proves the cap's normal against in place of
		// the tag and normalBound, so the axis direction, the frame lift and
		// every placement rounding are charged too; normalBound stays the
		// figure the gates that refuse a departed face read
		// (docs/evaluator-design.md §6).
		//
		// capAdmitAllow is revolveAxisAdmitBandCharge's own term (above): each
		// cap face's LOOP is walked from the axis-snapped profile, while its
		// area here is ig.Area, the Pappus engine's own integral over the
		// UNSNAPPED recorded one. The two agree exactly wherever
		// rp.ax.radialAdmitAllow is zero — every axis-aligned fixture — and
		// otherwise this is what keeps the published cap area from claiming a
		// tighter bound than the snap/unsnap mismatch can actually cost it.
		capAdmitAllow := revolvemass.AdmitBandCharge(rp.ax.radialAdmitAllow, rp.ax.axialExtentUpper)
		capStart = &Face{
			surface:     Plane{Frame: startFrame},
			origins:     []FeatureRef{{producer: ref, Role: roleCapStart}},
			body:        body,
			area:        ig.Area,
			areaBound:   proofbound.AbsSumUpper(ig.AreaBound, capAdmitAllow),
			normalBound: rp.phi0Delta(),
			denoted:     rp.capDenotation(rp.end0(), true),
		}
		capEnd = &Face{
			surface:     Plane{Frame: endFrame},
			origins:     []FeatureRef{{producer: ref, Role: roleCapEnd}},
			body:        body,
			area:        ig.Area,
			areaBound:   proofbound.AbsSumUpper(ig.AreaBound, capAdmitAllow),
			normalBound: rp.phi1Delta(),
			denoted:     rp.capDenotation(rp.end1(), false),
		}
	}

	// The section displacement's region charge (docs/surface-intersection-design.md
	// §7.2): the recorded and denoted regions differ by a band of at most
	// section.Band, so the cap area and the three axis-frame moments each move by
	// their integrand's envelope over it. The snap's charges above are about the
	// recorded region against the snapped one and compose beside this one. Zero
	// for every payload no construction displaced, and folded nowhere then.
	section, err := revolveSectionChargeOf(rp, work)
	if err != nil {
		return nil, err
	}
	if section.Band > 0 {
		q.Bound = proofbound.AbsSumUpper(q.Bound, section.First())
		mzr.Bound = proofbound.AbsSumUpper(mzr.Bound, section.Second())
		mrr.Bound = proofbound.AbsSumUpper(mrr.Bound, section.Second())
		if !rp.full {
			capStart.areaBound = proofbound.AbsSumUpper(capStart.areaBound, section.Band)
			capEnd.areaBound = proofbound.AbsSumUpper(capEnd.areaBound, section.Band)
		}
	}
	// Every charge above bounds the cap's plane-coordinate area; its image
	// under the denoted map L takes L's own stretch last (frameCharge).
	if !rp.full {
		for _, c := range []*Face{capStart, capEnd} {
			c.areaBound = frame.AreaOf(proofbound.MeasuredScalar(c.area, c.areaBound)).Bound
		}
	}

	sideArea := proofbound.BoundedScalar{}
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	perLoop := make([]revLoopParts, len(loops))
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		parts, err := buildRevolveLoop(ctx, body, ref, rp, b, li, loop, work, frame)
		if err != nil {
			return nil, err
		}
		perLoop[li] = parts
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
			faces = append(faces, group.faces...)
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
	if err := addBlendRoles(ctx, body, ref, rp.blendSegs, rp.blendKind); err != nil {
		return nil, err
	}

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
	volume.Bound = proofbound.AbsSumUpper(volume.Bound,
		revolvemass.AdmitVolumeCharge(rp.ax.radialAdmitAllow, rp.ax.axialExtentUpper))
	// The Pappus volume is the plane-coordinate solid's; L scales it by
	// exactly |det L| (frameCharge).
	volume = frame.VolumeOf(volume)
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
	lift := revolveCentroidLift{axialUpper: proofbound.AbsSumUpper(axial.Value, axial.Bound)}
	if !rp.full {
		// The in-plane term is the swept radial direction integrated over
		// the interval — closed form in the sweep angle; a full turn's is
		// identically zero, which is what puts its centroid on the axis.
		sin1, cos1 := revolveangle.EndSinCos(rp.den.Phi1, rp.phi1)
		sin0, cos0 := revolveangle.EndSinCos(rp.den.Phi0, rp.phi0)
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
		lift.rxUpper = proofbound.AbsSumUpper(rx.Value, rx.Bound)
		lift.ryUpper = proofbound.AbsSumUpper(ry.Value, ry.Bound)
		lift.scaleUpper = proofbound.AbsSumUpper(radialScale.Value, radialScale.Bound)
	}
	// An absent axis-lift charge folds nothing: proofbound.AbsSumUpper up-rounds
	// every term it folds, zero included, and an exact axis must read as it
	// did before the charge existed.
	if axisLift := lift.charge(rp, b); axisLift > 0 {
		centroidBound = proofbound.AbsSumUpper(centroidBound, axisLift)
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
	centroidBound = proofbound.AbsSumUpper(centroidBound,
		revolvemass.AdmitBandCharge(rp.ax.radialAdmitAllow, rp.ax.axialExtentUpper))
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
	// runs[i] is faces[i]'s run: the off-axis stretch of the loop between two
	// on-axis walks that holds it. A loop with fewer than two on-axis walks is
	// one run. outerRun is the run holding the loop's on-axis junction of least
	// axial coordinate.
	runs     []int
	outerRun int
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

// junctionCircle is the placed circle a junction at axis coordinates (z, ρ)
// sweeps: its centre on the axis at z, its axis the placed sweep axis (negated
// under a reflected placement, so the circle's sense stays the sweep's), and
// its radius ρ. buildRevolveLoop stamps every junction edge from it, and
// revolve_blend.go matches a selected edge against it, so the two read the
// same numbers from the same arithmetic.
func (rp revolvePayload) junctionCircle(b revolvemesh.RevolveBasis, z, rho float64) (r3.Vec, r3.Vec, units.Value) {
	sweepSign := 1.0
	if rp.reflected() {
		sweepSign = -1
	}
	return rp.point(b, z, 0, 0), rp.xform.ApplyDir(b.W).Scale(sweepSign), units.Millimeters(rho)
}

// revolveWalks is the shared plane and axis-coordinate resolution.
type revolveWalks = revolveaxis.ResolvedWalks

// buildRevolveLoop builds one loop's side faces with shared vertices and
// edges, returning the faces, the two caps' coedges in walk order, and the
// loop's side area.
func buildRevolveLoop(ctx context.Context, body *Body, ref producerID, rp revolvePayload, b revolvemesh.RevolveBasis, li int, loop LoopRecord, work *freeform.FreeformWork, frame massmoment.MapCharge) (revLoopParts, error) {
	resolved, err := revolveaxis.ResolveLoop(ctx, loop, work, "the revolve wall build", rp.chargedWalk, rp.ax.snapTol)
	if err != nil {
		return revLoopParts{}, err
	}
	walks, kinds, singleClosed := resolved.Walks, resolved.Kinds, resolved.SingleClosed
	n := len(walks)
	sweep := rp.sweep()
	dphi := sweep.Value

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
			// The recorded point the junction denotes: walk i's start is its
			// first recorded segment's own start (revolvesampling.MeridianJunctions,
			// tessellate_revolve.go, reads the same one), bounded against the
			// previous walk's denoted end too.
			prev := walks[(i+n-1)%n]
			at := rp.denotedPoint(revolveJunctionStart(resolved, prev, w))
			turn := prev.TanOutU*w.TanInV - prev.TanOutV*w.TanInU
			center, jAxis, jRadius := rp.junctionCircle(b, j.z, j.rho)
			switch {
			case rp.full && !j.onAxis:
				seam := rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end0())
				latitudeLength := 2 * math.Pi * j.rho
				latitudeBound := proofbound.ConservativeValueError(latitudeLength, proofbound.ProductUpper(w.AxisRadiusUpper, proofbound.TwoPiUpper()))
				if rhoEnc, ok := junctionRadiusInterval(j.rho, w.StartVBound); ok {
					enc := proofbound.IntervalMul(proofbound.TwoPiInterval(), rhoEnc)
					latitudeBound = math.Min(latitudeBound, proofbound.IntervalFloatError(enc, latitudeLength))
				}
				j.lat = &Edge{
					curve:       Circle3{Center: center, Axis: jAxis, Radius: jRadius},
					start:       seam,
					end:         seam,
					convex:      turn > 0,
					length:      latitudeLength,
					lengthBound: frame.LengthBound(latitudeLength, latitudeBound),
					// The CURVE half of the shared-denotation certificate
					// (denotation.go): this junction latitude circle is
					// shared, by construction, between side faces i-1 and i
					// of THIS build alone (fullRevLoops below), exactly the
					// role a prism's own rim edge plays for its two caps.
					denot: body.doc.mintCurve(),
				}
			case !rp.full:
				if j.onAxis {
					j.v0 = rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end0(), rp.end1())
					j.v1 = j.v0
				} else {
					j.v0 = rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end0())
					j.v1 = rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end1())
					arcLength := j.rho * dphi
					dphiUpper := proofbound.AbsSumUpper(math.Abs(dphi), sweep.Bound)
					arcBound := proofbound.ConservativeValueError(arcLength, proofbound.ProductUpper(w.AxisRadiusUpper, dphiUpper))
					if rhoEnc, ok := junctionRadiusInterval(j.rho, w.StartVBound); ok {
						if widthEnc, ok := rp.den.WidthInterval(); ok {
							enc := proofbound.IntervalMul(rhoEnc, widthEnc)
							arcBound = math.Min(arcBound, proofbound.IntervalFloatError(enc, arcLength))
						}
					}
					j.arc = &Edge{
						curve:       Arc3{Center: center, Axis: jAxis, Radius: jRadius},
						start:       j.v0,
						end:         j.v1,
						convex:      turn > 0,
						length:      arcLength,
						lengthBound: frame.LengthBound(arcLength, arcBound),
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
					lengthBound: frame.LengthBound(w.Length, w.LengthBound),
				}
				cap0[i], cap1[i] = shared, shared
				continue
			}
			var vs0, ve0, vs1, ve1 *Vertex
			if !singleClosed {
				vs0, ve0 = js[i].v0, js[(i+1)%n].v0
				vs1, ve1 = js[i].v1, js[(i+1)%n].v1
			}
			// A whole closed walk's seam vertex denotes its one recorded
			// segment's own start; every other walk's cap edge takes the
			// junction vertices above.
			seam := rp.denotedPoint(walkStart(resolved.Segs[w.Segs[0]], resolved.Plane[w.Segs[0]]))
			cap0[i] = rp.capEdge(b, w.SegmentWalk, singleClosed, vs0, ve0, seam, rp.end0(), holeLoop, frame)
			cap1[i] = rp.capEdge(b, w.SegmentWalk, singleClosed, vs1, ve1, seam, rp.end1(), holeLoop, frame)
		}
	}

	parts := revLoopParts{}
	axisWalks := 0
	for _, k := range kinds {
		if k == wallAxis {
			axisWalks++
		}
	}
	// A full turn sweeps each run into its own closed surface: an on-axis
	// junction sweeps a point, never an edge, so two runs share no edge. Of
	// those surfaces the outer one encloses every other, so its two axis ends
	// bracket theirs (fullRevolveShellsContext).
	seen, leastZ := 0, math.Inf(1)
	for i, w := range walks {
		if err := ctx.Err(); err != nil {
			return revLoopParts{}, err
		}
		run := 0
		if axisWalks > 1 {
			// The faces after the last on-axis walk close the loop onto the
			// faces before the first, so they share its run.
			run = seen % axisWalks
		}
		if kinds[i] == wallAxis {
			seen++
		} else {
			for _, end := range [...][2]float64{{w.StartU, w.StartV}, {w.EndU, w.EndV}} {
				if end[1] == 0 && end[0] < leastZ {
					leastZ, parts.outerRun = end[0], run
				}
			}
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
		faceArea := frame.AreaOf(proofbound.BoundedMul(rp.wallMoment(w.SegmentWalk, kinds[i], segs), sweep))
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.Value,
			areaBound: faceArea.Bound,
			reversed:  reversed,
			denoted:   rp.wallDenotation(w, kinds[i], resolved.Plane),
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
		parts.runs = append(parts.runs, run)
		parts.area = proofbound.BoundedAdd(parts.area, faceArea)
		if !rp.full {
			parts.startCo = append(parts.startCo, coedge{edge: cap0[i], forward: true})
			parts.endCo = append(parts.endCo, coedge{edge: cap1[i], forward: true})
		}
	}
	return parts, nil
}

// fullRevolveShellsContext builds one shell per non-empty run of each loop
// group. A loop meeting the axis along k ≥ 2 walks has k runs, and the full
// turn sweeps each into its own closed surface (a hollow cylinder's meridian
// sweeps an outer skin and a cavity wall); every other loop is one run. sheet
// is rp.surfaceResult: a full revolution's wall set already closes on itself,
// so every shell built here is closed regardless of kind, but shellIsOpen
// still runs rather than assuming it — the same discipline every other shell
// in this build follows (docs/surface-design.md §2.2). On a solid, a hole
// loop's run bounds its own toroidal cavity (evaluator §6), and so does every
// run of the outer loop but its outer one: the outer surface encloses the
// others, so its two axis ends bracket every other on-axis junction of the
// loop. A sheet bounds no cavity at all (decision B, §2.2), so void is always
// false there.
func fullRevolveShellsContext(ctx context.Context, perLoop []revLoopParts, sheet bool) ([]*Shell, error) {
	var shells []*Shell
	for li, parts := range perLoop {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		// Runs in order of first appearance, the outer run first.
		order := []int{parts.outerRun}
		groups := map[int][]*Face{}
		for i, f := range parts.faces {
			r := parts.runs[i]
			if _, ok := groups[r]; !ok && r != parts.outerRun {
				order = append(order, r)
			}
			groups[r] = append(groups[r], f)
		}
		for _, r := range order {
			group := groups[r]
			if len(group) == 0 {
				continue
			}
			void := !sheet && (li != 0 || r != parts.outerRun)
			shells = append(shells, &Shell{faces: group, open: shellIsOpen(group), void: void})
		}
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
// takes the loop's: outer convex, hole concave. end is THIS cap's end of the
// sweep (rp.end0() or rp.end1()), and seam the recorded point a closed walk's
// own seam vertex denotes; that vertex is bounded the same way every other cap
// vertex is (sweptVertex; docs/evaluator-design.md §6). An open walk reads
// neither.
func (rp revolvePayload) capEdge(b revolvemesh.RevolveBasis, w survey2d.SegmentWalk, closed bool, vs, ve *Vertex, seam sweptPoint, end sweptEnd, holeLoop bool, frame massmoment.MapCharge) *Edge {
	convex := !holeLoop
	if w.IsCircular() {
		convex = w.Th0 < w.Th1
	}
	e := &Edge{convex: convex, length: w.Length, lengthBound: frame.LengthBound(w.Length, w.LengthBound)}
	if !w.IsCircular() {
		e.curve = Line3{}
		e.start, e.end = vs, ve
		return e
	}
	// The cap plane at φ is the profile plane rotated about the axis, so
	// its normal is the rotated sweep-velocity direction; the walk's own
	// range order is the arc's CCW sense about it, inverted once by a
	// reflected placement.
	sin, cos := math.Sincos(end.phi)
	normal := b.E0.Scale(-sin).Add(b.E1.Scale(cos))
	sign := 1.0
	if w.Th1 < w.Th0 {
		sign = -1
	}
	if rp.reflected() {
		sign = -sign
	}
	axis := rp.xform.ApplyDir(normal).Scale(sign)
	center := rp.point(b, w.CU, w.CV, end.phi)
	radius := units.Millimeters(w.Radius)
	if closed {
		v := rp.sweptVertex(b, w.StartU, w.StartV, seam, curveToken{}, end)
		e.curve = Circle3{Center: center, Axis: axis, Radius: radius}
		e.start, e.end = v, v
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

// wallDenotation encloses the surface the wall w denotes: its recorded
// segments, read off the plane-local walks they were re-expressed from, swept
// about the recorded axis and placed (surfacenormal.Revolved). A straight
// wall reads each recorded segment's own two ends, in walk order; a circular
// wall reads each segment's recorded centre and radius. Face.NormalAt judges
// the wall's normal against it, and Stitch's flux path admits the wall only
// where its tag is exactly this surface (docs/evaluator-design.md §6,
// docs/surface-design.md §6.4).
func (rp revolvePayload) wallDenotation(w survey2d.SideWalk, kind wallKind, plane []survey2d.SegmentWalk) *surfacenormal.Revolved {
	lift, ab := rp.lift(), rp.axisBound()
	section := revolveaxis.SectionWholeCharges(rp.sectionWhole, rp.sectionDelta)
	var den surfacenormal.Revolved
	switch kind {
	case wallSphere, wallTorus:
		circles := make([]revolvemesh.RecordedMeridian, len(w.Segs))
		for i, si := range w.Segs {
			pw := plane[si]
			circles[i] = revolvemesh.RecordedMeridian{
				U: pw.CU, V: pw.CV, UV: revolveaxis.DenotedBound(proofbound.WalkEndBound{}, section),
				R: pw.Radius, RBound: revolveaxis.DenotedRadiusBound(pw.RadiusBound, section),
			}
		}
		den = lift.CircularWallNormal(ab, rp.xform, circles)
	default:
		ends := make([][2]revolvemesh.RecordedMeridian, len(w.Segs))
		for i, si := range w.Segs {
			pw := plane[si]
			ends[i] = [2]revolvemesh.RecordedMeridian{
				{U: pw.StartU, V: pw.StartV, UV: revolveaxis.DenotedBound(pw.StartBound, section)},
				{U: pw.EndU, V: pw.EndV, UV: revolveaxis.DenotedBound(pw.EndBound, section)},
			}
		}
		den = lift.StraightWallNormal(ab, rp.xform, ends)
	}
	return &den
}

// capDenotation encloses the plane a partial sweep's cap at end denotes: the
// plane through the recorded axis at the angle end's record states, placed
// (revolvemesh.RevolveLift.CapNormal). An end with no denotation — a
// ToFaceAngular stop, or a payload literal built without one — states no
// angle, so it returns nil and the cap keeps its tag's proof composed with
// its normalBound, which is +Inf there.
func (rp revolvePayload) capDenotation(end sweptEnd, start bool) *surfacenormal.Revolved {
	if !end.den.Valid() {
		return nil
	}
	sin, cos, ok := end.den.SinCosFor(end.phi)
	if !ok {
		return nil
	}
	den := rp.lift().CapNormal(rp.axisBound(), rp.xform, sin, cos, start)
	return &den
}

// This section is RevolveChain's own build (docs/surface-design.md §13.4):
// the open chain's counterpart of evalRevolveContextWork/buildRevolveLoop
// above, reusing rp.wallSurface, rp.capEdge, sideOriginsContext and
// revolvemass.WallAxisMoment unchanged. It differs only in TOPOLOGY: n segments place
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

	frame, err := rev.frameCharge()
	if err != nil {
		return nil, err
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
		resolved, err := revolveaxis.ResolveChain(ctx, rp.chains[ci], work, "the revolve wall build",
			view.chargedWalk, view.ax.snapTol)
		if err != nil {
			return nil, err
		}
		if len(resolved.Walks) == 0 {
			return nil, fmt.Errorf(`%w: a recorded chain holds no segments`, ErrDegenerate)
		}
		if err := revolveaxis.RequireChainAxisIncidence(resolved); err != nil {
			return nil, err
		}
		faces, walkArea, err := buildChainRevolveWalls(ctx, body, ref, view, ci, b, resolved, frame)
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
func buildChainRevolveWalls(ctx context.Context, body *Body, ref producerID, rp revolvePayload, loopIdx int, b revolvemesh.RevolveBasis, resolved revolveWalks, frame massmoment.MapCharge) ([]*Face, proofbound.BoundedScalar, error) {
	walks, kinds := resolved.Walks, resolved.Kinds
	n := len(walks)
	sweep := rp.sweep()
	dphi := sweep.Value
	sweepSign := 1.0
	if rp.reflected() {
		sweepSign = -1
	}
	wDir := rp.xform.ApplyDir(b.W)

	// junctionSource reads junction i's own (z, rho), the walk whose end it
	// belongs to, and the recorded point it denotes: junction i sits at walk
	// i's start for i < n, and at the LAST walk's own end for i == n — the
	// chain's own two free ends. A coalesced walk starts at its first recorded
	// segment's start and ends at its last one's end.
	junctionSource := func(i int) (z, rho, rhoBound, axisRadiusUpper float64, at sweptPoint) {
		switch i {
		case n:
			w := walks[n-1]
			last := w.Segs[len(w.Segs)-1]
			return w.EndU, w.EndV, w.EndVBound, w.AxisRadiusUpper, rp.denotedPoint(walkEnd(resolved.Segs[last], resolved.Plane[last]))
		case 0:
			w := walks[0]
			return w.StartU, w.StartV, w.StartVBound, w.AxisRadiusUpper, rp.denotedPoint(walkStart(resolved.Segs[w.Segs[0]], resolved.Plane[w.Segs[0]]))
		}
		w := walks[i]
		return w.StartU, w.StartV, w.StartVBound, w.AxisRadiusUpper, rp.denotedPoint(revolveJunctionStart(resolved, walks[i-1], w))
	}

	js := make([]revJunction, n+1)
	for i := 0; i <= n; i++ {
		if err := ctx.Err(); err != nil {
			return nil, proofbound.BoundedScalar{}, err
		}
		z, rho, rhoBound, axisRadiusUpper, at := junctionSource(i)
		j := revJunction{z: z, rho: rho, onAxis: rho == 0}
		var turn float64
		if i > 0 && i < n {
			prev, w := walks[i-1], walks[i]
			turn = prev.TanOutU*w.TanInV - prev.TanOutV*w.TanInU
		}
		center := rp.point(b, j.z, 0, 0)
		switch {
		case rp.full && !j.onAxis:
			seam := rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end0())
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
				lengthBound: frame.LengthBound(latitudeLength, latitudeBound),
				denot:       body.doc.mintCurve(),
			}
		case !rp.full:
			if j.onAxis {
				j.v0 = rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end0(), rp.end1())
				j.v1 = j.v0
			} else {
				j.v0 = rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end0())
				j.v1 = rp.sweptVertex(b, j.z, j.rho, at, body.doc.mintCurve(), rp.end1())
				arcLength := j.rho * dphi
				dphiUpper := proofbound.AbsSumUpper(math.Abs(dphi), sweep.Bound)
				arcBound := proofbound.ConservativeValueError(arcLength, proofbound.ProductUpper(axisRadiusUpper, dphiUpper))
				if rhoEnc, ok := junctionRadiusInterval(j.rho, rhoBound); ok {
					if widthEnc, ok := rp.den.WidthInterval(); ok {
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
					lengthBound: frame.LengthBound(arcLength, arcBound),
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
		faceArea := frame.AreaOf(proofbound.BoundedMul(rp.wallMoment(w.SegmentWalk, kinds[i], segs), sweep))
		face := &Face{
			surface:   surf,
			origins:   origins,
			body:      body,
			area:      faceArea.Value,
			areaBound: faceArea.Bound,
			reversed:  reversed,
			denoted:   rp.wallDenotation(w, kinds[i], resolved.Plane),
		}
		if rp.full {
			face.loops = fullRevLoops(js[i], js[i+1], kinds[i])
		} else {
			// holeLoop is always false: a chain has no hole, and its whole
			// walk takes the loop-0 (outer) convention
			// (docs/surface-design.md §13.4).
			cap0 := rp.capEdge(b, w.SegmentWalk, false, js[i].v0, js[i+1].v0, sweptPoint{}, rp.end0(), false, frame)
			cap1 := rp.capEdge(b, w.SegmentWalk, false, js[i].v1, js[i+1].v1, sweptPoint{}, rp.end1(), false, frame)
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
