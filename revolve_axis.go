package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file resolves a revolve's axis into the sketch plane and decides what
// the profile may do around it: axisLine2, the 2D line the axis projects to;
// axisFrame, the axis-local frame every later reading is taken in; and the
// two gates that refuse a profile crossing or touching the axis where the
// sweep would be degenerate.
//
// wallKind is decided here because it follows from the axis alone: a segment
// parallel to the axis sweeps a cylinder, one meeting it at an angle a cone,
// one perpendicular an annulus. A segment whose kind the axis cannot decide
// exactly refuses rather than being assigned the nearest one. See
// docs/evaluator-design.md §6.

// axisLine2 is a revolve axis resolved into the profile plane: a point on
// the axis and its unit direction, plane-local (u, v).
type axisLine2 struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
}

// finiteAxisValues reports whether every derived axis value is representable.
func finiteAxisValues(values ...float64) bool {
	return revolveaxis.FiniteAxisValues(values...)
}

// axisInPlane reads the public axis variant and adapts its resolved coordinates.
func axisInPlane(a Axis, frame r3.Frame) (axisLine2, error) {
	var input revolveaxis.AxisInput
	switch value := a.(type) {
	case SketchLine:
		input = revolveaxis.SketchLine{
			StartU: value.Start.U, StartV: value.Start.V,
			EndU: value.End.U, EndV: value.End.V,
		}
	case ConstructionAxis:
		input = revolveaxis.ConstructionAxis{Origin: value.Origin, Dir: value.Dir}
	default:
		return axisLine2{}, fmt.Errorf(`%w: axis %T is not supported by this evaluator`, ErrUnsupported, a)
	}
	line, err := revolveaxis.AxisInPlane(input, frame)
	if err != nil {
		return axisLine2{}, err
	}
	return axisLine2{
		aU: line.AU, aV: line.AV,
		aUBound: line.AUBound, aVBound: line.AVBound,
		dU: line.DU, dV: line.DV,
		dUBound: line.DUBound, dVBound: line.DVBound,
	}, nil
}

func axisDirectionSqrtBracket(du, dv *big.Rat, heldU, heldV float64) (float64, float64) {
	return revolveaxis.AxisDirectionSqrtBracket(du, dv, heldU, heldV)
}

// axisFrame is the revolve axis as a proper plane-local frame with the
// region on its non-negative side: z = (p−a)·d runs along the axis and
// ρ = cross(d, p−a) ≥ 0 is the radial coordinate. snapTol is the
// scale-relative tolerance that classified axis contact; a coordinate
// within it of the axis IS on the axis.
//
// radialAdmitAllow and axialExtentUpper are resolveAxisSide's own charge for
// admitting a region under UNCERTAINTY rather than proof: they are zero
// whenever resolveAxisSide proved the region's radial minimum non-negative —
// which is every axis-aligned fixture in the tree, since the arithmetic is
// then exact and admits nothing it has not proven — and otherwise they carry
// the worst case a genuine straddle leaves open. radialAdmitAllow bounds how
// far below zero the TRUE radial minimum can sit despite being admitted, and
// axialExtentUpper bounds the recorded region's own axial reach; together they
// are what revolve_build.go charges into the published volume, cap area and
// centroid bounds (revolvePayload.ax's own doc comment there), since the
// admitted region's faces are built from the SNAPPED profile while the
// integrals behind those measurements read the UNSNAPPED one. Both are the
// zero value for any axisFrame not built by resolveAxisSide (a full-sweep
// composite payload's own literal), which is the safe default: no admitted
// uncertainty, no charge.
//
// snap is the SNAP's own share of that same mismatch, and it answers a
// displacement decad COMMITS rather than one it admits without proof: wherever
// axisFrame.walk assigns an endpoint exactly 0 that the arithmetic put a
// positive distance out, the region whose boundary the built faces follow is no
// longer the recorded one, and every measurement integrated over the recorded
// one owes the difference. Its four fields bound how far the region's own area
// and its three axis-frame moments can move, and revolve_build.go adds each to
// the reading it belongs to. Every one of them is exactly zero for a profile
// whose on-axis endpoints already sit on the axis, which is every axis-incident
// fixture in the tree.
type axisFrame struct {
	aU, aV           float64
	aUBound, aVBound float64
	dU, dV           float64
	dUBound, dVBound float64
	snapTol          float64
	radialAdmitAllow float64
	// radialProof is a strict zero-threshold proof from this profile's own
	// build scan. Only a payload retaining that profile and axis may reuse it.
	radialProof      bool
	axialExtentUpper float64
	snap             regionSnapAllow
}

// numeric returns the axis values used to re-express one boundary walk.
func (ax axisFrame) numeric() revolveaxis.Frame {
	return revolveaxis.Frame{
		AU: ax.aU, AV: ax.aV, AUBound: ax.aUBound, AVBound: ax.aVBound,
		DU: ax.dU, DV: ax.dV, DUBound: ax.dUBound, DVBound: ax.dVBound,
		SnapTol: ax.snapTol,
	}
}

func (ax axisFrame) toAxis(u, v float64) (float64, float64) {
	return ax.numeric().ToAxis(u, v)
}

func (ax axisFrame) toAxisRhoBound(u, v float64) float64 {
	return ax.numeric().ToAxisRhoBound(u, v)
}

func (ax axisFrame) radialUpper(coordUpper float64) float64 {
	return ax.numeric().RadialUpper(coordUpper)
}

func (ax axisFrame) planeDirection(wg, k float64) (float64, float64) {
	return ax.numeric().PlaneDirection(wg, k)
}

func (ax axisFrame) walk(w survey2d.SegmentWalk) survey2d.SegmentWalk {
	return ax.numeric().Walk(w)
}

func (ax axisFrame) walkCharged(
	w survey2d.SegmentWalk, startCharge, endCharge proofbound.WalkEndBound,
) survey2d.SegmentWalk {
	return ax.numeric().WalkCharged(w, startCharge, endCharge)
}

// wallKind classifies what one boundary walk sweeps.
type wallKind int

const (
	// wallAxis is a line lying along the axis: it sweeps a zero-area set
	// and emits no face — the neighboring segments' faces close the solid.
	wallAxis wallKind = iota
	// wallCylinder is a line parallel to the axis.
	wallCylinder
	// wallPlane is a line perpendicular to the axis: a planar annulus, or a
	// disk when it reaches the axis.
	wallPlane
	// wallCone is an inclined line; an endpoint on the axis is its apex.
	wallCone
	// wallSphere is a circular walk whose center lies on the axis.
	wallSphere
	// wallTorus is a circular walk whose center lies off the axis.
	wallTorus
)

// classify names the surface of revolution one axis-coordinate walk sweeps.
func (ax axisFrame) classify(w survey2d.SegmentWalk) wallKind {
	if w.IsCircular() {
		if math.Abs(w.CV) <= ax.snapTol {
			return wallSphere
		}
		return wallTorus
	}
	if w.StartV == 0 && w.EndV == 0 {
		return wallAxis
	}
	dz, dr := w.EndU-w.StartU, w.EndV-w.StartV
	l := math.Hypot(dz, dr)
	if math.Abs(dr) <= 1e-9*l {
		return wallCylinder
	}
	if math.Abs(dz) <= 1e-9*l {
		return wallPlane
	}
	return wallCone
}

// IsAxis reports whether a meridian walk sweeps no face.
func (ax axisFrame) IsAxis(w survey2d.SegmentWalk) bool {
	return ax.classify(w) == wallAxis
}

// resolveAxisSide orients the axis so the recorded region lies on its
// non-negative-ρ side, enforcing the §6 half-plane and contact rules: a
// region with boundary on both sides of the axis is rejected, as is a curve
// tangent to the axis at an interior point (a circle kissing the axis would
// sweep a self-touching horn torus). It returns the oriented frame and the
// side sign the sweep interval must be remapped by.
//
// The two radial extremes are read through the boundary scan's bounded form,
// so a section whose extreme rides a computed arc radius arrives with a proven
// interval rather than a held float claiming to be exact. Each of the three
// decisions below is taken on that interval and must be DECIDED by it: the
// side sign is a structural fact about the region, not a measurement, so a
// radial extreme whose own interval straddles the ±tol band names no side and
// is refused rather than guessed. The band is 1e-9 of the section's own scale
// and the bound is the scan's own last-ulp figure, so an ordinary section is
// decided with eight orders of magnitude to spare; reaching the refusal means
// the region genuinely sits on the axis to within its own arithmetic.
//
// roff/zoff are the axis anchor's own offset along each functional: the scan
// above reads the profile about the FRAME origin, and the axis's anchor is
// what shifts that reading onto the axis. A bare `rlo -= roff` would leave
// the two products, their sum, and the subtraction's OWN round-to-nearest
// error unaccounted for, on top of whatever aU/aV/dU/dV's own proven bounds
// already are — the identical gap verify.go's revolvePayloadProvesSimple
// exists to close one level up, only here it feeds the gate that decides
// which side the region is admitted on, not a review AFTER the fact. Reading
// the offset through proofbound.BoundedMul/proofbound.BoundedAdd/proofbound.BoundedSub (the same vocabulary
// axisFrame.toAxisRhoBound already uses for the identical ρ formula at one
// point) charges every one of those roundings into rloB/rhiB, so the strict
// admission gate below reads a number the accompanying bound provably covers.
//
// THE ADMISSION GATE ITSELF is the two-part repair CLAUDE.md's reject-only
// rule requires: proofbound.AdmitBelow(near, 0) reads whether the CHOSEN side's
// near-axis extreme is proven negative, proven non-negative, or neither, off
// the extreme's own proven interval — never off a tolerance.
//
//   - Proven negative (proofbound.SurvAdmit) refuses outright, however the interval got
//     that wide: a region that truly dips across the axis is a real defect,
//     not an artifact of the bound charging it wide.
//   - Proven non-negative (proofbound.SurvReject) needs no allowance, and that holds
//     even where the bound is nonzero — an interval whose own lower end
//     already clears zero commits nothing further. This is what keeps
//     radialAdmitAllow at exactly zero for every axis-aligned fixture: their
//     arithmetic is exact (a zero bound), so proofbound.AdmitBelow can only answer
//     proofbound.SurvReject or proofbound.SurvAdmit, never straddle, and a zero-bound interval that
//     is not proven negative IS proven non-negative.
//   - Neither (proofbound.SurvStraddle) is the genuine case a nonzero bound creates — a
//     tilted or offset axis whose own direction/anchor rounding leaves the
//     true radial minimum undecided. Admitting it is sound only because the
//     interval's own worst case is bounded (by the coarse ±tol classification
//     above, which already proved the chosen side's near extreme sits no
//     lower than −tol), and radialAdmitAllow carries exactly that worst case
//     forward to revolve_build.go, which charges it into every published
//     measurement the snap/unsnap mismatch can touch.
func resolveAxisSide(ctx context.Context, profile ProfileRecord, line axisLine2, work *freeform.FreeformWork) (axisFrame, float64, error) {
	nU, nV := -line.dV, line.dU
	rlo, rhi, rBound, err := boundaryExtremesBoundedContext(ctx, profile, nU, nV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	zlo, zhi, zBound, err := boundaryExtremesBoundedContext(ctx, profile, line.dU, line.dV, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}

	// boundaryExtremesBoundedContext's own returned bound charges each
	// candidate's POSITIONAL uncertainty (a walked endpoint's own proven
	// displacement, a circular candidate's own enclosure) but NOT the
	// multiply-and-sum arithmetic of evaluating gu·u + gv·v itself for a
	// direction that is not exactly 0, 1 or −1 — the identical gap
	// axisExtremeContext closes for its own, structurally identical scan
	// through proofbound.PlaneDotDecompositionRoundAllow (internal/proofbound/bounds.go). A profile vertex
	// the record states verbatim therefore still commits real rounding
	// forming its dot with a tilted axis's own (nU, nV)/(dU, dV), and that
	// rounding is architecture-sensitive (FMA differs amd64/arm64):
	// left uncharged, the SAME recorded profile can compute as provably
	// negative on one platform and merely uncertain on another, for a region
	// whose true radial minimum is exactly zero. Folding it in here — once,
	// for THIS scan, never composed with axisExtremeContext's own copy of the
	// identical charge on a different reading — is what makes the admission
	// decision agree across platforms: the charge is zero for an axis-aligned
	// direction (proofbound.PlaneDotDecompositionRoundAllow's own trivial-coefficient
	// case), so it costs nothing for the tree's axis-aligned fixtures, and it
	// dominates a tilted axis's few-ulp discrepancy by orders of magnitude,
	// which turns a coin-flip sign into a proven straddle everywhere.
	coordUpper, err := profileCoordinateEnvelope(profile, work, nil)
	if err != nil {
		return axisFrame{}, 0, err
	}
	rBound = proofbound.AbsSumUpper(rBound, proofbound.PlaneDotDecompositionRoundAllow(nU, nV, coordUpper))
	zBound = proofbound.AbsSumUpper(zBound, proofbound.PlaneDotDecompositionRoundAllow(line.dU, line.dV, coordUpper))

	roffB := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(nU, line.dVBound), proofbound.MeasuredScalar(line.aU, line.aUBound)),
		proofbound.BoundedMul(proofbound.MeasuredScalar(nV, line.dUBound), proofbound.MeasuredScalar(line.aV, line.aVBound)),
	)
	zoffB := proofbound.BoundedAdd(
		proofbound.BoundedMul(proofbound.MeasuredScalar(line.dU, line.dUBound), proofbound.MeasuredScalar(line.aU, line.aUBound)),
		proofbound.BoundedMul(proofbound.MeasuredScalar(line.dV, line.dVBound), proofbound.MeasuredScalar(line.aV, line.aVBound)),
	)
	rloB := proofbound.BoundedSub(proofbound.MeasuredScalar(rlo, rBound), roffB)
	rhiB := proofbound.BoundedSub(proofbound.MeasuredScalar(rhi, rBound), roffB)
	zloB := proofbound.BoundedSub(proofbound.MeasuredScalar(zlo, zBound), zoffB)
	zhiB := proofbound.BoundedSub(proofbound.MeasuredScalar(zhi, zBound), zoffB)

	scale := math.Max(math.Max(math.Abs(rloB.Value), math.Abs(rhiB.Value)), math.Max(math.Abs(zloB.Value), math.Abs(zhiB.Value)))
	tol := 1e-9 * math.Max(1, scale)
	// Each side test is decided on the extreme's own proven interval: above
	// names "clear of the axis on the + side", below "clear on the − side",
	// and an interval spanning the threshold decides neither.
	hiAbove := proofbound.AdmitAbove(rhiB, tol)
	loBelow := proofbound.AdmitBelow(rloB, -tol)
	if hiAbove == proofbound.SurvStraddle || loBelow == proofbound.SurvStraddle {
		return axisFrame{}, 0, fmt.Errorf(`%w: the recorded region's radial extreme about this axis is known only to ±%v mm, which does not decide which side of the axis the region lies on`, ErrDegenerate, math.Max(rloB.Bound, rhiB.Bound))
	}
	switch {
	case hiAbove == proofbound.SurvAdmit && loBelow == proofbound.SurvAdmit:
		return axisFrame{}, 0, fmt.Errorf(`%w: the revolve axis passes through the region`, ErrDegenerate)
	case hiAbove == proofbound.SurvReject && loBelow == proofbound.SurvReject:
		return axisFrame{}, 0, fmt.Errorf(`%w: the region collapses onto the revolve axis`, ErrDegenerate)
	}
	side := 1.0
	near := rloB
	if hiAbove == proofbound.SurvReject {
		side = -1
		near = proofbound.BoundedNeg(rhiB)
	}

	var radialAdmitAllow float64
	var radialProof bool
	switch proofbound.AdmitBelow(near, 0) {
	case proofbound.SurvAdmit:
		return axisFrame{}, 0, fmt.Errorf(`%w: the recorded region's radial minimum about this axis is proven negative, so the axis cuts through material`, ErrDegenerate)
	case proofbound.SurvStraddle:
		radialAdmitAllow = math.Max(0, near.Bound-near.Value)
	case proofbound.SurvReject:
		// The positive-side reading is the same extreme and offset that
		// revolvePayloadProvesSimple would scan again. The build adds an
		// extra dot-product charge, so this proof is at least as strict.
		// A flipped axis keeps the old scan because its coefficient signs
		// and rounding path differ.
		radialProof = side > 0
	}

	axialExtent := proofbound.BoundedSub(zhiB, zloB)
	axialExtentUpper := proofbound.AbsSumUpper(axialExtent.Value, axialExtent.Bound)

	ax := axisFrame{
		aU: line.aU, aV: line.aV,
		aUBound: line.aUBound, aVBound: line.aVBound,
		dU: side * line.dU, dV: side * line.dV,
		dUBound:          line.dUBound,
		dVBound:          line.dVBound,
		snapTol:          tol,
		radialAdmitAllow: radialAdmitAllow,
		radialProof:      radialProof,
		axialExtentUpper: axialExtentUpper,
	}
	snap, err := ax.auditAxisContact(profile, work)
	if err != nil {
		return axisFrame{}, 0, err
	}
	ax.snap = snap
	return ax, side, nil
}

// regionSnapAllow is one profile's accumulated snap allowance, one field per
// integral the revolve publishes a measurement from: area bounds Σ|Δ∫dA| (the
// cap's own reading), first bounds Σ|Δ∫ρ dA| (Pappus's second theorem, so the
// volume), mixed bounds Σ|Δ∫zρ dA| (the solid centroid's axial term) and second
// bounds Σ|Δ∫ρ² dA| (a partial sweep's in-plane centroid term). ρ and z are the
// AXIS-frame coordinates, which is what keeps the last three tight: a profile
// far down the axis has a large |z| and a small ρ, and charging both at one
// frame-origin envelope would inflate the volume by the axial offset.
type regionSnapAllow struct {
	area   float64
	first  float64
	mixed  float64
	second float64
}

// add composes another walk's allowance into this one, each order summed
// outward. The integrals are additive over the boundary, so the region's total
// displacement is at most its walks' displacements summed.
func (a regionSnapAllow) add(b regionSnapAllow) regionSnapAllow {
	return regionSnapAllow{
		area:   proofbound.AbsSumUpper(a.area, b.area),
		first:  proofbound.AbsSumUpper(a.first, b.first),
		mixed:  proofbound.AbsSumUpper(a.mixed, b.mixed),
		second: proofbound.AbsSumUpper(a.second, b.second),
	}
}

// snapAllowOf is ONE walk's contribution to the region snap allowance, and the
// place the charge is derived.
//
// axisFrame.walk assigns an endpoint exactly 0 when the arithmetic put it
// within snapTol of the axis, discarding a magnitude δ ≤ snapTol. The built
// wall follows the snapped walk, the region integrals follow the recorded one,
// and the two curves bound a ribbon between them. Every point of that ribbon
// sits within δ of the recorded walk measured radially, so the ribbon lies in a
// band of width δ along a curve no longer than the longer of the two walks:
//
//	area(ribbon) ≤ δ · max(L, L') ≤ δ · (w.length + w.lengthBound)
//
// — w.lengthBound already carries both discarded magnitudes by the time this
// reads it (axisFrame.walk's own doc comment), so proofbound.AbsSumUpper over the pair
// covers whichever of the two is longer.
//
// The symmetric difference between the recorded region and the snapped one is
// contained in the union of those ribbons, so for any integrand f,
// |Δ∫f dA| ≤ area(ribbon) · sup|f| over the ribbon, and each order's charge is
// that product against the matching envelope: nothing for ∫dA, one radial
// envelope for ∫ρ dA, a radial and an axial one for ∫zρ dA, and two radial ones
// for ∫ρ² dA.
//
// Every step widens. proofbound.ProductUpper and proofbound.AbsSumUpper each round outward, δ is the
// discarded magnitude itself rather than an estimate of what it costs, and
// every sup|f| is replaced by an envelope that dominates it. A walk with
// nothing discarded contributes exactly zero, which is what leaves an
// axis-incident profile's published volume, cap area and centroid as proven as
// they were.
//
// A CIRCULAR walk with a snapped endpoint takes the same charge although its
// built surface keeps the recorded circle's own center, radius and angles: the
// snap still displaces the endpoint its neighbouring walls meet it at, by the
// same δ over the same walk, so the same ribbon dominates the difference.
func snapAllowOf(walked survey2d.SegmentWalk, discarded float64) regionSnapAllow {
	if !(discarded > 0) {
		return regionSnapAllow{}
	}
	ribbon := proofbound.ProductUpper(proofbound.AbsSumUpper(walked.Length, walked.LengthBound), discarded)
	rhoUp := walkRadialUpper(walked, discarded)
	// |z| = |(p−a)·d| ≤ |p−a| ≤ |p| + |a|, which is exactly what
	// axisFrame.radialUpper composes — it is the envelope of the whole axis-
	// local position, so it bounds the axial coordinate as well as the radial
	// one, and for a profile far down the axis it is the axial one that is
	// large.
	zUp := proofbound.AbsSumUpper(walked.AxisRadiusUpper, discarded)
	first := proofbound.ProductUpper(ribbon, rhoUp)
	return regionSnapAllow{
		area:   ribbon,
		first:  first,
		mixed:  proofbound.ProductUpper(first, zUp),
		second: proofbound.ProductUpper(first, rhoUp),
	}
}

// walkRadialUpper is a proven upper bound on |ρ| over one walk already
// re-expressed in axis coordinates, widened by the snap magnitude discarded
// along it. A straight walk's ρ runs linearly between its two endpoints, so its
// extremes ARE those endpoints, each read through the radial bound
// axisFrame.walk proved for it; a circular walk reaches at most its center's
// radial coordinate plus its radius, each read through its own bound. The
// answer is capped by the walk's own axis-radius envelope, which is proven
// independently, so this can only ever tighten and never widen it.
func walkRadialUpper(w survey2d.SegmentWalk, discarded float64) float64 {
	held := proofbound.AbsSumUpper(
		math.Max(math.Abs(w.StartV), math.Abs(w.EndV)),
		math.Max(w.StartVBound, w.EndVBound),
		discarded,
	)
	if w.IsCircular() {
		held = proofbound.AbsSumUpper(math.Abs(w.CV), w.CVBound, w.Radius, w.RadiusBound, discarded)
	}
	return math.Min(held, proofbound.AbsSumUpper(w.AxisRadiusUpper, discarded))
}

// snapDiscarded is the largest radial magnitude axisFrame.walk's snap discards
// over one walk's two endpoints, read from the walk BEFORE it was re-expressed
// — the same toAxis reading and the same snapTol comparison walk itself makes,
// so the two can never disagree about whether an endpoint snapped.
func (ax axisFrame) snapDiscarded(w survey2d.SegmentWalk) float64 {
	var discarded float64
	for _, end := range [][2]float64{{w.StartU, w.StartV}, {w.EndU, w.EndV}} {
		_, rho := ax.toAxis(end[0], end[1])
		if m := math.Abs(rho); m <= ax.snapTol && m > discarded {
			discarded = m
		}
	}
	return discarded
}

// auditAxisContact makes ONE pass over the profile's recorded walks for two
// jobs that both need every walk re-expressed in axis coordinates.
//
// It rejects the circular boundary walks a revolve cannot sweep soundly: a
// walk tangent to the axis at a point interior to
// the walk — the horn-torus contact §6 forbids — and a walk whose circle
// center lies across the axis, whose swept surface is a spindle-branch
// torus the shipped Torus (non-negative Major) cannot represent; the solid
// is valid, so that one is the staged ErrUnsupported, never a wrong face.
// For the tangency, the circle's radial minimum sits at its lowest angle;
// when that angle is strictly inside the walked range and the minimum
// reaches the axis, the contact is neither of the two allowed forms.
//
// It also sums the region snap allowance (snapAllowOf) the same walks earn,
// here rather than in a second pass of its own: re-walking a profile costs a
// second free-form conversion and a second rational bracket per segment for an
// answer this loop already holds.
func (ax axisFrame) auditAxisContact(profile ProfileRecord, work *freeform.FreeformWork) (regionSnapAllow, error) {
	const angEps = 1e-9
	var snap regionSnapAllow
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	for _, loop := range loops {
		for _, seg := range loop.Segments {
			w, err := walkOf(seg, work)
			if err != nil {
				return regionSnapAllow{}, err
			}
			if err := requireAnalyticWalk(w, "the revolve axis-contact audit"); err != nil {
				return regionSnapAllow{}, err
			}
			discarded := ax.snapDiscarded(w)
			w = ax.walk(w)
			snap = snap.add(snapAllowOf(w, discarded))
			if !w.IsCircular() {
				continue
			}
			if w.CV < -ax.snapTol {
				return regionSnapAllow{}, fmt.Errorf(`%w: a boundary arc centered across the revolve axis sweeps a spindle torus this evaluator cannot represent`, ErrUnsupported)
			}
			if w.CV-w.Radius > ax.snapTol {
				continue
			}
			lo, hi := math.Min(w.Th0, w.Th1), math.Max(w.Th0, w.Th1)
			if w.Closed {
				return regionSnapAllow{}, fmt.Errorf(`%w: a closed curve touching the revolve axis sweeps a self-touching solid`, ErrDegenerate)
			}
			// The minimum-ρ angle is −π/2 modulo a full turn.
			for th := -math.Pi/2 + 2*math.Pi*math.Floor((lo+math.Pi/2)/(2*math.Pi)); th <= hi+angEps; th += 2 * math.Pi {
				if th > lo+angEps && th < hi-angEps {
					return regionSnapAllow{}, fmt.Errorf(`%w: the boundary touches the revolve axis at an interior point`, ErrDegenerate)
				}
			}
		}
	}
	return snap, nil
}
