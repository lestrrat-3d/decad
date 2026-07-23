package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the mass-property engine of docs/evaluator-design.md §4:
// decad integrating its OWN records. sketch decides topology and
// admissibility; once a region is recorded, its areas and moments are decad's
// job, computed by closed-form boundary integrals (Green's theorem) per
// segment kind — exact for line, circle and arc walks, and ErrUnsupported for
// the free-form kinds.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a [Measurement] of Kind Area (mm²): a computed quantity carries
// its Exactness and Bound (docs/api-design.md §6). The closed forms here are
// exact, so the measurement reads Exact with a zero bound; each boundary
// segment contributes its Green's-theorem integral in walk order, so a hole's
// clockwise walk subtracts without a special case.
//
// A region whose boundary contains a free-form segment kind (ellipse,
// elliptical arc, conic, spline, closed spline, fit spline, NURBS) is
// [ErrUnsupported] (docs/evaluator-design.md §11) — never approximated.
// A malformed or open record is [ErrDegenerate]. A circle radius of the wrong
// kind is [ErrUnitKind], a negative radius is [ErrNegativeMagnitude], and a
// non-finite field or arithmetic result is [ErrNotFinite]. No measurement is
// returned on error.
func (r ProfileRecord) Area() (Measurement, error) {
	ig, err := r.integrals()
	if err != nil {
		return Measurement{}, err
	}
	return Measurement{
		Value:     units.SquareMillimeters(ig.area),
		Exactness: Exact,
		Bound:     units.SquareMillimeters(0),
	}, nil
}

// Centroid returns the recorded region's centroid from its exact first
// moments, as a [VecMeasurement] — a computed coordinate is a measurement
// (docs/api-design.md §6). The Value is PLANE-LOCAL: (u, v, 0) in the
// region's own plane coordinates, millimetres (§5.2), not a world position —
// lift it through the profile's PlaneRecord to place it in space. The closed
// forms are exact, so it reads Exact with a zero bound.
//
// A region whose net area is zero has no centroid and is [ErrDegenerate].
// Record validation and arithmetic errors match [ProfileRecord.Area].
func (r ProfileRecord) Centroid() (VecMeasurement, error) {
	ig, err := r.integrals()
	if err != nil {
		return VecMeasurement{}, err
	}
	if ig.area == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	u, v := ig.mu/ig.area, ig.mv/ig.area
	if !finiteMomentValues(u, v) {
		return VecMeasurement{}, fmt.Errorf(`%w: the region centroid is not finite`, ErrNotFinite)
	}
	return VecMeasurement{
		Value:     r3.NewVec(u, v, 0),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, nil
}

// SecondMoments is a recorded region's second moments of area about the
// plane origin, in the plane's own (u, v): every field is a Measurement of
// Kind SecondMomentOfArea (mm⁴), Exact with a zero bound — the closed forms
// admit no approximation. They are what a revolve's solid centroid is
// computed from (docs/evaluator-design.md §4/§6); to re-reference them to
// another axis, use the parallel-axis theorem with the region's Area and
// Centroid.
type SecondMoments struct {
	// UU is ∫u² dA, VV is ∫v² dA, UV is the mixed ∫uv dA.
	UU Measurement
	UV Measurement
	VV Measurement
}

// SecondMoments returns the region's exact second moments of area about the
// plane origin. The staging matches Area: a free-form boundary kind is
// [ErrUnsupported], and malformed or non-finite records are rejected before a
// measurement is constructed.
func (r ProfileRecord) SecondMoments() (SecondMoments, error) {
	ig, err := r.integrals()
	if err != nil {
		return SecondMoments{}, err
	}
	exact := func(x float64) Measurement {
		return Measurement{Value: units.QuarticMillimeters(x), Exactness: Exact, Bound: units.QuarticMillimeters(0)}
	}
	return SecondMoments{UU: exact(ig.muu), UV: exact(ig.muv), VV: exact(ig.mvv)}, nil
}

// regionIntegrals accumulates the boundary integrals of one region: the net
// signed area, the first moments ∫u dA and ∫v dA, and the second moments
// ∫u² dA, ∫uv dA and ∫v² dA.
type regionIntegrals struct {
	area float64
	mu   float64 // ∫u dA
	mv   float64 // ∫v dA
	muu  float64 // ∫u² dA
	muv  float64 // ∫uv dA
	mvv  float64 // ∫v² dA
}

func finiteMomentValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func (ig regionIntegrals) isFinite() bool {
	return finiteMomentValues(ig.area, ig.mu, ig.mv, ig.muu, ig.muv, ig.mvv)
}

// integrals walks the outer loop and every hole in recorded walk order and
// sums each segment's closed-form contribution. Walk order carries the sign:
// the outer loop is counter-clockwise (positive), holes are clockwise
// (negative), so the sum IS the net region integral.
func (r ProfileRecord) integrals() (regionIntegrals, error) {
	loops := append([]LoopRecord{r.Outer}, r.Holes...)
	var ig regionIntegrals
	for loopIndex, loop := range loops {
		if err := validateMomentLoop(loop); err != nil {
			return regionIntegrals{}, fmt.Errorf(`decad: profile loop %d is invalid: %w`, loopIndex, err)
		}
		var loopIntegrals regionIntegrals
		for segmentIndex, seg := range loop.Segments {
			if err := loopIntegrals.add(seg); err != nil {
				return regionIntegrals{}, err
			}
			if !loopIntegrals.isFinite() {
				return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed at loop %d segment %d`, ErrNotFinite, loopIndex, segmentIndex)
			}
		}
		if loopIndex == 0 && loopIntegrals.area <= 0 {
			return regionIntegrals{}, fmt.Errorf(`%w: the profile outer loop must run counter-clockwise`, ErrDegenerate)
		}
		if loopIndex > 0 && loopIntegrals.area >= 0 {
			return regionIntegrals{}, fmt.Errorf(`%w: profile hole %d must run clockwise`, ErrDegenerate, loopIndex-1)
		}
		ig.area += loopIntegrals.area
		ig.mu += loopIntegrals.mu
		ig.mv += loopIntegrals.mv
		ig.muu += loopIntegrals.muu
		ig.muv += loopIntegrals.muv
		ig.mvv += loopIntegrals.mvv
		if !ig.isFinite() {
			return regionIntegrals{}, fmt.Errorf(`%w: mass-property integration overflowed while combining profile loop %d`, ErrNotFinite, loopIndex)
		}
	}
	if err := validateMomentArrangement(loops); err != nil {
		return regionIntegrals{}, err
	}
	if ig.area <= 0 {
		return regionIntegrals{}, fmt.Errorf(`%w: the recorded region encloses no positive net area`, ErrDegenerate)
	}
	return ig, nil
}

// validateMomentArrangement applies the package's independent line/arc
// boundary audit after each loop's fields, closure and winding have passed.
// Caller-built and decoded records carry no upstream arrangement certificate,
// so crossing, outside or nested holes are malformed input here. Boundary
// tangency remains admissible for mass properties; consumers that require a
// manifold boundary apply their own stricter gate.
func validateMomentArrangement(loops []LoopRecord) error {
	segs, err := buildSegEntries(loops)
	if err != nil {
		return err
	}
	for i := range segs {
		for j := i + 1; j < len(segs); j++ {
			if adjacent(segs[i], segs[j]) {
				continue
			}
			if momentSegmentsCross(segs[i].w, segs[j].w) {
				return fmt.Errorf(`%w: the recorded profile boundaries cross`, ErrDegenerate)
			}
		}
	}
	if !momentNestingValid(segs, len(loops)) {
		return fmt.Errorf(`%w: each profile hole must lie inside the outer loop and outside every other hole`, ErrDegenerate)
	}
	return nil
}

// momentSegmentsCross reports only a proven transversal crossing. Tangencies
// are valid for mass properties, so the shared modify audit's stricter
// crossing-or-contact result cannot be used here.
func momentSegmentsCross(a, b segmentWalk) bool {
	switch {
	case !a.circular && !b.circular:
		return lineLineSegCross(a, b)
	case a.circular && b.circular:
		return momentArcsCross(a, b)
	case a.circular:
		return momentLineArcCross(b, a)
	default:
		return momentLineArcCross(a, b)
	}
}

func momentLineArcCross(line, arc segmentWalk) bool {
	dx, dy := line.endU-line.startU, line.endV-line.startV
	length := math.Hypot(dx, dy)
	if length == 0 {
		return false
	}
	ux, uy := dx/length, dy/length
	fx, fy := line.startU-arc.cU, line.startV-arc.cV
	bb := fx*ux + fy*uy
	cc := fx*fx + fy*fy - arc.radius*arc.radius
	discriminant := bb*bb - cc
	if discriminant < 0 {
		return false
	}
	root := math.Sqrt(discriminant)
	for _, distance := range []float64{-bb + root, -bb - root} {
		u, v := line.startU+distance*ux, line.startV+distance*uy
		if !interior(distance/length) || !momentAngleInterior(arc, u, v) {
			continue
		}
		radialU, radialV := u-arc.cU, v-arc.cV
		residual := dx*radialU + dy*radialV
		scale := math.Max(math.Abs(dx*radialU), math.Abs(dy*radialV))
		if !momentCoordinateJoins(residual, 0, scale) {
			return true
		}
	}
	return false
}

func momentArcsCross(a, b segmentWalk) bool {
	for _, point := range circleCircle(a.cU, a.cV, a.radius, b.cU, b.cV, b.radius) {
		if !momentAngleInterior(a, point[0], point[1]) || !momentAngleInterior(b, point[0], point[1]) {
			continue
		}
		au, av := point[0]-a.cU, point[1]-a.cV
		bu, bv := point[0]-b.cU, point[1]-b.cV
		residual := au*bv - av*bu
		scale := math.Max(math.Abs(au*bv), math.Abs(av*bu))
		if !momentCoordinateJoins(residual, 0, scale) {
			return true
		}
	}
	return false
}

func momentAngleInterior(arc segmentWalk, u, v float64) bool {
	return arc.closed || angleInterior(arc, u, v)
}

// momentNestingValid uses several boundary probes per segment. A probe proven
// outside the outer loop or inside another hole rejects the record. An
// undecidable probe can be a permitted tangency, so it never admits or rejects
// by itself; proven transversal crossings were already rejected above.
func momentNestingValid(segs []segEntry, nLoops int) bool {
	if nLoops <= 1 {
		return true
	}
	bounds := make([][]surveyElem, nLoops)
	probes := make([][][2]float64, nLoops)
	for _, seg := range segs {
		if elem, ok := elemOf(seg.w); ok {
			bounds[seg.loop] = append(bounds[seg.loop], elem)
		}
		probes[seg.loop] = append(probes[seg.loop], momentSegmentProbes(seg.w)...)
	}
	minU, minV, maxU, maxV, ok := sectionBBox(segs)
	if !ok {
		return false
	}
	scale := math.Max(1, math.Max(math.Max(math.Abs(minU), math.Abs(maxU)), math.Max(math.Abs(minV), math.Abs(maxV))))
	tolerance := contactEps * scale

	for hole := 1; hole < nLoops; hole++ {
		provenInside := false
		for _, probe := range probes[hole] {
			inside, decided := loopContains(bounds[0], probe[0], probe[1], tolerance)
			if decided && !inside {
				return false
			}
			provenInside = provenInside || decided && inside
		}
		if !provenInside {
			return false
		}
	}
	for a := 1; a < nLoops; a++ {
		for b := a + 1; b < nLoops; b++ {
			aOutsideB := false
			for _, probe := range probes[a] {
				inside, decided := loopContains(bounds[b], probe[0], probe[1], tolerance)
				if decided && inside {
					return false
				}
				aOutsideB = aOutsideB || decided && !inside
			}
			bOutsideA := false
			for _, probe := range probes[b] {
				inside, decided := loopContains(bounds[a], probe[0], probe[1], tolerance)
				if decided && inside {
					return false
				}
				bOutsideA = bOutsideA || decided && !inside
			}
			if !aOutsideB || !bOutsideA {
				return false
			}
		}
	}
	return true
}

func momentSegmentProbes(w segmentWalk) [][2]float64 {
	midU, midV := (w.startU+w.endU)/2, (w.startV+w.endV)/2
	if w.circular {
		midAngle := (w.th0 + w.th1) / 2
		midU = w.cU + w.radius*math.Cos(midAngle)
		midV = w.cV + w.radius*math.Sin(midAngle)
	}
	return [][2]float64{{w.startU, w.startV}, {midU, midV}}
}

type momentWalk struct {
	segmentWalk
	uScale float64
	vScale float64
}

// validateMomentLoop checks the part of ProfileRecord's structural contract
// the closed-form integrator relies on: every supported segment has finite,
// usable walk geometry and the directed walks join into one closed boundary.
func validateMomentLoop(loop LoopRecord) error {
	if len(loop.Segments) == 0 {
		return fmt.Errorf(`%w: a recorded loop holds no segments`, ErrDegenerate)
	}
	walks := make([]momentWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		walk, err := validateMomentSegment(seg)
		if err != nil {
			return fmt.Errorf(`segment %d: %w`, i, err)
		}
		walks[i] = walk
	}
	if len(walks) == 1 && walks[0].closed {
		if momentEndpointsJoin(walks[0], walks[0]) {
			return nil
		}
		return fmt.Errorf(
			`%w: the closed segment ends at (%g, %g), not its start (%g, %g)`,
			ErrDegenerate, walks[0].endU, walks[0].endV, walks[0].startU, walks[0].startV,
		)
	}
	for i, walk := range walks {
		if walk.closed {
			return fmt.Errorf(`%w: a closed segment cannot share a loop with another segment`, ErrDegenerate)
		}
		next := walks[(i+1)%len(walks)]
		if !momentEndpointsJoin(walk, next) {
			return fmt.Errorf(
				`%w: segment %d ends at (%g, %g), not segment %d's start (%g, %g)`,
				ErrDegenerate, i, walk.endU, walk.endV, (i+1)%len(walks), next.startU, next.startV,
			)
		}
	}
	return nil
}

func validateMomentSegment(seg CurveSegment) (momentWalk, error) {
	seg, err := normalizeSegment(seg)
	if err != nil {
		return momentWalk{}, err
	}
	if seg == nil {
		return momentWalk{}, errNilSegment
	}
	switch seg := seg.(type) {
	case LineSeg:
		if !finiteMomentValues(seg.Start.U, seg.Start.V, seg.End.U, seg.End.V, seg.TStart, seg.TEnd) {
			return momentWalk{}, fmt.Errorf(`%w: a line segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(seg.TStart, seg.TEnd); err != nil {
			return momentWalk{}, err
		}
	case CircleSeg:
		if _, err := magnitudeIn(seg.Radius, units.Length, units.Millimeter, "a circle segment's radius"); err != nil {
			return momentWalk{}, err
		}
		if !finiteMomentValues(seg.Center.U, seg.Center.V, seg.TStart, seg.TEnd) {
			return momentWalk{}, fmt.Errorf(`%w: a circle segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(seg.TStart, seg.TEnd); err != nil {
			return momentWalk{}, err
		}
	case ArcSeg:
		if !finiteMomentValues(
			seg.Center.U, seg.Center.V,
			seg.Start.U, seg.Start.V,
			seg.End.U, seg.End.V,
			seg.TStart, seg.TEnd,
		) {
			return momentWalk{}, fmt.Errorf(`%w: an arc segment field is not finite`, ErrNotFinite)
		}
		if err := validateMomentRange(seg.TStart, seg.TEnd); err != nil {
			return momentWalk{}, err
		}
		startRadius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		endRadius := math.Hypot(seg.End.U-seg.Center.U, seg.End.V-seg.Center.V)
		sourceScale := math.Max(
			math.Max(math.Abs(seg.Center.U), math.Abs(seg.Center.V)),
			math.Max(
				math.Max(math.Abs(seg.Start.U), math.Abs(seg.Start.V)),
				math.Max(math.Abs(seg.End.U), math.Abs(seg.End.V)),
			),
		)
		if !momentCoordinateJoins(startRadius, endRadius, sourceScale) {
			return momentWalk{}, fmt.Errorf(
				`%w: an arc segment's pinned start and end radii differ (%g and %g)`,
				ErrDegenerate, startRadius, endRadius,
			)
		}
	default:
		return momentWalk{}, fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, seg)
	}

	walk, err := walkOf(seg)
	if err != nil {
		return momentWalk{}, err
	}
	if !finiteMomentValues(
		walk.startU, walk.startV, walk.endU, walk.endV,
		walk.tanInU, walk.tanInV, walk.tanOutU, walk.tanOutV,
		walk.length, walk.cU, walk.cV, walk.radius, walk.th0, walk.th1,
	) {
		return momentWalk{}, fmt.Errorf(`%w: a segment's derived walk is not finite`, ErrNotFinite)
	}
	if walk.circular && walk.radius < 0 {
		return momentWalk{}, fmt.Errorf(`%w: a circular segment radius must be non-negative`, ErrNegativeMagnitude)
	}
	if walk.length <= 0 {
		return momentWalk{}, fmt.Errorf(`%w: a zero-length segment contributes no boundary`, ErrDegenerate)
	}
	out := momentWalk{segmentWalk: walk}
	switch seg := seg.(type) {
	case LineSeg:
		out.uScale = math.Max(math.Abs(seg.Start.U), math.Abs(seg.End.U))
		out.vScale = math.Max(math.Abs(seg.Start.V), math.Abs(seg.End.V))
	case CircleSeg:
		out.uScale = math.Max(math.Abs(seg.Center.U), math.Abs(walk.radius))
		out.vScale = math.Max(math.Abs(seg.Center.V), math.Abs(walk.radius))
	case ArcSeg:
		out.uScale = math.Max(
			math.Abs(walk.radius),
			math.Max(math.Abs(seg.Center.U), math.Max(math.Abs(seg.Start.U), math.Abs(seg.End.U))),
		)
		out.vScale = math.Max(
			math.Abs(walk.radius),
			math.Max(math.Abs(seg.Center.V), math.Max(math.Abs(seg.Start.V), math.Abs(seg.End.V))),
		)
	}
	return out, nil
}

func validateMomentRange(start, end float64) error {
	if start < 0 || start > 1 || end < 0 || end > 1 {
		return fmt.Errorf(`%w: a segment range must stay within [0, 1]`, ErrDegenerate)
	}
	if start == end {
		return fmt.Errorf(`%w: a zero-length segment range contributes no boundary`, ErrDegenerate)
	}
	return nil
}

// momentEndpointsJoin allows only the ULP-scale difference introduced when
// the same certified junction is re-evaluated through two segment formulas.
// It is intentionally unrelated to the model's geometric scale tolerance.
func momentEndpointsJoin(a, b momentWalk) bool {
	return momentCoordinateJoins(a.endU, b.startU, math.Max(a.uScale, b.uScale)) &&
		momentCoordinateJoins(a.endV, b.startV, math.Max(a.vScale, b.vScale))
}

func momentCoordinateJoins(a, b, sourceScale float64) bool {
	scale := math.Max(1, math.Max(sourceScale, math.Max(math.Abs(a), math.Abs(b))))
	ulp := scale - math.Nextafter(scale, 0)
	return math.Abs(a-b) <= 64*ulp
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction (TStart→TEnd, which TStart > TEnd runs
// against the curve's natural sense — the sign falls out of the integral
// limits, no special case).
func (ig *regionIntegrals) add(seg CurveSegment) error {
	// The codec accepts pointer variants and normalizes them to values
	// (record.go); the integrals read segments on the same terms, so a
	// *LineSeg integrates exactly like its value and a nil pointer is
	// rejected rather than misread as an unsupported kind.
	seg, err := normalizeSegment(seg)
	if err != nil {
		return err
	}
	switch seg := seg.(type) {
	case LineSeg:
		ig.addLine(seg)
		return nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		// The record carries the walk twice — the range order, and CCW —
		// and the two must agree (a reversed walk swaps the range AND flips
		// CCW, seam §2). A record that contradicts itself is malformed, not
		// a judgement call.
		if seg.CCW != (seg.TStart < seg.TEnd) {
			return fmt.Errorf(`%w: a circle segment's CCW flag contradicts its range order`, ErrDegenerate)
		}
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		ig.addCircular(seg.Center, r, 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd)
		return nil
	case ArcSeg:
		// geom.Arc's derived readings, computed on the record's own pinned
		// points: radius from Center→Start, angles about the center, the
		// sweep CCW-wrapped into (0, 2π].
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		ig.addCircular(seg.Center, radius, a0+seg.TStart*sweep, a0+seg.TEnd*sweep)
		return nil
	default:
		return fmt.Errorf(`%w: this evaluator computes mass properties over line, arc and circle profile segments only; the profile has a %T segment`, ErrUnsupported, seg)
	}
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(seg LineSeg) {
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
	// A     = ½ ∮ (u dv − v du)
	// ∫u dA = ½ ∮ u² dv
	// ∫v dA = −½ ∮ v² du
	ig.area += 0.5 * (u0*v1 - u1*v0)
	ig.mu += (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	ig.mv += -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6

	// ∫u² dA = ⅓ ∮ u³ dv;  ∫v² dA = −⅓ ∮ v³ du — the cubic sums are the
	// exact ∫₀¹ of the lerp cubed.
	ig.muu += (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	ig.mvv += -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12

	// ∫uv dA = ½ ∮ u²v dv: expand u(t)² v(t) and integrate the polynomial
	// exactly, term by term.
	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	ig.muv += 0.5 * dv * intU2V
}

// addCircular accumulates a circular path about center c with radius r, from
// angle th0 to th1 in the walk direction (th1 < th0 walks clockwise). The
// antiderivatives are exact, so a whole period, a fragment, and a reversed
// walk are all the same formula.
func (ig *regionIntegrals) addCircular(c Point2, r, th0, th1 float64) {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0

	// A = ½ ∫ (u v′ − v u′) dθ = ½ [r²·θ + c_u·r·sin θ + c_v·r·cos θ]
	ig.area += 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))

	// ∫u dA = ½ ∮ u² dv, with u = c_u + r cos θ, dv = r cos θ dθ:
	//   ½ r [c_u²·sin θ + 2 c_u r (θ/2 + sin 2θ/4) + r²(sin θ − sin³θ/3)]
	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	ig.mu += 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	// ∫v dA = −½ ∮ v² du, with v = c_v + r sin θ, du = −r sin θ dθ:
	//   ½ r [c_v²·(−cos θ)′... i.e. ½ r ∫ v² sin θ dθ]
	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	ig.mv += 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)

	// The quartic antiderivatives for the second moments:
	//   ∫cos⁴θ dθ = 3θ/8 + sin 2θ/4 + sin 4θ/32
	//   ∫sin⁴θ dθ = 3θ/8 − sin 2θ/4 + sin 4θ/32
	intCos4 := 3*dth/8 + (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	intSin4 := 3*dth/8 - (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32

	// ∫u² dA = ⅓ ∮ u³ dv, dv = r cos θ dθ, u³ expanded about the center.
	ig.muu += r / 3 * (c.U*c.U*c.U*intCos + 3*c.U*c.U*r*intCos2 + 3*c.U*r*r*intCos3 + r*r*r*intCos4)

	// ∫v² dA = −⅓ ∮ v³ du, du = −r sin θ dθ, v³ expanded about the center.
	ig.mvv += r / 3 * (c.V*c.V*c.V*intSin + 3*c.V*c.V*r*intSin2 + 3*c.V*r*r*intSin3 + r*r*r*intSin4)

	// ∫uv dA = ½ ∮ u²v dv: u²v = c_v·u² + r sin θ·u², with
	//   ∫sin θ cos θ dθ = sin²θ/2, ∫sin θ cos²θ dθ = −cos³θ/3,
	//   ∫sin θ cos³θ dθ = −cos⁴θ/4.
	intSC := (sin1*sin1 - sin0*sin0) / 2
	intSC2 := (cos0*cos0*cos0 - cos1*cos1*cos1) / 3
	intSC3 := (cos0*cos0*cos0*cos0 - cos1*cos1*cos1*cos1) / 4
	ig.muv += 0.5 * r * (c.V*(c.U*c.U*intCos+2*c.U*r*intCos2+r*r*intCos3) +
		r*(c.U*c.U*intSC+2*c.U*r*intSC2+r*r*intSC3))
}

// lerp2 returns the point at parameter t on the segment start→end.
func lerp2(start, end Point2, t float64) (float64, float64) {
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
