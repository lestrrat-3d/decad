package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/sketch/geom"
)

// loftFitCircleBlend proposes a numerical fillet at one CircleSeg/FitSplineSeg
// corner. It retains the fit points: the caller changes only the returned
// segment ranges. The residual checks below can reject a bad proposal; they
// do not certify either tangent contact or the rewritten section's closure.
func loftFitCircleBlend(segments []curveSegment, walks []survey2d.SideWalk, corner int, radius float64) (*cornerBlend, float64, float64, error) {
	if len(segments) != len(walks) || len(segments) < 2 || corner < 0 || corner >= len(segments) ||
		!finiteLoftFitValue(radius) || radius <= 0 {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle fillet has invalid corner inputs`, ErrUnsupported)
	}
	prev := (corner + len(segments) - 1) % len(segments)
	fitArrives := false
	var fit fitSplineSeg
	var circle circleSeg
	var fitWalk, circleWalk survey2d.SideWalk
	switch a := segments[prev].(type) {
	case fitSplineSeg:
		c, ok := segments[corner].(circleSeg)
		if !ok {
			return nil, 0, 0, fmt.Errorf(`%w: the loft corner is not a fit-spline/circle pair`, ErrUnsupported)
		}
		fit, circle, fitWalk, circleWalk, fitArrives = a, c, walks[prev], walks[corner], true
	case circleSeg:
		f, ok := segments[corner].(fitSplineSeg)
		if !ok {
			return nil, 0, 0, fmt.Errorf(`%w: the loft corner is not a circle/fit-spline pair`, ErrUnsupported)
		}
		fit, circle, fitWalk, circleWalk = f, a, walks[corner], walks[prev]
	default:
		return nil, 0, 0, fmt.Errorf(`%w: the loft corner is not a fit-spline/circle pair`, ErrUnsupported)
	}
	if fitWalk.Kind != survey2d.WalkFreeform || circleWalk.Kind != survey2d.WalkCircular ||
		len(fit.Fit) < 2 || !finiteLoftFitValue(circleWalk.Radius) || circleWalk.Radius <= 0 {
		return nil, 0, 0, fmt.Errorf(`%w: the loft corner has no usable fit-spline and circular walks`, ErrUnsupported)
	}
	for _, point := range fit.Fit {
		if !finiteLoftFitValue(point.U, point.V) {
			return nil, 0, 0, fmt.Errorf(`%w: the loft fit spline has a non-finite fit point`, ErrUnsupported)
		}
	}
	interp, err := geom.NewFitInterpolant(splinebezier.FitCoords(fit.Fit))
	if err != nil {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit spline cannot be evaluated: %v`, ErrUnsupported, err)
	}

	// Use the existing analytic construction only to choose the nearby branch.
	// Passing the free-form walk itself to offset2d.Fillet would incorrectly
	// classify it as a line through its start point.
	fitCorner := fit.TStart
	circleCorner := circle.TEnd
	if fitArrives {
		fitCorner, circleCorner = fit.TEnd, circle.TStart
	}
	px, py := interp.Eval(fitCorner)
	dx, dy := interp.EvalDeriv(fitCorner)
	if !finiteLoftFitValue(px, py, dx, dy) {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit spline has no finite corner tangent`, ErrUnsupported)
	}
	travel := math.Copysign(1, fit.TEnd-fit.TStart)
	dx, dy = travel*dx, travel*dy
	length := math.Hypot(dx, dy)
	if !finiteLoftFitValue(length) || length == 0 {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit spline has no corner direction`, ErrUnsupported)
	}
	dx, dy = dx/length, dy/length
	startU, startV, endU, endV := px, py, px+dx, py+dy
	if fitArrives {
		startU, startV, endU, endV = px-dx, py-dy, px, py
	}
	surrogate := survey2d.SideWalk{SegmentWalk: survey2d.SegmentWalk{
		StartU: startU, StartV: startV, EndU: endU, EndV: endV,
		TanInU: dx, TanInV: dy, TanOutU: dx, TanOutV: dy,
		Length: 1, Kind: survey2d.WalkLine,
	}}
	arrive, leave := circleWalk, surrogate
	if fitArrives {
		arrive, leave = surrogate, circleWalk
	}
	seed, err := offset2d.Fillet([]survey2d.SideWalk{arrive, leave}, 1, radius, filletTol)
	if err != nil {
		return nil, 0, 0, err
	}
	seedArc, ok := seed.Connector.(arcSeg)
	if !ok {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle fillet has no seed arc`, ErrUnsupported)
	}
	ax, ay := arrive.TanOutU, arrive.TanOutV
	bx, by := leave.TanInU, leave.TanInV
	cross := ax*by - ay*bx
	if !finiteLoftFitValue(cross) || math.Abs(cross) <= filletTol {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle corner has no distinct tangent directions`, ErrUnsupported)
	}
	offsetSign := math.Copysign(1, cross)
	insideSign := math.Copysign(1, circle.TEnd-circle.TStart)
	offsetRadius := circleWalk.Radius - offsetSign*insideSign*radius
	if !finiteLoftFitValue(offsetRadius) || offsetRadius <= filletTol {
		return nil, 0, 0, fmt.Errorf(`%w: the loft root circle cannot carry that fillet radius`, ErrUnsupported)
	}

	otherFit := fit.TEnd
	if fitArrives {
		otherFit = fit.TStart
	}
	trial := func(u float64) (float64, sectionrecord.Point2, sectionrecord.Point2, float64, float64, bool) {
		t := fitCorner + u*(otherFit-fitCorner)
		x, y := interp.Eval(t)
		du, dv := interp.EvalDeriv(t)
		du, dv = travel*du, travel*dv
		d := math.Hypot(du, dv)
		if !finiteLoftFitValue(t, x, y, du, dv, d) || d == 0 {
			return 0, sectionrecord.Point2{}, sectionrecord.Point2{}, 0, 0, false
		}
		ox := x - offsetSign*radius*dv/d
		oy := y + offsetSign*radius*du/d
		q := math.Hypot(ox-circleWalk.CU, oy-circleWalk.CV)
		if !finiteLoftFitValue(ox, oy, q) || q == 0 {
			return 0, sectionrecord.Point2{}, sectionrecord.Point2{}, 0, 0, false
		}
		return q - offsetRadius, sectionrecord.Point2{U: x, V: y}, sectionrecord.Point2{U: ox, V: oy}, du / d, dv / d, true
	}

	// Logarithmic stations find cutbacks close to the corner. Uniform stations
	// then inspect the rest of the recorded fragment for a competing crossing.
	previousU, previousF := 0.0, 0.0
	previousF, _, _, _, _, ok = trial(0)
	if !ok {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle offset cannot be evaluated at its corner`, ErrUnsupported)
	}
	if previousF == 0 {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle contact is at the original corner`, ErrUnsupported)
	}
	rootLo, rootHi, roots := 0.0, 0.0, 0
	inspect := func(u float64) bool {
		f, _, _, _, _, valid := trial(u)
		if !valid {
			return false
		}
		if f == 0 && previousF != 0 {
			rootLo, rootHi, roots = u, u, roots+1
		} else if f != 0 && previousF != 0 && math.Signbit(f) != math.Signbit(previousF) {
			rootLo, rootHi, roots = previousU, u, roots+1
		}
		previousU, previousF = u, f
		return true
	}
	for exponent := 24; exponent >= 9; exponent-- {
		if !inspect(math.Ldexp(1, -exponent)) {
			return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle offset has a non-finite search interval`, ErrUnsupported)
		}
	}
	for i := 1; i <= 256; i++ {
		if !inspect(float64(i) / 256) {
			return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle offset has a non-finite search interval`, ErrUnsupported)
		}
	}
	if roots != 1 || rootLo > rootHi {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle offset has %d observed contact roots`, ErrUnsupported, roots)
	}
	loF, _, _, _, _, _ := trial(rootLo)
	for i := 0; i < 54; i++ {
		mid := rootLo + (rootHi-rootLo)/2
		if mid == rootLo || mid == rootHi {
			break
		}
		f, _, _, _, _, valid := trial(mid)
		if !valid {
			return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle contact cannot be refined`, ErrUnsupported)
		}
		if f == 0 {
			rootLo, rootHi = mid, mid
			break
		}
		if math.Signbit(f) == math.Signbit(loF) {
			rootLo, loF = mid, f
		} else {
			rootHi = mid
		}
	}
	u := rootLo + (rootHi-rootLo)/2
	fitT := fitCorner + u*(otherFit-fitCorner)
	residual, fitFoot, center, ftx, fty, valid := trial(u)
	if !valid || !betweenLoftFitRange(fitT, fit.TStart, fit.TEnd) ||
		math.Abs(residual) > 1e-8*math.Max(1, radius) ||
		math.Hypot(center.U-seedArc.Center.U, center.V-seedArc.Center.V) > 8*radius {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle contact is outside its interior or seed branch`, ErrUnsupported)
	}
	centerDistance := math.Hypot(center.U-circleWalk.CU, center.V-circleWalk.CV)
	circleFoot := sectionrecord.Point2{
		U: circleWalk.CU + circleWalk.Radius*(center.U-circleWalk.CU)/centerDistance,
		V: circleWalk.CV + circleWalk.Radius*(center.V-circleWalk.CV)/centerDistance,
	}
	circleT, ok := loftCircleContactParameter(circleFoot, circle, circleCorner)
	if !ok {
		return nil, 0, 0, fmt.Errorf(`%w: the loft circle contact is outside its recorded fragment`, ErrUnsupported)
	}
	fa, fb := circleFoot, fitFoot
	if fitArrives {
		fa, fb = fitFoot, circleFoot
	}
	arcCCW := (-(fa.V-center.V)*ax + (fa.U-center.U)*ay) > 0
	arcSign := -1.0
	if arcCCW {
		arcSign = 1
	}
	arcTangent := func(p sectionrecord.Point2) (float64, float64) {
		x, y := arcSign*-(p.V-center.V), arcSign*(p.U-center.U)
		d := math.Hypot(x, y)
		return x / d, y / d
	}
	atFitU, atFitV := arcTangent(fitFoot)
	if atFitU*ftx+atFitV*fty < 1-1e-6 {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle arc misses the fit tangent`, ErrUnsupported)
	}
	circleSign := math.Copysign(1, circle.TEnd-circle.TStart)
	circleU := circleSign * -(circleFoot.V - circleWalk.CV) / circleWalk.Radius
	circleV := circleSign * (circleFoot.U - circleWalk.CU) / circleWalk.Radius
	atCircleU, atCircleV := arcTangent(circleFoot)
	if atCircleU*circleU+atCircleV*circleV < 1-1e-6 ||
		!finiteLoftFitValue(fa.U, fa.V, fb.U, fb.V, center.U, center.V) {
		return nil, 0, 0, fmt.Errorf(`%w: the loft fit-circle arc misses the circle tangent`, ErrUnsupported)
	}
	fitCut := math.Hypot(fitFoot.U-px, fitFoot.V-py)
	circleCut := circleWalk.Radius * 2 * math.Pi * math.Abs(circleT-circleCorner)
	blend := &cornerBlend{FA: fa, FB: fb,
		Connector: offset2d.ArcSegment(center, fa, fb, arcCCW)}
	if fitArrives {
		blend.CutbackA, blend.CutbackB = fitCut, circleCut
	} else {
		blend.CutbackA, blend.CutbackB = circleCut, fitCut
	}
	return blend, fitT, circleT, nil
}

func finiteLoftFitValue(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func betweenLoftFitRange(value, a, b float64) bool {
	return value > math.Min(a, b) && value < math.Max(a, b)
}

func loftCircleContactParameter(point sectionrecord.Point2, circle circleSeg, corner float64) (float64, bool) {
	angle := math.Atan2(point.V-circle.Center.V, point.U-circle.Center.U) / (2 * math.Pi)
	value, count := 0.0, 0
	for shift := -1; shift <= 2; shift++ {
		candidate := angle + float64(shift)
		if !betweenLoftFitRange(candidate, circle.TStart, circle.TEnd) {
			continue
		}
		if count == 0 || math.Abs(candidate-corner) < math.Abs(value-corner) {
			value = candidate
		}
		count++
	}
	return value, count == 1
}
