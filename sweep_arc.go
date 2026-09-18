package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/r3"
)

// This file owns the one-arc Sweep reduction. ArcThrough's three recorded
// points derive one exact rational circle. Its float axis and angle are held
// beside rational enclosures of the exact values they publish.

type sweepArcGeometry struct {
	line axisLine2
	phi  float64
	den  angleDenotation
}

type sweepRatVec [3]*big.Rat

type sweepArcRecord struct {
	center       sweepRatVec
	radiusStart  sweepRatVec
	radiusMiddle sweepRatVec
	radiusEnd    sweepRatVec
	axis         sweepRatVec
}

func evalArcSweepContext(
	ctx context.Context,
	d *Document,
	ref producerID,
	profile ProfileRecord,
	plane PlaneRecord,
	frame r3.Frame,
	path *Path,
	pathRecord pathSegmentRecord,
	work *freeformWork,
) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	geometry, err := deriveSweepArc(pathRecord, plane)
	if err != nil {
		return nil, err
	}
	ax, side, err := resolveAxisSide(ctx, profile, geometry.line, work)
	if err != nil {
		return nil, err
	}

	phi0, phi1 := 0.0, geometry.phi
	den := sweepDenotation{phi0: zeroAngleDenotation(), phi1: geometry.den}
	reverseCaps := false
	if side < 0 {
		phi0, phi1 = -phi1, -phi0
		den.phi0, den.phi1 = den.phi1.neg(), den.phi0.neg()
		reverseCaps = true
	}
	revolve := revolvePayload{
		profile: profile,
		frame:   frame,
		ax:      ax,
		phi0:    phi0,
		phi1:    phi1,
		den:     den,
		xform:   r3.Identity(),
	}
	body, err := evalRevolveContextWork(ctx, d, ref, revolve, work)
	if err != nil {
		return nil, err
	}

	// Verify's Sweep diameter arm reads this zero-height start-section witness.
	// Every one of its points lies on the real start cap, so its point-set
	// diameter is a sound lower bound on the swept body's diameter.
	witness := prismPayload{
		profile: profile,
		frame:   frame,
		xform:   r3.Identity(),
	}
	payload := sweepPayload{
		prism:          witness,
		revolve:        revolve,
		arc:            true,
		reverseArcCaps: reverseCaps,
		path:           path,
	}
	finishArcSweepBody(body, payload)
	return body, nil
}

func deriveSweepArc(pathRecord pathSegmentRecord, plane PlaneRecord) (sweepArcGeometry, error) {
	start := pathRecord.start
	normal := dvCross(dyVec(plane.U), dyVec(plane.V))
	relStart := dvSub(dyVec(start), dyVec(plane.Origin))
	if !dvDot(relStart, normal).isZero() {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep path must start in the profile plane`, ErrDegenerate)
	}

	if pathRecord.arc == nil {
		return sweepArcGeometry{}, fmt.Errorf(`%w: a sweep arc path record holds no circular carrier`, ErrDegenerate)
	}
	record := *pathRecord.arc
	centerRat := record.center
	r0, rm, r1 := record.radiusStart, record.radiusMiddle, record.radiusEnd
	axisRat := record.axis
	tangent := sweepRatCross(axisRat, r0)
	normalRat := sweepRatFromDyadic(normal)
	if !sweepRatIsZero(sweepRatCross(tangent, normalRat)) || sweepRatDot(tangent, normalRat).Sign() <= 0 {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep path's initial tangent must follow the profile plane's positive normal`, ErrDegenerate)
	}
	if sweepRatDot(sweepRatSub(centerRat, sweepRatVecOf(plane.Origin)), normalRat).Sign() != 0 {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep arc's derived axis does not lie in the profile plane`, ErrDegenerate)
	}

	// The carrier was solved from all three points. This equality catches an
	// arithmetic regression before a derived angle reaches the evaluator.
	radiusSquared := sweepRatDot(r0, r0)
	if sweepRatDot(rm, rm).Cmp(radiusSquared) != 0 || sweepRatDot(r1, r1).Cmp(radiusSquared) != 0 {
		return sweepArcGeometry{}, fmt.Errorf(`%w: the sweep arc points do not share one exact carrier`, ErrUnsupported)
	}
	line, err := sweepArcAxisLine(centerRat, axisRat, plane)
	if err != nil {
		return sweepArcGeometry{}, err
	}

	return sweepArcGeometry{
		line: line,
		phi:  pathRecord.arcPhi,
		den:  pathRecord.arcAngle,
	}, nil
}

func recordSweepArc(start, through, end r3.Vec) (sweepArcRecord, error) {
	p0, pm, p1 := sweepRatVecOf(start), sweepRatVecOf(through), sweepRatVecOf(end)
	a, b := sweepRatSub(pm, p0), sweepRatSub(p1, p0)
	aa, ab, bb := sweepRatDot(a, a), sweepRatDot(a, b), sweepRatDot(b, b)
	det := new(big.Rat).Sub(
		new(big.Rat).Mul(aa, bb),
		new(big.Rat).Mul(ab, ab),
	)
	if det.Sign() == 0 {
		return sweepArcRecord{}, fmt.Errorf(`%w: a sweep arc requires three non-collinear points`, ErrDegenerate)
	}
	twoDet := new(big.Rat).Mul(det, big.NewRat(2, 1))
	alpha := new(big.Rat).Quo(
		new(big.Rat).Mul(bb, new(big.Rat).Sub(aa, ab)),
		twoDet,
	)
	beta := new(big.Rat).Quo(
		new(big.Rat).Mul(aa, new(big.Rat).Sub(bb, ab)),
		twoDet,
	)
	centerRat := sweepRatAdd(p0, sweepRatScale(a, alpha), sweepRatScale(b, beta))
	r0 := sweepRatSub(p0, centerRat)
	rm := sweepRatSub(pm, centerRat)
	r1 := sweepRatSub(p1, centerRat)
	axisRat := sweepRatCross(a, b)
	return sweepArcRecord{
		center:       centerRat,
		radiusStart:  r0,
		radiusMiddle: rm,
		radiusEnd:    r1,
		axis:         axisRat,
	}, nil
}

func sweepArcAxisLine(center, axis sweepRatVec, plane PlaneRecord) (axisLine2, error) {
	origin := sweepRatVecOf(plane.Origin)
	u, v := sweepRatVecOf(plane.U), sweepRatVecOf(plane.V)
	rel := sweepRatSub(center, origin)
	aUExact, aVExact := sweepRatDot(rel, u), sweepRatDot(rel, v)
	dURaw, dVRaw := sweepRatDot(axis, u), sweepRatDot(axis, v)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(dURaw, dURaw),
		new(big.Rat).Mul(dVRaw, dVRaw),
	)
	if lengthSquared.Sign() == 0 {
		return axisLine2{}, fmt.Errorf(`%w: the sweep arc has no axis direction in the profile plane`, ErrDegenerate)
	}

	aU, aUBound, ok := sweepRatHeld(aUExact)
	if !ok {
		return axisLine2{}, fmt.Errorf(`%w: the sweep arc's axis anchor is outside the representable range`, ErrUnsupported)
	}
	aV, aVBound, ok := sweepRatHeld(aVExact)
	if !ok {
		return axisLine2{}, fmt.Errorf(`%w: the sweep arc's axis anchor is outside the representable range`, ErrUnsupported)
	}
	dU, dV, ok := sweepNormalizedRat2(dURaw, dVRaw, lengthSquared)
	if !ok {
		return axisLine2{}, fmt.Errorf(`%w: the sweep arc's axis direction is outside the representable range`, ErrUnsupported)
	}
	dUBound, dVBound := axisDirectionSqrtBracket(dURaw, dVRaw, dU, dV)
	if !finiteAxisValues(aU, aV, aUBound, aVBound, dU, dV, dUBound, dVBound) {
		return axisLine2{}, fmt.Errorf(`%w: the sweep arc's axis has no finite publication bound`, ErrUnsupported)
	}
	return axisLine2{
		aU: aU, aV: aV,
		aUBound: aUBound, aVBound: aVBound,
		dU: dU, dV: dV,
		dUBound: dUBound, dVBound: dVBound,
	}, nil
}

func sweepArcAngle(r0, r1, axis sweepRatVec) (float64, angleDenotation, error) {
	dot := sweepRatDot(r0, r1)
	cross := sweepRatCross(r0, r1)
	crossSquared := sweepRatDot(cross, cross)
	orientation := sweepRatDot(axis, cross).Sign()
	if dot.Sign() == 0 {
		if orientation > 0 {
			return math.Pi / 2, angleDenotation{rad: new(big.Rat), turn: big.NewRat(1, 4)}, nil
		}
		return 3 * math.Pi / 2, angleDenotation{rad: new(big.Rat), turn: big.NewRat(3, 4)}, nil
	}
	if orientation == 0 {
		if dot.Sign() >= 0 {
			return 0, angleDenotation{}, fmt.Errorf(`%w: the sweep arc closes without a directed span`, ErrDegenerate)
		}
		return math.Pi, angleDenotation{rad: new(big.Rat), turn: big.NewRat(1, 2)}, nil
	}
	sinMagnitude, ok := intervalSqrt(pointInterval(crossSquared))
	if !ok || sinMagnitude.lo.Sign() <= 0 {
		return 0, angleDenotation{}, fmt.Errorf(`%w: the sweep arc angle has no finite enclosure`, ErrUnsupported)
	}
	positive := sweepPositiveAtan2Span(sinMagnitude, dot)
	span := positive
	if orientation < 0 {
		span = intervalSub(twoPiInterval(), positive)
	}
	phiRat := new(big.Rat).Quo(new(big.Rat).Add(span.lo, span.hi), big.NewRat(2, 1))
	phi, _, ok := sweepRatHeld(phiRat)
	if !ok || phi <= 0 || phi >= 2*math.Pi {
		return 0, angleDenotation{}, fmt.Errorf(`%w: the sweep arc angle is outside the representable range`, ErrUnsupported)
	}
	return phi, angleDenotation{span: &span}, nil
}

func sweepPositiveAtan2Span(y ratInterval, x *big.Rat) ratInterval {
	if x.Sign() >= 0 {
		return interval(atan2Interval(y.lo, x, false).lo, atan2Interval(y.hi, x, false).hi)
	}
	return interval(atan2Interval(y.hi, x, false).lo, atan2Interval(y.lo, x, false).hi)
}

func sweepRatHeld(value *big.Rat) (float64, float64, bool) {
	held, _ := value.Float64()
	if math.IsNaN(held) || math.IsInf(held, 0) {
		return 0, 0, false
	}
	bound := rationalFloatError(value, held)
	return held, bound, !math.IsNaN(bound) && !math.IsInf(bound, 0)
}

func sweepNormalizedRat2(u, v, lengthSquared *big.Rat) (float64, float64, bool) {
	const precision = 256
	length := new(big.Float).SetPrec(precision).SetRat(lengthSquared)
	length.Sqrt(length)
	if length.Sign() == 0 {
		return 0, 0, false
	}
	component := func(value *big.Rat) float64 {
		f := new(big.Float).SetPrec(precision).SetRat(value)
		f.Quo(f, length)
		held, _ := f.Float64()
		return held
	}
	heldU, heldV := component(u), component(v)
	if !finiteAxisValues(heldU, heldV) {
		return 0, 0, false
	}
	return heldU, heldV, true
}

func sweepRatVecOf(v r3.Vec) sweepRatVec {
	return sweepRatVec{floatRat(v.X), floatRat(v.Y), floatRat(v.Z)}
}

func sweepRatFromDyadic(v dyV3) sweepRatVec {
	return sweepRatVec{v[0].rat(), v[1].rat(), v[2].rat()}
}

func sweepRatAdd(vectors ...sweepRatVec) sweepRatVec {
	out := sweepRatVec{new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, vector := range vectors {
		for i := range out {
			out[i].Add(out[i], vector[i])
		}
	}
	return out
}

func sweepRatSub(a, b sweepRatVec) sweepRatVec {
	return sweepRatVec{
		new(big.Rat).Sub(a[0], b[0]),
		new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func sweepRatScale(v sweepRatVec, scale *big.Rat) sweepRatVec {
	return sweepRatVec{
		new(big.Rat).Mul(v[0], scale),
		new(big.Rat).Mul(v[1], scale),
		new(big.Rat).Mul(v[2], scale),
	}
}

func sweepRatDot(a, b sweepRatVec) *big.Rat {
	return ratAdd(
		new(big.Rat).Mul(a[0], b[0]),
		new(big.Rat).Mul(a[1], b[1]),
		new(big.Rat).Mul(a[2], b[2]),
	)
}

func sweepRatCross(a, b sweepRatVec) sweepRatVec {
	component := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(
			new(big.Rat).Mul(a[i], b[j]),
			new(big.Rat).Mul(a[j], b[i]),
		)
	}
	return sweepRatVec{component(1, 2), component(2, 0), component(0, 1)}
}

func sweepRatIsZero(v sweepRatVec) bool {
	return v[0].Sign() == 0 && v[1].Sign() == 0 && v[2].Sign() == 0
}

func finishArcSweepBody(body *Body, payload sweepPayload) {
	if built, ok := body.payload.(revolvePayload); ok {
		payload.revolve = built
	}
	for _, face := range body.Faces() {
		for i, origin := range face.origins {
			switch {
			case strings.HasPrefix(origin.Role, "side("):
				origin.Role = "side(0," + strings.TrimPrefix(origin.Role, "side(")
			case payload.reverseArcCaps && origin.Role == roleCapStart:
				origin.Role = roleCapEnd
			case payload.reverseArcCaps && origin.Role == roleCapEnd:
				origin.Role = roleCapStart
			}
			face.origins[i] = origin
		}
	}
	body.payload = payload
}
