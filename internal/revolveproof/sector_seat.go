package revolveproof

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// revolveCrossingPoint lifts Sketch's rounded line evaluation and bounds the
// true cone crossing enclosed by its parameter interval. The returned point
// is only a mesh station; pointBound must enter every seam displacement proof.
func revolveCrossingPoint(src *facetedrecord.PointSectionSource, cert sketch.ConeLineCrossing) (
	r3.Vec, float64, bool) {
	if src == nil || cert.IsStale() || !cert.TExact() ||
		cert.SketchRevision() != src.Revision {
		return r3.Vec{}, 0, false
	}
	line := cert.Line()
	if line == nil {
		return r3.Vec{}, 0, false
	}
	start, end := line.Start.Geometry(), line.End.Geometry()
	point := cert.Point()
	t, lo, hi := cert.T(), 0.0, 0.0
	lo, hi = cert.TBounds()
	if !(0 < lo && lo <= t && t <= hi && hi < 1) ||
		!proofbound.FiniteVec(r3.Vec{X: point[0], Y: point[1], Z: t}) {
		return r3.Vec{}, 0, false
	}
	q := src.Frame.ToWorldUV(point[0], point[1])
	if !proofbound.FiniteVec(q) {
		return r3.Vec{}, 0, false
	}
	delta := proofbound.HeldDelta(r3.Vec{X: end.X, Y: end.Y},
		r3.Vec{X: start.X, Y: start.Y})
	lineLength := proofbound.DvLenUpper(delta)
	loGap := new(big.Rat).Sub(proofarith.FloatRat(t), proofarith.FloatRat(lo))
	hiGap := new(big.Rat).Sub(proofarith.FloatRat(hi), proofarith.FloatRat(t))
	if loGap.Sign() < 0 || hiGap.Sign() < 0 {
		return r3.Vec{}, 0, false
	}
	if hiGap.Cmp(loGap) > 0 {
		loGap = hiGap
	}
	paramGap := proofbound.RatFloatUp(loGap)
	coordError := func(a, b, rounded float64) float64 {
		exact := new(big.Rat).Add(proofarith.FloatRat(a),
			new(big.Rat).Mul(new(big.Rat).Sub(proofarith.FloatRat(b),
				proofarith.FloatRat(a)), proofarith.FloatRat(t)))
		return proofbound.RatAbsDiff(exact, rounded)
	}
	uError := coordError(start.X, end.X, point[0])
	vError := coordError(start.Y, end.Y, point[1])
	uLength := proofbound.DvLenUpper(proofbound.HeldDelta(src.Frame.U(), r3.Vec{}))
	vLength := proofbound.DvLenUpper(proofbound.HeldDelta(src.Frame.V(), r3.Vec{}))
	bound := proofbound.AbsSumUpper(
		proofbound.ProductUpper(paramGap, proofbound.ProductUpper(lineLength,
			proofbound.AbsSumUpper(uLength, vLength))),
		proofbound.ProductUpper(uError, uLength),
		proofbound.ProductUpper(vError, vLength),
		loftmesh.FrameLiftRoundAllow(src.Frame, max(math.Abs(point[0]), math.Abs(point[1]))))
	return q, bound, !proofbound.IsNonFinite(bound)
}

// revolveSeatPoint extends the reported-cone crossing to the exact straight
// meridian recorded by Revolve. F is strictly monotone along the whole line;
// its endpoint margins dominate the meridian perturbation. The inverse slope
// then encloses how far the true source contact can move along the connector.
// A general certificate bounds the meridian perturbation exactly instead
// (seatShiftAbout).
func revolveSeatPoint(src *facetedrecord.PointSectionSource, cert sketch.ConeLineCrossing,
	section facetedrecord.RevolveSectionCertificate, base facetedrecord.Payload) (r3.Vec, float64, bool) {
	point, pointBound, ok := revolveCrossingPoint(src, cert)
	if !ok || len(base.Verts) < 4 || base.MeshBound < 0 {
		return r3.Vec{}, 0, false
	}
	line := cert.Line()
	start, end := line.Start.Geometry(), line.End.Geometry()
	frame := src.Frame
	fr := [3][3]*big.Rat{}
	for axis, values := range [3][3]float64{
		{frame.Origin().X, frame.U().X, frame.V().X},
		{frame.Origin().Y, frame.U().Y, frame.V().Y},
		{frame.Origin().Z, frame.U().Z, frame.V().Z},
	} {
		for j, v := range values {
			fr[axis][j] = proofarith.FloatRat(v)
		}
	}
	var q, d [3]*big.Rat
	u0, v0 := proofarith.FloatRat(start.X), proofarith.FloatRat(start.Y)
	du := new(big.Rat).Sub(proofarith.FloatRat(end.X), u0)
	dv := new(big.Rat).Sub(proofarith.FloatRat(end.Y), v0)
	for i := range q {
		q[i] = new(big.Rat).Add(fr[i][0], new(big.Rat).Add(
			new(big.Rat).Mul(fr[i][1], u0), new(big.Rat).Mul(fr[i][2], v0)))
		d[i] = new(big.Rat).Add(new(big.Rat).Mul(fr[i][1], du),
			new(big.Rat).Mul(fr[i][2], dv))
	}
	slope := proofarith.FloatRat(section.ConeSection().Slope)
	slope2 := new(big.Rat).Mul(slope, slope)
	originX := proofarith.FloatRat(section.RootCone.Origin.X)
	form := func(p [3]*big.Rat) *big.Rat {
		x := new(big.Rat).Sub(p[0], originX)
		return new(big.Rat).Sub(new(big.Rat).Add(
			new(big.Rat).Mul(p[1], p[1]), new(big.Rat).Mul(p[2], p[2])),
			new(big.Rat).Mul(slope2, new(big.Rat).Mul(x, x)))
	}
	endQ := [3]*big.Rat{}
	for i := range q {
		endQ[i] = new(big.Rat).Add(q[i], d[i])
	}
	if section.General != nil {
		shift, ok := seatShiftAbout(section, q, endQ, d)
		if !ok {
			return r3.Vec{}, 0, false
		}
		bound, ok := seatAllowance(cert, frame, pointBound, shift)
		return point, bound, ok
	}
	f0, f1 := form(q), form(endQ)
	derivative := func(p [3]*big.Rat) *big.Rat {
		x := new(big.Rat).Sub(p[0], originX)
		return new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Sub(
			new(big.Rat).Add(new(big.Rat).Mul(p[1], d[1]),
				new(big.Rat).Mul(p[2], d[2])),
			new(big.Rat).Mul(slope2, new(big.Rat).Mul(x, d[0]))))
	}
	d0, d1 := derivative(q), derivative(endQ)
	if d0.Sign() == 0 || d0.Sign() != d1.Sign() || f0.Sign() == f1.Sign() {
		return r3.Vec{}, 0, false
	}
	m, intercept := section.MeridianEquation()
	sourceRadius := func(p [3]*big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(m, p[0]), intercept)
	}
	sourceForm := func(p [3]*big.Rat) *big.Rat {
		radius := sourceRadius(p)
		return new(big.Rat).Sub(new(big.Rat).Add(
			new(big.Rat).Mul(p[1], p[1]), new(big.Rat).Mul(p[2], p[2])),
			new(big.Rat).Mul(radius, radius))
	}
	if sourceRadius(q).Sign() <= 0 || sourceRadius(endQ).Sign() <= 0 ||
		sourceForm(q).Sign() == sourceForm(endQ).Sign() {
		return r3.Vec{}, 0, false
	}
	minD := new(big.Rat).Abs(d0)
	if minD.Cmp(new(big.Rat).Abs(d1)) > 0 {
		minD.Abs(d1)
	}
	reach := 0.0
	for _, p := range base.Verts[1:] {
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(p, r3.Vec{})))
	}
	reach = proofbound.AbsSumUpper(reach, base.MeshBound)
	xLo := proofbound.RatFloatDown(proofbound.RatMin(q[0], endQ[0]))
	xHi := proofbound.RatFloatUp(proofbound.RatMax(q[0], endQ[0]))
	meridianGap := revolveMeridianExtensionGap(section, xLo, xHi)
	fCost := proofbound.ProductUpper(meridianGap,
		proofbound.AbsSumUpper(proofbound.ProductUpper(2, reach), meridianGap))
	if proofbound.IsNonFinite(fCost) ||
		new(big.Rat).Abs(f0).Cmp(proofarith.FloatRat(fCost)) <= 0 ||
		new(big.Rat).Abs(f1).Cmp(proofarith.FloatRat(fCost)) <= 0 {
		return r3.Vec{}, 0, false
	}
	shift := proofbound.RatFloatUp(new(big.Rat).Quo(proofarith.FloatRat(fCost), minD))
	bound, ok := seatAllowance(cert, frame, pointBound, shift)
	return point, bound, ok
}

// seatAllowance turns a proven parameter shift of the true contact along the
// connector into a world displacement added to the crossing point's bound.
func seatAllowance(cert sketch.ConeLineCrossing, frame r3.Frame, pointBound, shift float64) (float64, bool) {
	lo, hi := cert.TBounds()
	if proofbound.IsNonFinite(shift) || !(lo > shift && hi+shift < 1) {
		return 0, false
	}
	line := cert.Line()
	start, end := line.Start.Geometry(), line.End.Geometry()
	localLength := proofbound.DvLenUpper(proofbound.HeldDelta(
		r3.Vec{X: end.X, Y: end.Y}, r3.Vec{X: start.X, Y: start.Y}))
	basis := proofbound.AbsSumUpper(
		proofbound.DvLenUpper(proofbound.HeldDelta(frame.U(), r3.Vec{})),
		proofbound.DvLenUpper(proofbound.HeldDelta(frame.V(), r3.Vec{})))
	worldShift := proofbound.ProductUpper(shift,
		proofbound.ProductUpper(localLength, basis))
	bound := proofbound.AbsSumUpper(pointBound, worldShift)
	return bound, !proofbound.IsNonFinite(bound)
}
