package revolveproof

import (
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// circleFragmentSlack widens the parameter range and each box side of a
// circle fragment, as a fraction of the radius, so the box encloses the true
// curve however the float trigonometry rounded. It is far above binary64
// rounding and far below any margin a real tooth root has.
const circleFragmentSlack = 1e-9

// CircleFragmentSide proves that the Partial circle fragment of edge lies
// strictly on one side of cone, over exactly the parameter range Sketch reports
// for it. Sketch's CertifyConeCurveSide covers a whole line, fitted spline or
// narrow arc but no circle fragment, so this decad-side proof stands in for it
// on the one shape a tooth root takes when the involute base circle lies inside
// the root circle. The proof is the same exact test Sketch's arc proof ends in:
// an interval box around the fragment, and the sign of the cone residual over
// that box. It admits only; any doubt (a range wider than a quarter turn, a
// non-finite input, a box that touches the cone) returns ok false.
//
// Sketch must report the fragment's bounds exact (TExact), and the recorded
// segment must carry the same centre, radius and range. The returned margin is
// an exact lower bound on |F| over the box, the quantity ImplicitMargin gives
// for a Sketch certificate. A cone about an axis A other than world X reads
// Sketch's own F = |q×A|² − s²(q·A)², whose margin is in |A|²·mm²
// (agent-docs/revolve-axis-design.md §5.3).
func CircleFragmentSide(frame r3.Frame, circle *sketch.Circle, edge sketch.BoundaryEdge,
	segment sectionrecord.CircleSeg, cone sketch.ConeSection) (sketch.ConeSide, *big.Rat, bool) {
	if circle == nil || circle.IsStale() || edge.Entity != sketch.Entity(circle) ||
		!edge.Partial || !edge.TExact {
		return 0, nil, false
	}
	cx, cy, r := circle.Center.X(), circle.Center.Y(), circle.R()
	lo, hi := edge.TStart, edge.TEnd
	if !proofbound.FiniteVec(r3.Vec{X: cx, Y: cy, Z: r}) || !(r > 0) ||
		!(0 <= lo && lo < hi && hi <= 1) || hi-lo >= 0.25 ||
		segment.Center != (sectionrecord.Point2{U: cx, V: cy}) || segment.Radius.Base() != r ||
		min(segment.TStart, segment.TEnd) != lo || max(segment.TStart, segment.TEnd) != hi ||
		!proofbound.FiniteVec(cone.Axis) || cone.Axis == (r3.Vec{}) ||
		!(cone.Slope > 0) || math.IsInf(cone.Slope, 0) {
		return 0, nil, false
	}
	a0, a1 := 2*math.Pi*lo-circleFragmentSlack, 2*math.Pi*hi+circleFragmentSlack
	xs, ys := []float64{math.Cos(a0), math.Cos(a1)}, []float64{math.Sin(a0), math.Sin(a1)}
	// Cosine and sine are monotone between quarter turns; an extremum inside
	// the range is a multiple of a quarter turn.
	for k := math.Ceil(a0 / (math.Pi / 2)); k*(math.Pi/2) <= a1; k++ {
		xs = append(xs, math.Cos(k*math.Pi/2))
		ys = append(ys, math.Sin(k*math.Pi/2))
	}
	slack := circleFragmentSlack * r
	uLo, uHi := cx+r*slices.Min(xs)-slack, cx+r*slices.Max(xs)+slack
	vLo, vHi := cy+r*slices.Min(ys)-slack, cy+r*slices.Max(ys)+slack
	if !proofbound.FiniteVec(r3.Vec{X: uLo, Y: uHi, Z: vLo}) || !proofbound.FiniteVec(r3.Vec{X: vHi}) {
		return 0, nil, false
	}
	// A float rounding of the box ends is itself charged: step each end
	// outward by one unit in the last place.
	uLo, uHi = math.Nextafter(uLo, math.Inf(-1)), math.Nextafter(uHi, math.Inf(1))
	vLo, vHi = math.Nextafter(vLo, math.Inf(-1)), math.Nextafter(vHi, math.Inf(1))
	world, ok := proofbound.FrameBoxWorld(frame, frame.Origin(), [2]float64{uLo, vLo}, [2]float64{uHi, vHi})
	if !ok {
		return 0, nil, false
	}
	apex := [3]float64{cone.Apex.X, cone.Apex.Y, cone.Apex.Z}
	var q [3]proofbound.RatInterval
	for i := range q {
		q[i] = proofbound.IntervalSub(world[i], proofbound.PointInterval(proofarith.FloatRat(apex[i])))
	}
	if cone.Axis != (r3.Vec{X: 1}) {
		residual, ok := coneResidualAbout(q, cone)
		if !ok {
			return 0, nil, false
		}
		return circleSide(residual)
	}
	// The axis is +X, so the half-cone test is q.X > 0 and the cross product
	// with the axis has components (0, q.Z, -q.Y).
	h := q[0]
	if h.Lo.Sign() <= 0 {
		return 0, nil, false
	}
	slope2 := new(big.Rat).Mul(proofarith.FloatRat(cone.Slope), proofarith.FloatRat(cone.Slope))
	residual := proofbound.IntervalSub(
		proofbound.IntervalAdd(proofbound.IntervalSquare(q[1]), proofbound.IntervalSquare(q[2])),
		proofbound.IntervalScale(proofbound.IntervalSquare(h), slope2))
	return circleSide(residual)
}

// circleSide reads the side and the exact margin from the residual's interval.
func circleSide(residual proofbound.RatInterval) (sketch.ConeSide, *big.Rat, bool) {
	if residual.Lo.Sign() > 0 {
		return sketch.ConeOutside, new(big.Rat).Set(residual.Lo), true
	}
	if residual.Hi.Sign() < 0 {
		return sketch.ConeInside, new(big.Rat).Neg(residual.Hi), true
	}
	return 0, nil, false
}
