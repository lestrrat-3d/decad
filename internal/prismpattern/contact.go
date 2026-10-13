package prismpattern

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// IVPoint is a plane point whose coordinates are rational intervals.
type IVPoint struct{ X, Y proofbound.RatInterval }

// IVPointOf returns the point interval at (x, y).
func IVPointOf(x, y *big.Rat) IVPoint {
	return IVPoint{proofbound.PointInterval(x), proofbound.PointInterval(y)}
}

// Add returns the interval sum p + q.
func (p IVPoint) Add(q IVPoint) IVPoint {
	return IVPoint{proofbound.IntervalAdd(p.X, q.X), proofbound.IntervalAdd(p.Y, q.Y)}
}

// Sub returns the interval difference p - q.
func (p IVPoint) Sub(q IVPoint) IVPoint {
	return IVPoint{proofbound.IntervalSub(p.X, q.X), proofbound.IntervalSub(p.Y, q.Y)}
}

// Scale returns p multiplied by the interval s.
func (p IVPoint) Scale(s proofbound.RatInterval) IVPoint {
	return IVPoint{proofbound.IntervalMul(p.X, s), proofbound.IntervalMul(p.Y, s)}
}

// Dot returns the interval dot product of p and q.
func (p IVPoint) Dot(q IVPoint) proofbound.RatInterval {
	return proofbound.IntervalAdd(proofbound.IntervalMul(p.X, q.X), proofbound.IntervalMul(p.Y, q.Y))
}

// IVSqrt returns the square-root bracket of a, or ErrUnsupported when it is
// not finite.
func IVSqrt(a proofbound.RatInterval) (proofbound.RatInterval, error) {
	value, ok := proofbound.IntervalSqrt(a)
	if !ok || proofbound.IsNonFinite(proofbound.RatFloatUp(value.Hi)) {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet has no finite square-root bracket",
			decaderr.ErrUnsupported)
	}
	return value, nil
}

// IVQuo returns the quotient a / b, or ErrUnsupported when b is not
// separated from zero.
func IVQuo(a, b proofbound.RatInterval) (proofbound.RatInterval, error) {
	value, ok := proofbound.IntervalQuo(a, b)
	if !ok {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet has no separated denominator",
			decaderr.ErrUnsupported)
	}
	return value, nil
}

// IVHeld reads the midpoint of p as a float point at height z and returns its
// interval error bound.
func IVHeld(p IVPoint, z float64) (r3.Vec, float64, error) {
	read := func(iv proofbound.RatInterval) float64 {
		mid := new(big.Rat).Add(iv.Lo, iv.Hi)
		mid.Quo(mid, big.NewRat(2, 1))
		value, _ := mid.Float64()
		return value
	}
	x, y := read(p.X), read(p.Y)
	point := r3.Vec{X: x, Y: y, Z: z}
	if !proofbound.FiniteVec(point) {
		return r3.Vec{}, 0, fmt.Errorf("%w: joined-prism fillet contact is non-finite", decaderr.ErrUnsupported)
	}
	errorBound := proofbound.AbsSumUpper(proofbound.IntervalFloatError(p.X, x),
		proofbound.IntervalFloatError(p.Y, y))
	return point, errorBound, nil
}

// ExactAngle returns the polar angle of an exact point, or ErrUnsupported
// when p is not a point or is the origin.
func ExactAngle(p IVPoint) (proofbound.RatInterval, error) {
	if p.X.Lo.Cmp(p.X.Hi) != 0 || p.Y.Lo.Cmp(p.Y.Hi) != 0 ||
		p.X.Lo.Sign() == 0 && p.Y.Lo.Sign() == 0 {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet has no exact polar point",
			decaderr.ErrUnsupported)
	}
	return proofbound.Atan2Interval(p.Y.Lo, p.X.Lo, false), nil
}

// HeldAngle returns the polar angle of a held point widened by its error
// bound.
func HeldAngle(p r3.Vec, errorBound float64) (proofbound.RatInterval, error) {
	if !proofbound.FiniteVec(p) || proofbound.IsNonFinite(errorBound) || errorBound < 0 {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet has no finite polar point",
			decaderr.ErrUnsupported)
	}
	radialSq := new(big.Rat).Add(new(big.Rat).Mul(proofarith.FloatRat(p.X), proofarith.FloatRat(p.X)),
		new(big.Rat).Mul(proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Y)))
	radialLo := new(big.Rat).Sub(proofarith.FloatRat(proofbound.RatSqrtDown(radialSq)),
		proofarith.FloatRat(errorBound))
	if radialLo.Sign() <= 0 {
		return proofbound.RatInterval{}, fmt.Errorf("%w: joined-prism fillet polar error reaches the axis",
			decaderr.ErrUnsupported)
	}
	angle := proofbound.Atan2Interval(proofarith.FloatRat(p.Y), proofarith.FloatRat(p.X), false)
	widen := proofbound.Atan2Interval(proofarith.FloatRat(errorBound), radialLo, false).Hi
	return proofbound.IntervalWiden(angle, widen), nil
}

// AngleNear picks the only 2π copy within 90 degrees of the reference. All accepted
// contacts and source connectors are local; distant or ambiguous arcs refuse.
func AngleNear(angle, reference proofbound.RatInterval) (proofbound.RatInterval, bool) {
	for k := int64(-1); k <= 1; k++ {
		shifted := proofbound.IntervalAdd(angle,
			proofbound.IntervalScale(proofbound.TwoPiInterval(), big.NewRat(k, 1)))
		diff := proofbound.IntervalSub(shifted, reference)
		if diff.Lo.Cmp(new(big.Rat).Neg(proofbound.HalfPiInterval().Lo)) > 0 &&
			diff.Hi.Cmp(proofbound.HalfPiInterval().Lo) < 0 {
			return shifted, true
		}
	}
	return proofbound.RatInterval{}, false
}

// RingPoint returns the ring point at the given angle and height with its
// rounding bound.
func RingPoint(radius, z, angle float64,
	angleIV proofbound.RatInterval) (r3.Vec, float64, error) {
	point := r3.Vec{X: radius * math.Cos(angle), Y: radius * math.Sin(angle), Z: z}
	if !proofbound.FiniteVec(point) {
		return r3.Vec{}, 0, fmt.Errorf("%w: circular prism ring is non-finite", decaderr.ErrUnsupported)
	}
	sine, cosine, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
	if !ok {
		return r3.Vec{}, 0, fmt.Errorf("%w: circular prism ring has no trigonometric enclosure", decaderr.ErrUnsupported)
	}
	r := proofarith.FloatRat(radius)
	pointRound := proofbound.Radius3D(max(
		proofbound.IntervalFloatError(proofbound.IntervalScale(cosine, r), point.X),
		proofbound.IntervalFloatError(proofbound.IntervalScale(sine, r), point.Y)))
	angleError := proofbound.IntervalFloatError(angleIV, angle)
	round := proofbound.AbsSumUpper(pointRound, proofbound.ProductUpper(radius, angleError))
	return point, round, nil
}
