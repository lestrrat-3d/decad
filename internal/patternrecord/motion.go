package patternrecord

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

type Point2 = sectionrecord.Point2

// Spec holds a validated pattern's inputs for its plane-local record motion.
type Spec struct {
	Count        int
	Circular     bool
	Dir          r3.Vec
	StepMM       float64
	StepRat      *big.Rat
	Center, Axis r3.Vec
}

// Perpendicular decides the exact dot product of the held direction and normal.
func Perpendicular(dir, normal r3.Vec) bool {
	return ratVecDot(ratVecOf(dir), ratVecOf(normal)).Sign() == 0
}

// ratVec is an exact rational 3-vector.
type ratVec [3]*big.Rat

func ratVecOf(v r3.Vec) ratVec {
	return ratVec{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func ratVecDot(a, b ratVec) *big.Rat {
	return proofbound.RatAdd(proofbound.RatMul(a[0], b[0]), proofbound.RatMul(a[1], b[1]), proofbound.RatMul(a[2], b[2]))
}

// placedAxes are the receiver's frame axes and origin as placed in world
// space, exact rationals of the held floats: B·U, B·V and B·O + t, with B the
// placement's basis and t its translation.
func placedAxes(frame r3.Frame, xform r3.Transform) (u, v, o ratVec) {
	basis := xform.Basis()
	ex, ey, ez := ratVecOf(basis.EX), ratVecOf(basis.EY), ratVecOf(basis.EZ)
	apply := func(w r3.Vec) ratVec {
		r := ratVecOf(w)
		var out ratVec
		for k := range out {
			out[k] = proofbound.RatAdd(proofbound.RatMul(ex[k], r[0]), proofbound.RatMul(ey[k], r[1]), proofbound.RatMul(ez[k], r[2]))
		}
		return out
	}
	u, v = apply(frame.U()), apply(frame.V())
	o = apply(frame.Origin())
	t := ratVecOf(xform.Translation())
	for k := range o {
		o[k] = new(big.Rat).Add(o[k], t[k])
	}
	return u, v, o
}

// PointMotion moves one plane-local point exactly and rounds it once,
// returning the held point beside the distance it can sit from the exact
// image (the two coordinates' errors summed, or the one that rounded).
type PointMotion func(Point2) (Point2, float64, error)

// heldOf rounds an exact coordinate interval to the float nearest its
// midpoint, beside the farthest the true coordinate can sit from it.
func heldOf(iv proofbound.RatInterval) (float64, float64, error) {
	mid := new(big.Rat).Add(iv.Lo, iv.Hi)
	mid.Quo(mid, big.NewRat(2, 1))
	held, _ := mid.Float64()
	if math.IsInf(held, 0) {
		return 0, 0, fmt.Errorf(`%w: a pattern instance coordinate overflows a float`, decaderr.ErrNotFinite)
	}
	return held, proofbound.IntervalFloatError(iv, held), nil
}

func heldPoint(u, v proofbound.RatInterval) (Point2, float64, error) {
	hu, eu, err := heldOf(u)
	if err != nil {
		return Point2{}, 0, err
	}
	hv, ev, err := heldOf(v)
	if err != nil {
		return Point2{}, 0, err
	}
	p := Point2{U: hu, V: hv}
	switch {
	case eu == 0:
		return p, ev, nil
	case ev == 0:
		return p, eu, nil
	default:
		return p, proofbound.AbsSumUpper(eu, ev), nil
	}
}

// LinearMotion is instance i's in-plane offset (§6.2): the world offset
// t = i·step·Dir/|Dir| read on the placed frame axes, (t·U, t·V). step is the
// exact rational the caller's Step denotes in millimetres, and 1/|Dir| is a
// certified enclosure from RatSqrtDown/RatSqrtUp over the exact Dir·Dir, of
// zero width when Dir·Dir is a float's square. Each moved coordinate is an
// exact interval rounded once at its midpoint, so the step's conversion, the
// enclosure's width and the sum's rounding are all inside the one charge.
func (rp Spec) LinearMotion(frame r3.Frame, xform r3.Transform, i int) (PointMotion, error) {
	dir := ratVecOf(rp.Dir)
	q := ratVecDot(dir, dir)
	lo, hi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
	if lo <= 0 || math.IsInf(hi, 0) {
		return nil, fmt.Errorf(`%w: the pattern direction's length has no certified enclosure`, decaderr.ErrUnsupported)
	}
	scale := new(big.Rat).Mul(big.NewRat(int64(i), 1), rp.StepRat)
	k := proofbound.IntervalScale(proofbound.Interval(
		new(big.Rat).Inv(proofarith.FloatRat(hi)), new(big.Rat).Inv(proofarith.FloatRat(lo))), scale)
	u, v, _ := placedAxes(frame, xform)
	ou := proofbound.IntervalScale(k, ratVecDot(dir, u))
	ov := proofbound.IntervalScale(k, ratVecDot(dir, v))
	return func(p Point2) (Point2, float64, error) {
		return heldPoint(
			proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(p.U)), ou),
			proofbound.IntervalAdd(proofbound.PointInterval(proofarith.FloatRat(p.V)), ov))
	}, nil
}

// CircularMotion is instance i's in-plane rotation (§6.2) by i/Count of a
// turn about the centre the pattern's axis passes through, read on the placed
// frame axes as an exact rational. The sense is the world rotation's, carried
// into the plane: reversed when Axis is −N, and reversed again when the
// placement is a reflection, which conjugates a rotation into its inverse.
//
// A half turn and a quarter turn are the exact maps (u, v) ↦ (−u, −v) and
// (−v, u) about the centre: the angle denotes exactly those, and no trig
// runs. Every other turn reads cos and sin from TurnSinCosInterval, the
// certified enclosure of sin(2πt) and cos(2πt) for a rational turn t, so each
// rotated coordinate is an exact interval rounded once at its midpoint.
func (rp Spec) CircularMotion(frame r3.Frame, xform r3.Transform, i int) PointMotion {
	u, v, o := placedAxes(frame, xform)
	c := ratVecOf(rp.Center)
	rel := ratVec{new(big.Rat).Sub(c[0], o[0]), new(big.Rat).Sub(c[1], o[1]), new(big.Rat).Sub(c[2], o[2])}
	cu, cv := ratVecDot(rel, u), ratVecDot(rel, v)
	sense := int64(1)
	if rp.Axis != xform.ApplyDir(frame.N()) {
		sense = -sense
	}
	if xform.IsReflection() {
		sense = -sense
	}
	turn := big.NewRat(sense*int64(i), int64(rp.Count))
	// Reduce into [0, 1) to name the exact quarter and half turns.
	whole := new(big.Int).Div(turn.Num(), turn.Denom())
	frac := new(big.Rat).Sub(turn, new(big.Rat).SetInt(whole))
	var cosIv, sinIv proofbound.RatInterval
	switch {
	case frac.Cmp(big.NewRat(1, 4)) == 0:
		cosIv, sinIv = proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(1, 1))
	case frac.Cmp(big.NewRat(1, 2)) == 0:
		cosIv, sinIv = proofbound.PointInterval(big.NewRat(-1, 1)), proofbound.PointInterval(new(big.Rat))
	case frac.Cmp(big.NewRat(3, 4)) == 0:
		cosIv, sinIv = proofbound.PointInterval(new(big.Rat)), proofbound.PointInterval(big.NewRat(-1, 1))
	default:
		sinIv, cosIv = proofbound.TurnSinCosInterval(frac)
	}
	return func(p Point2) (Point2, float64, error) {
		du := proofbound.PointInterval(new(big.Rat).Sub(proofarith.FloatRat(p.U), cu))
		dv := proofbound.PointInterval(new(big.Rat).Sub(proofarith.FloatRat(p.V), cv))
		ru := proofbound.IntervalAdd(proofbound.PointInterval(cu),
			proofbound.IntervalSub(proofbound.IntervalMul(cosIv, du), proofbound.IntervalMul(sinIv, dv)))
		rv := proofbound.IntervalAdd(proofbound.PointInterval(cv),
			proofbound.IntervalAdd(proofbound.IntervalMul(sinIv, du), proofbound.IntervalMul(cosIv, dv)))
		return heldPoint(ru, rv)
	}
}

// Motion is instance i's in-plane motion of the frame-keeping arm.
func (rp Spec) Motion(frame r3.Frame, xform r3.Transform, i int) (PointMotion, error) {
	if rp.Circular {
		return rp.CircularMotion(frame, xform, i), nil
	}
	return rp.LinearMotion(frame, xform, i)
}
