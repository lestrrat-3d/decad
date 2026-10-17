package loftcylinder

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// RootCollar bounds the root wall's cap collar from two recorded profiles.
// A matched pair of centered circular arcs has a constant radius at each
// cap and a linearly interpolated angular difference along its parameter.
// At every arc parameter, squared radius along a Loft ruling is convex in
// height. Both interior endpoint inequalities certify the interval between
// the returned cap collars. A positive cap radius roundoff is admitted only
// when this proof places the root wall inside the blank between them.
func RootCollar(xform r3.Transform, profiles [2]momentinput.Profile, radius, height float64) (float64, bool) {
	var q [2]*big.Rat
	var angles [2][2]proofbound.RatInterval
	basis := xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	for cap, profile := range profiles {
		arc, ok := profile.Outer.Segments[5].(sectionrecord.ArcSeg)
		if !ok {
			return 0, false
		}
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q[cap] = new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		for end, p := range []sectionrecord.Point2{arc.Start, arc.End} {
			angles[cap][end] = proofbound.Atan2Interval(
				proofarith.FloatRat(p.V), proofarith.FloatRat(p.U), false)
		}
		if angles[cap][1].Hi.Cmp(angles[cap][0].Lo) <= 0 {
			angles[cap][1] = proofbound.IntervalAdd(angles[cap][1], proofbound.TwoPiInterval())
		}
	}
	r2 := new(big.Rat).Mul(proofarith.FloatRat(radius), proofarith.FloatRat(radius))
	if q[0].Cmp(r2) <= 0 && q[1].Cmp(r2) <= 0 {
		return 0, true
	}
	minTwist := new(big.Rat)
	twistSign := 0
	for end := range 2 {
		delta := proofbound.IntervalSub(angles[1][end], angles[0][end])
		var lower *big.Rat
		sign := 0
		switch {
		case delta.Lo.Sign() > 0 && delta.Hi.Cmp(proofarith.FloatRat(0.4)) < 0:
			lower = delta.Lo
			sign = 1
		case delta.Hi.Sign() < 0 && delta.Lo.Cmp(proofarith.FloatRat(-0.4)) > 0:
			lower = new(big.Rat).Neg(delta.Hi)
			sign = -1
		default:
			return 0, false
		}
		if twistSign != 0 && sign != twistSign {
			return 0, false
		}
		twistSign = sign
		if end == 0 || lower.Cmp(minTwist) < 0 {
			minTwist = lower
		}
	}
	_, cosine, ok := proofbound.RadSinCosInterval(minTwist)
	if !ok {
		return 0, false
	}
	oneMinusCos := new(big.Rat).Sub(big.NewRat(1, 1), cosine.Hi)
	if oneMinusCos.Sign() <= 0 {
		return 0, false
	}
	rProductLo := proofarith.FloatRat(proofbound.RatSqrtDown(
		new(big.Rat).Mul(q[0], q[1])))
	for shift := 52; shift >= 8; shift-- {
		epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(shift)))
		oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), epsilon)
		inward := new(big.Rat).Mul(big.NewRat(2, 1), epsilon)
		inward.Mul(inward, oneMinus)
		inward.Mul(inward, rProductLo)
		inward.Mul(inward, oneMinusCos)
		bottom := new(big.Rat).Add(new(big.Rat).Mul(oneMinus, q[0]),
			new(big.Rat).Mul(epsilon, q[1]))
		top := new(big.Rat).Add(new(big.Rat).Mul(epsilon, q[0]),
			new(big.Rat).Mul(oneMinus, q[1]))
		if bottom.Sub(bottom, inward).Cmp(r2) < 0 && top.Sub(top, inward).Cmp(r2) < 0 {
			collar := proofbound.ProductUpper(proofbound.RatFloatUp(epsilon), height)
			return collar, !proofbound.IsNonFinite(collar)
		}
	}
	return 0, false
}

// SourceStrictRootCollar certifies strict interior occupancy at a zero collar.
// The root collar is zero when both cap radii are at most R. In
// that arm, convexity makes every interior ruling strict if one cap radius
// is strict. If both equal R, a certified nonzero angle change makes every
// interior ruling strict. Neither argument asserts strictness on a cap.
func SourceStrictRootCollar(xform r3.Transform, profiles [2]momentinput.Profile, radius, height float64) (float64, bool) {
	collar, ok := RootCollar(xform, profiles, radius, height)
	if !ok || collar > 0 {
		return collar, ok
	}
	basis := xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	r2 := new(big.Rat).Mul(proofarith.FloatRat(radius), proofarith.FloatRat(radius))
	var q [2]*big.Rat
	var angles [2][2]proofbound.RatInterval
	for cap, profile := range profiles {
		arc, valid := profile.Outer.Segments[5].(sectionrecord.ArcSeg)
		if !valid {
			return 0, false
		}
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q[cap] = new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		if q[cap].Cmp(r2) > 0 {
			return 0, false
		}
		for end, p := range []sectionrecord.Point2{arc.Start, arc.End} {
			angles[cap][end] = proofbound.Atan2Interval(
				proofarith.FloatRat(p.V), proofarith.FloatRat(p.U), false)
		}
		if angles[cap][1].Hi.Cmp(angles[cap][0].Lo) <= 0 {
			angles[cap][1] = proofbound.IntervalAdd(angles[cap][1], proofbound.TwoPiInterval())
		}
	}
	if q[0].Cmp(r2) < 0 || q[1].Cmp(r2) < 0 {
		return 0, true
	}
	twistSign := 0
	for end := range 2 {
		delta := proofbound.IntervalSub(angles[1][end], angles[0][end])
		sign := 0
		if delta.Lo.Sign() > 0 && delta.Hi.Cmp(proofarith.FloatRat(0.4)) < 0 {
			sign = 1
		} else if delta.Hi.Sign() < 0 && delta.Lo.Cmp(proofarith.FloatRat(-0.4)) > 0 {
			sign = -1
		}
		if sign == 0 || twistSign != 0 && sign != twistSign {
			return 0, false
		}
		twistSign = sign
	}
	return 0, true
}
