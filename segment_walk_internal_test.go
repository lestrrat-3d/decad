package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// requireSameFloatBits asserts two floats are the same IEEE-754 value, bit for
// bit: a published bound that moved by one ulp is a changed result.
func requireSameFloatBits(t *testing.T, want, got float64, msgAndArgs ...any) {
	t.Helper()
	require.Equal(t, math.Float64bits(want), math.Float64bits(got), msgAndArgs...)
}

// The three oracles below are the line-walk bounds as they were computed over
// big.Rat before the dyadic rewrite, kept verbatim so the rewrite is pinned to
// the values it replaced rather than to a model of them.

func ratLineWalkTangentBound(seg LineSeg, heldU, heldV float64) float64 {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1)
	}
	return math.Max(
		proofarith.RationalFloatError(new(big.Rat).Sub(u1, u0), heldU),
		proofarith.RationalFloatError(new(big.Rat).Sub(v1, v0), heldV),
	)
}

func ratLineWalkEndBound(seg LineSeg, t, heldU, heldV float64) proofbound.WalkEndBound {
	return proofbound.WalkEndBound{
		U: proofarith.RationalFloatError(ratLerp(seg.Start.U, seg.End.U, t), heldU),
		V: proofarith.RationalFloatError(ratLerp(seg.Start.V, seg.End.V, t), heldV),
	}
}

func ratLineWalkBounds(seg LineSeg, held float64) (float64, float64, float64) {
	u0 := ratLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := ratLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := ratLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := ratLerp(seg.Start.V, seg.End.V, seg.TEnd)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return math.Inf(1), math.Inf(1), math.Inf(1)
	}
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)
	lengthSquared := new(big.Rat).Add(
		new(big.Rat).Mul(du, du),
		new(big.Rat).Mul(dv, dv),
	)
	heldRat := proofarith.FloatRat(held)
	coordUpper := math.Max(ratL1Upper(u0, v0), ratL1Upper(u1, v1))
	if heldRat != nil && new(big.Rat).Mul(heldRat, heldRat).Cmp(lengthSquared) == 0 {
		return 0, held, coordUpper
	}
	l1 := new(big.Rat).Add(new(big.Rat).Abs(du), new(big.Rat).Abs(dv))
	upper, exact := l1.Float64()
	if !exact {
		upper = math.Nextafter(upper, math.Inf(1))
	}
	bound := math.Min(proofbound.ConservativeValueError(held, upper), ratSqrtIntervalError(lengthSquared, held))
	return bound, upper, coordUpper
}

// ratSqrtIntervalError is dySqrtIntervalError as it was computed over big.Rat
// before the dyadic rewrite, kept verbatim as the oracle the line-walk and
// straight-edge comparisons read.
func ratSqrtIntervalError(lengthSquared *big.Rat, held float64) float64 {
	lo, hi := proofarith.FloatRat(proofbound.RatSqrtDown(lengthSquared)), proofarith.FloatRat(proofbound.RatSqrtUp(lengthSquared))
	if lo == nil || hi == nil {
		return math.Inf(1)
	}
	return proofbound.IntervalFloatError(proofbound.Interval(lo, hi), held)
}

// TestLineWalkBoundsDyadicMatchRational pins lineWalkBounds,
// lineWalkTangentBound and lineWalkEndBound, computed over dyadics, to the
// big.Rat computation they replaced, bit for bit, on every output. The
// segments mix integer, eighth-unit, 1e-6, 1e2 and 1e6 coordinates, natural,
// midpoint and random parameters, and a non-finite coordinate now and then.
// Each is read at the walk's own held values and at a held length one ulp
// high, which exercises the inexact rounding paths.
//
// Shown to fail: publishing through dyFloatUp instead of dyNearestUp turns the
// test red on an inexact value whose nearest float rounded up, in
// dyRoundedFloatError alone (the tangent leg) and in dyL1Upper alone (the
// length-upper leg); deleting lineWalkBounds' dySquareEquals early return
// turns it red on an exact-length segment.
func TestLineWalkBoundsDyadicMatchRational(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 9))
	coord := func() float64 {
		switch rng.IntN(5) {
		case 0:
			return float64(rng.IntN(200) - 100)
		case 1:
			return float64(rng.IntN(2000)-1000) / 8
		case 2:
			return rng.NormFloat64() * 100
		case 3:
			return rng.NormFloat64() * 1e-6
		default:
			return rng.NormFloat64() * 1e6
		}
	}
	param := func() float64 {
		switch rng.IntN(4) {
		case 0:
			return 0
		case 1:
			return 1
		case 2:
			return 0.5
		default:
			return rng.Float64()
		}
	}
	for i := range 40000 {
		seg := LineSeg{
			Start: Point2{U: coord(), V: coord()}, End: Point2{U: coord(), V: coord()},
			TStart: param(), TEnd: param(),
		}
		if i%97 == 0 {
			seg.Start.U = math.Inf(1)
		}
		u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
		u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
		du, dv := u1-u0, v1-v0
		length := math.Hypot(du, dv)
		for _, held := range []float64{length, math.Nextafter(length, math.Inf(1))} {
			wb, wu, wc := ratLineWalkBounds(seg, held)
			gb, gu, gc := lineWalkBounds(seg, held)
			requireSameFloatBits(t, wb, gb, "length bound of %+v at %v", seg, held)
			requireSameFloatBits(t, wu, gu, "length upper of %+v at %v", seg, held)
			requireSameFloatBits(t, wc, gc, "coordinate upper of %+v at %v", seg, held)
		}
		requireSameFloatBits(t, ratLineWalkTangentBound(seg, du, dv), lineWalkTangentBound(seg, du, dv),
			"tangent bound of %+v", seg)
		for _, end := range [][3]float64{{seg.TStart, u0, v0}, {seg.TEnd, u1, v1}} {
			want := ratLineWalkEndBound(seg, end[0], end[1], end[2])
			got := lineWalkEndBound(seg, end[0], end[1], end[2])
			requireSameFloatBits(t, want.U, got.U, "end u of %+v at t=%v", seg, end[0])
			requireSameFloatBits(t, want.V, got.V, "end v of %+v at t=%v", seg, end[0])
		}
	}
}

// ratArcWalk is walkOf's ArcSeg arm as it was before the radius bracket was
// read out of circularWalkEnclosures, kept verbatim as the oracle below: the
// radius bound from arcWalkRadiusBound's own bracket, and the length bound
// from a second, separate circularLengthInterval.
func ratArcWalk(seg ArcSeg) survey2d.SegmentWalk {
	radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
	a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
	a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
	sweep := math.Mod(a1-a0, 2*math.Pi)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	w := circularWalk(
		seg.Center.U,
		seg.Center.V,
		radius,
		a0+seg.TStart*sweep,
		a0+seg.TEnd*sweep,
		arcRadiusUpper(seg),
		proofbound.CircularSweepUpper(seg.TStart, seg.TEnd),
	)
	w.RadiusBound = arcWalkRadiusBound(seg, radius)
	pinArcWalkEnds(&w, seg)
	if iv, ok := circularLengthInterval(seg); ok {
		w.LengthBound = math.Min(w.LengthBound, proofbound.IntervalFloatError(iv, w.Length))
	}
	return w
}

// TestArcWalkRadiusBoundMatchesEnclosureBracket pins walkOf's ArcSeg arm,
// which reads the radius bracket once out of circularWalkEnclosures, to the
// arm that built it twice, bit for bit on the radius bound and the length
// bound. The arcs mix integer (Pythagorean, so an exact radius), unit, 1e-3,
// 1e3 and 1e6 magnitudes over natural and trimmed ranges, and one arc whose
// squared radius overflows the bracket drives the refusal arm.
//
// Shown to fail: reading rIv.hi for both ends of the bracket turns the radius
// leg red on an arc whose radius the bracket does not pin to one float;
// multiplying the radius interval by itself instead of by the sweep turns the
// length leg red; dropping the arcWalkRadiusBound fallback turns the overflow
// arc red.
func TestArcWalkRadiusBoundMatchesEnclosureBracket(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(11, 13))
	check := func(t *testing.T, seg ArcSeg) {
		t.Helper()
		got, err := walkOf(seg, nil)
		require.NoError(t, err, "%+v", seg)
		want := ratArcWalk(seg)
		requireSameFloatBits(t, want.RadiusBound, got.RadiusBound, "radius bound of %+v", seg)
		requireSameFloatBits(t, want.LengthBound, got.LengthBound, "length bound of %+v", seg)
	}
	// 2000 arcs, not more: each walk builds two certified atan2 enclosures,
	// and at 10000 the test alone cost about 25s of a race shard.
	for range 2000 {
		scale := []float64{1, 1e-3, 1e3, 1e6}[rng.IntN(4)]
		c := Point2{U: rng.NormFloat64() * scale, V: rng.NormFloat64() * scale}
		var start, end Point2
		if rng.IntN(3) == 0 {
			k := float64(1 + rng.IntN(50))
			c = Point2{U: float64(rng.IntN(200) - 100), V: float64(rng.IntN(200) - 100)}
			start = Point2{U: c.U + 3*k, V: c.V + 4*k}
			end = Point2{U: c.U - 4*k, V: c.V + 3*k}
		} else {
			r := math.Abs(rng.NormFloat64())*scale + scale/100
			a, b := rng.Float64()*2*math.Pi, rng.Float64()*2*math.Pi
			start = Point2{U: c.U + r*math.Cos(a), V: c.V + r*math.Sin(a)}
			end = Point2{U: c.U + r*math.Cos(b), V: c.V + r*math.Sin(b)}
		}
		t0, t1 := 0.0, 1.0
		if rng.IntN(2) == 0 {
			t0, t1 = rng.Float64()/2, 0.5+rng.Float64()/2
		}
		check(t, ArcSeg{Center: c, Start: start, End: end, TStart: t0, TEnd: t1})
	}

	overflow := ArcSeg{
		Center: Point2{U: -1e308, V: 0},
		Start:  Point2{U: 1e308, V: 0},
		End:    Point2{U: -1e308, V: 1e308},
		TStart: 0, TEnd: 1,
	}
	_, _, ok := circularWalkEnclosures(overflow)
	require.False(t, ok, "the fixture must overflow the radius bracket to reach the refusal arm")
	check(t, overflow)
	got, err := walkOf(overflow, nil)
	require.NoError(t, err)
	require.True(t, math.IsInf(got.RadiusBound, 1), "an overflowed bracket refuses with +Inf")
}
