package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestRatMinMaxFollowExactOrder checks RatMin and RatMax against the order
// big.Rat.Cmp gives, over dyadics, fractions, equal values and opposite signs.
func TestRatMinMaxFollowExactOrder(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(5, 8))
	pick := func() *big.Rat {
		switch rng.IntN(3) {
		case 0:
			return proofarith.FloatRat(rng.NormFloat64())
		case 1:
			return big.NewRat(rng.Int64N(41)-20, rng.Int64N(9)+1)
		}
		return new(big.Rat).SetInt64(rng.Int64N(5) - 2)
	}
	for range 2000 {
		a, b := pick(), pick()
		lo, hi := proofbound.RatMin(a, b), proofbound.RatMax(a, b)
		require.True(t, lo.Cmp(a) <= 0 && lo.Cmp(b) <= 0, "RatMin(%v, %v) = %v", a, b, lo)
		require.True(t, hi.Cmp(a) >= 0 && hi.Cmp(b) >= 0, "RatMax(%v, %v) = %v", a, b, hi)
		require.True(t, lo.Cmp(a) == 0 || lo.Cmp(b) == 0)
		require.True(t, hi.Cmp(a) == 0 || hi.Cmp(b) == 0)
	}
}

// TestChordLocusLengthAllowMatchesRawQuotient reads the axial speed
// |axialSpan|/dc as an exact quotient. The reference rebuilds ChordLocusLengthAllow
// around big.Rat.Quo and requires the same float.
func TestChordLocusLengthAllowMatchesRawQuotient(t *testing.T) {
	t.Parallel()
	reference := func(speedUpper, dc, axialSpan, chordUpper float64) float64 {
		rdc, raxial := proofarith.FloatRat(dc), proofarith.FloatRat(axialSpan)
		zSpeed, exact := new(big.Rat).Quo(new(big.Rat).Abs(raxial), rdc).Float64()
		if !exact {
			zSpeed = math.Nextafter(zSpeed, math.Inf(1))
		}
		excess := proofbound.ProductUpper(dc, proofbound.Radius2D(speedUpper, zSpeed)) - chordUpper
		if excess <= 0 {
			return 0
		}
		return proofbound.UpRound(excess)
	}
	rng := rand.New(rand.NewPCG(9, 12))
	for range 500 {
		dc := math.Ldexp(rng.Float64()+0.5, rng.IntN(7)-3)
		axial := (rng.Float64() - 0.5) * 8
		if rng.IntN(3) == 0 {
			axial = float64(rng.IntN(9)-4) * dc
		}
		speed := rng.Float64() * 3
		got := proofbound.ChordLocusLengthAllow(speed, dc, axial, 0)
		require.Equal(t, reference(speed, dc, axial, 0), got, "speed %v dc %v axial %v", speed, dc, axial)
		require.Greater(t, got, 0.0)
	}
}

// TestCellChordPatchNormalLowerFlatSquare uses a flat square patch of side
// s, whose four corner area vectors all equal s² along z: the sum is 4s²,
// every corner dot is 4s⁴, and the proven lower bound on the normal's length
// is exactly s². The sides are powers of two and sums of them, so s² is
// representable and the rounded-down reading is exact.
func TestCellChordPatchNormalLowerFlatSquare(t *testing.T) {
	t.Parallel()
	for _, s := range []float64{0.5, 1, 1.5, 3, 7, 12.25} {
		got := proofbound.CellChordPatchNormalLower(
			r3.Vec{}, r3.Vec{X: s}, r3.Vec{Y: s}, r3.Vec{X: s, Y: s})
		require.Equal(t, s*s, got, "side %v", s)
	}
}

// TestExactFrameLocalRoundMatchesRawArithmetic compares ExactFrameLocalRound
// with a reference that forms (p - origin)·axis through big.Rat's own Sub, Mul
// and Add.
func TestExactFrameLocalRoundMatchesRawArithmetic(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(14, 15))
	vec := func(scale float64) r3.Vec {
		return r3.Vec{X: rng.NormFloat64() * scale, Y: rng.NormFloat64() * scale, Z: rng.NormFloat64() * scale}
	}
	rat := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	}
	for range 300 {
		frame, err := r3.NewFrame(vec(10), vec(1), vec(1))
		require.NoError(t, err)
		p, axis := vec(100), frame.U()
		held := frame.ToLocal(p).X
		pR, aR, oR := rat(p), rat(axis), rat(frame.Origin())
		exact := new(big.Rat)
		for i := range 3 {
			diff := new(big.Rat).Sub(pR[i], oR[i])
			exact.Add(exact, diff.Mul(diff, aR[i]))
		}
		want := proofarith.RationalFloatError(exact, held)
		require.Equal(t, want, proofbound.ExactFrameLocalRound(frame, p, axis, held))
	}
}

// TestLnIntervalEnclosesFloatLog checks LnInterval at points whose binary
// scaling lands the mantissa on each side of 1 (below 1 for 1/3 and 5/7), and
// at integers and dyadics, against math.Log within a few ulps.
func TestLnIntervalEnclosesFloatLog(t *testing.T) {
	t.Parallel()
	points := []*big.Rat{
		big.NewRat(1, 3), big.NewRat(5, 7), big.NewRat(3, 2), big.NewRat(3, 1), big.NewRat(10, 1),
		big.NewRat(1, 10), big.NewRat(7, 5), big.NewRat(1, 1000), big.NewRat(123456789, 1000),
		proofarith.FloatRat(0.75), proofarith.FloatRat(40.5), proofarith.FloatRat(1.0 / 1024),
	}
	for _, y := range points {
		f, _ := y.Float64()
		iv, ok := proofbound.LnInterval(proofbound.PointInterval(y))
		require.True(t, ok)
		lo, _ := iv.Lo.Float64()
		hi, _ := iv.Hi.Float64()
		require.InDelta(t, math.Log(f), lo, 1e-12, "ln %v lower", y)
		require.InDelta(t, math.Log(f), hi, 1e-12, "ln %v upper", y)
		require.LessOrEqual(t, lo, hi)
		require.LessOrEqual(t, lo, math.Log(f)+1e-15)
		require.GreaterOrEqual(t, hi, math.Log(f)-1e-15)
	}
}

// TestAtan2IntervalEnclosesFloatAtan2 checks Atan2Interval in all four
// quadrants for slopes on both sides of 1/2 and of 1, against math.Atan2.
func TestAtan2IntervalEnclosesFloatAtan2(t *testing.T) {
	t.Parallel()
	slopes := []*big.Rat{
		big.NewRat(3, 10), big.NewRat(1, 2), big.NewRat(7, 10), big.NewRat(1, 1), big.NewRat(3, 2),
		big.NewRat(3, 1), big.NewRat(1, 7), big.NewRat(20, 3),
	}
	for _, slope := range slopes {
		for _, sx := range []int64{1, -1} {
			for _, sy := range []int64{1, -1} {
				for _, flip := range []bool{false, true} {
					x, y := big.NewRat(sx, 1), new(big.Rat).Mul(slope, big.NewRat(sy, 1))
					if flip {
						x, y = y, x
					}
					fx, _ := x.Float64()
					fy, _ := y.Float64()
					iv := proofbound.Atan2Interval(y, x, false)
					lo, _ := iv.Lo.Float64()
					hi, _ := iv.Hi.Float64()
					want := math.Atan2(fy, fx)
					require.InDelta(t, want, lo, 1e-12, "atan2(%v, %v) lower", y, x)
					require.InDelta(t, want, hi, 1e-12, "atan2(%v, %v) upper", y, x)
					require.LessOrEqual(t, lo, want+1e-15)
					require.GreaterOrEqual(t, hi, want-1e-15)
				}
			}
		}
	}
}

// TestAtan2IntervalPicksTheOctantFormula pins the endpoints of the first
// quadrant to the formula each side of the diagonal uses: atan(y/x) when
// y <= x, and pi/2 - atan(x/y) otherwise. Both enclose the same angle, so only
// the exact endpoints tell them apart.
func TestAtan2IntervalPicksTheOctantFormula(t *testing.T) {
	t.Parallel()
	same := func(a, b proofbound.RatInterval) bool { return a.Lo.Cmp(b.Lo) == 0 && a.Hi.Cmp(b.Hi) == 0 }
	for _, c := range [][2]int64{{1, 3}, {2, 5}, {7, 9}, {1, 1}, {9, 7}, {5, 2}, {3, 1}, {40, 3}} {
		y, x := big.NewRat(c[0], 1), big.NewRat(c[1], 1)
		got := proofbound.Atan2Interval(y, x, false)
		var want proofbound.RatInterval
		if c[0] <= c[1] {
			want = proofbound.AtanPositiveInterval(new(big.Rat).Quo(y, x))
		} else {
			want = proofbound.IntervalSub(proofbound.HalfPiInterval(), proofbound.AtanPositiveInterval(new(big.Rat).Quo(x, y)))
		}
		require.True(t, same(want, got), "atan2(%v, %v)", y, x)
	}
}
