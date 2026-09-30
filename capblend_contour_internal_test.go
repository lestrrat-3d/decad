package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ratSquaredDistance3Oracle and ratStraightEdgeBound are ratSquaredDistance3
// and straightEdgeBound as they were computed over big.Rat before the dyadic
// rewrite, kept verbatim as the oracles below.

func ratSquaredDistance3Oracle(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	sum := new(big.Rat)
	for _, pair := range [3][2]float64{{a0, b0}, {a1, b1}, {a2, b2}} {
		x, y := floatRat(pair[0]), floatRat(pair[1])
		if x == nil || y == nil {
			return nil
		}
		diff := new(big.Rat).Sub(x, y)
		sum.Add(sum, diff.Mul(diff, diff))
	}
	return sum
}

func ratStraightEdgeBound(held float64, squared *big.Rat, endpointDeltas ...float64) float64 {
	if squared == nil {
		return math.Inf(1)
	}
	return absSumUpper(append([]float64{ratSqrtIntervalError(squared, held)}, endpointDeltas...)...)
}

// TestStraightEdgeBoundDyadicMatchesRational pins straightEdgeBound and its
// squared length, computed over dyadics, to the big.Rat computation they
// replaced, bit for bit, on random 3D point pairs of mixed magnitudes with a
// non-finite coordinate now and then. It reads every caller's spelling: the
// general three-coordinate form (capSlantEdge, loftEdgeLength,
// compositeLineSweepSpan), capEdgeLengthBound's plane form, the big.Rat that
// ratSquaredDistance3 still hands its dividing callers, and the dot product of
// the exact tangent validateStraightSweepPath substitutes for the distance.
//
// Shown to fail: dropping dySquaredDistance3's third coordinate turns the
// squared-length leg red; answering a finite bound where the squared length
// could not be stated turns a non-finite pair at held length 1 red; passing
// capEdgeLengthBound's delta for one endpoint only turns the plane leg red;
// publishing through dyFloatUp instead of dyNearestUp in dyRoundedFloatError
// turns an inexact gap at held length 1 red. Without the held length of 1 the
// last two stay green: a held length computed from a non-finite pair is itself
// non-finite and refuses on its own, and a held length's gap from a bracket
// end of its own magnitude is exact.
func TestStraightEdgeBoundDyadicMatchesRational(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(17, 19))
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
	for i := range 20000 {
		a := r3.Vec{X: coord(), Y: coord(), Z: coord()}
		b := r3.Vec{X: coord(), Y: coord(), Z: coord()}
		if i%10 == 0 {
			b.Z = a.Z
		}
		if i%97 == 0 {
			a.Y = math.NaN()
		}
		if i%89 == 0 {
			b.X = math.Inf(-1)
		}
		held := a.Sub(b).Len()
		delta := 0.0
		if rng.IntN(2) == 0 {
			delta = math.Abs(rng.NormFloat64()) * 1e-9
		}

		want := ratSquaredDistance3Oracle(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
		squared, ok := dySquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
		require.Equal(t, want != nil, ok, "whether %v and %v state a squared distance", a, b)
		gotRat := ratSquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
		if want == nil {
			require.Nil(t, gotRat, "%v and %v", a, b)
		} else {
			require.Equal(t, want.Num().String(), gotRat.Num().String(), "squared numerator of %v and %v", a, b)
			require.Equal(t, want.Denom().String(), gotRat.Denom().String(), "squared denominator of %v and %v", a, b)
			tangent := dvSub(dyVec(b), dyVec(a))
			require.Zero(t, dyCmp(dvDot(tangent, tangent), squared), "the tangent's dot product of %v and %v", a, b)
		}

		// A held length of 1 stands apart from the pair: against a non-finite
		// pair it is the only finite held value, so it is what reaches the
		// refusal on an unstated squared length, and against a length far
		// from 1 its gap from the bracket needs more than 53 bits, which is
		// what reaches the inexact rounding path.
		for _, h := range []float64{held, math.Nextafter(held, math.Inf(1)), 1} {
			requireSameFloatBits(t, ratStraightEdgeBound(h, want), straightEdgeBound(h, squared, ok),
				"bound of %v to %v at %v", a, b, h)
			requireSameFloatBits(t, ratStraightEdgeBound(h, want, delta, delta), straightEdgeBound(h, squared, ok, delta, delta),
				"displaced bound of %v to %v at %v", a, b, h)
		}

		end, start := Point2{U: a.X, V: a.Y}, Point2{U: b.X, V: b.Y}
		planeHeld := math.Hypot(end.U-start.U, end.V-start.V)
		requireSameFloatBits(t,
			ratStraightEdgeBound(planeHeld, ratSquaredDistance3Oracle(end.U, end.V, 0, start.U, start.V, 0), delta, delta),
			capEdgeLengthBound(planeHeld, end, start, delta),
			"plane bound of %v to %v", end, start)
	}
}
