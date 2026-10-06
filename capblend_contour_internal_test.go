package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ratSquaredDistance3Oracle and ratStraightEdgeBound are ratSquaredDistance3
// and straightEdgeBound as they were computed over big.Rat before the dyadic
// rewrite, kept verbatim as the oracles below.

func ratSquaredDistance3Oracle(a0, a1, a2, b0, b1, b2 float64) *big.Rat {
	sum := new(big.Rat)
	for _, pair := range [3][2]float64{{a0, b0}, {a1, b1}, {a2, b2}} {
		x, y := proofarith.FloatRat(pair[0]), proofarith.FloatRat(pair[1])
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
	return proofbound.AbsSumUpper(append([]float64{ratSqrtIntervalError(squared, held)}, endpointDeltas...)...)
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
			tangent := proofarith.DvSub(proofarith.DyVec(b), proofarith.DyVec(a))
			require.Zero(t, proofarith.DyCmp(proofarith.DvDot(tangent, tangent), squared), "the tangent's dot product of %v and %v", a, b)
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

// TestStraightEdgeBoundExactSquareSkipsTheBracket pins straightEdgeBound's
// exact-square shortcut to the rational computation it skips. A non-negative
// held length whose exact square is the squared length must read a zero
// square-root term, where the oracle may read zero or at most one ulp; every
// other held length must read the oracle bit for bit. Pairs sweep
// axis-aligned full-width lengths, whose squares need up to 106 bits, scaled
// Pythagorean triples, and general pairs whose Hypot is usually not exact,
// each read at the held length, one ulp above it, and its negation.
//
// Shown to fail: dropping the dySquareEquals test (so every non-negative held
// length reads a zero square-root term) turns the 1+2^-60 fixture, the random
// sweep and TestStraightEdgeBoundDyadicMatchesRational red. Dropping the
// held < 0 test turns the negated fixture and the random sweep's negated held
// lengths red: their square matches while the length is off by twice its
// magnitude.
func TestStraightEdgeBoundExactSquareSkipsTheBracket(t *testing.T) {
	t.Parallel()
	var shortcut, wideShortcut, bracket int
	check := func(t *testing.T, a, b r3.Vec, held, delta float64) {
		t.Helper()
		want := ratSquaredDistance3Oracle(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
		squared, ok := dySquaredDistance3(a.X, a.Y, a.Z, b.X, b.Y, b.Z)
		got := straightEdgeBound(held, squared, ok, delta, delta)
		old := ratStraightEdgeBound(held, want, delta, delta)
		if want != nil && held >= 0 && new(big.Rat).Mul(proofarith.FloatRat(held), proofarith.FloatRat(held)).Cmp(want) == 0 {
			requireSameFloatBits(t, proofbound.AbsSumUpper(0, delta, delta), got,
				"an exact length %v from %v to %v must read no square-root term", held, a, b)
			require.LessOrEqual(t, old, proofbound.AbsSumUpper(proofbound.UpRound(proofbound.UlpOf(held)), delta, delta),
				"the oracle may miss an exact length %v by one ulp at most", held)
			shortcut++
			if !isExactFloat(want) {
				wideShortcut++
			}
			return
		}
		requireSameFloatBits(t, old, got, "bound of %v to %v at %v", a, b, held)
		if want != nil && !proofbound.IsNonFinite(held) {
			require.Positive(t, got, "%v is not the length from %v to %v and must not read exact", held, a, b)
			bracket++
		}
	}

	t.Run("fixtures", func(t *testing.T) {
		// Hypot(1, 2^-30) rounds to 1, but the squared length is 1+2^-60:
		// only the exact comparison keeps the held 1 from reading exact.
		near := r3.Vec{X: 1, Y: 0x1p-30}
		require.Equal(t, 1.0, near.Len(), "the fixture's held length must round to 1")
		check(t, near, r3.Vec{}, near.Len(), 0)

		// -5 squares to 25 exactly, and the true length is 5, ten away.
		pyth := r3.Vec{X: 3, Y: 4}
		check(t, pyth, r3.Vec{}, -5, 0)
		require.GreaterOrEqual(t, straightEdgeBound(-5, proofarith.DyInt(25), true), 10.0,
			"a negated length is off by twice its magnitude")
		check(t, pyth, r3.Vec{}, 5, 1e-9)
	})

	t.Run("random", func(t *testing.T) {
		rng := rand.New(rand.NewPCG(31, 37))
		triples := [][3]float64{{3, 4, 5}, {5, 12, 13}, {8, 15, 17}, {1, 2, 3}, {2, 3, 6}}
		for range 10000 {
			delta := 0.0
			if rng.IntN(2) == 0 {
				delta = math.Abs(rng.NormFloat64()) * 1e-9
			}
			origin := r3.Vec{X: float64(rng.IntN(200) - 100), Y: float64(rng.IntN(200) - 100), Z: rng.NormFloat64()}

			axis := math.Ldexp(1+rng.Float64(), rng.IntN(200)-100)
			pairs := [][2]r3.Vec{{{X: axis, Z: origin.Z}, {Z: origin.Z}}}

			// {1,2,3} is not a triple and {2,3,6} is the 3D one (4+9+36 = 49).
			tri := triples[rng.IntN(len(triples))]
			scale := math.Ldexp(1, rng.IntN(40)-20)
			pairs = append(pairs, [2]r3.Vec{origin, origin.Add(r3.Vec{X: tri[0] * scale, Y: tri[1] * scale, Z: 0})})
			if tri == triples[4] {
				pairs = append(pairs, [2]r3.Vec{{}, {X: 2 * scale, Y: 3 * scale, Z: 6 * scale}})
			}

			pairs = append(pairs, [2]r3.Vec{
				{X: rng.NormFloat64() * 100, Y: rng.NormFloat64() * 100, Z: rng.NormFloat64() * 100},
				{X: rng.NormFloat64() * 100, Y: rng.NormFloat64() * 100, Z: rng.NormFloat64() * 100},
			})
			for _, pair := range pairs {
				held := pair[0].Sub(pair[1]).Len()
				for _, h := range []float64{held, math.Nextafter(held, math.Inf(1)), -held} {
					check(t, pair[0], pair[1], h, delta)
				}
			}
		}
	})

	require.Positive(t, shortcut, "the sweep must reach the shortcut")
	require.Positive(t, wideShortcut, "the sweep must reach the shortcut on a squared length past 53 bits")
	require.Positive(t, bracket, "the sweep must reach the bracket")
}

// isExactFloat reports whether q is exactly a float64.
func isExactFloat(q *big.Rat) bool {
	_, exact := q.Float64()
	return exact
}
