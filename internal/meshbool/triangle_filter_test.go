package meshbool_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFloatIntervalArithmeticSanity(t *testing.T) {
	t.Parallel()
	a := meshbool.FloatInterval{Lo: 1, Hi: 2}
	b := meshbool.FloatInterval{Lo: 3, Hi: 4}

	require.True(t, a.Disjoint(b))
	require.False(t, a.Disjoint(meshbool.FloatInterval{Lo: 1.5, Hi: 5}))
	require.True(t, a.Contains(1.5))
	require.False(t, a.Contains(2.5))

	sum := a.Add(b)
	require.LessOrEqual(t, sum.Lo, 4.0)
	require.GreaterOrEqual(t, sum.Hi, 6.0)

	diff := b.Sub(a)
	require.LessOrEqual(t, diff.Lo, 1.0)
	require.GreaterOrEqual(t, diff.Hi, 3.0)

	prod := a.Mul(b)
	require.LessOrEqual(t, prod.Lo, 3.0)
	require.GreaterOrEqual(t, prod.Hi, 8.0)
	for _, tc := range []struct {
		name     string
		a, b     meshbool.FloatInterval
		wantLo   float64
		wantHigh float64
	}{
		{name: "positive-positive", a: meshbool.FloatInterval{Lo: 1, Hi: 2}, b: meshbool.FloatInterval{Lo: 3, Hi: 4}, wantLo: 3, wantHigh: 8},
		{name: "positive-negative", a: meshbool.FloatInterval{Lo: 1, Hi: 2}, b: meshbool.FloatInterval{Lo: -4, Hi: -3}, wantLo: -8, wantHigh: -3},
		{name: "positive-spanning", a: meshbool.FloatInterval{Lo: 1, Hi: 2}, b: meshbool.FloatInterval{Lo: -4, Hi: 3}, wantLo: -8, wantHigh: 6},
		{name: "negative-positive", a: meshbool.FloatInterval{Lo: -2, Hi: -1}, b: meshbool.FloatInterval{Lo: 3, Hi: 4}, wantLo: -8, wantHigh: -3},
		{name: "negative-negative", a: meshbool.FloatInterval{Lo: -2, Hi: -1}, b: meshbool.FloatInterval{Lo: -4, Hi: -3}, wantLo: 3, wantHigh: 8},
		{name: "negative-spanning", a: meshbool.FloatInterval{Lo: -2, Hi: -1}, b: meshbool.FloatInterval{Lo: -4, Hi: 3}, wantLo: -6, wantHigh: 8},
		{name: "spanning-positive", a: meshbool.FloatInterval{Lo: -2, Hi: 1}, b: meshbool.FloatInterval{Lo: 3, Hi: 4}, wantLo: -8, wantHigh: 4},
		{name: "spanning-negative", a: meshbool.FloatInterval{Lo: -2, Hi: 1}, b: meshbool.FloatInterval{Lo: -4, Hi: -3}, wantLo: -4, wantHigh: 8},
		{name: "spanning-spanning", a: meshbool.FloatInterval{Lo: -2, Hi: 1}, b: meshbool.FloatInterval{Lo: -4, Hi: 3}, wantLo: -6, wantHigh: 8},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got := tc.a.Mul(tc.b)
			require.LessOrEqual(t, got.Lo, tc.wantLo)
			require.GreaterOrEqual(t, got.Hi, tc.wantHigh)
		})
	}

	quot := b.Div(a)
	require.LessOrEqual(t, quot.Lo, 1.5)
	require.GreaterOrEqual(t, quot.Hi, 4.0)

	straddling := meshbool.FloatInterval{Lo: -1, Hi: 1}
	require.True(t, a.Div(straddling).Abstains(),
		`a denominator interval containing zero must abstain, never resolve`)

	require.True(t, meshbool.FivPoint(math.NaN()).Abstains())
	require.True(t, meshbool.FivPoint(math.Inf(1)).Abstains())
	require.True(t, meshbool.FloatInterval{Lo: math.Inf(1), Hi: math.Inf(1)}.Add(a).Abstains())
}

func TestFloatIntervalNextMatchesMath(t *testing.T) {
	t.Parallel()
	values := []float64{
		-math.MaxFloat64,
		-1,
		-math.SmallestNonzeroFloat64,
		math.Copysign(0, -1),
		0,
		math.SmallestNonzeroFloat64,
		1,
		math.MaxFloat64,
	}
	rng := rand.New(rand.NewPCG(3, 4))
	for len(values) < 10_000 {
		bits := rng.Uint64()
		if bits&(uint64(0x7ff)<<52) == uint64(0x7ff)<<52 {
			continue
		}
		values = append(values, math.Float64frombits(bits))
	}
	for _, value := range values {
		wantDown := math.Float64bits(math.Nextafter(value, math.Inf(-1)))
		wantUp := math.Float64bits(math.Nextafter(value, math.Inf(1)))
		require.Equal(t, wantDown, math.Float64bits(meshbool.FivNextDown(value)))
		require.Equal(t, wantUp, math.Float64bits(meshbool.FivNextUp(value)))
	}
}

// TestTriTriIntervalEnclosesExact is fu158 task 1's proof obligation: for a
// few thousand pseudo-random float64 triples, the interval evaluation of
// a*b - c*d and of a/(a-b) must enclose the big.Rat value of the very same
// expression — checked as a big.Rat comparison, never a float one, so the
// test cannot pass by the same rounding the filter has to survive.
func TestTriTriIntervalEnclosesExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	sample := func() float64 { return (rng.Float64()*2 - 1) * 1000 }

	for range 5000 {
		a, b, c, d := sample(), sample(), sample(), sample()

		got := meshbool.FivPoint(a).Mul(meshbool.FivPoint(b)).Sub(meshbool.FivPoint(c).Mul(meshbool.FivPoint(d)))
		exact := new(big.Rat).Sub(
			new(big.Rat).Mul(polynomial.MustRatOf(a), polynomial.MustRatOf(b)),
			new(big.Rat).Mul(polynomial.MustRatOf(c), polynomial.MustRatOf(d)),
		)
		require.False(t, got.Abstains(), `a finite product/difference must never abstain`)
		require.LessOrEqual(t, polynomial.MustRatOf(got.Lo).Cmp(exact), 0)
		require.GreaterOrEqual(t, polynomial.MustRatOf(got.Hi).Cmp(exact), 0)

		if a == b {
			continue // a/(a-b) is undefined; the filter's own div guard covers it
		}
		gotDiv := meshbool.FivPoint(a).Div(meshbool.FivPoint(a).Sub(meshbool.FivPoint(b)))
		if gotDiv.Abstains() {
			continue // an abstained bound trivially encloses every value
		}
		exactDiv := new(big.Rat).Quo(polynomial.MustRatOf(a), new(big.Rat).Sub(polynomial.MustRatOf(a), polynomial.MustRatOf(b)))
		require.LessOrEqual(t, polynomial.MustRatOf(gotDiv.Lo).Cmp(exactDiv), 0)
		require.GreaterOrEqual(t, polynomial.MustRatOf(gotDiv.Hi).Cmp(exactDiv), 0)
	}
}

// TestHomogeneousProjectedCrossMatchesRational verifies that the loft audit's
// cached homogeneous projection path returns the same exact orientation value
// as the existing rational projection path on every coordinate-plane choice.
func TestHomogeneousProjectedCrossMatchesRational(t *testing.T) {
	t.Parallel()
	points := [3]proof.Xpt{
		proof.XptOf(r3.NewVec(0.1, 2.0, -3.5)),
		proof.XptOf(r3.NewVec(4.25, -1.5, 0.75)),
		proof.XptOf(r3.NewVec(-2.0, 3.125, 5.5)),
	}
	for _, axes := range [][2]int{{0, 1}, {2, 0}, {1, 2}} {
		hom := [3]meshbool.Xp2{
			meshbool.NewXP2FromXpt(points[0], axes[0], axes[1]),
			meshbool.NewXP2FromXpt(points[1], axes[0], axes[1]),
			meshbool.NewXP2FromXpt(points[2], axes[0], axes[1]),
		}
		rat := [3]meshbool.Xp2{
			meshbool.NewXP2(meshbool.RatCoordOf(points[0], axes[0]), meshbool.RatCoordOf(points[0], axes[1])),
			meshbool.NewXP2(meshbool.RatCoordOf(points[1], axes[0]), meshbool.RatCoordOf(points[1], axes[1])),
			meshbool.NewXP2(meshbool.RatCoordOf(points[2], axes[0]), meshbool.RatCoordOf(points[2], axes[1])),
		}
		want := meshbool.Cross2x(rat[0], rat[1], rat[2])
		got := meshbool.Cross2x(hom[0], hom[1], hom[2])
		require.Zero(t, got.Cmp(want), "projection axes %v must preserve the exact cross product", axes)
		require.Equal(t, meshbool.Cross2xSign(rat[0], rat[1], rat[2]), meshbool.Cross2xSign(hom[0], hom[1], hom[2]))
	}
}

// referenceMeshParityContext is the PRE-OPTIMIZATION reference copy of
// meshbool.MeshParityContext, captured verbatim before the query projection was hoisted
// out of the triangle loop: same arithmetic, same branch order, same returns,
// and the same per-triangle meshbool.NewXP2 construction the hoist removes.
//
// It is frozen on purpose. A later production edit must NEVER be mirrored into
// this function — the whole value of the comparison is that the reference
// still states the behaviour production had before the change. If production
// and this function disagree, production changed a classification and the
// change is wrong, not this copy.
