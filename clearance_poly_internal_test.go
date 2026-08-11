package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRatPolyOfRejectsNonFiniteCoefficient(t *testing.T) {
	for _, coeff := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		p, ok := ratPolyOf(1, coeff, 2)
		require.False(t, ok)
		require.Nil(t, p)
	}
}

func TestLineCircleBracketsRejectsNonFinitePolynomial(t *testing.T) {
	cp := circleParam{
		c: [3]float64{math.MaxFloat64, 0, 0},
		u: [3]float64{1, 0, 0},
		v: [3]float64{0, 1, 0},
		r: 1,
	}
	brackets, nonConstant, err := lineCircleBracketsContext(
		t.Context(),
		cp,
		[3]float64{},
		[3]float64{0, 0, 1},
		1e-9,
	)
	require.ErrorIs(t, err, errNonFiniteClearancePolynomial)
	require.False(t, nonConstant)
	require.Empty(t, brackets)
}

func TestLineCircleBracketsAcceptsFinitePolynomial(t *testing.T) {
	cp := circleParam{
		c: [3]float64{3, 0, 0},
		u: [3]float64{1, 0, 0},
		v: [3]float64{0, 1, 0},
		r: 1,
	}
	brackets, nonConstant, err := lineCircleBracketsContext(
		t.Context(),
		cp,
		[3]float64{},
		[3]float64{0, 0, 1},
		1e-9,
	)
	require.NoError(t, err)
	require.True(t, nonConstant)
	require.NotEmpty(t, brackets)
}

func TestTorusCrossingsRejectsNonFinitePolynomial(t *testing.T) {
	face := cFace{
		kind:   ckTorus,
		axis:   r3.NewVec(0, 0, 1),
		major:  math.MaxFloat64,
		radius: 1,
	}
	count, decided, err := face.torusCrossings(
		t.Context(),
		r3.Vec{},
		r3.NewVec(1, 0, 0),
		1e-9,
	)
	require.NoError(t, err)
	require.False(t, decided)
	require.Zero(t, count)
}

// p8SpineFixture and p8SpineFixture2 are the P8 cell's own circles, chosen
// generic and non-axis-aligned so circleCircleBracketsContext's chart never
// hits a degenerate branch. The same pair backs BenchmarkCircleCircleBrackets
// below, so the benchmark measures the same polynomial the correctness tests
// exercise.
var (
	p8SpineFixture = circleParam{
		c: [3]float64{0.3, -0.7, 1.1},
		u: [3]float64{0.8017837257372732, 0.5345224838248488, 0.2672612419124244},
		v: [3]float64{-0.3563483225498992, 0.7126966450997984, -0.6047820818352262},
		r: 5.25,
	}
	p8SpineFixture2 = circleParam{
		c: [3]float64{12.7, 3.3, -4.9},
		u: [3]float64{0.8, 0.6, 0.0},
		v: [3]float64{-0.42426406871192851, 0.56568542494923812, 0.70710678118654746},
		r: 3.125,
	}
	p8SpineNormal2 = [3]float64{0.4242640687119285, -0.5656854249492381, 0.7071067811865476}
)

// p8SpineChain rebuilds circleCircleBracketsContext's own degree-8 spine
// polynomial and its Sturm chain, by the same csPoly steps that function
// runs internally, so the exactness proof below runs against the actual
// production P8 chain rather than a stand-in.
func p8SpineChain(t *testing.T) []ratPoly {
	t.Helper()
	c1, c2, n2 := p8SpineFixture, p8SpineFixture2, p8SpineNormal2
	dot := func(x, y [3]float64) float64 { return x[0]*y[0] + x[1]*y[1] + x[2]*y[2] }
	m := [3]float64{c1.c[0] - c2.c[0], c1.c[1] - c2.c[1], c1.c[2] - c2.c[2]}
	wConst, ok := csConst(dot(m, m) + c1.r*c1.r)
	require.True(t, ok)
	wLin, ok := csLin(0, 2*c1.r*dot(m, c1.u), 2*c1.r*dot(m, c1.v))
	require.True(t, ok)
	h, ok := csLin(dot(m, n2), c1.r*dot(c1.u, n2), c1.r*dot(c1.v, n2))
	require.True(t, ok)
	w := csAdd(wConst, wLin)
	sp := csSub(w, csMul(h, h))
	wd := csDerivTheta(w)
	sd := csDerivTheta(sp)
	r2, ok := csConst(c2.r * c2.r)
	require.True(t, ok)
	f := csSub(csMul(sp, csMul(wd, wd)), csMul(r2, csMul(sd, sd)))

	p := rpSquareFree(rpTrim(csToT(f)))
	require.Equal(t, 8, rpDeg(p), "the P8 fixture must isolate the degree-8 spine polynomial")
	return sturmChain(p)
}

// TestSturmVarAtMatchesRationalHornerSigns is the exactness proof behind the
// integer sign path: for the real production P8 chain plus a handful of
// small hand-written chains (a zero coefficient, a constant last member, and
// the empty ratPoly{}), every chain member's sign under ipSign must match
// rpEval(...).Sign() at points that are deliberately not all dyadic — the
// entry's claim that evaluation points are dyadic did not survive
// investigation, so this test does not assume it — and the sign-change count
// sturmVarAt reports over the integer chain must match the count the old
// rpEval-based algorithm would have computed from the very same signs.
func TestSturmVarAtMatchesRationalHornerSigns(t *testing.T) {
	rationalRoots := ratPoly{big.NewRat(-6, 1), big.NewRat(11, 1), big.NewRat(-6, 1), big.NewRat(1, 1)} // (x-1)(x-2)(x-3)
	zeroCoeff := ratPoly{big.NewRat(6, 1), big.NewRat(-5, 1), new(big.Rat), big.NewRat(1, 1)}           // x^3 - 5x + 6, x^2 coefficient is zero
	constLastMember := ratPoly{big.NewRat(2, 1), big.NewRat(3, 1)}                                      // 2 + 3x; its chain's last member is the constant 3

	cases := []struct {
		name  string
		chain []ratPoly
	}{
		{"p8", p8SpineChain(t)},
		{"rationalRoots", sturmChain(rationalRoots)},
		{"zeroCoeff", sturmChain(zeroCoeff)},
		{"constLastMember", sturmChain(constLastMember)},
		{"empty", sturmChain(ratPoly{})},
	}

	points := []*big.Rat{
		big.NewRat(-7, 3),
		big.NewRat(22, 7),
		big.NewRat(1, 1000003),
		big.NewRat(0, 1),
		big.NewRat(1, 1),  // an exact root of rationalRoots: (x-1)(x-2)(x-3)
		big.NewRat(2, 1),  // ditto
		big.NewRat(3, 1),  // ditto
		big.NewRat(3, 8),  // dyadic
		big.NewRat(-3, 8), // dyadic
		big.NewRat(5, 4),  // dyadic
		big.NewRat(-11, 13),
		big.NewRat(100, 1),
	}
	require.GreaterOrEqual(t, len(points), 12, "the design calls for at least twelve evaluation points")

	for _, c := range cases {
		chainInt := newSturmChainInt(c.chain)
		require.Len(t, chainInt, len(c.chain))
		for _, x := range points {
			num, den := x.Num(), x.Denom()
			wantVars, prevWant := 0, 0
			for i, p := range c.chain {
				wantSign := rpEval(p, x).Sign()
				gotSign := ipSign(chainInt[i], num, den, new(big.Int), new(big.Int), new(big.Int))
				require.Equalf(t, wantSign, gotSign, "chain %s member %d at x=%s", c.name, i, x.RatString())
				if wantSign == 0 {
					continue
				}
				if prevWant != 0 && wantSign != prevWant {
					wantVars++
				}
				prevWant = wantSign
			}
			require.Equalf(t, wantVars, sturmVarAt(chainInt, x), "chain %s sign-change count at x=%s", c.name, x.RatString())
		}
	}
}

// TestClearDenomsIsAPositiveRescaling proves clearDenoms only ever rescales
// by one positive rational shared across every coefficient — the single
// admissible transformation the design permits (docs/clearance-design.md
// §4's "cannot lie" guarantee).
func TestClearDenomsIsAPositiveRescaling(t *testing.T) {
	p := ratPoly{big.NewRat(1, 3), big.NewRat(-2, 7), big.NewRat(5, 1), big.NewRat(-11, 13)}
	out := clearDenoms(p)
	require.Len(t, out, len(p))

	for i := range p {
		for j := range p {
			if p[j].Sign() == 0 {
				continue
			}
			want := new(big.Rat).Quo(p[i], p[j])
			got := new(big.Rat).SetFrac(out[i], out[j])
			require.Equalf(t, want.RatString(), got.RatString(), "out[%d]/out[%d] must equal p[%d]/p[%d]", i, j, i, j)
		}
	}

	var scale *big.Rat
	for i, c := range p {
		if c.Sign() == 0 {
			continue
		}
		ratio := new(big.Rat).Quo(new(big.Rat).SetInt(out[i]), c)
		if scale == nil {
			scale = ratio
			require.Positive(t, scale.Sign(), "the common rescaling factor D must be positive")
			continue
		}
		require.Equalf(t, scale.RatString(), ratio.RatString(), "out[%d]/p[%d] must equal the same D as every other coefficient", i, i)
	}
	require.NotNil(t, scale, "fixture must have at least one nonzero coefficient")
}

// TestRefineRootCachedVariationMatchesRecount proves the cached varLo in
// rpRefineRootContext takes the identical branch the original recomputation
// did: it isolates and refines x^2 - 2's positive root and checks the
// straddle (rpEval's sign flips low-to-high across the narrowed interval)
// and the numeric answer (√2) the branch history must have produced.
func TestRefineRootCachedVariationMatchesRecount(t *testing.T) {
	p := ratPoly{big.NewRat(-2, 1), new(big.Rat), big.NewRat(1, 1)}
	chain := newSturmChainInt(sturmChain(p))
	ivs, err := rpIsolateRootsContext(t.Context(), p, chain)
	require.NoError(t, err)
	require.NotEmpty(t, ivs)

	var iv ratIv
	found := false
	for _, cand := range ivs {
		lo, _ := cand.lo.Float64()
		if lo >= 0 {
			iv = cand
			found = true
		}
	}
	require.True(t, found, "must isolate the positive root of x^2 - 2")

	narrow := func(lo, hi float64) bool { return hi-lo <= 1e-12 }
	iv, err = rpRefineRootContext(t.Context(), chain, iv, narrow)
	require.NoError(t, err)

	loF, _ := iv.lo.Float64()
	hiF, _ := iv.hi.Float64()
	require.LessOrEqual(t, hiF-loF, 1e-12)
	require.Negative(t, rpEval(p, iv.lo).Sign(), "the interval must still straddle sqrt(2) from below")
	require.Positive(t, rpEval(p, iv.hi).Sign(), "the interval must still straddle sqrt(2) from above")
	require.InDelta(t, math.Sqrt2, (loF+hiF)/2, 1e-12)
}

// BenchmarkLineCircleBrackets measures the P4 cell end to end
// (lineCircleBracketsContext): chart construction, Sturm chain, isolation
// and refinement of every root. The fixture is a generic, non-axis-aligned
// circle and line chosen to return 4 brackets without hitting a degenerate
// branch.
func BenchmarkLineCircleBrackets(b *testing.B) {
	cp := circleParam{
		c: [3]float64{0.3, -0.7, 1.1},
		u: [3]float64{0.8017837257372732, 0.5345224838248488, 0.2672612419124244},
		v: [3]float64{-0.3563483225498992, 0.7126966450997984, -0.6047820818352262},
		r: 5.25,
	}
	a := [3]float64{7.3, -2.1, 4.4}
	d := [3]float64{0.2672612419124244, 0.5345224838248488, 0.8017837257372732}
	ctx := context.Background()

	b.ReportAllocs()
	for b.Loop() {
		brackets, ok, err := lineCircleBracketsContext(ctx, cp, a, d, 1e-9)
		if err != nil || !ok || len(brackets) == 0 {
			b.Fatalf("lineCircleBracketsContext: brackets=%d ok=%v err=%v", len(brackets), ok, err)
		}
	}
}

// BenchmarkCircleCircleBrackets measures the P8 cell end to end
// (circleCircleBracketsContext), the degree-8 spine problem and the
// dominant cost this change targets. The fixture is the same generic pair
// p8SpineChain rebuilds above, so the correctness proof and the benchmark
// exercise the same polynomial.
func BenchmarkCircleCircleBrackets(b *testing.B) {
	c1, c2, n2 := p8SpineFixture, p8SpineFixture2, p8SpineNormal2
	ctx := context.Background()

	b.ReportAllocs()
	for b.Loop() {
		brackets, ok, err := circleCircleBracketsContext(ctx, c1, c2, n2, 1e-9)
		if err != nil || !ok || len(brackets) == 0 {
			b.Fatalf("circleCircleBracketsContext: brackets=%d ok=%v err=%v", len(brackets), ok, err)
		}
	}
}
