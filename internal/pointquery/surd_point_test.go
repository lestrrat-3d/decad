package pointquery

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestSurdSign checks both arms of surdSign: agreeing terms, and the squared
// comparison of the others, including a vanishing term and exact
// cancellation.
func TestSurdSign(t *testing.T) {
	t.Parallel()
	rat := func(n, d int64) *big.Rat { return big.NewRat(n, d) }
	cases := []struct {
		name    string
		a, b, r *big.Rat
		want    int
	}{
		{name: "no root term", a: rat(-5, 1), b: rat(3, 1), r: rat(0, 1), want: -1},
		{name: "no linear term", a: rat(2, 1), b: rat(0, 1), r: rat(7, 1), want: 1},
		{name: "all zero", a: rat(0, 1), b: rat(0, 1), r: rat(0, 1), want: 0},
		{name: "zero constant", a: rat(0, 1), b: rat(-1, 3), r: rat(2, 1), want: -1},
		{name: "agreeing terms", a: rat(1, 1), b: rat(1, 1), r: rat(4, 1), want: 1},
		{name: "agreeing terms of equal magnitude", a: rat(2, 1), b: rat(1, 1), r: rat(4, 1), want: 1},
		{name: "constant wins", a: rat(-3, 1), b: rat(1, 1), r: rat(4, 1), want: -1},
		{name: "root term wins", a: rat(-1, 1), b: rat(1, 1), r: rat(4, 1), want: 1},
		{name: "exact cancellation", a: rat(2, 1), b: rat(-1, 1), r: rat(4, 1), want: 0},
		{name: "irrational root", a: rat(1, 1), b: rat(-1, 1), r: rat(2, 1), want: -1},
		{name: "irrational root below", a: rat(-7, 5), b: rat(1, 1), r: rat(2, 1), want: 1},
	}
	for _, tc := range cases {
		require.Equal(t, tc.want, surdSign(tc.a, tc.b, tc.r), tc.name)
	}
}

// TestSurdPointMatchesRational reads surdPoints whose root is rational, s =
// ±3, against the planePoint at the same coordinates: every region test must
// answer alike.
func TestSurdPointMatchesRational(t *testing.T) {
	t.Parallel()
	rat := func(n, d int64) *big.Rat { return big.NewRat(n, d) }
	p := surdPoint{u0: rat(1, 2), u1: rat(-1, 3), v0: rat(2, 1), v1: rat(1, 6), r: rat(9, 1)}
	// s = √9 = 3: the plus point is (−1/2, 5/2), the mirrored one (3/2, 3/2).
	points := map[string]struct {
		q    surdPoint
		want planePoint
	}{
		"plus":     {q: p, want: planePoint{u: rat(-1, 2), v: rat(5, 2)}},
		"mirrored": {q: p.mirrored(), want: planePoint{u: rat(3, 2), v: rat(3, 2)}},
	}
	a, b := planePoint{u: rat(-1, 1), v: rat(2, 1)}, planePoint{u: rat(0, 1), v: rat(3, 1)}
	c := planePoint{u: rat(0, 1), v: rat(2, 1)}
	for name, point := range points {
		q, want := point.q, point.want
		require.Equal(t, want.sideOf(a, b), q.sideOf(a, b), name)
		require.Equal(t, want.sideOf(b, c), q.sideOf(b, c), name)
		require.Equal(t, want.spanOf(a, b), q.spanOf(a, b), name)
		require.Equal(t, want.spanOf(b, c), q.spanOf(b, c), name)
		for _, v := range []*big.Rat{rat(5, 2), rat(3, 2), rat(2, 1)} {
			require.Equal(t, want.heightOf(v), q.heightOf(v), name)
		}
		for _, r2 := range []*big.Rat{rat(1, 2), rat(5, 2), rat(9, 2), rat(1, 1)} {
			require.Equal(t, want.radialOf(c, r2), q.radialOf(c, r2), name)
		}
	}
	// The plus point lies on the line through a and b, inside the segment,
	// and on the circle of squared radius 1/2 about c: zeros are exact.
	require.Equal(t, 0, p.sideOf(a, b))
	require.Equal(t, -1, p.spanOf(a, b))
	require.Equal(t, 0, p.radialOf(c, rat(1, 2)))
}
