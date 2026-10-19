package loftcylinder

import (
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

// TestLoftCylinderEdgesMeetXPAgreesWithRational drives the float-filtered
// edge contact predicate and the rational one over segment pairs that touch
// at an endpoint, cross, overlap collinearly, and stay disjoint with
// overlapping bounding boxes.
func TestLoftCylinderEdgesMeetXPAgreesWithRational(t *testing.T) {
	t.Parallel()
	pt := func(x, y float64) XY {
		return XY{new(big.Rat).SetFloat64(x), new(big.Rat).SetFloat64(y)}
	}
	ratPt := func(x, y *big.Rat) XY { return XY{x, y} }
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 100))
	onePlusTiny := new(big.Rat).Add(big.NewRat(1, 1), tiny)
	for _, tc := range []struct {
		name       string
		a, b, c, d XY
		want       bool
	}{
		{"touch at endpoint", pt(0, 0), pt(2, 0), pt(2, 0), pt(3, 3), true},
		{"endpoint on interior", pt(0, 0), pt(4, 0), pt(2, 0), pt(2, 3), true},
		{"cross", pt(0, 0), pt(2, 2), pt(0, 2), pt(2, 0), true},
		{"collinear overlap", pt(0, 0), pt(3, 0), pt(2, 0), pt(5, 0), true},
		{"collinear disjoint", pt(0, 0), pt(1, 0), pt(2, 0), pt(5, 0), false},
		{"overlapping boxes disjoint", pt(0, 0), pt(4, 4), pt(0, 1), pt(2, 3.5), false},
		{"near miss by one ulp", pt(0, 0), pt(1, 1), pt(1.0000000000000002, 1), pt(3, 0), false},
		{"rational gap rounded away", pt(0, 0), pt(1, 0), ratPt(onePlusTiny, big.NewRat(0, 1)), pt(2, 0), false},
		{"rational crossing", pt(0, 0), ratPt(big.NewRat(1, 1), big.NewRat(1, 3)),
			ratPt(big.NewRat(0, 1), big.NewRat(1, 3)), pt(1, 0), true},
		{"parallel apart", pt(0, 0), pt(2, 2), pt(0, 1), pt(2, 3), false},
		{"line crossing beyond the segment", pt(0, 0), pt(0.5, 0.5), pt(0, 2), pt(2, 0), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			require.Equal(t, tc.want, EdgesMeet(tc.a, tc.b, tc.c, tc.d))
			xp := func(p XY) meshbool.Xp2 { return meshbool.NewXP2(p.X, p.Y) }
			require.Equal(t, tc.want, edgesMeetXP(xp(tc.a), xp(tc.b), xp(tc.c), xp(tc.d)))
			// A point without a finite float image takes the rational body.
			huge := new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 2000), big.NewInt(1))
			far := XY{huge, huge}
			require.Equal(t, EdgesMeet(tc.a, tc.b, tc.c, far),
				edgesMeetXP(xp(tc.a), xp(tc.b), xp(tc.c), xp(far)))
		})
	}
}

// edgesMeetRaw is EdgesMeet as it read before its comparisons went through
// proof.CmpRat: the same bounding-box rejection and exact orientation tests,
// with big.Rat.Cmp.
func edgesMeetRaw(a, b, c, d XY) bool {
	between := func(a, b, p *big.Rat) bool {
		return (a.Cmp(p) <= 0 && p.Cmp(b) <= 0) ||
			(b.Cmp(p) <= 0 && p.Cmp(a) <= 0)
	}
	if a.X.Cmp(c.X) > 0 && a.X.Cmp(d.X) > 0 &&
		b.X.Cmp(c.X) > 0 && b.X.Cmp(d.X) > 0 ||
		a.X.Cmp(c.X) < 0 && a.X.Cmp(d.X) < 0 &&
			b.X.Cmp(c.X) < 0 && b.X.Cmp(d.X) < 0 ||
		a.Y.Cmp(c.Y) > 0 && a.Y.Cmp(d.Y) > 0 &&
			b.Y.Cmp(c.Y) > 0 && b.Y.Cmp(d.Y) > 0 ||
		a.Y.Cmp(c.Y) < 0 && a.Y.Cmp(d.Y) < 0 &&
			b.Y.Cmp(c.Y) < 0 && b.Y.Cmp(d.Y) < 0 {
		return false
	}
	abc, abd := Orient(a, b, c).Sign(), Orient(a, b, d).Sign()
	cda, cdb := Orient(c, d, a).Sign(), Orient(c, d, b).Sign()
	if abc == 0 && between(a.X, b.X, c.X) && between(a.Y, b.Y, c.Y) ||
		abd == 0 && between(a.X, b.X, d.X) && between(a.Y, b.Y, d.Y) ||
		cda == 0 && between(c.X, d.X, a.X) && between(c.Y, d.Y, a.Y) ||
		cdb == 0 && between(c.X, d.X, b.X) && between(c.Y, d.Y, b.Y) {
		return true
	}
	return abc != abd && cda != cdb
}

// TestEdgesMeetMatchesRawComparisons runs EdgesMeet and its raw-comparison
// form over 30,000 random segment pairs. Most take their coordinates from a
// 5x5 grid of integers, thirds and halves, so shared endpoints, collinear
// overlaps and touching boxes are common; the rest use huge-denominator
// rationals and float lifts.
func TestEdgesMeetMatchesRawComparisons(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(81, 83))
	grid := func() *big.Rat {
		den := []int64{1, 1, 3, 2}[rng.IntN(4)]
		return big.NewRat(rng.Int64N(7)-2, den)
	}
	wide := func() *big.Rat {
		if rng.IntN(2) == 0 {
			return new(big.Rat).SetFloat64(float64(rng.IntN(7)-3) / 4)
		}
		den := new(big.Int).Lsh(big.NewInt(2*rng.Int64N(1000)+1), uint(rng.IntN(80)))
		return new(big.Rat).SetFrac(big.NewInt(rng.Int64N(9)-4), den)
	}
	meets := 0
	for i := range 30000 {
		coord := grid
		if i%5 == 0 {
			coord = wide
		}
		pts := make([]XY, 4)
		for k := range pts {
			pts[k] = XY{coord(), coord()}
		}
		got := EdgesMeet(pts[0], pts[1], pts[2], pts[3])
		want := edgesMeetRaw(pts[0], pts[1], pts[2], pts[3])
		require.Equal(t, want, got, "segments %v-%v and %v-%v", pts[0], pts[1], pts[2], pts[3])
		require.Equal(t, got, EdgesMeet(pts[3], pts[2], pts[1], pts[0]), "the predicate is symmetric")
		if got {
			meets++
		}
	}
	require.Greater(t, meets, 3000, "the sample must hold plenty of touching pairs")
	require.Less(t, meets, 27000, "the sample must hold plenty of disjoint pairs")
}
