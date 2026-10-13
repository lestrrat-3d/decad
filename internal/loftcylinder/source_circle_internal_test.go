package loftcylinder

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// loftCylinderCrossingBoxReference is the rational bisection crossingBox
// replaced: the same decisions taken with big.Rat arithmetic.
func loftCylinderCrossingBoxReference(c Connector, tLo, tHi, radius2 *big.Rat) XYBox {
	low, high := big.NewRat(0, 1), big.NewRat(2, 1)
	for range 52 {
		mid := new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1))
		p0, p1 := c.point(mid, tLo), c.point(mid, tHi)
		q0, q1 := SourceDot(p0, p0), SourceDot(p1, p1)
		if q0.Cmp(radius2) < 0 && q1.Cmp(radius2) < 0 {
			low = mid
			continue
		}
		if sourceRadiusMin2(p0, p1).Cmp(radius2) > 0 {
			high = mid
			continue
		}
		if tLo.Cmp(tHi) == 0 && q0.Cmp(radius2) == 0 {
			low, high = mid, mid
		}
		break
	}
	box := sourceBoxOf(
		c.point(low, tLo), c.point(low, tHi), c.point(high, tLo), c.point(high, tHi))
	if low.Cmp(big.NewRat(1, 1)) < 0 && high.Cmp(big.NewRat(1, 1)) > 0 {
		middle := big.NewRat(1, 1)
		box = sourceUnionBox(box, sourceBoxOf(c.point(middle, tLo), c.point(middle, tHi)))
	}
	return box
}

// TestLoftCylinderSourceCrossingBoxMatchesRationalBisection compares the
// integer bisection against the rational one on connectors whose crossing
// lands before, across and after the middle vertex, and on the collapsed
// tLo == tHi case where the midpoint sits exactly on the circle.
func TestLoftCylinderSourceCrossingBoxMatchesRationalBisection(t *testing.T) {
	t.Parallel()
	xy := func(x, y int64, den int64) XY {
		return XY{big.NewRat(x, den), big.NewRat(y, den)}
	}
	connectors := map[string]Connector{
		"straight": {
			a: [2]XY{xy(2, 0, 1), xy(5, 1, 3)},
			d: [2]XY{xy(2, 0, 1), xy(5, 1, 3)},
			f: [2]XY{xy(7, 1, 1), xy(22, 5, 3)},
		},
		"bent at d": {
			a: [2]XY{xy(1, 0, 1), xy(3, 1, 7)},
			d: [2]XY{xy(3, 2, 1), xy(11, 9, 5)},
			f: [2]XY{xy(2, 6, 1), xy(5, 13, 3)},
		},
		"descending": {
			a: [2]XY{xy(9, 1, 2), xy(17, 3, 4)},
			d: [2]XY{xy(1, 1, 2), xy(3, 1, 4)},
			f: [2]XY{xy(1, 1, 2), xy(3, 1, 4)},
		},
	}
	ts := [][2]*big.Rat{
		{big.NewRat(0, 1), big.NewRat(1, 1)},
		{big.NewRat(1, 8), big.NewRat(3, 8)},
		{big.NewRat(1, 3), big.NewRat(1, 3)},
		{big.NewRat(9, 16), big.NewRat(15, 16)},
	}
	for name, c := range connectors {
		for _, radius := range []*big.Rat{big.NewRat(1, 2), big.NewRat(2, 1), big.NewRat(7, 2), big.NewRat(5, 1), big.NewRat(40, 1)} {
			r2 := new(big.Rat).Mul(radius, radius)
			for _, pair := range ts {
				want := loftCylinderCrossingBoxReference(c, pair[0], pair[1], r2)
				got := c.crossingBox(pair[0], pair[1], r2)
				require.Zero(t, want.lo.X.Cmp(got.lo.X), "%s r=%v t=%v lo.x", name, radius, pair)
				require.Zero(t, want.lo.Y.Cmp(got.lo.Y), "%s r=%v t=%v lo.y", name, radius, pair)
				require.Zero(t, want.hi.X.Cmp(got.hi.X), "%s r=%v t=%v hi.x", name, radius, pair)
				require.Zero(t, want.hi.Y.Cmp(got.hi.Y), "%s r=%v t=%v hi.y", name, radius, pair)
			}
		}
		// The first midpoint is the middle vertex: put the circle through it.
		for _, tt := range ts {
			mid := c.point(big.NewRat(1, 1), tt[0])
			r2 := SourceDot(mid, mid)
			want := loftCylinderCrossingBoxReference(c, tt[0], tt[0], r2)
			got := c.crossingBox(tt[0], tt[0], r2)
			require.Zero(t, want.lo.X.Cmp(got.lo.X), "%s collapse lo.x", name)
			require.Zero(t, want.lo.Y.Cmp(got.lo.Y), "%s collapse lo.y", name)
			require.Zero(t, want.hi.X.Cmp(got.hi.X), "%s collapse hi.x", name)
			require.Zero(t, want.hi.Y.Cmp(got.hi.Y), "%s collapse hi.y", name)
		}
	}
}

// point is the exact wall point at wall parameter s and height parameter t.
func (c Connector) point(s, t *big.Rat) XY {
	var lo, hi [2]XY
	u := new(big.Rat).Set(s)
	if s.Cmp(big.NewRat(1, 1)) <= 0 {
		lo, hi = c.a, c.d
	} else {
		lo, hi = c.d, c.f
		u.Sub(u, big.NewRat(1, 1))
	}
	p0 := sourceLerp(lo[0], hi[0], u)
	p1 := sourceLerp(lo[1], hi[1], u)
	return sourceLerp(p0, p1, t)
}
