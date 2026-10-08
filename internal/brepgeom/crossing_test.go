package brepgeom_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type pt = sectionrecord.Point2

func line(a, b pt) sectionrecord.LineSeg {
	return sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1}
}

func circle(c pt, r float64) sectionrecord.CircleSeg {
	return sectionrecord.CircleSeg{Center: c, Radius: units.Millimeters(r), TStart: 0, TEnd: 1, CCW: true}
}

// distSquared is |p − x|² over exact rationals.
func distSquared(p, x pt) *big.Rat {
	du := new(big.Rat).Sub(new(big.Rat).SetFloat64(p.U), new(big.Rat).SetFloat64(x.U))
	dv := new(big.Rat).Sub(new(big.Rat).SetFloat64(p.V), new(big.Rat).SetFloat64(x.V))
	return du.Add(du.Mul(du, du), dv.Mul(dv, dv))
}

// requireCovers checks off ≥ |p − x| over exact rationals.
func requireCovers(t *testing.T, off float64, p, x pt) {
	t.Helper()
	require.False(t, math.IsInf(off, 0))
	o := new(big.Rat).SetFloat64(off)
	require.GreaterOrEqualf(t, o.Mul(o, o).Cmp(distSquared(p, x)), 0, "offset %g does not reach %v from %v", off, x, p)
}

// TestCrossingOffsetUpperCoversEachCrossing holds the bound to the exact
// distance from points about the crossing of two carriers, for every pair of
// carrier kinds: two oblique lines crossing at (4, 2), an oblique line and a
// circle, an axis-aligned line and an arc, and two circles, the last three
// crossing at (3, 4). The bound is zero at the crossing itself, reaches every
// point 1e-9 off it in eight directions, and stays within four times that
// distance.
func TestCrossingOffsetUpperCoversEachCrossing(t *testing.T) {
	t.Parallel()
	arc := sectionrecord.ArcSeg{Center: pt{}, Start: pt{U: 5}, End: pt{V: 5}, TStart: 0, TEnd: 1}
	cases := []struct {
		name string
		a, b sectionrecord.CurveSegment
		x    pt
	}{
		{"line and line", line(pt{}, pt{U: 8, V: 4}), line(pt{V: 6}, pt{U: 6}), pt{U: 4, V: 2}},
		{"oblique line and circle", line(pt{}, pt{U: 6, V: 8}), circle(pt{}, 5), pt{U: 3, V: 4}},
		{"axis line and arc", line(pt{U: 3, V: -10}, pt{U: 3, V: 10}), arc, pt{U: 3, V: 4}},
		{"circle and circle", circle(pt{}, 5), circle(pt{U: 6}, 5), pt{U: 3, V: 4}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Zero(t, brepgeom.CrossingOffsetUpper(tc.a, tc.b, tc.x))
			require.Zero(t, brepgeom.CrossingOffsetUpper(tc.b, tc.a, tc.x), `the pair reads alike either way round`)
			for k := range 8 {
				angle := float64(k) * math.Pi / 4
				p := pt{U: tc.x.U + 1e-9*math.Cos(angle), V: tc.x.V + 1e-9*math.Sin(angle)}
				off := brepgeom.CrossingOffsetUpper(tc.a, tc.b, p)
				requireCovers(t, off, p, tc.x)
				require.Less(t, off, 4e-9)
			}
		})
	}
}

// TestCrossingOffsetUpperCoversBothCrossingsNearTheFoot holds a point too
// close to the centre's foot to name which crossing it stands for to both:
// on the line u = 3, 1e-12 above the centre's foot, the radius-5 circle about
// the origin crosses at (3, 4) and (3, −4).
func TestCrossingOffsetUpperCoversBothCrossingsNearTheFoot(t *testing.T) {
	t.Parallel()
	p := pt{U: 3, V: 1e-12}
	off := brepgeom.CrossingOffsetUpper(line(pt{U: 3, V: -10}, pt{U: 3, V: 10}), circle(pt{}, 5), p)
	requireCovers(t, off, p, pt{U: 3, V: 4})
	requireCovers(t, off, p, pt{U: 3, V: -4})
}

// TestCrossingOffsetUpperStatesNoCrossing answers +Inf for every pair that
// names no crossing to measure from.
func TestCrossingOffsetUpperStatesNoCrossing(t *testing.T) {
	t.Parallel()
	p := pt{U: 3, V: 4}
	pairs := map[string][2]sectionrecord.CurveSegment{
		"parallel lines":    {line(pt{}, pt{U: 1}), line(pt{V: 1}, pt{U: 1, V: 1})},
		"one circle twice":  {circle(pt{}, 5), circle(pt{}, 5)},
		"concentric":        {circle(pt{}, 5), circle(pt{}, 4)},
		"line misses":       {line(pt{U: 6, V: -1}, pt{U: 6, V: 1}), circle(pt{}, 5)},
		"degenerate line":   {line(pt{U: 1}, pt{U: 1}), circle(pt{}, 5)},
		"unsupported kinds": {sectionrecord.SplineSeg{}, circle(pt{}, 5)},
	}
	for name, pair := range pairs {
		require.Truef(t, math.IsInf(brepgeom.CrossingOffsetUpper(pair[0], pair[1], p), 1), "%s", name)
	}
}

// TestSameCarrier tells fragments of one carrier from crossing carriers.
func TestSameCarrier(t *testing.T) {
	t.Parallel()
	arc := sectionrecord.ArcSeg{Center: pt{}, Start: pt{U: 5}, End: pt{V: 5}, TStart: 0, TEnd: 1}
	require.True(t, brepgeom.SameCarrier(line(pt{}, pt{U: 2, V: 1}), line(pt{U: 4, V: 2}, pt{U: -2, V: -1})))
	require.True(t, brepgeom.SameCarrier(circle(pt{}, 5), arc))
	require.False(t, brepgeom.SameCarrier(line(pt{}, pt{U: 2, V: 1}), line(pt{V: 1}, pt{U: 2, V: 2})))
	require.False(t, brepgeom.SameCarrier(circle(pt{}, 5), circle(pt{}, 4)))
	require.False(t, brepgeom.SameCarrier(line(pt{}, pt{U: 2, V: 1}), circle(pt{}, 5)))
}
