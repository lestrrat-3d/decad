package brepgeom_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// TestJoinLoopChargesTheCrossingOffset joins a loop 1e6 mm from the origin
// whose junctions are crossings of the wall u = 20 with the radius-5 circle
// about (18, c), c = 1e6 + 1.3: the circle's outside part, an arc pinned at
// the two crossings, and the wall's fragment between them, its parameter cut
// 3e-12 off each crossing. JoinLoop pins each junction at the wall's walked
// point, which is that far from the exact crossing, and the returned
// allowance must reach it. The crossings sit at c ± √(r² − 4), r² the arc's
// squared Start radius, computed here in math/big.
//
// Shown-to-fail: without the crossing offset (CrossingOffsetUpper in
// JoinLoop) the allowance is the walk rounding alone, below both distances.
func TestJoinLoopChargesTheCrossingOffset(t *testing.T) {
	t.Parallel()
	const y0, prec = 1e6, 256
	c := y0 + 1.3
	half := math.Sqrt(21)
	lower, upper := pt{U: 20, V: c - half}, pt{U: 20, V: c + half}
	arc := sectionrecord.ArcSeg{Center: pt{U: 18, V: c}, Start: lower, End: upper, TStart: 0, TEnd: 1}
	// The wall runs down from y0 + 20 to y0 − 20: V(t) = y0 + 20 − 40·t, so
	// 3e-12 of t moves the walked point 1.2e-10 mm, about one ulp of 1e6.
	tUpper := (y0+20-upper.V)/40 + 3e-12
	tLower := (y0+20-lower.V)/40 - 3e-12
	wall := sectionrecord.LineSeg{Start: pt{U: 20, V: y0 + 20}, End: pt{U: 20, V: y0 - 20}, TStart: tUpper, TEnd: tLower}

	joined, allow, err := brepgeom.JoinLoop(sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{arc, wall}})
	require.NoError(t, err)
	got, ok := joined.Segments[1].(sectionrecord.LineSeg)
	require.True(t, ok)
	require.Less(t, allow, 1e-6)

	bf := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	du, dv := new(big.Float).SetPrec(prec).Sub(bf(lower.U), bf(18)), new(big.Float).SetPrec(prec).Sub(bf(lower.V), bf(c))
	r2 := new(big.Float).SetPrec(prec).Mul(du, du)
	r2.Add(r2, new(big.Float).SetPrec(prec).Mul(dv, dv))
	root := new(big.Float).SetPrec(prec).Sub(r2, bf(4))
	root.Sqrt(root)
	for _, end := range []struct {
		at   pt
		sign float64
	}{{got.Start, 1}, {got.End, -1}} {
		require.Equal(t, 20.0, end.at.U)
		crossing := new(big.Float).SetPrec(prec).Mul(root, bf(end.sign))
		crossing.Add(crossing, bf(c))
		dist := new(big.Float).SetPrec(prec).Sub(bf(end.at.V), crossing)
		dist.Abs(dist)
		d, _ := dist.Float64()
		require.Positive(t, d, `the wall's walked point is off the crossing`)
		require.LessOrEqualf(t, dist.Cmp(bf(allow)), 0, "the joined vertex is %.3e from the crossing, beyond the allowance %.3e", d, allow)
	}
}

// TestJoinLoopChargesAnArcNaturalEnd joins two arcs of the radius-5 circle
// about the origin: one from (3, 4) whose natural End sits 500 ulps outside
// the radius near (−3, 4), the other from (−3, 4) round to (3, 4). Their
// junction is not one float, and the arc's End is lexicographically smaller,
// so JoinLoop pins the junction there and rewrites the second arc to start at
// it. The arc denotes Start's radius at End's angle, so the pinned point sits
// |End| − 5 off the circle both arcs lie on, and the allowance must reach it.
// One carrier names no crossing, so only the denoted end bound covers it.
//
// Shown-to-fail: charging the walk's own EndBound instead of
// boundarywalk.DenotedEndBound leaves the allowance at zero.
func TestJoinLoopChargesAnArcNaturalEnd(t *testing.T) {
	t.Parallel()
	out := func(v float64) float64 {
		for range 500 {
			v = math.Nextafter(v, 2*v)
		}
		return v
	}
	end := pt{U: out(-3), V: out(4)}
	first := sectionrecord.ArcSeg{Center: pt{}, Start: pt{U: 3, V: 4}, End: end, TStart: 0, TEnd: 1}
	second := sectionrecord.ArcSeg{Center: pt{}, Start: pt{U: -3, V: 4}, End: pt{U: 3, V: 4}, TStart: 0, TEnd: 1}
	require.True(t, brepgeom.SameCarrier(first, second))

	joined, allow, err := brepgeom.JoinLoop(sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{first, second}})
	require.NoError(t, err)
	got, ok := joined.Segments[1].(sectionrecord.ArcSeg)
	require.True(t, ok)
	require.Equal(t, end, got.Start, `the junction is pinned at the arc's natural End`)

	const prec = 256
	radius := new(big.Float).SetPrec(prec).Mul(big.NewFloat(end.U), big.NewFloat(end.U))
	radius.Add(radius, new(big.Float).SetPrec(prec).Mul(big.NewFloat(end.V), big.NewFloat(end.V)))
	radius.Sqrt(radius)
	off := radius.Sub(radius, big.NewFloat(5))
	require.Positive(t, off.Sign())
	d, _ := off.Float64()
	require.LessOrEqualf(t, off.Cmp(big.NewFloat(allow)), 0, "the pinned point is %.3e off the circle, beyond the allowance %.3e", d, allow)
}
