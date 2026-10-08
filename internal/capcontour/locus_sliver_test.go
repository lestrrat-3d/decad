package capcontour_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// parabolaHull records the locus P(t) = (t, t²) over n equal sub-ranges of
// [0, span]: exact positions at each end and the exact velocity box
// {1} × [2·t0, 2·t1] on each sub-range.
func parabolaHull(span float64, n int) capcontour.LocusVelocityHull {
	pt := func(t float64) capcontour.Point {
		r := new(big.Rat).SetFloat64(t)
		return capcontour.Point{
			U: proofbound.PointInterval(r),
			V: proofbound.PointInterval(new(big.Rat).Mul(r, r)),
		}
	}
	var h capcontour.LocusVelocityHull
	t0 := 0.0
	for k := range n {
		t1 := float64(k+1) * span / float64(n)
		if k == n-1 {
			t1 = span
		}
		box := capcontour.Point{
			U: proofbound.PointInterval(big.NewRat(1, 1)),
			V: proofbound.Interval(new(big.Rat).SetFloat64(2*t0), new(big.Rat).SetFloat64(2*t1)),
		}
		h.Add(t0, t1, pt(t0), pt(t1), box)
		t0 = t1
	}
	return h
}

// TestSliverEnclosureHoldsTheExactSliver checks the per-range enclosure of
// W(dc) = ∫₀^dc (P(t) − Q(t)) dt against the parabola P(t) = (t, t²), whose
// chord Q(t) = (t, t·dc) gives W = (0, −dc³/6) exactly. With lo = hi the
// enclosure must hold W and sit within 1% of it; with lo < hi it must hold W
// at both ends and the middle of [lo, hi]. Tiles with a gap and an empty hull
// refuse.
//
// Shown to fail on 2026-10-09: with the remainder past lo left out, the
// lo < hi rows lose W, and with the per-range spread term left
// out, the lo = hi row's enclosure misses W.
func TestSliverEnclosureHoldsTheExactSliver(t *testing.T) {
	t.Parallel()
	exact := func(dc float64) *big.Rat {
		r := new(big.Rat).SetFloat64(dc)
		return new(big.Rat).Quo(new(big.Rat).Mul(r, new(big.Rat).Mul(r, r)), big.NewRat(-6, 1))
	}
	holds := func(iv proofbound.RatInterval, x *big.Rat) bool {
		return iv.Lo.Cmp(x) <= 0 && x.Cmp(iv.Hi) <= 0
	}

	h := parabolaHull(2, 32)
	w, ok := h.SliverEnclosure(2, 2)
	require.True(t, ok)
	require.True(t, holds(w.V, exact(2)), `the enclosure must hold W at dc = 2`)
	require.True(t, holds(w.U, new(big.Rat)), `the chord's U component matches the locus`)
	width, _ := new(big.Rat).Sub(w.V.Hi, w.V.Lo).Float64()
	want, _ := exact(2).Float64()
	require.Less(t, width, 0.01*-want, `the enclosure must sit within 1%% of W`)

	for _, lo := range []float64{1.9, 1.5} {
		w, ok := h.SliverEnclosure(lo, 2)
		require.True(t, ok)
		for _, dc := range []float64{lo, (lo + 2) / 2, 2} {
			require.True(t, holds(w.V, exact(dc)), `lo=%v: the enclosure must hold W at dc = %v`, lo, dc)
		}
	}

	_, ok = h.SliverEnclosure(2, 3)
	require.False(t, ok, `sub-ranges that stop short of hi do not tile it`)
	var empty capcontour.LocusVelocityHull
	_, ok = empty.SliverEnclosure(1, 1)
	require.False(t, ok, `an empty hull states nothing`)
}
