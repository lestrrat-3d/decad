package sectionaudit_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func lineWalk(u0, v0, u1, v1 float64) survey2d.SegmentWalk {
	return survey2d.SegmentWalk{Kind: survey2d.WalkLine, StartU: u0, StartV: v0, EndU: u1, EndV: v1,
		Length: math.Hypot(u1-u0, v1-v0)}
}

func arcWalk(cu, cv, r, th0, th1 float64) survey2d.SegmentWalk {
	return survey2d.SegmentWalk{Kind: survey2d.WalkCircular, CU: cu, CV: cv, Radius: r, Th0: th0, Th1: th1,
		StartU: cu + r*math.Cos(th0), StartV: cv + r*math.Sin(th0),
		EndU: cu + r*math.Cos(th1), EndV: cv + r*math.Sin(th1)}
}

// samples returns dense points of a walk.
func samples(w survey2d.SegmentWalk, n int) [][2]float64 {
	out := make([][2]float64, 0, n+1)
	for i := 0; i <= n; i++ {
		f := float64(i) / float64(n)
		if w.IsLine() {
			out = append(out, [2]float64{w.StartU + f*(w.EndU-w.StartU), w.StartV + f*(w.EndV-w.StartV)})
			continue
		}
		th := w.Th0 + f*(w.Th1-w.Th0)
		out = append(out, [2]float64{w.CU + w.Radius*math.Cos(th), w.CV + w.Radius*math.Sin(th)})
	}
	return out
}

func sampledGap(a, b survey2d.SegmentWalk) float64 {
	best := math.Inf(1)
	pa, pb := samples(a, 1500), samples(b, 1500)
	for _, p := range pa {
		for _, q := range pb {
			best = math.Min(best, math.Hypot(p[0]-q[0], p[1]-q[1]))
		}
	}
	return best
}

func TestSegGapLowerCrossingNearAnEnd(t *testing.T) {
	t.Parallel()
	// Two lines cross 1e-8 of a length from the end of one: segMinDist's
	// interior margin reads a positive gap, SegGapLower proves none.
	a := lineWalk(0, 0, 10, 0)
	b := lineWalk(1e-7, -1, 1e-7, 1) // crosses a at u = 1e-7, 1e-8 of a's length from its start
	require.Greater(t, sectionaudit.SegMinDist(a, b), 0.0)
	require.Zero(t, sectionaudit.SegGapLower(a, b))
	// A line and an arc crossing near the arc's end.
	arc := arcWalk(0, 0, 5, 0, math.Pi/2)
	ln := lineWalk(4.9999999, -1, 4.9999999, 1)
	require.Zero(t, sectionaudit.SegGapLower(arc, ln))
}

func TestSegGapLowerAtMostExactDistance(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewSource(7))
	pt := func() (float64, float64) { return rng.Float64()*10 - 5, rng.Float64()*10 - 5 }
	mk := func() survey2d.SegmentWalk {
		switch rng.Intn(3) {
		case 0:
			u0, v0 := pt()
			u1, v1 := pt()
			return lineWalk(u0, v0, u1, v1)
		case 1:
			cu, cv := pt()
			th0 := rng.Float64() * 2 * math.Pi
			sweep := 0.3 + rng.Float64()*5
			if rng.Intn(2) == 0 {
				return arcWalk(cu, cv, 0.5+rng.Float64()*3, th0, th0+sweep)
			}
			return arcWalk(cu, cv, 0.5+rng.Float64()*3, th0+sweep, th0)
		default:
			cu, cv := pt()
			w := arcWalk(cu, cv, 0.5+rng.Float64()*3, 0, 2*math.Pi)
			w.Closed = true
			return w
		}
	}
	positive := 0
	for range 150 {
		a, b := mk(), mk()
		lower := sectionaudit.SegGapLower(a, b)
		exact := sampledGap(a, b)
		// the sampled distance overestimates the true one by at most the
		// sample spacing
		require.LessOrEqual(t, lower, exact+0.02, "%+v %+v", a, b)
		if lower > 0 {
			positive++
		}
	}
	require.Greater(t, positive, 20)
}

func TestSegGapLowerTightForSeparatedPieces(t *testing.T) {
	t.Parallel()
	// Concentric arcs 1 mm apart, a line 2 mm from a circle.
	a := arcWalk(0, 0, 5, 0, math.Pi)
	b := arcWalk(0, 0, 6, 0.5, 2)
	require.InDelta(t, 1, sectionaudit.SegGapLower(a, b), 1e-9)
	circle := arcWalk(0, 0, 3, 0, 2*math.Pi)
	circle.Closed = true
	ln := lineWalk(5, -3, 5, 3)
	require.InDelta(t, 2, sectionaudit.SegGapLower(circle, ln), 1e-9)
	// A line piece whose carrier meets the arc's carrier outside both pieces.
	short := arcWalk(0, 0, 3, 0, 0.5)
	far := lineWalk(-4, -4, -4, 4)
	require.Greater(t, sectionaudit.SegGapLower(short, far), 3.0)
}

// WalkGapBox reads an arc as SegGapLower does (§6.2): its ends are the points
// at the radius on the float ends' bearings, not the float ends themselves.
func TestWalkGapBoxUsesTheEndpointEnclosures(t *testing.T) {
	t.Parallel()
	const r = 5.0
	arc := arcWalk(0, 0, r, 0.2, 1.0)
	// Both float ends sit 0.01 inside the circle, so the points SegGapLower
	// reads lie outside the box of the float ends.
	arc.StartU, arc.StartV = (r-0.01)*math.Cos(0.2), (r-0.01)*math.Sin(0.2)
	arc.EndU, arc.EndV = (r-0.01)*math.Cos(1.0), (r-0.01)*math.Sin(1.0)
	u0, v0, u1, v1, ok := sectionaudit.WalkGapBox(arc)
	require.True(t, ok)
	for _, th := range []float64{0.2, 1.0} {
		x, y := r*math.Cos(th), r*math.Sin(th)
		require.LessOrEqual(t, u0, x)
		require.LessOrEqual(t, v0, y)
		require.GreaterOrEqual(t, u1, x)
		require.GreaterOrEqual(t, v1, y)
	}
	// The arc reaches no cardinal point, so the box is the ends' box and no wider.
	require.InDelta(t, r*math.Cos(1.0), u0, 1e-12)
	require.InDelta(t, r*math.Cos(0.2), u1, 1e-12)
	require.InDelta(t, r*math.Sin(0.2), v0, 1e-12)
	require.InDelta(t, r*math.Sin(1.0), v1, 1e-12)

	// An arc over the top reaches the cardinal point (0, r) but not (r, 0).
	top := arcWalk(1, 2, r, 0.5, 2.5)
	_, _, maxU, maxV, ok := sectionaudit.WalkGapBox(top)
	require.True(t, ok)
	require.InDelta(t, 2+r, maxV, 1e-12)
	require.Less(t, maxU, 1+r-0.1)

	// A line's box is its two exact ends.
	u0, v0, u1, v1, ok = sectionaudit.WalkGapBox(lineWalk(3, -1, -2, 4))
	require.True(t, ok)
	require.Equal(t, [4]float64{-2, -1, 3, 4}, [4]float64{u0, v0, u1, v1})

	// A closed circle's box is its full square.
	circle := arcWalk(0, 0, 3, 0, 2*math.Pi)
	circle.Closed = true
	u0, v0, u1, v1, ok = sectionaudit.WalkGapBox(circle)
	require.True(t, ok)
	require.Equal(t, [4]float64{-3, -3, 3, 3}, [4]float64{u0, v0, u1, v1})
}
