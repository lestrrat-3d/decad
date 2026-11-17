package brepmodify

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestEndClipReliefPolygon reads the end clip of a concave end turned 10 degrees with and without a relief. inClip
// claims a point inside the sector's angles out to the clip radius, and a point outside them only within the circle the
// relief polygon holds. Every relief vertex lies on the relief's circle, so the clip removes nothing outside it, and the
// clip stays one simple ring wound one way.
func TestEndClipReliefPolygon(t *testing.T) {
	t.Parallel()
	const turn, relief, clip = 10 * math.Pi / 180, 0.004, 2.0
	// The run arrives at the origin along +X and the next wall leaves it turned 10 degrees to the right, along t.
	tdir := pt2{math.Cos(-turn), math.Sin(-turn)}
	e := runEnd{v: pt2{}, away: pt2{1, 0}, e2away: tdir, concave: true, turn: turn}
	from, sweep := e.clipSector()
	rest := 2*math.Pi - sweep
	at := func(angle, r float64) pt2 { return pt2{r * math.Cos(angle), r * math.Sin(angle)} }

	inSector := from + sweep/2
	require.True(t, e.inClip(at(inSector, clip*0.99), clip, 0))
	require.False(t, e.inClip(at(inSector, clip*1.01), clip, relief))
	outside := from + sweep + rest/2
	require.False(t, e.inClip(at(outside, relief/2), clip, 0), "no relief claims nothing outside the sector")
	require.True(t, e.inClip(at(outside, relief/2), clip, relief))
	require.False(t, e.inClip(at(outside, relief*1.001), clip, relief))
	// Midway along a chord the polygon reaches only its inradius.
	cells := reliefCells(rest)
	mid := from + sweep + rest/float64(cells)/2
	inradius := relief * math.Cos(rest/float64(cells)/2)
	require.True(t, e.inClip(at(mid, inradius*0.999), clip, relief))
	require.False(t, e.inClip(at(mid, (inradius+relief)/2), clip, relief))

	l := testLoop()
	l.relief = relief
	m, err := l.endClipMesh(t.Context(), e, 0.2, 0.1)
	require.NoError(t, err)
	onCircle := 0
	for _, p := range m.verts {
		r := math.Hypot(p.X, p.Y)
		require.LessOrEqual(t, r, e.clipRadius(0.2, 0.1)*(1+1e-12))
		if math.Abs(r-relief) < 1e-12 {
			onCircle++
		}
	}
	// Both heights hold every relief vertex, and nothing lies nearer the corner.
	require.Equal(t, 2*(cells+1), onCircle)
	for _, p := range m.verts {
		require.GreaterOrEqual(t, math.Hypot(p.X, p.Y), relief*(1-1e-12))
	}
}
