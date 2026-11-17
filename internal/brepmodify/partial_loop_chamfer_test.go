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
		if math.Abs(math.Hypot(p.X, p.Y)-relief) < 1e-12 {
			onCircle++
		}
	}
	// Both heights hold every relief vertex.
	require.Equal(t, 2*(cells+1), onCircle)
}

// TestEndClipReliefRing builds the relieved end clip over turns from 1e-7 to 90 degrees, in both senses, with clip
// radii under, near and over the relief. Every ring is simple, nothing lies nearer the corner than the smaller of the
// relief's inradius and the clip radius or past the larger of the two radii, and the mesh is closed and consistently
// wound.
func TestEndClipReliefRing(t *testing.T) {
	t.Parallel()
	const relief = 0.004
	l := testLoop()
	l.relief = relief
	for _, deg := range []float64{1e-7, 0.01, 1, 10, 22.5, 45, 60, 89, 90} {
		for _, sense := range []float64{1, -1} {
			for _, foot := range []float64{0, 0.001, 0.0039, 0.01} {
				turn := deg * math.Pi / 180
				e := runEnd{v: pt2{}, away: pt2{1, 0}, e2away: pt2{math.Cos(sense * turn), math.Sin(sense * turn)},
					concave: true, turn: turn, foot: foot}
				m, err := l.endClipMesh(t.Context(), e, 0.2, 0.1)
				require.NoError(t, err, "turn %v sense %v foot %v", deg, sense, foot)
				n := len(m.verts) / 2
				ring := make([]pt2, n)
				outer := max(relief, e.clipRadius(0.2, 0.1))
				inner := min(relief*math.Cos(math.Pi/16), e.clipRadius(0.2, 0.1))
				for k := range n {
					ring[k] = pt2{m.verts[k].X, m.verts[k].Y}
					r := ring[k].len()
					require.LessOrEqual(t, r, outer*(1+1e-12))
					require.GreaterOrEqual(t, r, inner*(1-1e-12))
				}
				for a := range n {
					for b := a + 2; b < n; b++ {
						if a == 0 && b == n-1 {
							continue
						}
						require.False(t, segmentsCross(ring[a], ring[(a+1)%n], ring[b], ring[(b+1)%n]),
							"turn %v sense %v foot %v: edges %d and %d cross", deg, sense, foot, a, b)
					}
				}
				seen := map[[2]int]int{}
				for _, tr := range m.tris {
					for k := range 3 {
						seen[[2]int{tr[k], tr[(k+1)%3]}]++
					}
				}
				for edge, count := range seen {
					require.Equal(t, 1, count)
					require.Equal(t, 1, seen[[2]int{edge[1], edge[0]}])
				}
			}
		}
	}
}

// segmentsCross reports whether the segments ab and cd meet, ends included.
func segmentsCross(a, b, c, d pt2) bool {
	orient := func(p, q, r pt2) float64 { return q.sub(p).cross(r.sub(p)) }
	d1, d2 := orient(c, d, a), orient(c, d, b)
	d3, d4 := orient(a, b, c), orient(a, b, d)
	return (d1 > 0) != (d2 > 0) && (d3 > 0) != (d4 > 0) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0
}
