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
	// This end has no band, so the clip's width does not limit it.
	noWidth := math.Inf(1)

	inSector := from + sweep/2
	require.True(t, e.inClip(at(inSector, clip*0.99), clip, noWidth, 0))
	require.False(t, e.inClip(at(inSector, clip*1.01), clip, noWidth, relief))
	outside := from + sweep + rest/2
	require.False(t, e.inClip(at(outside, relief/2), clip, noWidth, 0), "no relief claims nothing outside the sector")
	require.True(t, e.inClip(at(outside, relief/2), clip, noWidth, relief))
	require.False(t, e.inClip(at(outside, relief*1.001), clip, noWidth, relief))
	// Midway along a chord the polygon reaches only its inradius.
	cells := reliefCells(rest)
	mid := from + sweep + rest/float64(cells)/2
	inradius := relief * math.Cos(rest/float64(cells)/2)
	require.True(t, e.inClip(at(mid, inradius*0.999), clip, noWidth, relief))
	require.False(t, e.inClip(at(mid, (inradius+relief)/2), clip, noWidth, relief))

	l := testLoop()
	l.relief = relief
	m, err := l.endClipMesh(t.Context(), e, math.Inf(1), 0.2, 0.1)
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
				m, err := l.endClipMesh(t.Context(), e, math.Inf(1), 0.2, 0.1)
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

// tongueRun is the run up the right side of a tongue 2.1 wide and h tall standing on a block, across its top and down
// its left side. Each side meets the block's top through a short wall turned by turn off it, a concave corner.
func tongueRun(h, turn float64) chamferRun {
	const w = 2.1
	sn, cs := math.Sin(turn), math.Cos(turn)
	l := testLoop(
		lineSeg(pt2{w + sn, -cs}, pt2{w, 0}),
		lineSeg(pt2{w, 0}, pt2{w, h}),
		lineSeg(pt2{w, h}, pt2{0, h}),
		lineSeg(pt2{0, h}, pt2{0, 0}),
		lineSeg(pt2{0, 0}, pt2{-sn, -cs}),
		lineSeg(pt2{-sn, -cs}, pt2{-10, -cs}),
		lineSeg(pt2{-10, -cs}, pt2{-10, -5}),
		lineSeg(pt2{-10, -5}, pt2{12, -5}),
		lineSeg(pt2{12, -5}, pt2{12, -cs}),
		lineSeg(pt2{12, -cs}, pt2{w + sn, -cs}),
	)
	return chamferRun{loop: l, first: 1, count: 3}
}

// TestEndClipKeepsToTheLinesBand reads the end clip of a concave end turned 5.625 degrees beside a line, at the foot
// of a tongue's side. The clip's sector reaches 1.5 reach/sin(turn), 13 mm here, past the tongue's top and its other
// side, but the clip holds only the band within the cutter's reach of the side's line. It still takes the side's band
// past the next wall's plane carried back, and leaves the tongue's other side and top alone, so the run does not come
// back into it. Every vertex of the clip's ring lies within that reach of the line, and the ring holds the band's edge
// as a straight side.
func TestEndClipKeepsToTheLinesBand(t *testing.T) {
	t.Parallel()
	const d, over = 0.5, 0.25
	turn := 5.625 * math.Pi / 180
	r := tongueRun(12, turn)
	ends, ok := r.ends()
	require.True(t, ok)
	e := ends[0]
	require.True(t, e.concave)
	require.InDelta(t, turn, e.turn, 1e-12)
	require.Equal(t, pt2{-1, 0}, e.band)
	reach := partialReach(d, over)
	width := r.clipWidth(e, true, d, over)
	require.InDelta(t, reach, width, 1e-12)
	clip := e.clipRadius(d, over)
	require.Greater(t, clip, 12.0)

	// The side's band past the next wall's plane, which leans into the material by the turn, is in the clip; the same
	// point without the band's limit is in it too.
	past := pt2{2.1 - (d + over), 1}
	require.True(t, e.inClip(past, clip, width, 0))
	// The other side's band and the top's band lie in the sector but farther than reach from the line.
	for _, q := range []pt2{{d, 2}, {d + over, 8}, {d, 12 - d}} {
		require.True(t, e.inClip(q, clip, math.Inf(1), 0), "%v in the sector", q)
		require.False(t, e.inClip(q, clip, width, 0), "%v in the clip", q)
	}
	require.NoError(t, r.checkClipLocal(e, true, width, d, over))

	m, err := r.loop.endClipMesh(t.Context(), e, width, d, over)
	require.NoError(t, err)
	onEdge := 0
	for _, p := range m.verts {
		off := pt2{p.X, p.Y}.sub(e.v).dot(e.band)
		require.LessOrEqual(t, off, width*(1+1e-12))
		if math.Abs(off-width) < 1e-12 {
			onEdge++
		}
	}
	// The band's edge runs from the sector's side to the circle, at both heights.
	require.GreaterOrEqual(t, onEdge, 4)

	// A tongue shorter than the taper comes back into the clip at its top.
	short := tongueRun(3, turn)
	ends, ok = short.ends()
	require.True(t, ok)
	err = short.checkClipLocal(ends[0], true, short.clipWidth(ends[0], true, d, over), d, over)
	require.ErrorContains(t, err, "comes back into the clip")
}

// TestClipWidthHoldsTheTaperStretch reads the clip width at a concave end turned 30 degrees beside a line 0.3 long
// that a quarter arc of radius 1 continues tangentially into the material, onto a line back over it. The smooth
// stretch within reach/sin(turn) of the corner, where the taper lies, bends deeper into the clip's sector than the
// cutter's reach from the corner's line, so the width grows to hold that stretch's band, and every sampled point of it
// in the sector lies in the clip.
func TestClipWidthHoldsTheTaperStretch(t *testing.T) {
	t.Parallel()
	const d, over = 0.5, 0.25
	turn := 30 * math.Pi / 180
	sn, cs := math.Sin(turn), math.Cos(turn)
	l := testLoop(
		lineSeg(pt2{sn, -cs}, pt2{0, 0}),
		lineSeg(pt2{0, 0}, pt2{0, 0.3}),
		arcSeg(pt2{-1, 0.3}, 1, pt2{0, 0.3}, pt2{-1, 1.3}, true),
		lineSeg(pt2{-1, 1.3}, pt2{-6, 1.3}),
		lineSeg(pt2{-6, 1.3}, pt2{-6, -5}),
		lineSeg(pt2{-6, -5}, pt2{5, -5}),
		lineSeg(pt2{5, -5}, pt2{5, -cs}),
		lineSeg(pt2{5, -cs}, pt2{sn, -cs}),
	)
	r := chamferRun{loop: l, first: 1, count: 3}
	ends, ok := r.ends()
	require.True(t, ok)
	e := ends[0]
	require.True(t, e.concave)
	width := r.clipWidth(e, true, d, over)
	require.Greater(t, width, partialReach(d, over))
	clip := e.clipRadius(d, over)
	inSector := 0
	r.walkFromCorner(true, e.nearLen(d, over), func(p, in pt2, near bool) bool {
		if !near {
			return false
		}
		for _, off := range clipOffsets(d, over) {
			q := p.add(in.scale(off))
			if e.inClip(q, clip, math.Inf(1), 0) {
				inSector++
				require.True(t, e.inClip(q, clip, width, 0), "%v", q)
			}
		}
		return true
	})
	require.Positive(t, inSector)
}

// segmentsCross reports whether the segments ab and cd meet, ends included.
func segmentsCross(a, b, c, d pt2) bool {
	orient := func(p, q, r pt2) float64 { return q.sub(p).cross(r.sub(p)) }
	d1, d2 := orient(c, d, a), orient(c, d, b)
	d3, d4 := orient(a, b, c), orient(a, b, d)
	return (d1 > 0) != (d2 > 0) && (d3 > 0) != (d4 > 0) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0
}
