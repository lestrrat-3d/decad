package brepmodify

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// hubChains reads the hub loop as a test loop and returns its runs, the three arms' chains of selected segments.
func hubChains(t *testing.T, hub []hubSeg) []chamferRun {
	t.Helper()
	segs := make([]loopSeg, len(hub))
	sel := make([]bool, len(hub))
	for i, h := range hub {
		segs[i] = lineSeg(h.p0, h.p1)
		if h.arc {
			segs[i] = arcSeg(h.c, h.r, h.p0, h.p1, h.ccw)
		}
		sel[i] = h.sel
	}
	l := testLoop(segs...)
	l.materialLeft = l.signedArea() > 0
	l.depth = 10
	runs, ok := l.runs(sel)
	require.True(t, ok)
	require.Len(t, runs, 3)
	return runs
}

type edgeKey struct{ a, b int }

// TestPartialCutterLayersOnlyBesideCurvedColumns checks the layered cutter of three chains that hold sharp corners beside
// arcs: the one-way clutch hub's arm, a D keyhole's rim, and a line into an arc. The layers are the fewest that hold the
// bound. A column that is not flagged curved is a straight line, and a cell between two such columns is planar, so only
// the cells beside a curved column carry the rings between, and every other cell is the two triangles the two-ring cutter
// holds. The mesh is closed and consistently wound.
func TestPartialCutterLayersOnlyBesideCurvedColumns(t *testing.T) {
	t.Parallel()
	const d, over = 0.5, 0.25
	for _, c := range []struct {
		name   string
		run    chamferRun
		layers int // 0 when only the fewest-layers property is read
	}{
		{"hub arm", hubChains(t, hubLoop605zzCw)[0], 18},
		{"hub arm, other sense", hubChains(t, hubLoop605zzCcw)[0], 18},
		{"D keyhole", chamferRun{loop: keyholeLoop(4, 3), first: 2, count: 3}, 0},
		{"line then arc", chamferRun{loop: cornerCases()[0].loop, first: 0, count: 2}, 0},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			r := c.run
			cr, err := r.cutterRings(d, over)
			require.NoError(t, err)
			layers := cr.layers
			if c.layers != 0 {
				require.Equal(t, c.layers, layers)
			}
			require.LessOrEqual(t, cr.departure, partialSagitta)
			// The cutter steps through no more layers than it needs.
			requireFewestLayers(t, r, d, over, -(d + over/2), layers)

			// Every column that is not flagged curved is a straight line through all the heights.
			n := len(cr.rings[0])
			require.Equal(t, n, len(cr.curved))
			point := func(k, j int) r3.Vec { return r.loop.to3(cr.rings[k][j], cr.heights[k]) }
			straight := func(j int) float64 {
				a, b := point(0, j), point(layers, j)
				dir, ok := b.Sub(a).Normalize()
				require.True(t, ok)
				worst := 0.0
				for k := 1; k < layers; k++ {
					off := point(k, j).Sub(a)
					worst = max(worst, off.Sub(dir.Scale(off.Dot(dir))).Len())
				}
				return worst
			}
			bent := 0
			for j := range n {
				if cr.curved[j] {
					bent++
					continue
				}
				require.Less(t, straight(j), 1e-12, "column %d", j)
			}
			require.Positive(t, bent)
			// Every cell between two straight columns lies in a plane.
			for j := range n {
				j1 := (j + 1) % n
				if cr.curved[j] || cr.curved[j1] {
					continue
				}
				a, b, e := point(0, j), point(0, j1), point(layers, j)
				normal, ok := b.Sub(a).Cross(e.Sub(a)).Normalize()
				if !ok {
					continue
				}
				for k := 0; k <= layers; k++ {
					for _, jj := range []int{j, j1} {
						require.Less(t, math.Abs(point(k, jj).Sub(a).Dot(normal)), 1e-12, "cell %d at ring %d", j, k)
					}
				}
			}

			// The cells beside a curved column keep every layer, and the others are the two-ring cutter's.
			full := make([]bool, n)
			for j := range n {
				full[j] = cr.curved[j] || cr.curved[(j+n-1)%n] || cr.curved[(j+1)%n]
			}
			want := 2*n + 2*(n-2) // the two-ring sides and the two caps
			for j := range n {
				switch a, b := full[j], full[(j+1)%n]; {
				case a && b:
					want += 2 * (layers - 1)
				case a != b:
					want += layers - 1
				}
			}
			m, err := r.cutterMesh(t.Context(), d, over)
			require.NoError(t, err)
			require.Equal(t, want, len(m.tris))
			require.InDelta(t, partialSagitta+cr.departure, m.bound, 1e-15)

			// Closed and consistently wound: every directed edge occurs once and its reverse once.
			seen := map[edgeKey]int{}
			for _, tr := range m.tris {
				for k := range 3 {
					seen[edgeKey{tr[k], tr[(k+1)%3]}]++
				}
			}
			for e, count := range seen {
				require.Equal(t, 1, count, "directed edge %v", e)
				require.Equal(t, 1, seen[edgeKey{e.b, e.a}], "reverse of %v", e)
			}
		})
	}
}
