package retainedblend

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireGridKeepsPairs checks that, for every query, the grid's candidates
// include every strip box the query meets.
func requireGridKeepsPairs(t *testing.T, strip, queries []facetBox) int {
	t.Helper()
	grid := newBoxGrid(strip)
	var near []int
	pairs := 0
	for qi, b := range queries {
		near = grid.near(b, near[:0])
		var want, got []int
		for i, s := range strip {
			if b.meets(s) {
				want = append(want, i)
			}
		}
		for _, i := range near {
			if b.meets(strip[i]) {
				got = append(got, i)
			}
		}
		require.Equal(t, want, got, "query %d", qi)
		pairs += len(want)
	}
	return pairs
}

func pointBox(x, y, z float64) facetBox {
	p := r3.NewVec(x, y, z)
	return facetBox{lo: p, hi: p}
}

func spanBox(lo, hi r3.Vec) facetBox { return facetBox{lo: lo, hi: hi} }

func TestBoxGridKeepsEveryMeetingPair(t *testing.T) {
	t.Parallel()

	t.Run("random boxes", func(t *testing.T) {
		t.Parallel()
		rng := rand.New(rand.NewPCG(1, 2))
		box := func(scale float64) facetBox {
			lo := r3.NewVec(rng.Float64()*10, rng.Float64()*10, rng.Float64()*10)
			return spanBox(lo, lo.Add(r3.NewVec(rng.Float64()*scale, rng.Float64()*scale, rng.Float64()*scale)))
		}
		strip := make([]facetBox, 500)
		for i := range strip {
			strip[i] = box(0.5)
		}
		queries := make([]facetBox, 2000)
		for i := range queries {
			queries[i] = box(2)
		}
		require.Positive(t, requireGridKeepsPairs(t, strip, queries))
	})

	t.Run("degenerate axes, far queries, touching and boundary boxes", func(t *testing.T) {
		t.Parallel()
		// 27 strip boxes: a grid of 3 cells per axis. z spans only 1e-30, and
		// the boxes are points, flat squares and unit cubes whose faces lie
		// on the cell boundaries x, y = 0, 1, 2, 3.
		var strip []facetBox
		for i := range 3 {
			for j := range 3 {
				x, y := float64(i), float64(j)
				switch (i + j) % 3 {
				case 0:
					strip = append(strip, pointBox(x, y, 0))
				case 1:
					strip = append(strip, spanBox(r3.NewVec(x, y, 0), r3.NewVec(x+1, y+1, 0)))
				default:
					strip = append(strip, spanBox(r3.NewVec(x, y, 0), r3.NewVec(x+1, y+1, 1e-30)))
				}
				strip = append(strip, spanBox(r3.NewVec(x, y, 1e-30), r3.NewVec(x+1, y+1, 1e-30)),
					pointBox(x+1, y+1, 1e-30))
			}
		}
		inf := math.Inf(1)
		queries := []facetBox{
			// Reaching far beyond the joint box on every side.
			spanBox(r3.NewVec(2.5, 2.5, 0), r3.NewVec(1e16, 1e16, 1e16)),
			spanBox(r3.NewVec(2.5, 2.5, 0), r3.NewVec(1e308, 1e308, 1e308)),
			spanBox(r3.NewVec(2.5, 2.5, 0), r3.NewVec(inf, inf, inf)),
			spanBox(r3.NewVec(-inf, -inf, -inf), r3.NewVec(0.5, 0.5, 0)),
			spanBox(r3.NewVec(-inf, -inf, -inf), r3.NewVec(inf, inf, inf)),
			spanBox(r3.NewVec(-1e308, 2.5, -1e308), r3.NewVec(1e308, 2.5, 1e308)),
			// Touching the joint box's faces exactly, from outside.
			spanBox(r3.NewVec(3, 0, 0), r3.NewVec(5, 5, 1)),
			spanBox(r3.NewVec(-2, -2, -1), r3.NewVec(0, 0, 0)),
			spanBox(r3.NewVec(0, 0, 1e-30), r3.NewVec(3, 3, 1)),
			// Points and flat boxes on cell boundaries.
			pointBox(1, 1, 0),
			pointBox(2, 2, 1e-30),
			pointBox(3, 3, 1e-30),
			spanBox(r3.NewVec(1, 0, 0), r3.NewVec(1, 3, 1e-30)),
			spanBox(r3.NewVec(0, 2, 0), r3.NewVec(3, 2, 0)),
			// Missing everything, just past the joint box.
			spanBox(r3.NewVec(3.5, 3.5, 0), r3.NewVec(inf, inf, inf)),
			pointBox(1, 1, 2e-30),
		}
		require.Positive(t, requireGridKeepsPairs(t, strip, queries))
	})
}

// stripWork is a work whose result keeps rest and whose strip is strip, over
// verts.
func stripWork(verts []r3.Vec, rest, strip [][3]int) *work {
	return &work{fp: facetedrecord.Payload{Tris: rest}, verts: verts, out: strip}
}

func TestCheckStripClear(t *testing.T) {
	t.Parallel()
	strip := [][3]int{{0, 1, 2}}
	base := []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(0, 4, 0)}

	t.Run("a facet clear of the strip passes", func(t *testing.T) {
		t.Parallel()
		verts := append(append([]r3.Vec(nil), base...),
			r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 1), r3.NewVec(0, 1, 1))
		require.NoError(t, stripWork(verts, [][3]int{{3, 4, 5}}, strip).checkStripClear(t.Context(), 0, nil))
	})

	t.Run("a facet sharing a vertex with the strip is not tested", func(t *testing.T) {
		t.Parallel()
		verts := append(append([]r3.Vec(nil), base...), r3.NewVec(4, 4, 0), r3.NewVec(4, 0, 1))
		require.NoError(t, stripWork(verts, [][3]int{{1, 3, 4}}, strip).checkStripClear(t.Context(), 0, nil))
	})

	t.Run("a crossing facet is refused at its contact", func(t *testing.T) {
		t.Parallel()
		verts := append(append([]r3.Vec(nil), base...),
			r3.NewVec(1, 1, -1), r3.NewVec(1, 1, 1), r3.NewVec(1, 3, 1))
		err := stripWork(verts, [][3]int{{3, 4, 5}}, strip).checkStripClear(t.Context(), 0, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		// The facet meets the strip along the segment from (1,1,0) to (1,2,0).
		require.Regexp(t, `the blend meets another facet of the body at \{1 [12] 0\}`, err.Error())
	})

	t.Run("a coplanar overlapping facet is refused", func(t *testing.T) {
		t.Parallel()
		verts := append(append([]r3.Vec(nil), base...),
			r3.NewVec(1, 1, 0), r3.NewVec(2, 1, 0), r3.NewVec(1, 2, 0))
		err := stripWork(verts, [][3]int{{3, 4, 5}}, strip).checkStripClear(t.Context(), 0, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the blend meets another facet of the body at {1 1 0}")
	})

	t.Run("a flat facet near the strip is refused", func(t *testing.T) {
		t.Parallel()
		verts := append(append([]r3.Vec(nil), base...),
			r3.NewVec(1, 1, -1), r3.NewVec(1, 1, 0), r3.NewVec(1, 1, 1))
		err := stripWork(verts, [][3]int{{3, 4, 5}}, strip).checkStripClear(t.Context(), 0, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "the blend's clearance from a flat facet is undecided")
	})

	t.Run("a facet crossing a taken cap is refused", func(t *testing.T) {
		t.Parallel()
		// w.out holds the cap at z = 0 and then the strip at z = 5; the held
		// facet crosses the cap only.
		verts := append(append([]r3.Vec(nil), base...),
			r3.NewVec(0, 0, 5), r3.NewVec(4, 0, 5), r3.NewVec(0, 4, 5),
			r3.NewVec(1, 1, -1), r3.NewVec(1, 1, 1), r3.NewVec(1, 3, 1))
		w := stripWork(verts, [][3]int{{6, 7, 8}}, [][3]int{{0, 1, 2}, {3, 4, 5}})
		require.NoError(t, w.checkStripClear(t.Context(), 1, nil), "the strip alone is clear")
		err := w.checkStripClear(t.Context(), 1, [][2]int{{0, 1}})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Regexp(t, `the cap an end face takes meets another facet of the body at \{1 [12] 0\}`, err.Error())
	})

	t.Run("a facet sharing a held vertex with a taken cap meets it only there", func(t *testing.T) {
		t.Parallel()
		// w.out holds the cap at z = 0, corners 0, 1, 2, then the strip at
		// z = 5. heldN of the vertices are held; the rest the construction
		// made.
		verts := append(append([]r3.Vec(nil), base...),
			r3.NewVec(0, 0, 5), r3.NewVec(4, 0, 5), r3.NewVec(0, 4, 5),
			r3.NewVec(2, -3, 1), r3.NewVec(2, 1, -1), r3.NewVec(2, 1, 1))
		capWork := func(heldN int, rest [3]int) *work {
			w := stripWork(verts, [][3]int{rest}, [][3]int{{0, 1, 2}, {3, 4, 5}})
			w.fp.Verts = verts[:heldN]
			return w
		}
		taken := [][2]int{{0, 1}}

		// Along the shared edge from vertex 0 to vertex 1 only.
		require.NoError(t, capWork(len(verts), [3]int{0, 1, 6}).checkStripClear(t.Context(), 1, taken))

		// Through vertex 0 and on across the cap to (2,1,0).
		err := capWork(len(verts), [3]int{0, 7, 8}).checkStripClear(t.Context(), 1, taken)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		// The contact is the segment from (0,0,0) to (2,1,0); either end names it.
		require.Regexp(t, `the cap an end face takes meets another facet of the body at \{(0 0 0|2 1 0)\}`, err.Error())

		// The same pair through a vertex the construction made is its seam.
		require.NoError(t, capWork(0, [3]int{0, 7, 8}).checkStripClear(t.Context(), 1, taken))
	})
}
