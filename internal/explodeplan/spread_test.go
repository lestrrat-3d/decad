package explodeplan_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// spreadGap is the gap g of every spread test, in mm.
const spreadGap = 2

// runSpread runs the layer spread over plain boxes with no stage: part 0 is the ground. The axis is Z, a row
// of cells runs along Y and the rows advance along X.
func runSpread(t *testing.T, boxes []explodeplan.Box, layer []int, scale float64, thin bool) explodeplan.Outcome {
	t.Helper()
	order, nodes := explodeplan.Walk(len(boxes), nil, 0)
	partGap := make([]float64, len(boxes))
	for i := range partGap {
		partGap[i] = nan()
	}
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Ground: 0, Order: order, Nodes: nodes,
		DefaultGap: 10, PartGap: partGap, Scale: scale, Blocker: blocker{limit: 1e9},
		Spread: &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Across: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1}, Layer: layer, Gap: spreadGap,
			Thin: thin}})
	require.NoError(t, err)
	return out
}

// moved is the box after the part's offset.
func moved(b explodeplan.Box, o r3.Vec) explodeplan.Box {
	return explodeplan.Box{Min: b.Min.Add(o), Max: b.Max.Add(o)}
}

func TestSpreadSlabs(t *testing.T) {
	// Gap 2. The base is [0, 10]^2 x [0, 2]. Layer +1 at z 5..6 stays (6 is past 2 + 2). Layer +2 at z 3..4
	// would sit beside it, so it moves +5 to z 8..9. Layer -1 at z -3..-1 moves -1 to z -4..-2.
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 2), box(20, 0, 5, 26, 6, 6), box(40, 0, 3, 46, 6, 4),
		box(60, 0, -3, 66, 6, -1)}
	out := runSpread(t, boxes, []int{0, 1, 2, -1}, 1, true)
	require.True(t, out.Spread.Ran)
	want := []float64{0, 0, 5, -1}
	for i, w := range want {
		require.InDelta(t, w, out.Parts[i].Offset.Z, 1e-12, "part %d", i)
		require.InDelta(t, w, out.Parts[i].Slide.Z, 1e-12, "part %d slide", i)
	}
	zs := func(i int) (float64, float64) {
		b := moved(boxes[i], out.Parts[i].Offset)
		return b.Min.Z, b.Max.Z
	}
	lo, hi := zs(2)
	require.InDelta(t, 8, lo, 1e-12)
	require.InDelta(t, 9, hi, 1e-12)
	lo, hi = zs(3)
	require.InDelta(t, -4, lo, 1e-12)
	require.InDelta(t, -2, hi, 1e-12)

	// The push is one stop, the move to the cell another; the last stop is the offset.
	for i, p := range out.Parts {
		require.Equal(t, r3.Vec{}, p.Stops[0], "part %d starts at zero", i)
		require.Equal(t, p.Offset, p.Stops[len(p.Stops)-1], "part %d ends at its offset", i)
	}
	require.Equal(t, r3.Vec{Z: 5}, out.Parts[2].Stops[1], "the push of layer +2")
	require.Len(t, out.Parts[0].Stops, 1, "the base never moves")

	// A scale doubles the slide and every stop with the offset.
	scaled := runSpread(t, boxes, []int{0, 1, 2, -1}, 2, true)
	for i, p := range scaled.Parts {
		require.Equal(t, out.Parts[i].Offset.Scale(2), p.Offset, "part %d", i)
		require.Equal(t, out.Parts[i].Slide.Scale(2), p.Slide, "part %d slide", i)
		require.Equal(t, p.Offset, p.Stops[len(p.Stops)-1], "part %d", i)
	}
}

func TestSpreadGrid(t *testing.T) {
	// Three layers whose 10 x 10 footprints stand apart. Gap 2: one column is 10 x 34 (aspect 3.4), two
	// columns 22 x 22 (aspect 1), three columns 34 x 10.
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 1), box(30, 0, 5, 40, 10, 6), box(60, 0, 10, 70, 10, 11)}
	out := runSpread(t, boxes, []int{0, 1, 2}, 1, true)
	require.True(t, out.Spread.Ran)
	require.False(t, out.Spread.Coaxial)
	require.Equal(t, 2, out.Spread.Columns)
	require.InDelta(t, 22, out.Spread.Width, 1e-9)
	require.InDelta(t, 22, out.Spread.Depth, 1e-9)
	// The base stays; layer 1 takes the next column (y 12..22, as x 0..10) and layer 2 the next row
	// (x 12..22, y 0..10).
	require.Equal(t, r3.Vec{}, out.Parts[0].Offset)
	require.Equal(t, r3.Vec{X: -30, Y: 12}, out.Parts[1].Slide)
	require.Equal(t, r3.Vec{X: -48}, out.Parts[2].Slide)

	// A base and one layer tie at 2.2 for one column and two, and the smaller count wins.
	out = runSpread(t, boxes[:2], []int{0, 1}, 1, true)
	require.Equal(t, 1, out.Spread.Columns)
	require.InDelta(t, 10, out.Spread.Width, 1e-9)
	require.InDelta(t, 22, out.Spread.Depth, 1e-9)

	// A layer below the base lies in the cell before it: base in the middle column.
	out = runSpread(t, boxes, []int{1, 0, -1}, 1, true)
	require.Equal(t, 2, out.Spread.Columns)
	require.Equal(t, r3.Vec{}, out.Parts[1].Slide, "the base cell is anchored at its own footprint")
}

func TestSpreadCleanViews(t *testing.T) {
	// Seven layers, three below the base and three above, of varied size and position along Z.
	type spec struct {
		layer      int
		x, y       float64
		w, d, z, h float64
	}
	specs := []spec{
		{0, 0, 0, 40, 30, 0, 2},
		{1, 5, 3, 12, 7, 3, 2}, {2, -10, 8, 20, 9, 1, 3}, {3, 30, -12, 6, 14, 4, 1},
		{-1, -8, -5, 9, 6, -1, 2}, {-2, 12, 14, 15, 5, -4, 2}, {-3, -20, -9, 7, 11, 0, 3},
	}
	var boxes []explodeplan.Box
	var layer []int
	for _, s := range specs {
		boxes = append(boxes, box(s.x, s.y, s.z, s.x+s.w, s.y+s.d, s.z+s.h))
		layer = append(layer, s.layer)
	}
	out := runSpread(t, boxes, layer, 1, true)
	require.True(t, out.Spread.Ran)
	final := make([]explodeplan.Box, len(boxes))
	for i := range boxes {
		final[i] = moved(boxes[i], out.Parts[i].Offset)
	}
	overlaps := func(view r3.Vec) int {
		n := 0
		for a := range final {
			for b := a + 1; b < len(final); b++ {
				if layer[a] == layer[b] {
					continue
				}
				if _, cover := explodeplan.ProjectedCover(final[a], final[b], view); cover > 0 {
					n++
				}
			}
		}
		return n
	}
	for _, v := range []r3.Vec{{X: 1, Y: 1, Z: -1}, {Z: -1}, {Z: 1}, {X: 1}, {Y: 1}, {Y: 1, Z: -3}, {X: 2, Y: 1, Z: -1}} {
		require.Zero(t, overlaps(v), "no layer hides another along %v", v)
	}
	require.Positive(t, overlaps(r3.Vec{X: -1, Y: -1, Z: -1}), "a view outside the clean set sees a layer hide another")

	// Every cross-layer pair keeps the gap along Z, and along X or Y.
	for a := range final {
		for b := a + 1; b < len(final); b++ {
			if layer[a] == layer[b] {
				continue
			}
			z := math.Max(final[a].Min.Z-final[b].Max.Z, final[b].Min.Z-final[a].Max.Z)
			xy := math.Max(math.Max(final[a].Min.X-final[b].Max.X, final[b].Min.X-final[a].Max.X),
				math.Max(final[a].Min.Y-final[b].Max.Y, final[b].Min.Y-final[a].Max.Y))
			require.GreaterOrEqual(t, z, 2-1e-9, "layers %d and %d along Z", layer[a], layer[b])
			require.GreaterOrEqual(t, xy, 2-1e-9, "layers %d and %d across Z", layer[a], layer[b])
		}
	}
}

func TestSpreadCoaxial(t *testing.T) {
	// Concentric footprints: every centre lies inside every footprint, so nothing moves.
	boxes := []explodeplan.Box{box(-20, -20, 0, 20, 20, 2), box(-10, -10, 5, 10, 10, 7), box(-5, -5, 9, 5, 5, 11)}
	out := runSpread(t, boxes, []int{0, 1, 2}, 1, true)
	require.True(t, out.Spread.Coaxial)
	require.False(t, out.Spread.Ran)
	for i, p := range out.Parts {
		require.Equal(t, r3.Vec{}, p.Offset, "part %d", i)
		require.Equal(t, r3.Vec{}, p.Slide, "part %d", i)
		require.Len(t, p.Stops, 1, "part %d", i)
	}

	// One footprint moved off the axis spreads the layers.
	boxes[2] = box(15, 15, 9, 25, 25, 11)
	out = runSpread(t, boxes, []int{0, 1, 2}, 1, true)
	require.False(t, out.Spread.Coaxial)
	require.True(t, out.Spread.Ran)
	require.NotEqual(t, r3.Vec{}, out.Parts[2].Slide)

	// An assembly that is not thin is reported but never moved.
	out = runSpread(t, boxes, []int{0, 1, 2}, 1, false)
	require.False(t, out.Spread.Coaxial)
	require.False(t, out.Spread.Ran)
	require.Equal(t, r3.Vec{}, out.Parts[2].Offset)
}
