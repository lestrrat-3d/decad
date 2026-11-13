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
	return runPlan(t, boxes, nil, scale, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Across: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
		Layer: layer, Gap: spreadGap, Thin: thin})
}

// runPlan runs the stages, then the spread, over plain boxes: part 0 is the ground and no joint exists.
func runPlan(t *testing.T, boxes []explodeplan.Box, stages []explodeplan.Stage, scale float64, sp *explodeplan.Spread) explodeplan.Outcome {
	t.Helper()
	order, nodes := explodeplan.Walk(len(boxes), nil, 0)
	partGap := make([]float64, len(boxes))
	for i := range partGap {
		partGap[i] = nan()
	}
	out, err := explodeplan.Run(t.Context(), explodeplan.Input{Boxes: boxes, Ground: 0, Order: order, Nodes: nodes,
		Stages: stages, DefaultGap: 10, PartGap: partGap, Scale: scale, Blocker: blocker{limit: 1e9}, Spread: sp})
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

	// A scale doubles the slide with the offset.
	scaled := runSpread(t, boxes, []int{0, 1, 2, -1}, 2, true)
	for i, p := range scaled.Parts {
		require.Equal(t, out.Parts[i].Offset.Scale(2), p.Offset, "part %d", i)
		require.Equal(t, out.Parts[i].Slide.Scale(2), p.Slide, "part %d slide", i)
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

func TestSpreadLayers(t *testing.T) {
	// Ground 0 and six 2 mm cubes on the floor. Explicit stages A (part 1, up) and B (part 2, down) each make a
	// layer, C (part 6) has no direction and no joint so it does not move and stays in the base. Derived parts 3
	// and 4 leave up in rounds 2 and 1, part 5 leaves down in round 1.
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 2)}
	for k := range 6 {
		x := float64(20 + 10*k)
		boxes = append(boxes, box(x, 0, 0, x+2, 2, 2))
	}
	up, down := r3.Vec{Z: 1}, r3.Vec{Z: -1}
	stages := []explodeplan.Stage{
		{Members: []int{1}, Gap: nan(), Clear: explodeplan.ClearAll, Dir: up},
		{Members: []int{2}, Gap: nan(), Clear: explodeplan.ClearAll, Dir: down},
		{Members: []int{6}, Gap: nan(), Clear: explodeplan.ClearAll},
		{Members: []int{3}, Gap: nan(), Clear: explodeplan.ClearAll, Dir: up, Round: 2},
		{Members: []int{4}, Gap: nan(), Clear: explodeplan.ClearAll, Dir: up, Round: 1},
		{Members: []int{5}, Gap: nan(), Clear: explodeplan.ClearAll, Dir: down, Round: 1},
	}
	for _, thin := range []bool{true, false} {
		out := runPlan(t, boxes, stages, 1, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Across: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
			Layer: []int{0, 0, 0, 1, 2, -1, 0}, Gap: spreadGap, Thin: thin})
		require.Equal(t, []int{0, 1, -1, 2, 3, -2, 0}, out.Spread.Layer, "thin %v", thin)
		require.Equal(t, 6, out.Spread.Layers, "thin %v", thin)
		require.Equal(t, r3.Vec{}, out.Parts[6].Offset, "the part with no direction stays")
	}
}

func TestSpreadStacks(t *testing.T) {
	// Gap 2, Z is the axis. G is the base. Layer +1 holds B, a 20 mm plate, S, a pin through it that sticks out
	// above, R, a ring above the pin, and Q, apart from them. Layer -1 holds B2 and its pin S2.
	boxes := []explodeplan.Box{
		box(0, 0, 0, 40, 40, 2),    // 0 G
		box(0, 0, 3, 20, 20, 5),    // 1 B
		box(8, 8, 2, 12, 12, 9),    // 2 S
		box(8, 8, 10, 12, 12, 11),  // 3 R
		box(30, 30, 3, 34, 34, 4),  // 4 Q
		box(50, 0, -5, 70, 20, -3), // 5 B2
		box(58, 8, -9, 62, 12, -2), // 6 S2
	}
	layer := []int{0, 1, 1, 1, 1, -1, -1}
	out := runPlan(t, boxes, nil, 1, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Across: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
		Layer: layer, Gap: spreadGap, Thin: true, Stacks: [][]int{{1, 2}, {5, 6}}})
	require.True(t, out.Spread.Ran)
	require.False(t, out.Spread.Coaxial)
	require.Equal(t, []int{0, 1, 2, 0, 0, 1, 2}, out.Spread.Stack)
	require.Equal(t, 3, out.Spread.Layers)

	slide := func(p int) r3.Vec { return out.Parts[p].Slide }
	// S moves past B grown by 2 (t = 5) and past R, which it meets on the way (t = 11).
	require.InDelta(t, 11, slide(2).Z-slide(1).Z, 1e-9)
	require.InDelta(t, 0, slide(2).X-slide(1).X, 1e-9)
	require.InDelta(t, 0, slide(2).Y-slide(1).Y, 1e-9)
	require.InDelta(t, -5, slide(6).Z-slide(5).Z, 1e-9)
	for _, p := range []int{3, 4} {
		require.Equal(t, slide(1), slide(p), "part %d moves with the layer", p)
	}
	require.Equal(t, slide(5).X, slide(6).X)

	final := make([]explodeplan.Box, len(boxes))
	for i := range boxes {
		final[i] = moved(boxes[i], out.Parts[i].Offset)
	}
	for a := 1; a <= 4; a++ {
		for b := a + 1; b <= 4; b++ {
			require.Positive(t, explodeplan.BoxDistance(final[a], final[b]), "parts %d and %d stand apart", a, b)
		}
	}
	require.GreaterOrEqual(t, final[2].Min.Z-final[3].Max.Z, spreadGap-1e-9, "S is past R by the gap")
}
