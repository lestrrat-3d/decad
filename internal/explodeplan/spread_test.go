package explodeplan_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// spreadGap is the gap g of every spread test, in mm.
const spreadGap = 2

// runSpread runs the layer spread over plain boxes with no stage: part 0 is the ground. The axis is Z, the layers
// step along Y and X is the third axis.
func runSpread(t *testing.T, boxes []explodeplan.Box, layer []int, scale float64, thin bool) explodeplan.Outcome {
	t.Helper()
	return runPlan(t, boxes, nil, scale, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Shear: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
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

func TestSpreadSettle(t *testing.T) {
	// Gap 2. The base is [0, 10]^2 x [0, 2]. Layer +1 [20, 26] x [0, 6] x [5, 6] stays: its footprint stands apart.
	// Layer +2 [4, 8]^2 x [3, 4] lies over the base and moves 1 to z 4..5. Layer +3 [21, 25] x [1, 5] x [2, 3]
	// lies under layer +1 and moves 6 to z 8..9. Layer -1 [60, 66] x [0, 6] x [-3, -1] stays. Layer -2
	// [2, 8]^2 x [-1, 1] lies inside the base and moves -3 to z -4..-2.
	boxes := []explodeplan.Box{box(0, 0, 0, 10, 10, 2), box(20, 0, 5, 26, 6, 6), box(4, 4, 3, 8, 8, 4),
		box(21, 1, 2, 25, 5, 3), box(60, 0, -3, 66, 6, -1), box(2, 2, -1, 8, 8, 1)}
	layer := []int{0, 1, 2, 3, -1, -2}
	out := runSpread(t, boxes, layer, 1, true)
	require.True(t, out.Spread.Ran)
	want := []float64{0, 0, 1, 6, 0, -3}
	for i, w := range want {
		require.InDelta(t, w, out.Parts[i].Offset.Z, 1e-12, "part %d", i)
		require.InDelta(t, w, out.Parts[i].Slide.Z, 1e-12, "part %d slide", i)
		require.Zero(t, out.Parts[i].Offset.X, "part %d", i)
		require.Zero(t, out.Parts[i].Offset.Y, "part %d", i)
	}
	zs := func(i int) (float64, float64) {
		b := moved(boxes[i], out.Parts[i].Offset)
		return b.Min.Z, b.Max.Z
	}
	for i, w := range map[int][2]float64{2: {4, 5}, 3: {8, 9}, 5: {-4, -2}} {
		lo, hi := zs(i)
		require.InDelta(t, w[0], lo, 1e-12, "part %d", i)
		require.InDelta(t, w[1], hi, 1e-12, "part %d", i)
	}

	// A scale doubles the slide with the offset.
	scaled := runSpread(t, boxes, layer, 2, true)
	for i, p := range scaled.Parts {
		require.Equal(t, out.Parts[i].Offset.Scale(2), p.Offset, "part %d", i)
		require.Equal(t, out.Parts[i].Slide.Scale(2), p.Slide, "part %d slide", i)
	}
}

func TestSpreadSteps(t *testing.T) {
	// Gap 2, Shear Y, Rows X. G is the base. P [10, 20]^2 x [4, 6] (+1), Q [5, 25]^2 x [8, 10] (+2) and
	// R [10, 20]^2 x [12, 14] (+3) lie over it; U [10, 20]^2 x [-6, -4] (-1) and V [5, 25]^2 x [-10, -8] (-2) under.
	const (
		g = iota
		p
		q
		r
		u
		v
	)
	boxes := []explodeplan.Box{box(0, 0, 0, 50, 50, 2), box(10, 10, 4, 20, 20, 6), box(5, 5, 8, 25, 25, 10),
		box(10, 10, 12, 20, 20, 14), box(10, 10, -6, 20, 20, -4), box(5, 5, -10, 25, 25, -8)}
	layer := []int{0, 1, 2, 3, -1, -2}
	mates := []explodeplan.Mate{{A: g, B: p, Coaxial: true}, {A: p, B: q}, {A: q, B: r, Coaxial: true},
		{A: p, B: r, Coaxial: true}, {A: u, B: v}}
	out := runPlan(t, boxes, nil, 1, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Shear: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
		Layer: layer, Gap: spreadGap, Thin: true, Mates: mates})
	require.True(t, out.Spread.Ran)
	// Q steps by 10 - (5 - 2) = 7. R keeps Q's step because their pin stays on its axis, and {P, R}, off its axis
	// after the step, finds P's band of 7 toward -Y already. V steps -7 the same way.
	want := map[int]r3.Vec{g: {}, p: {}, q: {Y: 7}, r: {Y: 7}, u: {}, v: {Y: -7}}
	for i, w := range want {
		require.InDelta(t, w.X, out.Parts[i].Slide.X, 1e-9, "part %d", i)
		require.InDelta(t, w.Y, out.Parts[i].Slide.Y, 1e-9, "part %d", i)
		require.InDelta(t, 0, out.Parts[i].Slide.Z, 1e-9, "part %d", i)
	}
	final := make([]explodeplan.Box, len(boxes))
	for i := range boxes {
		final[i] = moved(boxes[i], out.Parts[i].Offset)
	}
	// ProjectedCover(a, b, v) names the nearer box (0 for a) and the share of the farther one it covers. Along
	// each view of the set the inner part P is in front or only partly covered; likewise U against V.
	for _, tc := range []struct {
		inner, outer int
		views        []r3.Vec
	}{
		{p, q, []r3.Vec{{Z: -1}, {Y: 1, Z: -1}, {Y: 1, Z: -3}, {Y: 3, Z: -1}}},
		{u, v, []r3.Vec{{Z: 1}, {Y: -1, Z: 1}, {Y: -1, Z: 3}}},
	} {
		for _, view := range tc.views {
			front, cover := explodeplan.ProjectedCover(final[tc.inner], final[tc.outer], view)
			require.True(t, front == 0 || cover < 1-1e-9, "the outer part wholly hides the inner one along %v: %d %v %v", view, front, cover, tc)
		}
	}
	front, cover := explodeplan.ProjectedCover(final[p], final[q], r3.Vec{Z: -1})
	require.Equal(t, 1, front)
	require.InDelta(t, 0.8, cover, 1e-9)
	front, cover = explodeplan.ProjectedCover(final[u], final[v], r3.Vec{Z: 1})
	require.Equal(t, 1, front)
	require.InDelta(t, 0.8, cover, 1e-9)
	// Along (0, -1, -1), outside the set, Q covers P with Covered 1.
	front, cover = explodeplan.ProjectedCover(final[p], final[q], r3.Vec{Y: -1, Z: -1})
	require.Equal(t, 1, front)
	require.InDelta(t, 1, cover, 1e-9)
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
	for i, p := range out.Parts {
		require.Equal(t, r3.Vec{}, p.Slide, "part %d: the layers already stand apart and no mate is given", i)
	}

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
		out := runPlan(t, boxes, stages, 1, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Shear: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
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
	out := runPlan(t, boxes, nil, 1, &explodeplan.Spread{Axis: r3.Vec{Z: 1}, Shear: r3.Vec{Y: 1}, Rows: r3.Vec{X: 1},
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
