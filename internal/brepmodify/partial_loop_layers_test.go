package brepmodify

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// testLoop is a loop of a z-normal face with its frame at the origin, whose material lies to the left of the walk.
func testLoop(segs ...loopSeg) *chamferLoop {
	return &chamferLoop{segs: segs, outer: true, materialLeft: true, origin: r3.Vec{}, u: r3.Vec{X: 1}, v: r3.Vec{Y: 1},
		n: r3.Vec{Z: 1}, depth: 100}
}

func lineSeg(a, b pt2) loopSeg { return loopSeg{p0: a, p1: b} }

func arcSeg(c pt2, r float64, p0, p1 pt2, ccw bool) loopSeg {
	return loopSeg{arc: true, p0: p0, p1: p1, c: c, r: r, ccw: ccw}
}

// keyholeHalfSlot is the half width of the keyhole's slot.
const keyholeHalfSlot = 0.5

// keyholeLoop is the loop of the keyhole plate's face of the gallery's slot rim: a bore of radius r about the origin
// opening into a slot of half width w through the bottom edge. A positive flat is a chord across the bore at y = flat.
// The arcs run clockwise from the left wall to the right one, with the material outside them.
func keyholeLoop(r, flat float64) *chamferLoop {
	const w = keyholeHalfSlot
	y0 := math.Sqrt(r*r - w*w)
	o := pt2{}
	segs := []loopSeg{
		lineSeg(pt2{-10, -10}, pt2{-w, -10}),
		lineSeg(pt2{-w, -10}, pt2{-w, -y0}),
	}
	if flat > 0 {
		qx := math.Sqrt(r*r - flat*flat)
		segs = append(segs,
			arcSeg(o, r, pt2{-w, -y0}, pt2{-qx, flat}, false),
			lineSeg(pt2{-qx, flat}, pt2{qx, flat}),
			arcSeg(o, r, pt2{qx, flat}, pt2{w, -y0}, false))
	} else {
		segs = append(segs, arcSeg(o, r, pt2{-w, -y0}, pt2{w, -y0}, false))
	}
	segs = append(segs,
		lineSeg(pt2{w, -y0}, pt2{w, -10}),
		lineSeg(pt2{w, -10}, pt2{10, -10}),
		lineSeg(pt2{10, -10}, pt2{10, 10}),
		lineSeg(pt2{10, 10}, pt2{-10, 10}),
		lineSeg(pt2{-10, 10}, pt2{-10, -10}))
	return testLoop(segs...)
}

// cornerCase is a chain of two segments that meet at a sharp corner, one of them an arc.
type cornerCase struct {
	name string
	loop *chamferLoop
}

func cornerCases() []cornerCase {
	// A line along y = 0 into a clockwise arc about (30,0): the corner at (20,0) lies on the line through the centre.
	lineArc := testLoop(
		lineSeg(pt2{0, 0}, pt2{20, 0}),
		arcSeg(pt2{30, 0}, 10, pt2{20, 0}, pt2{30, 10}, false),
		lineSeg(pt2{30, 10}, pt2{30, 20}), lineSeg(pt2{30, 20}, pt2{0, 20}), lineSeg(pt2{0, 20}, pt2{0, 0}))
	// Two counter-clockwise arcs, so that both offsets move toward their centres, meeting at (7.5, 6.614).
	vy := math.Sqrt(100 - 7.5*7.5)
	v := pt2{7.5, vy}
	sameSign := testLoop(
		arcSeg(pt2{0, 0}, 10, pt2{10, 0}, v, true),
		arcSeg(pt2{12, 0}, 8, v, pt2{4, 0}, true),
		lineSeg(pt2{4, 0}, pt2{10, 0}))
	// The second arc clockwise, so that its offset moves away from its centre.
	oppositeSign := testLoop(
		arcSeg(pt2{0, 0}, 10, pt2{10, 0}, v, true),
		arcSeg(pt2{12, 0}, 8, v, pt2{12, 8}, false),
		lineSeg(pt2{12, 8}, pt2{10, 0}))
	return []cornerCase{{"line then arc", lineArc}, {"two arcs, same sign", sameSign},
		{"two arcs, opposite sign", oppositeSign}}
}

// TestLayerDepartureBoundsCornerPath compares the closed form with the exact path of the corner: between two rings the
// cutter joins the corner's positions by a straight edge, and the corner at every offset in between lies within the
// bound of that edge.
func TestLayerDepartureBoundsCornerPath(t *testing.T) {
	t.Parallel()
	const (
		d      = 0.5
		top    = 0.25
		bottom = -0.625
		perRun = 512
	)
	for _, c := range cornerCases() {
		for _, layers := range []int{2, 4, 8} {
			t.Run(c.name, func(t *testing.T) {
				t.Parallel()
				r := chamferRun{loop: c.loop, first: 0, count: 2}
				a, b := r.seg(0), r.seg(1)
				for k := range layers {
					t0 := d + top + (bottom-top)*float64(k)/float64(layers)
					t1 := d + top + (bottom-top)*float64(k+1)/float64(layers)
					p, err := c.loop.corner(a, b, t0)
					require.NoError(t, err)
					q, err := c.loop.corner(a, b, t1)
					require.NoError(t, err)
					bound, err := c.loop.cornerPathDeparture(a, b, t0, t1, p, q)
					require.NoError(t, err)
					bound *= (t1 - t0) * (t1 - t0) / 8
					sampled := 0.0
					for i := 1; i < perRun; i++ {
						f := float64(i) / perRun
						exact, err := c.loop.corner(a, b, t0+f*(t1-t0))
						require.NoError(t, err)
						chord := p.add(q.sub(p).scale(f))
						sampled = max(sampled, exact.dist(chord))
					}
					require.LessOrEqual(t, sampled, bound*(1+1e-9)+1e-15, "layer %d of %d", k, layers)
					// The bound is a bound on the corner path, not a slack allowance.
					require.LessOrEqual(t, bound, 20*sampled+1e-12, "layer %d of %d", k, layers)
				}
			})
		}
	}
}

func TestLayerDepartureRefusesArcFoot(t *testing.T) {
	t.Parallel()
	loop := cornerCases()[0].loop
	a, b := loop.segs[0], loop.segs[1]
	// A corner at the foot of the arc's centre on the line, v = 0 at one end of the layer.
	_, err := loop.cornerPathDeparture(a, b, 0, 0.1, pt2{30, 0}, pt2{20, 0})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

// TestPartialLayersFollowTheCorners keeps two rings for a chain without a sharp corner beside an arc and steps through
// layers, until the charge is within the sagitta, for one with such a corner.
func TestPartialLayersFollowTheCorners(t *testing.T) {
	t.Parallel()
	const (
		d      = 0.5
		top    = 0.25
		bottom = -0.625
	)
	t.Run("one arc between two lines", func(t *testing.T) {
		t.Parallel()
		r := chamferRun{loop: keyholeLoop(4, 0), first: 2, count: 1}
		layers, dep, err := r.layers(d, top, bottom)
		require.NoError(t, err)
		require.Equal(t, 1, layers)
		require.Zero(t, dep)
	})
	for _, flat := range []float64{3, 1.5} {
		t.Run("arcs and the flat of a D", func(t *testing.T) {
			t.Parallel()
			r := chamferRun{loop: keyholeLoop(4, flat), first: 2, count: 3}
			layers, dep, err := r.layers(d, top, bottom)
			require.NoError(t, err)
			require.Greater(t, layers, 1)
			require.LessOrEqual(t, layers, maxPartialLayers)
			require.LessOrEqual(t, dep, partialSagitta)
			requireFewestLayers(t, r, d, top, bottom, layers)
		})
	}
	t.Run("line then arc", func(t *testing.T) {
		t.Parallel()
		r := chamferRun{loop: cornerCases()[0].loop, first: 0, count: 2}
		layers, dep, err := r.layers(d, top, bottom)
		require.NoError(t, err)
		require.Greater(t, layers, 2)
		require.LessOrEqual(t, dep, partialSagitta)
	})
}

// requireFewestLayers checks that layers is the fewest layers of rings between top and bottom that hold the bevel: it
// puts no ring near the wall's level, and every smaller count that also puts none there misses partialSagitta.
func requireFewestLayers(t *testing.T, r chamferRun, d, top, bottom float64, layers int) {
	t.Helper()
	require.False(t, ringNearWall(d, top, bottom, layers))
	for fewer := 2; fewer < layers; fewer++ {
		if ringNearWall(d, top, bottom, fewer) {
			continue
		}
		heights := make([]float64, fewer+1)
		for k := range heights {
			heights[k] = top + (bottom-top)*float64(k)/float64(fewer)
		}
		dep, err := r.layerDeparture(d, heights)
		require.NoError(t, err)
		require.Greater(t, dep, partialSagitta, "%d layers", fewer)
	}
}

// TestRingNearWallPassesOverRingsAtTheWall reads the layer counts a cutter between top and bottom passes over. With an
// overshoot of half the setback, the wall's level lies six sevenths of the way down, so every seventh count puts a ring
// on the wall itself. A count whose nearest ring lies within a quarter of a layer of that level is passed over, and one
// whose nearest ring lies farther is kept.
func TestRingNearWallPassesOverRingsAtTheWall(t *testing.T) {
	t.Parallel()
	const d, over = 0.2, 0.1
	top, bottom := over, -(d + over/2)
	for layers := 2; layers <= maxPartialLayers; layers++ {
		step := (top - bottom) / float64(layers)
		nearest := math.Inf(1)
		for k := 1; k < layers; k++ {
			nearest = min(nearest, math.Abs(top-float64(k)*step+d))
		}
		require.Equal(t, nearest < step/4, ringNearWall(d, top, bottom, layers), "%d layers", layers)
		if layers%7 == 0 {
			require.True(t, ringNearWall(d, top, bottom, layers), "%d layers", layers)
		}
	}
	require.False(t, ringNearWall(d, top, bottom, 1))
}

// TestArcRunOutFollowsItsCircle reads the points an arc run's cutter holds past its convex ends: every ring holds the
// same number of them, and they lie on the arc's moved circle, the source arc's reach in.
func TestArcRunOutFollowsItsCircle(t *testing.T) {
	t.Parallel()
	const d, over = 0.5, 0.25
	for _, r := range []float64{2.55, 4, 4.05} {
		run := chamferRun{loop: keyholeLoop(r, 0), first: 2, count: 1}
		ends, ok := run.ends()
		require.True(t, ok)
		runOut, err := run.runOutArcs(ends, d, over)
		require.NoError(t, err)
		cells := []int{1 + int(math.Ceil(run.seg(0).sweep()/0.01))}
		seg := run.seg(0)
		var counts []int
		for _, tt := range []float64{-0.25, 0, 0.2, d + over} {
			pts, _, err := run.polyline(tt, 0, 0, cells, runOut)
			require.NoError(t, err)
			counts = append(counts, len(pts))
			for _, p := range pts {
				require.InDelta(t, r+tt, p.dist(seg.c), 1e-9, "radius %v moved by %v", r, tt)
			}
			// The first and last points turn the run-out angle past the arc's own ends, along the circle.
			a0 := math.Atan2(seg.p0.y, seg.p0.x)
			a1 := math.Atan2(seg.p1.y, seg.p1.x)
			first, last := pts[0], pts[len(pts)-1]
			require.InDelta(t, math.Abs(runOut.alpha[0]), math.Abs(math.Remainder(math.Atan2(first.y, first.x)-a0, 2*math.Pi)), 1e-9)
			require.InDelta(t, math.Abs(runOut.alpha[1]), math.Abs(math.Remainder(math.Atan2(last.y, last.x)-a1, 2*math.Pi)), 1e-9)
		}
		for _, n := range counts {
			require.Equal(t, counts[0], n)
		}
	}
}

// TestPartialCutterBound charges the layered cutter's departure: a chain without a corner beside an arc keeps the
// sagitta alone, and a chain with one pays the departure on top, within twice the sagitta.
func TestPartialCutterBound(t *testing.T) {
	t.Parallel()
	const d, over = 0.5, 0.25
	arc := chamferRun{loop: keyholeLoop(4, 0), first: 2, count: 1}
	m, err := arc.cutterMesh(t.Context(), d, over)
	require.NoError(t, err)
	require.InDelta(t, partialSagitta, m.bound, 1e-15)
	dee := chamferRun{loop: keyholeLoop(4, 3), first: 2, count: 3}
	m, err = dee.cutterMesh(t.Context(), d, over)
	require.NoError(t, err)
	require.Greater(t, m.bound, partialSagitta)
	require.LessOrEqual(t, m.bound, 2*partialSagitta)
}
