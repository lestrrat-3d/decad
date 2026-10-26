package drawing_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/drawing"
	"github.com/lestrrat-3d/decad/internal/raster"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cube returns a 2 mm cube at the origin, wound counter-clockwise seen from outside.
func cube() ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(2, 0, 2), r3.NewVec(2, 2, 2), r3.NewVec(0, 2, 2),
	}
	t := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7},
	}
	return v, t
}

// oblique looks at the cube's (2,2,2) corner from the (0,0,0) side, slightly
// turned so no two edges project onto one line.
var oblique = raster.Camera{Direction: r3.NewVec(0.55, 0.8, 0.45), Up: r3.NewVec(0, 0, 1)}

func length(s drawing.Segment) float64 { return math.Hypot(s.B[0]-s.A[0], s.B[1]-s.A[1]) }

func TestProjectCubeFromAboveACornerHasNineVisibleAndThreeHiddenEdges(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := drawing.Project(t.Context(), v, tris, oblique)
	require.NoError(t, err)
	// Looking along (0.55, 0.8, 0.45) the camera is at the (0,0,0) corner's
	// side: the far corner (2,2,2) hides the three edges that meet at it.
	require.Len(t, got.Visible, 9)
	require.Len(t, got.Hidden, 3)
	right, up, _ := oblique.Axes()
	far := r3.NewVec(2, 2, 2)
	farPoint := [2]float64{right.Dot(far), up.Dot(far)}
	for _, s := range got.Hidden {
		touches := math.Hypot(s.A[0]-farPoint[0], s.A[1]-farPoint[1]) < 1e-9 ||
			math.Hypot(s.B[0]-farPoint[0], s.B[1]-farPoint[1]) < 1e-9
		require.True(t, touches, "a hidden edge meets at the far corner: %v", s)
	}
	// Each cube edge is whole in the drawing: its projected length is 2 mm
	// scaled by the share of the edge that lies in the image plane.
	total := 0.0
	for _, s := range append(append([]drawing.Segment(nil), got.Visible...), got.Hidden...) {
		total += length(s)
	}
	want := 0.0
	for _, edge := range [][2]int{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}} {
		d := v[edge[1]].Sub(v[edge[0]])
		want += math.Hypot(right.Dot(d), up.Dot(d))
	}
	require.InDelta(t, want, total, 1e-9, "no edge is split or lost")
}

func TestProjectLeavesOutTheDiagonalsOfFlatFaces(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := drawing.Project(t.Context(), v, tris, oblique)
	require.NoError(t, err)
	// 12 triangulated faces' diagonals would add six more segments.
	require.Len(t, got.Visible, 9)
}

func TestProjectDrawsASilhouetteWhereACurvedSurfaceTurnsAway(t *testing.T) {
	t.Parallel()
	// A 24-sided prism of radius 5 and height 10 seen from the side along +Y:
	// the side wall is smooth (each facet turns 15 degrees, below the crease
	// angle), so only the silhouette and the rims are features.
	const sides = 24
	var v []r3.Vec
	for i := range sides {
		a := 2 * math.Pi * float64(i) / sides
		v = append(v, r3.NewVec(5*math.Cos(a), 5*math.Sin(a), 0), r3.NewVec(5*math.Cos(a), 5*math.Sin(a), 10))
	}
	var tris [][3]int
	for i := range sides {
		b0, t0 := 2*i, 2*i+1
		b1, t1 := 2*((i+1)%sides), 2*((i+1)%sides)+1
		tris = append(tris, [3]int{b0, b1, t1}, [3]int{b0, t1, t0})
	}
	// Caps: fans from vertex 0 (counter-clockwise from above for the top).
	for i := 1; i < sides-1; i++ {
		tris = append(tris, [3]int{1, 2*i + 1, 2*(i+1) + 1}, [3]int{0, 2 * (i + 1), 2 * i})
	}
	got, err := drawing.Project(t.Context(), v, tris, raster.Camera{Direction: r3.NewVec(0, 1, 0), Up: r3.NewVec(0, 0, 1)})
	require.NoError(t, err)
	minX, maxX, minY, maxY := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, s := range got.Visible {
		for _, p := range [][2]float64{s.A, s.B} {
			minX, maxX = math.Min(minX, p[0]), math.Max(maxX, p[0])
			minY, maxY = math.Min(minY, p[1]), math.Max(maxY, p[1])
		}
	}
	require.InDelta(t, 10, maxX-minX, 1e-9, "the silhouette spans the 10 mm diameter")
	require.InDelta(t, 10, maxY-minY, 1e-9, "and the 10 mm height")
	// The wall's facet edges between the silhouettes are not features.
	for _, s := range got.Visible {
		vertical := math.Abs(s.A[0]-s.B[0]) < 1e-12 && math.Abs(length(s)-10) < 1e-9
		if vertical {
			require.InDelta(t, 5, math.Abs(s.A[0]), 1e-9, "a vertical line only at a silhouette")
		}
	}
}

func TestProjectRefusals(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	_, err := drawing.Project(t.Context(), v, tris, raster.Camera{Up: r3.NewVec(0, 0, 1)})
	require.ErrorIs(t, err, raster.ErrDegenerate)
	_, err = drawing.Project(t.Context(), v, nil, oblique)
	require.ErrorIs(t, err, raster.ErrDegenerate)
}

// ringSegments are the n chords of a polygon with its corners on the circle c.
func ringSegments(c drawing.Circle, n int) []drawing.Segment {
	pt := func(i int) [2]float64 {
		a := 2 * math.Pi * float64(i%n) / float64(n)
		return [2]float64{c.Centre[0] + c.Radius*math.Cos(a), c.Centre[1] + c.Radius*math.Sin(a)}
	}
	out := make([]drawing.Segment, n)
	for i := range out {
		out[i] = drawing.Segment{A: pt(i), B: pt(i + 1)}
	}
	return out
}

func TestCirculateTakesAWholeRingAndLeavesOtherSegments(t *testing.T) {
	t.Parallel()
	c := drawing.Circle{Centre: [2]float64{3, 4}, Radius: 5}
	other := drawing.Segment{A: [2]float64{0, 0}, B: [2]float64{1, 0}}
	lines := drawing.Lines{Visible: append(ringSegments(c, 48), other)}
	lines.Circulate([]drawing.Circle{c})
	require.Equal(t, []drawing.Circle{c}, lines.VisibleCircles)
	require.Equal(t, []drawing.Segment{other}, lines.Visible, "only the ring's chords are taken")
	require.Empty(t, lines.HiddenCircles)
}

func TestCirculateTakesAHiddenRingToo(t *testing.T) {
	t.Parallel()
	c := drawing.Circle{Radius: 2}
	lines := drawing.Lines{Hidden: ringSegments(c, 32)}
	lines.Circulate([]drawing.Circle{c})
	require.Equal(t, []drawing.Circle{c}, lines.HiddenCircles)
	require.Empty(t, lines.Hidden)
	require.Empty(t, lines.VisibleCircles)
}

func TestCirculateLeavesARingWithAChordMissing(t *testing.T) {
	t.Parallel()
	c := drawing.Circle{Radius: 5}
	ring := ringSegments(c, 48)
	lines := drawing.Lines{Visible: ring[1:]}
	lines.Circulate([]drawing.Circle{c})
	require.Empty(t, lines.VisibleCircles, "one chord short of the whole way round")
	require.Len(t, lines.Visible, 47)
}

func TestCirculateLeavesARingWhoseChordIsCutShort(t *testing.T) {
	t.Parallel()
	c := drawing.Circle{Radius: 5}
	ring := ringSegments(c, 48)
	// A surface hides the far half of one chord: its visible run ends inside the chord, off the circle.
	ring[3].B = [2]float64{(ring[3].A[0] + ring[3].B[0]) / 2, (ring[3].A[1] + ring[3].B[1]) / 2}
	lines := drawing.Lines{Visible: ring}
	lines.Circulate([]drawing.Circle{c})
	require.Empty(t, lines.VisibleCircles)
	require.Len(t, lines.Visible, 48)
}

func TestCirculateArcsTakesAWholeArcAndLeavesItsNeighbours(t *testing.T) {
	t.Parallel()
	c := drawing.Circle{Centre: [2]float64{1, 2}, Radius: 5}
	ring := ringSegments(c, 48)
	// A quarter arc from 90 to 180 degrees is the chords from index 12 to 24; the other chords are
	// other edges of the drawing.
	arc := drawing.Arc{Centre: c.Centre, Radius: 5, Start: math.Pi / 2, End: math.Pi}
	lines := drawing.Lines{Visible: ring}
	lines.CirculateArcs([]drawing.Arc{arc})
	require.Equal(t, []drawing.Arc{arc}, lines.VisibleArcs)
	require.Len(t, lines.Visible, 48-12, "the quarter's twelve chords are taken")
}

func TestCirculateArcsLeavesAnArcWithAChordMissing(t *testing.T) {
	t.Parallel()
	c := drawing.Circle{Radius: 5}
	ring := ringSegments(c, 48)
	arc := drawing.Arc{Radius: 5, Start: math.Pi / 2, End: math.Pi}
	lines := drawing.Lines{Visible: append(append([]drawing.Segment(nil), ring[:15]...), ring[16:]...)}
	lines.CirculateArcs([]drawing.Arc{arc})
	require.Empty(t, lines.VisibleArcs)
	require.Len(t, lines.Visible, 47)
}

func TestArcBoundsIncludeTheExtremesItPassesThrough(t *testing.T) {
	t.Parallel()
	// A half circle from -45 to 135 degrees passes through 0 and 90: its box reaches the radius at x and y.
	a := drawing.Arc{Centre: [2]float64{10, 10}, Radius: 4, Start: -math.Pi / 4, End: 3 * math.Pi / 4}
	minX, maxX, minY, maxY := a.Bounds()
	require.InDelta(t, 14, maxX, 1e-12)
	require.InDelta(t, 14, maxY, 1e-12)
	require.InDelta(t, 10-4*math.Sqrt2/2, minX, 1e-12)
	require.InDelta(t, 10-4*math.Sqrt2/2, minY, 1e-12)
}
