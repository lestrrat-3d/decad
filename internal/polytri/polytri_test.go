package polytri_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polytri"
	"github.com/stretchr/testify/require"
)

func area(points []polytri.Point, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		a, b, c := points[t[0]], points[t[1]], points[t[2]]
		total += ((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])) / 2
	}
	return total
}

func requireCounterClockwise(t *testing.T, points []polytri.Point, tris [][3]int) {
	t.Helper()
	for _, tri := range tris {
		a, b, c := points[tri[0]], points[tri[1]], points[tri[2]]
		require.Positive(t, (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]), "a triangle winds clockwise or has no area")
	}
}

func TestTriangulateSquare(t *testing.T) {
	t.Parallel()
	points, tris, err := polytri.Triangulate([]polytri.Point{{0, 0}, {4, 0}, {4, 3}, {0, 3}}, nil)
	require.NoError(t, err)
	require.Len(t, tris, 2)
	require.InDelta(t, 12, area(points, tris), 1e-12)
	requireCounterClockwise(t, points, tris)
}

func TestTriangulateConcaveL(t *testing.T) {
	t.Parallel()
	outer := []polytri.Point{{0, 0}, {6, 0}, {6, 2}, {2, 2}, {2, 6}, {0, 6}}
	points, tris, err := polytri.Triangulate(outer, nil)
	require.NoError(t, err)
	require.Len(t, tris, 4)
	require.InDelta(t, 6*2+2*4, area(points, tris), 1e-12)
	requireCounterClockwise(t, points, tris)
}

func TestTriangulateSquareWithAHole(t *testing.T) {
	t.Parallel()
	outer := []polytri.Point{{0, 0}, {10, 0}, {10, 10}, {0, 10}}
	hole := []polytri.Point{{3, 3}, {3, 7}, {7, 7}, {7, 3}} // clockwise
	points, tris, err := polytri.Triangulate(outer, [][]polytri.Point{hole})
	require.NoError(t, err)
	require.InDelta(t, 100-16, area(points, tris), 1e-9)
	requireCounterClockwise(t, points, tris)
}

func TestTriangulateTwoHolesAndACollinearPoint(t *testing.T) {
	t.Parallel()
	outer := []polytri.Point{{0, 0}, {5, 0}, {10, 0}, {10, 6}, {0, 6}} // (5, 0) is collinear
	h1 := []polytri.Point{{1, 1}, {1, 3}, {3, 3}, {3, 1}}
	h2 := []polytri.Point{{6, 2}, {6, 5}, {9, 5}, {9, 2}}
	points, tris, err := polytri.Triangulate(outer, [][]polytri.Point{h1, h2})
	require.NoError(t, err)
	require.InDelta(t, 60-4-9, area(points, tris), 1e-9)
	requireCounterClockwise(t, points, tris)
}

func TestTriangulateAPolygonWithManySides(t *testing.T) {
	t.Parallel()
	const n = 40
	outer := make([]polytri.Point, n)
	for i := range outer {
		a := 2 * math.Pi * float64(i) / n
		r := 10.0
		if i%2 == 1 {
			r = 6 // a star: concave at every second vertex
		}
		outer[i] = polytri.Point{r * math.Cos(a), r * math.Sin(a)}
	}
	points, tris, err := polytri.Triangulate(outer, nil)
	require.NoError(t, err)
	require.Len(t, tris, n-2)
	// The shoelace area of the star is the triangulation's.
	want := 0.0
	for i := range outer {
		j := (i + 1) % n
		want += outer[i][0]*outer[j][1] - outer[j][0]*outer[i][1]
	}
	require.InDelta(t, want/2, area(points, tris), 1e-9)
	requireCounterClockwise(t, points, tris)
}

func TestTriangulateRefuses(t *testing.T) {
	t.Parallel()
	square := []polytri.Point{{0, 0}, {4, 0}, {4, 4}, {0, 4}}
	for name, call := range map[string]func() error{
		"two points": func() error { _, _, err := polytri.Triangulate(square[:2], nil); return err },
		"clockwise outer": func() error {
			_, _, err := polytri.Triangulate([]polytri.Point{{0, 0}, {0, 4}, {4, 4}, {4, 0}}, nil)
			return err
		},
		"a counter-clockwise hole": func() error {
			_, _, err := polytri.Triangulate(square, [][]polytri.Point{{{1, 1}, {2, 1}, {2, 2}, {1, 2}}})
			return err
		},
		"a hole outside": func() error {
			_, _, err := polytri.Triangulate(square, [][]polytri.Point{{{10, 1}, {10, 2}, {11, 2}, {11, 1}}})
			return err
		},
		"collinear points": func() error {
			_, _, err := polytri.Triangulate([]polytri.Point{{0, 0}, {1, 0}, {2, 0}}, nil)
			return err
		},
	} {
		require.ErrorIs(t, call(), polytri.ErrDegenerate, name)
	}
}
