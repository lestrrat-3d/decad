package meshbody_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbody"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cube returns a 2 mm cube at the origin, wound counter-clockwise seen from
// outside, as 8 vertices and 12 triangles.
func cube() ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(2, 0, 2), r3.NewVec(2, 2, 2), r3.NewVec(0, 2, 2),
	}
	t := [][3]int{
		{0, 2, 1}, {0, 3, 2}, // bottom, normal -z
		{4, 5, 6}, {4, 6, 7}, // top
		{0, 1, 5}, {0, 5, 4}, // front, normal -y
		{2, 3, 7}, {2, 7, 6}, // back
		{1, 2, 6}, {1, 6, 5}, // right
		{3, 0, 4}, {3, 4, 7}, // left
	}
	return v, t
}

func TestAnalyzeClosedCube(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := meshbody.Analyze(t.Context(), v, tris)
	require.NoError(t, err)
	require.True(t, got.Closed())
	require.Equal(t, 1, got.Shells)
	require.Equal(t, "8", got.SignedVolume.RatString(), "a 2 mm cube holds 8 mm^3")
	require.Equal(t, "24", got.AreaLow.RatString(), "six 2 x 2 faces; the right triangles' lengths are exact squares")
	require.Zero(t, got.AreaLow.Cmp(got.AreaHigh))
}

func TestAnalyzeInvertedCubeHasNegativeVolume(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	for i := range tris {
		tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
	}
	got, err := meshbody.Analyze(t.Context(), v, tris)
	require.NoError(t, err)
	require.True(t, got.Closed())
	require.Equal(t, "-8", got.SignedVolume.RatString())
}

func TestAnalyzeOpenCubeHasBoundary(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := meshbody.Analyze(t.Context(), v, tris[2:]) // no bottom
	require.NoError(t, err)
	require.False(t, got.Closed())
	require.Equal(t, 4, got.BoundaryEdges, "the missing bottom leaves its four edges free")
}

func TestAnalyzeMisorientedTriangle(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	tris[0][1], tris[0][2] = tris[0][2], tris[0][1]
	got, err := meshbody.Analyze(t.Context(), v, tris)
	require.NoError(t, err)
	require.False(t, got.Closed())
	require.Positive(t, got.MisorientedEdges)
}

func TestAnalyzeNonManifoldEdge(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	v = append(v, r3.Vec{X: 1, Y: 1, Z: -3})
	tris = append(tris, [3]int{0, 1, 8}) // a third triangle on the edge 0-1
	got, err := meshbody.Analyze(t.Context(), v, tris)
	require.NoError(t, err)
	require.False(t, got.Closed())
	require.Equal(t, 1, got.NonManifoldEdges)
}

func TestAnalyzeDegenerateTriangles(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	v = append(v, r3.Vec{X: 5}) // collinear with vertices 0 and 1 on the x axis
	tris = append(tris, [3]int{0, 0, 1}, [3]int{0, 1, 0}, [3]int{0, 8, 1})
	got, err := meshbody.Analyze(t.Context(), v, tris)
	require.NoError(t, err)
	require.Equal(t, 3, got.DegenerateTriangles, "two repeated vertices and one collinear triple")
	require.True(t, got.Closed(), "degenerate triangles join no edge")
	require.Equal(t, "8", got.SignedVolume.RatString())
}

func TestAnalyzeCountsShells(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	shifted := make([]r3.Vec, len(v))
	for i, p := range v {
		shifted[i] = p.Add(r3.Vec{X: 10})
	}
	all := append(append([]r3.Vec(nil), v...), shifted...)
	more := append([][3]int(nil), tris...)
	for _, tri := range tris {
		more = append(more, [3]int{tri[0] + 8, tri[1] + 8, tri[2] + 8})
	}
	got, err := meshbody.Analyze(t.Context(), all, more)
	require.NoError(t, err)
	require.True(t, got.Closed())
	require.Equal(t, 2, got.Shells)
	require.Equal(t, "16", got.SignedVolume.RatString())
}

func TestAnalyzeAreaEnclosesIrrationalSides(t *testing.T) {
	t.Parallel()
	// A right triangle with legs 1 and 1 has area 1/2; the slanted one has
	// area sqrt(3)/4 · (sqrt 2)^2 = sqrt(3)/2.
	got, err := meshbody.Analyze(t.Context(), []r3.Vec{{}, {X: 1, Y: 1}, {Y: 1, Z: 1}}, [][3]int{{0, 1, 2}})
	require.NoError(t, err)
	low, _ := got.AreaLow.Float64()
	high, _ := got.AreaHigh.Float64()
	require.LessOrEqual(t, low, math.Sqrt(3)/2)
	require.GreaterOrEqual(t, high, math.Sqrt(3)/2)
	require.Less(t, high-low, 1e-15)
}

func TestAnalyzeRefusals(t *testing.T) {
	t.Parallel()
	_, err := meshbody.Analyze(t.Context(), []r3.Vec{{}, {X: math.NaN()}, {Y: 1}}, [][3]int{{0, 1, 2}})
	require.Error(t, err, "a NaN vertex")
	_, err = meshbody.Analyze(t.Context(), []r3.Vec{{}, {X: 1}, {Y: 1}}, [][3]int{{0, 1, 3}})
	require.Error(t, err, "an index out of range")
	_, err = meshbody.Analyze(t.Context(), []r3.Vec{{}, {X: 1}, {Y: 1}}, [][3]int{{0, 1, -1}})
	require.Error(t, err, "a negative index")
}

func TestIntervalOfAnExactValueHasNoBound(t *testing.T) {
	t.Parallel()
	v, b, ok := meshbody.Interval(big.NewRat(8, 1), big.NewRat(8, 1))
	require.True(t, ok)
	require.Equal(t, 8.0, v)
	require.Zero(t, b)
	v, b, ok = meshbody.Interval(big.NewRat(1, 3), big.NewRat(1, 3))
	require.True(t, ok)
	require.InDelta(t, 1.0/3, v, 1e-16)
	require.Positive(t, b, "a third is not a float")
}
