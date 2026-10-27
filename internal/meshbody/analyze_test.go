package meshbody_test

import (
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbody"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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

func TestAnalyzeShellsWithInterleavedDegenerateTriangles(t *testing.T) {
	t.Parallel()
	vertices, faces := cube()
	for _, p := range slices.Clone(vertices) {
		vertices = append(vertices, p.Add(r3.Vec{X: 10}))
	}
	triangles := [][3]int{{0, 12, 12}}
	for _, tri := range faces {
		triangles = append(triangles, tri, [3]int{0, 12, 12},
			[3]int{tri[0] + 8, tri[1] + 8, tri[2] + 8}, [3]int{0, 12, 12})
	}
	for order := range 2 {
		got, err := meshbody.Analyze(t.Context(), vertices, triangles)
		require.NoError(t, err)
		require.True(t, got.Closed())
		require.Equal(t, 2, got.Shells, "degenerate triangles cannot join the cubes")
		require.Equal(t, 25, got.DegenerateTriangles)
		require.Equal(t, "16", got.SignedVolume.RatString())
		require.Equal(t, "48", got.AreaLow.RatString())
		require.Equal(t, "48", got.AreaHigh.RatString())
		// These two live faces connect edges from the separate cubes.
		bridged := append(slices.Clone(triangles), [3]int{0, 1, 12}, [3]int{1, 12, 13})
		got, err = meshbody.Analyze(t.Context(), vertices, bridged)
		require.NoError(t, err)
		require.False(t, got.Closed())
		require.Equal(t, 1, got.Shells, "order %d", order)
		require.Equal(t, 2, got.NonManifoldEdges)
		require.Equal(t, 1, got.MisorientedEdges)
		require.Equal(t, 2, got.BoundaryEdges)
		require.Equal(t, 25, got.DegenerateTriangles)
		volume, low, high, _ := rationalGeometry(vertices, bridged)
		require.Zero(t, got.SignedVolume.Cmp(volume))
		require.Zero(t, got.AreaLow.Cmp(low))
		require.Zero(t, got.AreaHigh.Cmp(high))
		slices.Reverse(triangles)
	}
}

func TestAnalyzeRepeatedEdgesStayNonManifold(t *testing.T) {
	t.Parallel()
	vertices, faces := cube()
	triangles := make([][3]int, 512)
	for i := range triangles {
		triangles[i] = faces[0]
		if i%2 != 0 {
			triangles[i][1], triangles[i][2] = triangles[i][2], triangles[i][1]
		}
	}
	got, err := meshbody.Analyze(t.Context(), vertices, triangles)
	require.NoError(t, err)
	require.False(t, got.Closed())
	require.Equal(t, 3, got.NonManifoldEdges)
	require.Zero(t, got.BoundaryEdges)
	require.Zero(t, got.MisorientedEdges)
	require.Zero(t, got.DegenerateTriangles)
	require.Equal(t, 1, got.Shells)
	require.Equal(t, "0", got.SignedVolume.RatString())
	require.Equal(t, "1024", got.AreaLow.RatString())
	require.Equal(t, "1024", got.AreaHigh.RatString())
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

func TestAnalyzeMatchesIndependentRationalGeometry(t *testing.T) {
	t.Parallel()
	base := []r3.Vec{{X: 0.1, Y: -0.2, Z: 0.3}, {X: 1.1, Y: 0.25, Z: -0.7},
		{X: -0.4, Y: 1.7, Z: 0.4}, {X: 0.3, Y: 0.6, Z: 2.1}}
	tris := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	for _, exponent := range []int{-600, -300, -20, 0, 20, 300, 500} {
		for _, offsetBits := range []int{0, 40, 54} {
			scale := math.Ldexp(1, exponent)
			var offset r3.Vec
			if offsetBits != 0 {
				offset = r3.NewVec(0.7, -0.6, 0.9).Scale(math.Ldexp(1, exponent+offsetBits))
			}
			verts := make([]r3.Vec, len(base))
			for i, p := range base {
				verts[i] = p.Scale(scale).Add(offset)
			}
			volume, low, high, degenerate := rationalGeometry(verts, tris)
			got, err := meshbody.Analyze(t.Context(), verts, tris)
			require.NoError(t, err)
			require.Zero(t, got.SignedVolume.Cmp(volume), "volume at exponent %d offset %d", exponent, offsetBits)
			require.Zero(t, got.AreaLow.Cmp(low), "lower area at exponent %d offset %d", exponent, offsetBits)
			require.Zero(t, got.AreaHigh.Cmp(high), "upper area at exponent %d offset %d", exponent, offsetBits)
			require.Equal(t, degenerate, got.DegenerateTriangles, "exponent %d offset %d", exponent, offsetBits)
		}
	}
}

// rationalGeometry computes the original rational expressions directly with
// math/big, independently of the dyadic vector operations under test.
func rationalGeometry(verts []r3.Vec, tris [][3]int) (*big.Rat, *big.Rat, *big.Rat, int) {
	vec := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	sub := func(a, b [3]*big.Rat) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
	}
	cross := func(a, b [3]*big.Rat) [3]*big.Rat {
		var out [3]*big.Rat
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			out[i] = new(big.Rat).Sub(new(big.Rat).Mul(a[j], b[k]), new(big.Rat).Mul(a[k], b[j]))
		}
		return out
	}
	dot := func(a, b [3]*big.Rat) *big.Rat {
		out := new(big.Rat)
		for i := range 3 {
			out.Add(out, new(big.Rat).Mul(a[i], b[i]))
		}
		return out
	}
	volume, low, high := new(big.Rat), new(big.Rat), new(big.Rat)
	degenerate := 0
	for _, tri := range tris {
		a, b, c := vec(verts[tri[0]]), vec(verts[tri[1]]), vec(verts[tri[2]])
		normal := cross(sub(b, a), sub(c, a))
		lengthSquared := dot(normal, normal)
		if lengthSquared.Sign() == 0 {
			degenerate++
			continue
		}
		volume.Add(volume, new(big.Rat).Quo(dot(a, cross(b, c)), big.NewRat(6, 1)))
		low.Add(low, new(big.Rat).SetFloat64(proofbound.RatSqrtDown(lengthSquared)))
		high.Add(high, new(big.Rat).SetFloat64(proofbound.RatSqrtUp(lengthSquared)))
	}
	return volume, low.Quo(low, big.NewRat(2, 1)), high.Quo(high, big.NewRat(2, 1)), degenerate
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
