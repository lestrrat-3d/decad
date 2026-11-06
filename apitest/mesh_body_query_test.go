package apitest_test

import (
	"context"
	"fmt"
	"math"
	"math/rand/v2"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests MeshBody construction and agent-docs/mesh-import-design.md
// §5's public point queries, on a 2 mm cube at the origin.

func queryCubeParts() ([]r3.Vec, [][3]int) {
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

func queryCube(t *testing.T, inverted bool) *decad.MeshBody {
	t.Helper()
	v, tris := queryCubeParts()
	if inverted {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	m, err := decad.NewMeshBody(t.Context(), v, tris)
	require.NoError(t, err)
	return m
}

func TestNewMeshBodyCopiesInputAndAccessorSlices(t *testing.T) {
	t.Parallel()
	vertices, triangles := queryCubeParts()
	wantVertices := append([]r3.Vec(nil), vertices...)
	wantTriangles := append([][3]int(nil), triangles...)
	m, err := decad.NewMeshBody(t.Context(), vertices, triangles)
	require.NoError(t, err)
	wantReport := m.Report()
	wantBounds := m.Bounds()
	wantArea, err := m.Area()
	require.NoError(t, err)
	wantVolume, err := m.Volume()
	require.NoError(t, err)

	vertices[0] = r3.NewVec(100, 100, 100)
	triangles[0] = [3]int{7, 7, 7}
	gotVertices, gotTriangles := m.Vertices(), m.Triangles()
	gotVertices[0] = r3.NewVec(-100, -100, -100)
	gotTriangles[0] = [3]int{6, 6, 6}

	require.Equal(t, wantVertices, m.Vertices())
	require.Equal(t, wantTriangles, m.Triangles())
	require.Equal(t, wantReport, m.Report())
	require.Equal(t, wantBounds, m.Bounds())
	gotArea, err := m.Area()
	require.NoError(t, err)
	require.Equal(t, wantArea, gotArea)
	gotVolume, err := m.Volume()
	require.NoError(t, err)
	require.Equal(t, wantVolume, gotVolume)
}

func TestMeshBodyLocatePoint(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		at   r3.Vec
		want decad.PointLocation
	}{
		{"inside", r3.NewVec(1, 1, 1), decad.PointInside},
		{"outside beside a face", r3.NewVec(3, 1, 1), decad.PointOutside},
		{"outside past a corner", r3.NewVec(3, 3, 3), decad.PointOutside},
		{"on a face", r3.NewVec(2, 1, 1), decad.PointOnBoundary},
		{"on an edge", r3.NewVec(2, 2, 1), decad.PointOnBoundary},
		{"on a corner", r3.NewVec(2, 2, 2), decad.PointOnBoundary},
		{"a hair outside", r3.NewVec(math.Nextafter(2, 3), 1, 1), decad.PointOutside},
		{"a hair inside", r3.NewVec(math.Nextafter(2, 1), 1, 1), decad.PointInside},
	}
	for _, inverted := range []bool{false, true} {
		for _, tc := range cases {
			t.Run(tc.name, func(t *testing.T) {
				t.Parallel()
				got, err := queryCube(t, inverted).LocatePoint(t.Context(), tc.at)
				require.NoError(t, err)
				require.Equal(t, tc.want, got, "inverted %v", inverted)
			})
		}
	}
}

func TestMeshBodyLocatePointConcurrent(t *testing.T) {
	t.Parallel()
	mesh := queryCube(t, false)
	queries := []struct {
		point r3.Vec
		want  decad.PointLocation
	}{
		{r3.NewVec(1, 1, 1), decad.PointInside},
		{r3.NewVec(3, 1, 1), decad.PointOutside},
		{r3.NewVec(2, 1, 1), decad.PointOnBoundary},
	}
	errs := make(chan error, len(queries))
	ctx := t.Context()
	var wg sync.WaitGroup
	for _, query := range queries {
		wg.Go(func() {
			got, err := mesh.LocatePoint(ctx, query.point)
			if err != nil {
				errs <- err
				return
			}
			if got != query.want {
				errs <- fmt.Errorf("LocatePoint(%v) = %s, want %s", query.point, got, query.want)
			}
		})
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		require.NoError(t, err)
	}
}

func TestMeshBodyDistanceToPoint(t *testing.T) {
	t.Parallel()
	m := queryCube(t, false)
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want float64
	}{
		{"inside", r3.NewVec(1, 1, 1), 0},
		{"on a face", r3.NewVec(2, 1, 1), 0},
		{"beside a face", r3.NewVec(5, 1, 1), 3},
		{"past an edge", r3.NewVec(3, 3, 1), math.Sqrt2},
		{"past a corner", r3.NewVec(3, 3, 3), math.Sqrt(3)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := m.DistanceToPoint(t.Context(), tc.at)
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(got.Value.Base()-tc.want), got.Bound.Base()+1e-15)
			if tc.want == 0 || tc.want == 3 {
				require.Equal(t, decad.Exact, got.Exactness)
				require.Zero(t, got.Bound.Base())
			} else {
				require.Less(t, got.Bound.Base(), 1e-15, "a root is bounded by float rounding alone")
			}
		})
	}
}

func TestMeshBodyClosestPoint(t *testing.T) {
	t.Parallel()
	m := queryCube(t, false)
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want r3.Vec
	}{
		{"inside is its own", r3.NewVec(1, 0.5, 1.5), r3.NewVec(1, 0.5, 1.5)},
		{"on a face is its own", r3.NewVec(2, 1, 1), r3.NewVec(2, 1, 1)},
		{"beside a face", r3.NewVec(5, 0.5, 1.25), r3.NewVec(2, 0.5, 1.25)},
		{"past an edge", r3.NewVec(3, 3, 1), r3.NewVec(2, 2, 1)},
		{"past a corner", r3.NewVec(-1, 4, 3), r3.NewVec(0, 2, 2)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := m.ClosestPoint(t.Context(), tc.at)
			require.NoError(t, err)
			require.Equal(t, tc.want, got.Point, "the mesh is exact, so the nearest point is exact here")
			require.Zero(t, got.OffSurface.Base())
		})
	}
}

func TestMeshBodyQueriesRefuseWhatEncloseNothing(t *testing.T) {
	t.Parallel()
	v, tris := queryCubeParts()
	open, err := decad.NewMeshBody(t.Context(), v, tris[2:]) // no bottom
	require.NoError(t, err)
	_, err = open.LocatePoint(t.Context(), r3.NewVec(1, 1, 1))
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = open.DistanceToPoint(t.Context(), r3.NewVec(1, 1, 1))
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = open.ClosestPoint(t.Context(), r3.NewVec(1, 1, 1))
	require.ErrorIs(t, err, decad.ErrNotSolid)

	closed := queryCube(t, false)
	_, err = closed.LocatePoint(context.Context(nil), r3.NewVec(1, 1, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	got, err := closed.LocatePoint(t.Context(), r3.NewVec(1, 1, 1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, got)
	_, err = closed.LocatePoint(canceled, r3.NewVec(1, 1, 1))
	require.ErrorIs(t, err, context.Canceled)
	got, err = closed.LocatePoint(t.Context(), r3.NewVec(1, 1, 1))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, got)
	_, err = closed.DistanceToPoint(t.Context(), r3.NewVec(math.NaN(), 0, 0))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	var none *decad.MeshBody
	_, err = none.ClosestPoint(t.Context(), r3.NewVec(1, 1, 1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func BenchmarkMeshBodyLocatePointRepeated(b *testing.B) {
	vertices, triangles := meshBodyBenchmarkCube(12)
	mesh, err := decad.NewMeshBody(b.Context(), vertices, triangles)
	if err != nil {
		b.Fatal(err)
	}
	rng := rand.New(rand.NewPCG(0x5ca1, 0xab1e))
	queries := make([]r3.Vec, 32)
	for i := range queries {
		queries[i] = r3.NewVec(0.5+0.9*rng.Float64(), 0.5+0.9*rng.Float64(), 0.5+0.9*rng.Float64())
	}
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		for _, point := range queries {
			if _, err := mesh.LocatePoint(b.Context(), point); err != nil {
				b.Fatal(err)
			}
		}
	}
}

func meshBodyBenchmarkCube(k int) ([]r3.Vec, [][3]int) {
	vertices := make([]r3.Vec, 0, 6*(k+1)*(k+1))
	ids := make(map[[3]int]int, cap(vertices))
	at := func(i, j, l int) int {
		key := [3]int{i, j, l}
		if id, ok := ids[key]; ok {
			return id
		}
		id := len(vertices)
		ids[key] = id
		vertices = append(vertices, r3.NewVec(float64(i)/float64(k), float64(j)/float64(k), float64(l)/float64(k)))
		return id
	}
	triangles := make([][3]int, 0, 12*k*k)
	quad := func(a, b, c, d int) {
		triangles = append(triangles, [3]int{a, b, c}, [3]int{a, c, d})
	}
	for i := range k {
		for j := range k {
			quad(at(i, j, 0), at(i, j+1, 0), at(i+1, j+1, 0), at(i+1, j, 0))
			quad(at(i, j, k), at(i+1, j, k), at(i+1, j+1, k), at(i, j+1, k))
			quad(at(0, i, j), at(0, i, j+1), at(0, i+1, j+1), at(0, i+1, j))
			quad(at(k, i, j), at(k, i+1, j), at(k, i+1, j+1), at(k, i, j+1))
			quad(at(i, 0, j), at(i+1, 0, j), at(i+1, 0, j+1), at(i, 0, j+1))
			quad(at(i, k, j), at(i, k, j+1), at(i+1, k, j+1), at(i+1, k, j))
		}
	}
	return vertices, triangles
}
