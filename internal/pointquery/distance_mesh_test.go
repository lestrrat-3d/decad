package pointquery

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// eagerMeshDistanceSquared is the straightforward form of MeshDistanceSquared
// that converts every vertex to rationals before the triangle loop. It is the
// reference the optimized form must match.
func eagerMeshDistanceSquared(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (*big.Rat, error) {
	if len(tris) == 0 {
		return nil, decaderr.ErrUnsupported
	}
	budget := proofbound.NewWorkBudget(ctx)
	point := exactVec(p)
	converted := make([]ratVec, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if !proofbound.FiniteVec(v) {
			return nil, decaderr.ErrUnsupported
		}
		converted[i] = exactVec(v)
	}
	var minimum *big.Rat
	var minimumUpper float64
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		for _, index := range tri {
			if index < 0 || index >= len(converted) {
				return nil, decaderr.ErrUnsupported
			}
		}
		a, b, c := converted[tri[0]], converted[tri[1]], converted[tri[2]]
		if minimum != nil && (boxDistanceSquaredLower(p, verts[tri[0]], verts[tri[1]], verts[tri[2]]) >= minimumUpper ||
			boxDistanceSquared(point, a, b, c).Cmp(minimum) >= 0) {
			continue
		}
		distance, err := triangleDistanceSquared(point, a, b, c)
		if err != nil {
			return nil, err
		}
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
			minimumUpper = proofbound.RatFloatUp(minimum)
		}
	}
	return minimum, nil
}

// uvSphere builds a closed triangulated sphere with rings*segments*2 triangles.
func uvSphere(rings, segments int, radius float64) ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{{Z: radius}}
	for i := 1; i < rings; i++ {
		phi := math.Pi * float64(i) / float64(rings)
		for j := range segments {
			theta := 2 * math.Pi * float64(j) / float64(segments)
			verts = append(verts, r3.NewVec(
				radius*math.Sin(phi)*math.Cos(theta),
				radius*math.Sin(phi)*math.Sin(theta),
				radius*math.Cos(phi)))
		}
	}
	verts = append(verts, r3.Vec{Z: -radius})
	south := len(verts) - 1
	ring := func(i, j int) int { return 1 + (i-1)*segments + j%segments }
	var tris [][3]int
	for j := range segments {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
		tris = append(tris, [3]int{south, ring(rings-1, j+1), ring(rings-1, j)})
	}
	for i := 1; i < rings-1; i++ {
		for j := range segments {
			tris = append(tris,
				[3]int{ring(i, j), ring(i+1, j), ring(i+1, j+1)},
				[3]int{ring(i, j), ring(i+1, j+1), ring(i, j+1)})
		}
	}
	return verts, tris
}

// TestMeshDistanceSquaredMatchesEagerReference holds the distance calculation to
// the eager reference on random spheres, whole and cut to a random prefix of
// shuffled triangles. Under the race detector it runs the first 4 trials.
func TestMeshDistanceSquaredMatchesEagerReference(t *testing.T) {
	t.Parallel()
	trials := 60
	if raceDetector {
		trials = 4
	}
	rng := rand.New(rand.NewPCG(7, 11))
	for trial := range trials {
		verts, tris := uvSphere(3+rng.IntN(6), 3+rng.IntN(8), 0.5+rng.Float64()*4)
		rng.Shuffle(len(tris), func(i, j int) { tris[i], tris[j] = tris[j], tris[i] })
		if trial%3 == 0 {
			tris = tris[:1+rng.IntN(len(tris))]
		}
		for range 6 {
			scale := []float64{0.1, 1, 6, 50}[rng.IntN(4)]
			p := r3.NewVec((rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale)
			want, werr := eagerMeshDistanceSquared(t.Context(), p, verts, tris)
			got, gerr := MeshDistanceSquared(t.Context(), p, verts, tris)
			require.NoError(t, werr)
			require.NoError(t, gerr)
			require.Zero(t, want.Cmp(got), "trial %d point %v: lazy %s eager %s", trial, p, got.RatString(), want.RatString())
		}
	}
}

func TestMeshDistanceSquaredErrors(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)}
	p := r3.NewVec(0.2, 0.2, 1)
	run := func(verts []r3.Vec, tris [][3]int) error {
		got, err := MeshDistanceSquared(t.Context(), p, verts, tris)
		require.Nil(t, got)
		return err
	}
	require.ErrorIs(t, run(verts, nil), decaderr.ErrUnsupported)
	require.ErrorContains(t, run(verts, nil), "no triangles")
	// A non-finite vertex is reported before any triangle problem.
	bad := []r3.Vec{{}, r3.NewVec(math.NaN(), 0, 0), r3.NewVec(0, 1, 0)}
	err := run(bad, [][3]int{{0, 1, 9}})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "non-finite vertex")
	inf := []r3.Vec{{}, r3.NewVec(1, 0, 0), r3.NewVec(0, math.Inf(1), 0)}
	require.ErrorContains(t, run(inf, [][3]int{{0, 1, 2}}), "non-finite vertex")
	// An invalid index in an unvisited-for-distance later triangle still errors.
	for _, tri := range [][3]int{{0, 1, 3}, {-1, 1, 2}} {
		err = run(verts, [][3]int{{0, 1, 2}, tri})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "invalid triangle index")
	}
	require.ErrorContains(t, run(verts, [][3]int{{0, 1, 5}}), "invalid triangle index")
}

func TestMeshDistanceSquaredBudgetCancelled(t *testing.T) {
	t.Parallel()
	verts, tris := uvSphere(8, 8, 1)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, want := eagerMeshDistanceSquared(ctx, r3.NewVec(3, 0, 0), verts, tris)
	_, got := MeshDistanceSquared(ctx, r3.NewVec(3, 0, 0), verts, tris)
	require.Equal(t, want == nil, got == nil)
}

func benchmarkMesh(b *testing.B, fn func(context.Context, r3.Vec, []r3.Vec, [][3]int) (*big.Rat, error), p r3.Vec) {
	verts, tris := uvSphere(40, 50, 10)
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		got, err := fn(b.Context(), p, verts, tris)
		require.NoError(b, err)
		require.NotNil(b, got)
	}
}

func BenchmarkMeshDistanceSquared(b *testing.B) {
	near, far := r3.NewVec(10.5, 0.3, 0.2), r3.NewVec(40, 25, -30)
	b.Run("near/lazy", func(b *testing.B) { benchmarkMesh(b, MeshDistanceSquared, near) })
	b.Run("near/eager", func(b *testing.B) { benchmarkMesh(b, eagerMeshDistanceSquared, near) })
	b.Run("far/lazy", func(b *testing.B) { benchmarkMesh(b, MeshDistanceSquared, far) })
	b.Run("far/eager", func(b *testing.B) { benchmarkMesh(b, eagerMeshDistanceSquared, far) })
}

// These two functions freeze the rational kernel independently of the
// dyadic implementation, including its closed-edge and degenerate cases.
func segmentDistanceSquared(p, a, b ratVec) *big.Rat {
	axis := subtract(b, a)
	lengthSquared := dot(axis, axis)
	fromStart := subtract(p, a)
	if lengthSquared.Sign() == 0 {
		return dot(fromStart, fromStart)
	}
	along := dot(fromStart, axis)
	if along.Sign() <= 0 {
		return dot(fromStart, fromStart)
	}
	if along.Cmp(lengthSquared) >= 0 {
		fromEnd := subtract(p, b)
		return dot(fromEnd, fromEnd)
	}
	parallelSquared := new(big.Rat).Quo(new(big.Rat).Mul(along, along), lengthSquared)
	return new(big.Rat).Sub(dot(fromStart, fromStart), parallelSquared)
}

func triangleDistanceSquared(p, a, b, c ratVec) (*big.Rat, error) {
	normal := cross(subtract(b, a), subtract(c, a))
	normalSquared := dot(normal, normal)
	if normalSquared.Sign() == 0 {
		return nil, fmt.Errorf(`%w: point query mesh contains a degenerate triangle`, decaderr.ErrUnsupported)
	}
	triangle := [3]ratVec{a, b, c}
	projectionInside := true
	for i := range triangle {
		start, end := triangle[i], triangle[(i+1)%3]
		side := dot(cross(subtract(end, start), subtract(p, start)), normal)
		if side.Sign() < 0 {
			projectionInside = false
			break
		}
	}
	if projectionInside {
		offset := dot(subtract(p, a), normal)
		return new(big.Rat).Quo(new(big.Rat).Mul(offset, offset), normalSquared), nil
	}
	var minimum *big.Rat
	for i := range triangle {
		distance := segmentDistanceSquared(p, triangle[i], triangle[(i+1)%3])
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
		}
	}
	return minimum, nil
}
