package pointquery

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ratSegmentDistanceSquared and ratTriangleDistanceSquared are the big.Rat
// form of the exact distance kernel, the reference the dyadic kernel must
// match.
func ratSegmentDistanceSquared(p, a, b ratVec) *big.Rat {
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

func ratTriangleDistanceSquared(p, a, b, c ratVec) (*big.Rat, error) {
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
		distance := ratSegmentDistanceSquared(p, triangle[i], triangle[(i+1)%3])
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
		}
	}
	return minimum, nil
}

// eagerMeshDistanceSquared is the index-order triangle scan that converts every
// vertex to rationals before the triangle loop. It is the reference
// MeshDistanceSquared must match in value, budget steps and errors.
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
			return nil, fmt.Errorf(`%w: point query mesh has a non-finite vertex`, decaderr.ErrUnsupported)
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
				return nil, fmt.Errorf(`%w: point query mesh has an invalid triangle index`, decaderr.ErrUnsupported)
			}
		}
		a, b, c := converted[tri[0]], converted[tri[1]], converted[tri[2]]
		if minimum != nil && (boxDistanceSquaredLower(p, verts[tri[0]], verts[tri[1]], verts[tri[2]]) >= minimumUpper ||
			boxDistanceSquared(point, a, b, c).Cmp(minimum) >= 0) {
			continue
		}
		distance, err := ratTriangleDistanceSquared(point, a, b, c)
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

// TestMeshDistanceSquaredMatchesEagerReference holds the lazy conversion to
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

// plainMeshDistanceSquared is the exact minimum of the big.Rat reference
// kernel over every triangle, with no filter and no visiting order.
func plainMeshDistanceSquared(p r3.Vec, verts []r3.Vec, tris [][3]int) (*big.Rat, error) {
	point := exactVec(p)
	var minimum *big.Rat
	for _, tri := range tris {
		distance, err := ratTriangleDistanceSquared(point, exactVec(verts[tri[0]]), exactVec(verts[tri[1]]), exactVec(verts[tri[2]]))
		if err != nil {
			return nil, err
		}
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
		}
	}
	return minimum, nil
}

// sliverMesh builds n random thin triangles: two far-apart corners and a
// third a tiny offset from their midpoint, so each is nearly collinear.
func sliverMesh(rng *rand.Rand, n int, scale float64) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	var tris [][3]int
	random := func() r3.Vec {
		return r3.NewVec((rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale)
	}
	for range n {
		a, b := random(), random()
		thin := []float64{1e-3, 1e-9, 1e-15}[rng.IntN(3)] * scale
		c := a.Add(b).Scale(0.5).Add(r3.NewVec(thin*rng.Float64(), thin*rng.Float64(), thin))
		tris = append(tris, [3]int{len(verts), len(verts) + 1, len(verts) + 2})
		verts = append(verts, a, b, c)
	}
	return verts, tris
}

// meshQueryPoints returns points far from, near, on and inside the mesh:
// random points at several scales, every vertex exactly, and vertices nudged
// by a tiny offset.
func meshQueryPoints(rng *rand.Rand, verts []r3.Vec, count int) []r3.Vec {
	var points []r3.Vec
	for range count {
		scale := []float64{0.1, 1, 6, 50}[rng.IntN(4)]
		points = append(points, r3.NewVec((rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale))
		v := verts[rng.IntN(len(verts))]
		offset := []float64{0, 1e-12, 1e-6, 0.05}[rng.IntN(4)]
		points = append(points, v.Add(r3.NewVec(offset, -offset, offset/2)))
	}
	return points
}

// TestMeshDistanceSquaredMatchesPlainScan holds MeshDistanceSquared to the
// unfiltered exact minimum on spheres, shuffled and cut spheres and sliver
// meshes, at far, near, on-surface and interior points. Under the race
// detector it runs the first 3 trials of each mesh family.
func TestMeshDistanceSquaredMatchesPlainScan(t *testing.T) {
	t.Parallel()
	trials := 24
	if raceDetector {
		trials = 3
	}
	rng := rand.New(rand.NewPCG(19, 23))
	check := func(name string, verts []r3.Vec, tris [][3]int) {
		for _, p := range meshQueryPoints(rng, verts, 4) {
			want, werr := plainMeshDistanceSquared(p, verts, tris)
			require.NoError(t, werr)
			got, gerr := MeshDistanceSquared(t.Context(), p, verts, tris)
			require.NoError(t, gerr)
			require.Zero(t, want.Cmp(got), "%s point %v: got %s want %s", name, p, got.RatString(), want.RatString())
		}
	}
	for trial := range trials {
		verts, tris := uvSphere(3+rng.IntN(10), 3+rng.IntN(12), 0.5+rng.Float64()*4)
		check("sphere", verts, tris)
		rng.Shuffle(len(tris), func(i, j int) { tris[i], tris[j] = tris[j], tris[i] })
		check("shuffled sphere", verts, tris[:1+rng.IntN(len(tris))])
		verts, tris = sliverMesh(rng, 1+rng.IntN(40), []float64{0.01, 1, 100}[trial%3])
		check("slivers", verts, tris)
	}
}

// TestMeshDistanceSquaredDegenerateMatchesIndexScan holds meshes with exactly
// degenerate triangles to the index-order reference: the same value when the
// scan never reaches them and the same error when it does, including ahead of
// a later invalid index.
func TestMeshDistanceSquaredDegenerateMatchesIndexScan(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(29, 31))
	verts, sphere := uvSphere(6, 8, 2)
	collinear := len(verts)
	verts = append(verts, r3.NewVec(5, 5, 5), r3.NewVec(6, 6, 6), r3.NewVec(7, 7, 7))
	degenerate := [][3]int{{collinear, collinear + 1, collinear + 2}, {0, 0, 1}, {3, 4, 3}}
	reached, skipped := 0, 0
	for trial := range 40 {
		tris := append([][3]int(nil), sphere...)
		for range 1 + rng.IntN(3) {
			at := rng.IntN(len(tris) + 1)
			tris = slices.Insert(tris, at, degenerate[rng.IntN(len(degenerate))])
		}
		if trial%4 == 0 {
			tris = append(tris, [3]int{0, 1, len(verts)})
		}
		for _, p := range meshQueryPoints(rng, verts, 3) {
			want, werr := eagerMeshDistanceSquared(t.Context(), p, verts, tris)
			got, gerr := MeshDistanceSquared(t.Context(), p, verts, tris)
			if werr != nil {
				reached++
				require.Nil(t, got)
				require.EqualError(t, gerr, werr.Error(), "trial %d point %v", trial, p)
				continue
			}
			skipped++
			require.NoError(t, gerr, "trial %d point %v", trial, p)
			require.Zero(t, want.Cmp(got), "trial %d point %v: got %s want %s", trial, p, got.RatString(), want.RatString())
		}
	}
	require.Positive(t, reached, "some scans reach a degenerate triangle")
	require.Positive(t, skipped, "some scans skip every degenerate triangle")
}

// TestNormalNonzeroFloatNeverClaimsADegenerateTriangle checks the float test,
// and the integer degeneracy test, against triangleDistanceSquared's rational
// one on degenerate and nearly degenerate triangles.
func TestNormalNonzeroFloatNeverClaimsADegenerateTriangle(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(37, 41))
	proved := 0
	for range 2000 {
		a := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
		d := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
		s, u := float64(rng.IntN(5)), rng.Float64()
		b := a.Add(d)
		c := a.Add(d.Scale(s))
		if rng.IntN(2) == 0 {
			c = c.Add(r3.NewVec(0, 0, u*1e-14))
		}
		got := normalNonzeroFloat(a, b, c)
		_, err := ratTriangleDistanceSquared(exactVec(r3.Vec{}), exactVec(a), exactVec(b), exactVec(c))
		require.Equal(t, err != nil, degenerateTriangle(a, b, c), "a %v b %v c %v", a, b, c)
		if err != nil {
			require.False(t, got, "a %v b %v c %v is degenerate", a, b, c)
			continue
		}
		if got {
			proved++
		}
	}
	require.Positive(t, proved)
	// Slivers a few units in the last place off a line straddle the float bound.
	undecided := 0
	verts, tris := sliverMesh(rng, 3000, 1)
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		if !normalNonzeroFloat(a, b, c) {
			undecided++
			continue
		}
		require.False(t, degenerateTriangle(a, b, c), "a %v b %v c %v is degenerate", a, b, c)
	}
	require.Positive(t, undecided)
	require.False(t, normalNonzeroFloat(r3.Vec{}, r3.NewVec(1, 1, 1), r3.NewVec(2, 2, 2)))
	require.False(t, normalNonzeroFloat(r3.Vec{}, r3.NewVec(0x1p300, 0, 0), r3.NewVec(0, 1, 0)), "out of range is undecided")
	require.True(t, normalNonzeroFloat(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)))
}

// TestTriangleDistanceSquaredMatchesRatKernel holds the dyadic kernel to the
// big.Rat reference on random, sliver and degenerate triangles at mixed
// scales, with points in every face, edge and vertex region.
func TestTriangleDistanceSquaredMatchesRatKernel(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(43, 47))
	random := func(scale float64) r3.Vec {
		return r3.NewVec(rng.NormFloat64()*scale, rng.NormFloat64()*scale, rng.NormFloat64()*scale)
	}
	degenerate := 0
	for trial := range 3000 {
		scale := []float64{1e-6, 1, 1e5}[trial%3]
		a, b := random(scale), random(scale)
		c := random(scale)
		switch trial % 5 {
		case 1:
			c = a.Add(b).Scale(0.5).Add(random(scale * 1e-14))
		case 2:
			c = b
		}
		p := random(scale * 2)
		if trial%7 == 0 {
			p = a
		}
		want, werr := ratTriangleDistanceSquared(exactVec(p), exactVec(a), exactVec(b), exactVec(c))
		got, gerr := triangleDistanceSquared(proof.DyVec(p), proof.DyVec(a), proof.DyVec(b), proof.DyVec(c))
		if werr != nil {
			degenerate++
			require.EqualError(t, gerr, werr.Error())
			require.Nil(t, got)
			continue
		}
		require.NoError(t, gerr)
		require.Zero(t, want.Cmp(got), "p %v a %v b %v c %v: got %s want %s", p, a, b, c, got.RatString(), want.RatString())
		require.Equal(t, want.RatString(), got.RatString())
	}
	require.Positive(t, degenerate)
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
	// The sphere has 3,900 triangles. near sits 0.5 outside its wall and inner
	// 0.2 inside it.
	near, inner, far := r3.NewVec(10.5, 0.3, 0.2), r3.NewVec(-3.0, 5.4, -7.6), r3.NewVec(40, 25, -30)
	b.Run("near/lazy", func(b *testing.B) { benchmarkMesh(b, MeshDistanceSquared, near) })
	b.Run("near/eager", func(b *testing.B) { benchmarkMesh(b, eagerMeshDistanceSquared, near) })
	b.Run("inner/lazy", func(b *testing.B) { benchmarkMesh(b, MeshDistanceSquared, inner) })
	b.Run("inner/eager", func(b *testing.B) { benchmarkMesh(b, eagerMeshDistanceSquared, inner) })
	b.Run("far/lazy", func(b *testing.B) { benchmarkMesh(b, MeshDistanceSquared, far) })
	b.Run("far/eager", func(b *testing.B) { benchmarkMesh(b, eagerMeshDistanceSquared, far) })
}
