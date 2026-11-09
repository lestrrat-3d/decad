package meshbool

import (
	"context"
	"math"
	"math/big"
	"math/rand"
	"sync"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestParityObliqueRays queries the octahedron |x| + |y| + |z| ≤ 1 at points
// on its symmetry axes. Every axis ray from the centre runs through a vertex,
// so all six are ambiguous; an oblique ray crosses one face and places the
// centre inside, both through the subset scan and the all-facets scan. A
// point beyond a vertex on an axis is outside, and a vertex itself, where
// every ray meets the boundary, stays a genuine failure.
//
// Shown to fail: without the oblique fallback the centre is refused with
// "every parity ray was ambiguous".
func TestParityObliqueRays(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}
	var tris [][3]int
	for _, x := range []int{0, 1} {
		for _, y := range []int{2, 3} {
			for _, z := range []int{4, 5} {
				tri := [3]int{x, y, z}
				// Orient each face outward: (y−x)×(z−x) along the face's octant.
				a, b, c := verts[x], verts[y], verts[z]
				n := b.Sub(a).Cross(c.Sub(a))
				if n.Dot(a.Add(b).Add(c)) < 0 {
					tri[1], tri[2] = tri[2], tri[1]
				}
				tris = append(tris, tri)
			}
		}
	}
	all := make([]int, len(tris))
	for i := range all {
		all[i] = i
	}
	bm, err := PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	bm.SetVertexBounds(make([]float64, len(verts)))
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	canceledInside, canceledBoundary, canceledDecided, err := bm.Parity.obliqueParity(
		canceled, proof.XptOf(r3.Vec{}), len(all), func(i int) int { return all[i] })
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, canceledInside || canceledBoundary || canceledDecided)
	require.Nil(t, bm.Parity.obliqueScales)
	axisPoint := proof.XptOf(r3.Vec{X: 0.1, Y: 0.2, Z: 0.3})
	inside, boundary, err := MeshParityAllContext(t.Context(), axisPoint, bm.Parity)
	require.NoError(t, err)
	require.True(t, inside)
	require.False(t, boundary)
	require.Nil(t, bm.Parity.obliqueScales)
	centre := proof.XptOf(r3.Vec{})
	wantInside, wantBoundary, wantDecided, wantErr := NewParityMesh(verts, tris).obliqueParity(
		t.Context(), centre, len(all), func(i int) int { return all[i] })
	type result struct {
		inside, boundary, decided bool
		err                       error
	}
	results := make(chan result, 8)
	var wg sync.WaitGroup
	for range 8 {
		wg.Go(func() {
			i, b, d, e := bm.Parity.obliqueParity(t.Context(), centre, len(all), func(i int) int { return all[i] })
			results <- result{i, b, d, e}
		})
	}
	wg.Wait()
	close(results)
	for got := range results {
		require.Equal(t, wantErr, got.err)
		require.Equal(t, []bool{wantInside, wantBoundary, wantDecided}, []bool{got.inside, got.boundary, got.decided})
	}
	at := func(x, y, z int64) proof.Xpt {
		return ratXpt(big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1))
	}
	for _, tc := range []struct {
		name   string
		p      proof.Xpt
		inside bool
	}{
		{"centre", at(0, 0, 0), true},
		{"beyond a vertex", at(0, 0, 2), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			wantInside, wantBoundary, wantDecided, wantErr := NewParityMesh(verts, tris).obliqueParity(
				t.Context(), tc.p, len(all), func(i int) int { return all[i] })
			gotInside, gotBoundary, gotDecided, gotErr := bm.Parity.obliqueParity(
				t.Context(), tc.p, len(all), func(i int) int { return all[i] })
			require.Equal(t, wantErr, gotErr)
			require.Equal(t, []bool{wantInside, wantBoundary, wantDecided}, []bool{gotInside, gotBoundary, gotDecided})
			inside, onBoundary, err := MeshParityPreparedContext(t.Context(), tc.p, NewParityMesh(verts, tris), all)
			require.NoError(t, err)
			require.False(t, onBoundary)
			require.Equal(t, tc.inside, inside)
			inside, onBoundary, err = MeshParityAllContext(t.Context(), tc.p, NewParityMesh(verts, tris))
			require.NoError(t, err)
			require.False(t, onBoundary)
			require.Equal(t, tc.inside, inside)
		})
	}
	t.Run("a vertex", func(t *testing.T) {
		t.Parallel()
		_, _, err := MeshParityAllContext(t.Context(), at(1, 0, 0), NewParityMesh(verts, tris))
		require.ErrorContains(t, err, "every parity ray was ambiguous")
	})
}

func TestPreparedObliqueBoxMatchesStandalone(t *testing.T) {
	rng := rand.New(rand.NewSource(12))
	special := []float64{0, math.Copysign(0, -1), math.SmallestNonzeroFloat64,
		-math.SmallestNonzeroFloat64, 0x1p500, math.Nextafter(0x1p500, math.Inf(1)),
		-0x1p500, math.MaxFloat64}
	value := func(i int) float64 {
		if i < len(special) {
			return special[i]
		}
		return math.Ldexp(rng.Float64()*2-1, rng.Intn(1200)-600)
	}
	for i := range 20000 {
		v := make([]r3.Vec, 4)
		for j := range v {
			v[j] = r3.Vec{X: value((i + j) % 25), Y: value((i + 2*j + 1) % 25), Z: value((i + 3*j + 2) % 25)}
		}
		bm := &BoolMesh{Verts: v[:3], Tris: [][3]int{{0, 1, 2}}, Boxes: [][2]r3.Vec{TriBox(v, [3]int{0, 1, 2})}}
		bm.SetVertexBounds([]float64{0, 0, 0})
		_, err := bm.Parity.initObliqueScales(t.Context())
		require.NoError(t, err)
		p, box := v[3], bm.Boxes[0]
		scale := max(1, math.Abs(p.X), math.Abs(p.Y), math.Abs(p.Z), bm.Parity.obliqueScales[0])
		refScale := 1.0
		for k := range 3 {
			refScale = max(refScale, math.Abs(CoordOf(box[0], k)), math.Abs(CoordOf(box[1], k)), math.Abs(CoordOf(p, k)))
		}
		require.Equal(t, math.Float64bits(refScale), math.Float64bits(scale), "scale case %d", i)
		for _, ray := range obliqueRays {
			d := [3]float64{float64(ray[0]), float64(ray[1]), float64(ray[2])}
			want := rayMissesBox(p, d, v[0], v[1], v[2])
			got := rayMissesPreparedBox(p, d, box[0], box[1], scale)
			require.Equal(t, want, got, "case %d, direction %v", i, ray)
		}
	}
}

type cancelAfterPolls struct {
	polls int
}

func (c *cancelAfterPolls) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *cancelAfterPolls) Done() <-chan struct{}       { return nil }
func (c *cancelAfterPolls) Value(any) any               { return nil }

func (c *cancelAfterPolls) Err() error {
	c.polls++
	if c.polls >= 3 {
		return context.Canceled
	}
	return nil
}

func TestObliqueScaleBuildCancellationRetries(t *testing.T) {
	pm := &ParityMesh{obliqueBoxes: make([][2]r3.Vec, 600)}
	ctx := &cancelAfterPolls{}
	_, err := pm.initObliqueScales(ctx)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, pm.obliqueScales)
	scales, err := pm.initObliqueScales(t.Context())
	require.NoError(t, err)
	require.Len(t, scales, 600)
	for _, scale := range scales {
		require.Equal(t, 1.0, scale)
	}
}
