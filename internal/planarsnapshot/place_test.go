package planarsnapshot_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// tetrahedronSnapshot is the unit tetrahedron at the origin with outward
// triangles, as a cached identity-pose snapshot.
func tetrahedronSnapshot(t *testing.T) *planarsnapshot.SnapshotEntry {
	t.Helper()
	corners := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	verts := make([]proofarith.DyV3, len(corners))
	for i, c := range corners {
		verts[i] = proofarith.DyVec(c)
	}
	// The cached slices have spare capacity, as a built snapshot's may.
	tris := append(make([][3]int, 0, 16), [3]int{0, 2, 1}, [3]int{0, 1, 3}, [3]int{0, 3, 2}, [3]int{1, 2, 3})
	faces := append(make([]int, 0, 16), 0, 1, 2, 3)
	topology, ok, err := planar.NewPlanarTopology(len(verts), tris, func() error { return nil })
	require.NoError(t, err)
	require.True(t, ok)
	return &planarsnapshot.SnapshotEntry{OK: true, Solid: planar.PlanarSolid{Verts: verts, Tris: tris, Faces: faces},
		Topology: topology}
}

func TestPlace(t *testing.T) {
	t.Parallel()

	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }

	t.Run("maps every vertex through the pose and shares the topology", func(t *testing.T) {
		t.Parallel()
		snapshot := tetrahedronSnapshot(t)
		move, err := r3.Translation(r3.Vec{X: 3, Y: -2, Z: 0.5})
		require.NoError(t, err)
		solid, ok, err := planarsnapshot.Place(budget(), snapshot, move)
		require.NoError(t, err)
		require.True(t, ok)
		require.Len(t, solid.Verts, 4)
		for i, v := range snapshot.Solid.Verts {
			for axis, offset := range [3]float64{3, -2, 0.5} {
				want := v[axis].Rat()
				want.Add(want, proofarith.MustDyOf(offset).Rat())
				require.Zero(t, want.Cmp(solid.Verts[i][axis].Rat()), "vertex %d axis %d", i, axis)
			}
		}
		require.Equal(t, snapshot.Solid.Tris, solid.Tris)
		require.Equal(t, snapshot.Solid.Faces, solid.Faces)
		require.Equal(t, len(snapshot.Solid.Tris), cap(solid.Tris), "the shared slices are clipped")
		require.Equal(t, len(snapshot.Solid.Faces), cap(solid.Faces))
		_ = append(solid.Tris, [3]int{9, 9, 9})
		require.Equal(t, [3]int{}, snapshot.Solid.Tris[:5][4], "an append cannot reach the cache's storage")
		require.Len(t, snapshot.Solid.Verts, 4, "the cache keeps its own vertices")
		require.Zero(t, snapshot.Solid.Verts[1][0].Rat().Cmp(proofarith.MustDyOf(1).Rat()))
	})
	t.Run("a pose that turns the solid inside out is refused", func(t *testing.T) {
		t.Parallel()
		mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		flip, err := r3.Reflection(mirror)
		require.NoError(t, err)
		solid, ok, err := planarsnapshot.Place(budget(), tetrahedronSnapshot(t), flip)
		require.NoError(t, err)
		require.False(t, ok)
		require.Empty(t, solid.Verts)
	})
	t.Run("a spent budget stops the placement", func(t *testing.T) {
		t.Parallel()
		spent := errors.New("spent")
		b := &proofbound.WorkBudget{StepFn: func() error { return spent }}
		_, ok, err := planarsnapshot.Place(b, tetrahedronSnapshot(t), r3.Identity())
		require.ErrorIs(t, err, spent)
		require.False(t, ok)
	})
	t.Run("a canceled budget stops the audit", func(t *testing.T) {
		t.Parallel()
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		b := &proofbound.WorkBudget{StepFn: func() error { return ctx.Err() }}
		_, ok, err := planarsnapshot.Place(b, tetrahedronSnapshot(t), r3.Identity())
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
	})
}
