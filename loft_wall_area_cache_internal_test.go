package decad

import (
	"context"
	"math"
	"slices"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/sketch"
)

// TestLoftWallAreaCacheMatchesHeldTriangles pins the topology-to-mass area
// mapping on the public circular build, including every charged chord cell.
func TestLoftWallAreaCacheMatchesHeldTriangles(t *testing.T) {
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	s0, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c0 := s0.CreatePoint(0, 0)
	s0.Fix(c0)
	s0.CreateCircle(c0, 10)
	_, err = s0.Solve(t.Context())
	require.NoError(t, err)
	s1, err := w.CreateSketch(top)
	require.NoError(t, err)
	c1 := s1.CreatePoint(0, 0)
	s1.Fix(c1)
	s1.CreateCircle(c1, 5)
	_, err = s1.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Loft(t.Context(), s0, s0.Profiles()[0], s1, s1.Profiles()[0])
	require.NoError(t, err)
	pl, ok := body.n.Payload.(loftPayload)
	require.True(t, ok)
	b := loftMassAtTarget(t, pl, 0)
	a := b.a
	verts, tris := slices.Clone(a.Verts), slices.Clone(a.Tris)
	cap0 := loftmesh.CapPolygonAreaRat(a.Pts0, a.LoopIdx0)
	cap1 := loftmesh.CapPolygonAreaRat(a.Pts1, a.LoopIdx1)
	_, _, walls, areas, err := loftrecord.BuildTopology(t.Context(), envOf(body.n.Doc, body.n.Origin.Producer), bodyNode(body), a, cap0, cap1)
	require.NoError(t, err)
	require.Equal(t, a.Walls, len(walls))
	require.Equal(t, a.Walls, len(areas))
	require.Equal(t, 2*len(a.VIdx[0]), a.Walls)
	for k, tri := range a.Tris[:a.Walls] {
		x := proofarith.XptOf(a.Verts[tri[0]])
		y := proofarith.XptOf(a.Verts[tri[1]])
		z := proofarith.XptOf(a.Verts[tri[2]])
		lo, hi := loftmesh.WallTriangleArea(proofarith.Xsub(y, x), proofarith.Xsub(z, x))
		require.Equal(t, math.Float64bits(lo), math.Float64bits(areas[k].Lo), "lower wall %d", k)
		require.Equal(t, math.Float64bits(hi), math.Float64bits(areas[k].Hi), "upper wall %d", k)
		require.Equal(t, math.Float64bits(lo), math.Float64bits(walls[k].Area), "face %d", k)
		require.Equal(t, uint8(k%2), a.Side[k])
		if k%2 == 1 {
			require.Equal(t, a.Cell[k-1], a.Cell[k])
		}
	}
	cached := buildLoftMassWithWallAreas(pl, a, b.pairs, b.sectionDelta, b.sectionMatchedDelta, areas)
	require.Equal(t, b.mass.MassAccumulator, cached.MassAccumulator)
	wantArea, gotArea := b.mass.area(cap0, cap1), cached.area(cap0, cap1)
	require.Equal(t, math.Float64bits(wantArea.Value.Base()), math.Float64bits(gotArea.Value.Base()))
	require.Equal(t, math.Float64bits(wantArea.Bound.Base()), math.Float64bits(gotArea.Bound.Base()))
	require.Equal(t, verts, a.Verts)
	require.Equal(t, tris, a.Tris)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	cancelStart, cancelEnd, cancelWalls, noAreas, err := loftrecord.BuildTopology(ctx, envOf(body.n.Doc, body.n.Origin.Producer), bodyNode(body), a, cap0, cap1)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, cancelStart)
	require.Nil(t, cancelEnd)
	require.Nil(t, cancelWalls)
	require.Nil(t, noAreas)
}
