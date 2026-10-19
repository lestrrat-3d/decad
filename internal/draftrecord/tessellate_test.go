package draftrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// squareTaper is a 10 mm square tapered inward by 5° from the sketch plane
// over [z0, z1]; nearStart says which end is the sketch plane.
func squareTaper(t *testing.T, z0, z1 float64, nearStart bool) draftrecord.Payload {
	t.Helper()
	return draftrecord.Payload{
		Profile: centeredSquare(5), Frame: xyFrame(t), Z0: z0, Z1: z1,
		NearStart: nearStart, Taper: 5 * units.Degree.Factor(), Xform: r3.Identity(),
	}
}

// requireCornersOnFrustum checks every mesh vertex sits, within the mesh's
// own Bound, on a corner of the square section the taper denotes at its
// level: half side 5 at the sketch plane, shrinking by tan 5° per millimetre
// of distance from it.
func requireCornersOnFrustum(t *testing.T, m *topo.Mesh) {
	t.Helper()
	tan := math.Tan(5 * units.Degree.Factor())
	for _, v := range m.Vertices {
		half := 5 - math.Abs(v.Z)*tan
		require.InDelta(t, half, math.Abs(v.X), m.Bound+1e-12)
		require.InDelta(t, half, math.Abs(v.Y), m.Bound+1e-12)
	}
}

func TestTessellateDraftBand(t *testing.T) {
	t.Parallel()
	env := topo.Env{Doc: &topo.Document{}, Ref: 1, Alloc: topotest.Alloc{}}

	t.Run("SingleSlab", func(t *testing.T) {
		t.Parallel()
		body, err := draftrecord.Build(t.Context(), env, squareTaper(t, 0, 8, true))
		require.NoError(t, err)
		dp, ok := body.Payload.(draftrecord.Payload)
		require.True(t, ok)
		m, err := draftrecord.Tessellate(t.Context(), body, dp, 0.01, true)
		require.NoError(t, err)
		require.Len(t, m.Vertices, 8, "a line-only band shares its two rings")
		require.Len(t, m.Triangles, 12)
		require.NoError(t, topobuild.RequireClosedMesh(m))
		require.True(t, m.SymDiffOK, "a line-line mitred band carries the occupied-volume proof")
		requireCornersOnFrustum(t, m)
	})

	t.Run("NoBandGeometryRefuses", func(t *testing.T) {
		t.Parallel()
		body, err := draftrecord.Build(t.Context(), env, squareTaper(t, 0, 8, true))
		require.NoError(t, err)
		dp, ok := body.Payload.(draftrecord.Payload)
		require.True(t, ok)
		dp.Patches = nil
		_, err = draftrecord.Tessellate(t.Context(), body, dp, 0.01, true)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "states no wall band geometry")
	})

	t.Run("TwoSidedSewsTheMiddleRim", func(t *testing.T) {
		t.Parallel()
		tp := draftrecord.TwoSidedPayload{
			Negative: squareTaper(t, -6, 0, false),
			Positive: squareTaper(t, 0, 8, true),
		}
		body, err := draftrecord.BuildTwoSided(t.Context(), env, tp)
		require.NoError(t, err)
		got, ok := body.Payload.(draftrecord.TwoSidedPayload)
		require.True(t, ok)
		m, err := draftrecord.TessellateTwoSided(t.Context(), body, env, got, 0.01, true)
		require.NoError(t, err)
		require.Len(t, m.Vertices, 12, "the middle rim's four corners are shared")
		require.Len(t, m.Triangles, 20, "both middle caps are dropped")
		require.NoError(t, tessellation.RequireClosedMesh(m.Triangles))
		require.True(t, m.SymDiffOK)
		requireCornersOnFrustum(t, m)
		live := map[*topo.Face]struct{}{}
		for _, f := range topobuild.Faces(body) {
			live[f] = struct{}{}
		}
		for _, f := range m.Source {
			_, ok := live[f]
			require.True(t, ok, "every facet names a face of the sewn body")
		}
		beta, err := topobuild.VertexBounds(m)
		require.NoError(t, err)
		require.Len(t, beta, 12)
	})
}
