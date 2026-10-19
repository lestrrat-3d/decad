package draftrecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
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
		body, got := twoSidedBody(t, env, centeredSquare(5))
		m, err := draftrecord.TessellateTwoSided(t.Context(), body, env, got, 0.01, true)
		require.NoError(t, err)
		require.Len(t, m.Vertices, 12, "the middle rim's four corners are shared")
		require.Len(t, m.Triangles, 20, "both middle caps are dropped")
		require.NoError(t, tessellation.RequireClosedMesh(m.Triangles))
		requireCornersOnFrustum(t, m)
		requireHalvesCompose(t, body, env, got, m)
	})

	t.Run("TwoSidedCarriesARefusedHalfProof", func(t *testing.T) {
		t.Parallel()
		// A join the build reads as tangent but the record does not make
		// exactly tangent is not admitted, so neither half's band carries the
		// occupied-volume proof and the sewn mesh carries none.
		body, got := twoSidedBody(t, env, nearTangentSlot())
		m, err := draftrecord.TessellateTwoSided(t.Context(), body, env, got, 0.01, true)
		require.NoError(t, err)
		require.NoError(t, tessellation.RequireClosedMesh(m.Triangles))
		require.False(t, m.SymDiffOK)
		requireHalvesCompose(t, body, env, got, m)
	})

	t.Run("TwoSidedRefusesAMismatchedMiddleRim", func(t *testing.T) {
		t.Parallel()
		body, got := twoSidedBody(t, env, centeredSquare(5))
		got.Positive.Profile = centeredSquare(4)
		_, err := draftrecord.TessellateTwoSided(t.Context(), body, env, got, 0.01, true)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "two-sided draft middle rim")
	})
}

// nearTangentSlot is a 30 × 10 slot whose right arc's center sits 2⁻⁴⁰ off
// the line ends it meets: the build reads both joins as tangent, but the
// record does not make that corner exactly tangent.
func nearTangentSlot() momentinput.Profile {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	a, b, c, d := p(-15, -5), p(15, -5), p(15, 5), p(-15, 5)
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(15+math.Ldexp(1, -40), 0), Start: b, End: c, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Center: p(-15, 0), Start: d, End: a, TStart: 0, TEnd: 1},
	}}}
}

// twoSidedBody builds a taper of section over [-6, 8] with the sketch plane
// at zero, as two halves sewn at their middle rim.
func twoSidedBody(t *testing.T, env topo.Env, section momentinput.Profile) (*topo.Body, draftrecord.TwoSidedPayload) {
	t.Helper()
	negative, positive := squareTaper(t, -6, 0, false), squareTaper(t, 0, 8, true)
	negative.Profile, positive.Profile = section, section
	body, err := draftrecord.BuildTwoSided(t.Context(), env, draftrecord.TwoSidedPayload{Negative: negative, Positive: positive})
	require.NoError(t, err)
	got, ok := body.Payload.(draftrecord.TwoSidedPayload)
	require.True(t, ok)
	return body, got
}

// requireHalvesCompose meshes each half alone and checks the sewn mesh m is
// their composition: the summed area slack and occupied volume, the joint
// proof flag, every middle-rim vertex's bound the larger of its two halves',
// and a bound for every facet's source face.
func requireHalvesCompose(t *testing.T, body *topo.Body, env topo.Env, tp draftrecord.TwoSidedPayload, m *topo.Mesh) {
	t.Helper()
	var halves [2]*topo.Mesh
	halfBeta := [2]map[r3.Vec]float64{{}, {}}
	for i, p := range []draftrecord.Payload{tp.Negative, tp.Positive} {
		part, err := draftrecord.Build(t.Context(), env, p)
		require.NoError(t, err)
		pp, ok := part.Payload.(draftrecord.Payload)
		require.True(t, ok)
		halves[i], err = draftrecord.Tessellate(t.Context(), part, pp, 0.01, true)
		require.NoError(t, err)
		beta, err := topobuild.VertexBounds(halves[i])
		require.NoError(t, err)
		for v, pos := range halves[i].Vertices {
			halfBeta[i][pos] = beta[v]
		}
	}
	require.Equal(t, proofbound.AbsSumUpper(proofbound.AbsSumUpper(0, halves[0].AreaSlack), halves[1].AreaSlack), m.AreaSlack)
	require.Equal(t, proofbound.AbsSumUpper(proofbound.AbsSumUpper(0, halves[0].VolSymDiff), halves[1].VolSymDiff), m.VolSymDiff)
	require.Equal(t, halves[0].SymDiffOK && halves[1].SymDiffOK, m.SymDiffOK)

	beta, err := topobuild.VertexBounds(m)
	require.NoError(t, err)
	middle := 0
	for v, pos := range m.Vertices {
		b0, in0 := halfBeta[0][pos]
		b1, in1 := halfBeta[1][pos]
		if in0 && in1 {
			middle++
			require.Equal(t, max(b0, b1), beta[v], "a middle-rim vertex keeps the larger half's bound")
		}
	}
	require.Positive(t, middle)

	live := map[*topo.Face]struct{}{}
	for _, f := range topobuild.Faces(body) {
		live[f] = struct{}{}
	}
	for _, f := range m.Source {
		_, ok := live[f]
		require.True(t, ok, "every facet names a face of the sewn body")
		_, ok = m.FaceBound[f]
		require.True(t, ok, "every facet's source face states its bound")
	}
}
