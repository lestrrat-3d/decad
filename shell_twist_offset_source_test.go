package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestTwistShellOffsetSourceFromRoundedSketch(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	ends := [8]*sketch.Point{
		point(-2.5, -3), point(2.5, -3), point(4, -1.5), point(4, 1.5),
		point(2.5, 3), point(-2.5, 3), point(-4, 1.5), point(-4, -1.5),
	}
	s.CreateLine(ends[0], ends[1])
	s.CreateArc(point(2.5, -1.5), ends[1], ends[2])
	s.CreateLine(ends[2], ends[3])
	s.CreateArc(point(2.5, 1.5), ends[3], ends[4])
	s.CreateLine(ends[4], ends[5])
	s.CreateArc(point(-2.5, 1.5), ends[5], ends[6])
	s.CreateLine(ends[6], ends[7])
	s.CreateArc(point(-2.5, -1.5), ends[7], ends[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 8 && len(candidate.Holes) == 0 {
			profile = candidate
			break
		}
	}
	require.NotNil(t, profile)
	path, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	body, err := Sweep(t.Context(), New(), s, profile, path, WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	payload, ok := body.n.Payload.(twistedSweepPayload)
	require.True(t, ok)
	proof, err := twistsweep.TwistOffsetSource(payload.Profile, 5, payload.Angle, 1, 0)
	require.NoError(t, err)
	identityGap, err := twistsweep.TwistShellPlacedProof(payload.Profile, 5, 1,
		payload.Angle, proof, r3.Identity())
	require.NoError(t, err)
	require.Zero(t, identityGap.Boundary)
	require.Zero(t, identityGap.Volume)
	require.Zero(t, identityGap.Area)
	turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(23))
	require.NoError(t, err)
	placed, err := twistsweep.TwistShellPlacedProof(payload.Profile, 5, 1, payload.Angle, proof, turn)
	require.NoError(t, err)
	require.Positive(t, placed.Defect)
	require.Less(t, placed.Boundary, 1e-10)
	require.Less(t, placed.Volume, 1e-8)
	t.Logf("placed semantic: defect=%g boundary=%g volume=%g area=%g",
		placed.Defect, placed.Boundary, placed.Volume, placed.Area)
	want := twistsweep.TwistRat(23.615752285840543)
	require.Negative(t, proof.Area.Lo.Cmp(want))
	require.Positive(t, proof.Area.Hi.Cmp(want))
	require.Positive(t, proof.ClearanceLo.Sign())
	require.Positive(t, proof.SpeedLo.Sign())
	require.Positive(t, proof.TurnRateLo.Sign())
	require.Positive(t, proof.RadialLo.Sign())
	require.Zero(t, proof.FirstMomentX.Sign())
	require.Zero(t, proof.FirstMomentY.Sign())
	stations, err := twistsweep.TwistOffsetSampleStations(payload.Profile, 5, payload.Angle, 1, 0, 8, 16)
	require.NoError(t, err)
	require.Len(t, stations.Outer, 96)
	require.Len(t, stations.Inner, 96)
	require.True(t, twistsweep.TwistOffsetConvexPolygon(stations.Outer))
	require.True(t, twistsweep.TwistOffsetConvexPolygon(stations.Inner))
	var outerVertex, innerVertex float64
	for i := range stations.Outer {
		outerVertex = max(outerVertex, stations.OuterVertex[i])
		innerVertex = max(innerVertex, stations.InnerVertex[i])
	}
	t.Logf("stations=%d outerChord=%g innerChord=%g outerVertex=%g innerVertex=%g",
		len(stations.Outer), stations.OuterChord, stations.InnerChord,
		outerVertex, innerVertex)
	poly, err := twistsweep.TwistPolygonOf(payload.Profile)
	require.NoError(t, err)
	held, err := twistsweep.TwistShellSampledMesh(t.Context(), stations, 5, 1,
		payload.Angle, poly.AreaInterval)
	require.NoError(t, err)
	t.Logf("held shell: vertices=%d triangles=%d vertexError=%g outerBound=%g innerBound=%g volumeBound=%g areaSlack=%g",
		len(held.Verts), len(held.Tris), held.VertexError, held.OuterBound,
		held.InnerBound, held.VolumeBound, held.AreaSlack)
	innerNode, err := twistsweep.ShellFacetedBody(t.Context(), envOf(documentNode(body.Document()), 1), held,
		poly.AreaInterval, proof.Area, 5, 1)
	require.NoError(t, err)
	inner := bodyHandle(innerNode)
	volume, err := inner.Volume()
	require.NoError(t, err)
	t.Logf("cup body: faces=%d lumps=%d volume=%g ±%g", len(inner.Faces()),
		len(inner.Lumps()), volume.Value.Base(), volume.Bound.Base())
	t.Logf("area=[%.9g,%.9g] clearance>=%.9g speed>=%.9g turn>=%.9g",
		proofbound.RatFloatDown(proof.Area.Lo), proofbound.RatFloatUp(proof.Area.Hi),
		proofbound.RatFloatDown(proof.ClearanceLo), proofbound.RatFloatDown(proof.SpeedLo),
		proofbound.RatFloatDown(proof.TurnRateLo))
}
