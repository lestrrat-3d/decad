package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	payload, ok := body.payload.(twistedSweepPayload)
	require.True(t, ok)
	proof, err := twistOffsetSource(payload.profile, 5, payload.angle, 1, 0)
	require.NoError(t, err)
	want := twistRat(23.615752285840543)
	require.Negative(t, proof.area.Lo.Cmp(want))
	require.Positive(t, proof.area.Hi.Cmp(want))
	require.Positive(t, proof.clearanceLo.Sign())
	require.Positive(t, proof.speedLo.Sign())
	require.Positive(t, proof.turnRateLo.Sign())
	require.Zero(t, proof.firstMomentX.Sign())
	require.Zero(t, proof.firstMomentY.Sign())
	stations, err := twistOffsetSampleStations(payload.profile, 5, payload.angle, 1, 0, 8, 16)
	require.NoError(t, err)
	require.Len(t, stations.outer, 96)
	require.Len(t, stations.inner, 96)
	require.True(t, twistOffsetConvexPolygon(stations.outer))
	require.True(t, twistOffsetConvexPolygon(stations.inner))
	var outerVertex, innerVertex float64
	for i := range stations.outer {
		outerVertex = max(outerVertex, stations.outerVertex[i])
		innerVertex = max(innerVertex, stations.innerVertex[i])
	}
	t.Logf("stations=%d outerChord=%g innerChord=%g outerVertex=%g innerVertex=%g",
		len(stations.outer), stations.outerChord, stations.innerChord,
		outerVertex, innerVertex)
	poly, err := twistPolygonOf(payload.profile)
	require.NoError(t, err)
	held, err := twistShellSampledMesh(t.Context(), stations, 5, 1,
		payload.angle, poly.areaInterval)
	require.NoError(t, err)
	t.Logf("held shell: vertices=%d triangles=%d vertexError=%g outerBound=%g innerBound=%g volumeBound=%g areaSlack=%g",
		len(held.verts), len(held.tris), held.vertexError, held.outerBound,
		held.innerBound, held.volumeBound, held.areaSlack)
	inner, err := twistShellFacetedBody(t.Context(), body.Document(), 1, held,
		poly.areaInterval, proof.area, 5, 1)
	require.NoError(t, err)
	volume, err := inner.Volume()
	require.NoError(t, err)
	t.Logf("cup body: faces=%d lumps=%d volume=%g ±%g", len(inner.Faces()),
		len(inner.Lumps()), volume.Value.Base(), volume.Bound.Base())
	t.Logf("area=[%.9g,%.9g] clearance>=%.9g speed>=%.9g turn>=%.9g",
		proofbound.RatFloatDown(proof.area.Lo), proofbound.RatFloatUp(proof.area.Hi),
		proofbound.RatFloatDown(proof.clearanceLo), proofbound.RatFloatDown(proof.speedLo),
		proofbound.RatFloatDown(proof.turnRateLo))
}
