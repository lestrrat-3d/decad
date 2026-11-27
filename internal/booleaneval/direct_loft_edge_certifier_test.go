package booleaneval

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestDirectLoftEdgeCertifierTargetPlane(t *testing.T) {
	for _, tc := range []struct {
		name string
		edit func(*testing.T, *directLoftEdgeFixture)
		want bool
	}{
		{name: "exact face", want: true},
		{name: "face with a held bound", edit: func(_ *testing.T, f *directLoftEdgeFixture) {
			f.target.FaceBound[f.target.Source[0]] = 0.001
		}},
		{name: "retained tag stating a displacement", edit: func(_ *testing.T, f *directLoftEdgeFixture) {
			face := f.target.Source[0]
			face.Surface = surfacegeom.Plane{}
			face.RetainedTag, face.ExtentDelta = true, 1e-9
			f.target.FaceBound[face] = 0.001
		}},
		{name: "retained tag stating a normal bound", edit: func(_ *testing.T, f *directLoftEdgeFixture) {
			face := f.target.Source[0]
			face.Surface = surfacegeom.Plane{}
			face.RetainedTag, face.NormalBound = true, 1e-12
			f.target.FaceBound[face] = 0.001
		}},
		{name: "retained tag that does not hold the facet", edit: func(t *testing.T, f *directLoftEdgeFixture) {
			frame, err := r3.NewFrame(r3.Vec{Z: 50}, r3.Vec{X: 1}, r3.Vec{Y: 1})
			require.NoError(t, err)
			face := f.target.Source[0]
			face.Surface = surfacegeom.Plane{Frame: frame}
			face.RetainedTag = true
			f.target.FaceBound[face] = 0.001
		}},
		{name: "stale prepared facet", edit: func(_ *testing.T, f *directLoftEdgeFixture) {
			f.target.Triangles = append([][3]int(nil), f.target.Triangles...)
			f.target.Triangles[0] = [3]int{0, 1, 3}
		}},
		{name: "moved prepared apex on exact face", edit: func(_ *testing.T, f *directLoftEdgeFixture) {
			f.targetBool.Verts = append([]r3.Vec(nil), f.targetBool.Verts...)
			f.targetBool.Verts[f.targetBool.Tris[0][2]].X += 0.25
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := makeDirectLoftEdgeFixture(t)
			if tc.edit != nil {
				tc.edit(t, &f)
			}
			require.Equal(t, tc.want, certifierFor(t, f).targetPlaneExact(0))
		})
	}
}

func TestDirectLoftEdgeCertifierCapAndWall(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	cert := certifierFor(t, f)
	lp := f.body.Payload.(loftrecord.Payload)
	require.True(t, cert.capFacetsInPlane(lp, f.loftFacet, f.loftEdge))
	// The cap's plane is z = 0, so a facet lifted off it is refused.
	f.loftBool.Verts = append([]r3.Vec(nil), f.loftBool.Verts...)
	capFacet, _ := capSide(t, f)
	f.loftBool.Verts[f.loftBool.Tris[capFacet][0]].Z = 1e-300
	require.False(t, cert.capFacetsInPlane(lp, f.loftFacet, f.loftEdge))
}

func TestDirectStartWallTriangleExact(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	lp := f.body.Payload.(loftrecord.Payload)
	tri := f.loftBool.Tris[f.loftFacet]
	points := [3]r3.Vec{f.loftBool.Verts[tri[0]], f.loftBool.Verts[tri[1]], f.loftBool.Verts[tri[2]]}
	check := func(body *topo.Body, side int, points [3]r3.Vec) bool {
		ok, err := lp.DirectStartWallTriangleExact(t.Context(), body, f.loft, f.loftFacet,
			lp.Cell[f.loftFacet][1], side, points, proofbound.NewWorkBudget(t.Context()))
		require.NoError(t, err)
		return ok
	}
	require.True(t, check(f.body, 0, points))
	require.False(t, check(f.body, 1, points))
	require.False(t, check(new(topo.Body), 0, points))
	points[2].X += 0.125
	require.False(t, check(f.body, 0, points))
}

func TestDirectStartWallTriangleRefusesRoundedLift(t *testing.T) {
	f := makeDirectLoftEdgeFixtureOffset(t, 0.1)
	lp := f.body.Payload.(loftrecord.Payload)
	tri := f.loftBool.Tris[f.loftFacet]
	points := [3]r3.Vec{f.loftBool.Verts[tri[0]], f.loftBool.Verts[tri[1]], f.loftBool.Verts[tri[2]]}
	ok, err := lp.DirectStartWallTriangleExact(t.Context(), f.body, f.loft, f.loftFacet,
		lp.Cell[f.loftFacet][1], int(lp.Side[f.loftFacet]), points, proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, err)
	require.False(t, ok)
}

func TestDirectLoftEdgeCertifierRefusesBadIndices(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	cert := certifierFor(t, f)
	for _, tc := range [][4]int{{-1, 0, f.loftFacet, f.loftEdge}, {0, 3, f.loftFacet, f.loftEdge},
		{0, 0, -1, f.loftEdge}, {0, 0, len(f.loftBool.Tris), 0}, {0, 0, f.loftFacet, 3}, {99, 0, f.loftFacet, f.loftEdge}} {
		got, err := cert.CertifySharedEdge(t.Context(), tc[0], tc[1], tc[2], tc[3])
		require.NoError(t, err)
		require.False(t, got, "%v", tc)
	}
	// An edge of the loft that does not touch the cap is not the shared edge.
	for e := range 3 {
		if e == f.loftEdge {
			continue
		}
		twin, ok := f.loftBool.TwinFacet(f.loftFacet, e)
		require.True(t, ok)
		if f.loft.Source[twin] == f.loft.Source[f.loftFacet] {
			continue
		}
		got, err := cert.CertifySharedEdge(t.Context(), 0, 0, f.loftFacet, e)
		require.NoError(t, err)
		require.False(t, got)
	}
}

func TestNewDirectLoftEdgeCertifierRefusesOtherBodies(t *testing.T) {
	f := makeDirectLoftEdgeFixture(t)
	require.Nil(t, newDirectLoftEdgeCertifier(f.target, f.targetBool, new(topo.Body), f.loft, f.loftBool))
	require.Nil(t, newDirectLoftEdgeCertifier(nil, f.targetBool, f.body, f.loft, f.loftBool))
	require.Nil(t, newDirectLoftEdgeCertifier(f.target, f.targetBool, f.body, f.loft, nil))
}

func TestOnPlane(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	require.True(t, onPlane(frame, []r3.Vec{{X: 3, Y: -2}, {X: 1}}))
	require.False(t, onPlane(frame, []r3.Vec{{X: 3, Z: 1e-300}}))
	require.False(t, onPlane(r3.Frame{}, []r3.Vec{{}}))
}
