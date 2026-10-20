package capedge_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capedge"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func trapezoid(t *testing.T) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	corners := []sectionrecord.Point2{{}, {U: 40}, {U: 30, V: 20}, {U: 10, V: 20}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TEnd: 1})
	}
	return prismrecord.Payload{Frame: frame, Xform: r3.Identity(), Z0: 0, Z1: 10,
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}}
}

func edgeBetween(a, b r3.Vec) *topo.Edge {
	return &topo.Edge{Start: &topo.Vertex{Position: a}, End: &topo.Vertex{Position: b}}
}

func TestObliqueProfile(t *testing.T) {
	t.Parallel()

	pp := trapezoid(t)
	low, high := r3.NewVec(0, 0, 0), r3.NewVec(40, 0, 0)
	require.True(t, capedge.ObliqueProfile(pp, edgeBetween(low, high), 2), "bottom cap edge")
	require.True(t, capedge.ObliqueProfile(pp, edgeBetween(high, low), 2), "either direction")
	require.True(t, capedge.ObliqueProfile(pp, edgeBetween(r3.NewVec(0, 0, 10), r3.NewVec(40, 0, 10)), 2), "top cap edge")

	require.False(t, capedge.ObliqueProfile(pp, edgeBetween(low, r3.NewVec(40, 0, 1)), 2), "not a cap edge")
	require.False(t, capedge.ObliqueProfile(pp, edgeBetween(low, r3.NewVec(40.01, 0, 0)), 2), "endpoint off by more than 1e-6")
	require.True(t, capedge.ObliqueProfile(pp, edgeBetween(low, r3.NewVec(40+5e-7, 0, 0)), 2), "endpoint within 1e-6")
	require.True(t, capedge.ObliqueProfile(pp, edgeBetween(low, r3.NewVec(40+5e-7, 0, 0)), 2))
	require.False(t, capedge.ObliqueProfile(pp, edgeBetween(low, r3.NewVec(40+5e-4, 0, 0)), 2), "endpoint off by 5e-4")
	require.False(t, capedge.ObliqueProfile(pp, edgeBetween(low, high), 5), "radius too large for the clearance")

	// The edge's own index in the walk selects its neighbours and clearance.
	slant := edgeBetween(r3.NewVec(40, 0, 0), r3.NewVec(30, 20, 0))
	require.True(t, capedge.ObliqueProfile(pp, slant, 2))
	require.False(t, capedge.ObliqueProfile(pp, slant, 4.5), "the far corner is within 4r of this edge but not of the first")
	require.True(t, capedge.ObliqueProfile(pp, edgeBetween(low, high), 4.5))

	// The same edge at both levels of a flat prism is ambiguous.
	flat := pp
	flat.Z1 = 0
	require.False(t, capedge.ObliqueProfile(flat, edgeBetween(low, high), 2))

	// A section with an arc is not a straight convex section.
	curved := pp
	curved.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), pp.Profile.Outer.Segments...)
	curved.Profile.Outer.Segments[1] = sectionrecord.ArcSeg{}
	require.False(t, capedge.ObliqueProfile(curved, edgeBetween(low, high), 2))
}
