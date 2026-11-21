package booleaneval_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMixedLoftWallUsesItsOwnPlane(t *testing.T) {
	// A quarter disk gives two straight source segments and one curved one.
	origin, u, v := momentinput.Point2{}, momentinput.Point2{U: 2}, momentinput.Point2{V: 2}
	profile := momentinput.Profile{Outer: momentinput.LoopRecord{Segments: []momentinput.CurveSegment{
		momentinput.LineSeg{Start: origin, End: u, TEnd: 1},
		momentinput.ArcSeg{Center: origin, Start: u, End: v, TEnd: 1},
		momentinput.LineSeg{Start: v, End: origin, TEnd: 1},
	}}}
	p0 := sectionrecord.PlaneRecord{U: r3.Vec{X: 1}, V: r3.Vec{Y: 1}}
	p1 := p0
	p1.Origin = r3.Vec{Z: 3}
	f0, err := r3.NewFrame(p0.Origin, p0.U, p0.V)
	require.NoError(t, err)
	f1, err := r3.NewFrame(p1.Origin, p1.U, p1.V)
	require.NoError(t, err)
	input := loftrecord.Payload{Profile0: profile, Profile1: profile, Frame0: f0, Frame1: f1, Plane0: p0, Plane1: p1,
		Xform: r3.Identity(), RecordArea: [2]float64{math.Pi, math.Pi}, AuthenticatedReconstruction: true}
	env := topo.Env{Doc: new(topo.Document), Alloc: topotest.Alloc{}, Ref: 1}
	body, err := loftrecord.Build(t.Context(), env, input, proofbound.NewWorkBudget(t.Context()),
		freeform.NewFreeformWork(), freeform.NewFreeformWork())
	require.NoError(t, err)
	loft := body.Payload.(loftrecord.Payload)
	require.Greater(t, loft.Proof.FacetDeparture, 1e-5)
	var straight, curved int
	for _, face := range body.Lumps[0].Shells[0].Faces {
		if face.CapRecord != nil {
			continue
		}
		group, ok := facetedrecord.SourceTag(face, false)
		require.True(t, ok)
		got := booleaneval.ChordedLoftPlane(group, body, face)
		// The two radial side planes contain the origin. Each circular
		// chord's wall plane is offset from it by nearly the circle radius.
		normal := group.Surface.(surfacegeom.Plane).Frame.N()
		if normal.X == 0 || normal.Y == 0 {
			require.Less(t, got.AxialDelta, 1e-10)
			_, has := loft.StraightWallPlaneBound(new(topo.Face), group.Surface.(surfacegeom.Plane))
			require.False(t, has)
			_, has = loft.StraightWallPlaneBound(face, surfacegeom.Plane{Frame: f0})
			require.False(t, has)
			straight++
		} else {
			require.Equal(t, loft.Proof.FacetDeparture, got.AxialDelta)
			curved++
		}
	}
	require.Equal(t, 4, straight)
	require.Positive(t, curved)
}
