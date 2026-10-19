package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCanonicalFrame(t *testing.T) {
	frame, err := loftcylinder.CanonicalFrame(2.5)
	require.NoError(t, err)
	require.Equal(t, r3.Vec{Z: 2.5}, frame.Origin())
	require.Equal(t, r3.Vec{X: 1}, frame.U())
	require.Equal(t, r3.Vec{Y: 1}, frame.V())
	require.Equal(t, r3.Vec{Z: 1}, frame.N())
}

// placedToothPayload is the smallest Loft record PlacedToothShape accepts:
// both sections carry the six circular-Union curve kinds, joined at the root
// arc's endpoints.
func placedToothPayload() loftrecord.Payload {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	section := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(1, 0), End: p(3, 0), TStart: 0, TEnd: 1},
		sectionrecord.FitSplineSeg{},
		sectionrecord.ArcSeg{},
		sectionrecord.FitSplineSeg{},
		sectionrecord.LineSeg{Start: p(3, 1), End: p(0, 1), TStart: 0, TEnd: 1},
		sectionrecord.ArcSeg{Start: p(1, 0), End: p(0, 1), TStart: 1, TEnd: 0},
	}}}
	return loftrecord.Payload{
		AuthenticatedReconstruction: true,
		SourceIdentity:              &loftrecord.SourceIdentity{Origin: 4},
		Profile0:                    section,
		Profile1:                    section,
		Segment:                     [][]int{{0}},
		Verts:                       make([]r3.Vec, 16),
	}
}

func TestPlacedToothShape(t *testing.T) {
	require.True(t, loftcylinder.PlacedToothShape(&topo.Body{Payload: placedToothPayload()}))
	refusals := []struct {
		name string
		edit func(lp *loftrecord.Payload)
	}{
		{name: "unauthenticated", edit: func(lp *loftrecord.Payload) { lp.AuthenticatedReconstruction = false }},
		{name: "surface result", edit: func(lp *loftrecord.Payload) { lp.SurfaceResult = true }},
		{name: "no source identity", edit: func(lp *loftrecord.Payload) { lp.SourceIdentity = nil }},
		{name: "a hole", edit: func(lp *loftrecord.Payload) {
			lp.Profile1.Holes = []sectionrecord.LoopRecord{lp.Profile1.Outer}
		}},
		{name: "five curves", edit: func(lp *loftrecord.Payload) {
			lp.Profile0.Outer.Segments = lp.Profile0.Outer.Segments[:5]
		}},
		{name: "two segments", edit: func(lp *loftrecord.Payload) { lp.Segment = [][]int{{0}, {1}} }},
		{name: "odd vertex count", edit: func(lp *loftrecord.Payload) { lp.Verts = make([]r3.Vec, 17) }},
		{name: "too few vertices", edit: func(lp *loftrecord.Payload) { lp.Verts = make([]r3.Vec, 14) }},
		{name: "wrong curve kind", edit: func(lp *loftrecord.Payload) {
			segments := append([]sectionrecord.CurveSegment(nil), lp.Profile1.Outer.Segments...)
			segments[2] = sectionrecord.LineSeg{}
			lp.Profile1.Outer.Segments = segments
		}},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			lp := placedToothPayload()
			tc.edit(&lp)
			require.False(t, loftcylinder.PlacedToothShape(&topo.Body{Payload: lp}))
		})
	}
	require.False(t, loftcylinder.PlacedToothShape(&topo.Body{Payload: prismrecord.Payload{}}), "not a Loft")
}

func TestSourceIdentitiesMatch(t *testing.T) {
	source := &loftrecord.SourceIdentity{Origin: 4}
	tooth := func(id *loftrecord.SourceIdentity) *topo.Body {
		return &topo.Body{Payload: loftrecord.Payload{SourceIdentity: id}}
	}
	require.True(t, loftcylinder.SourceIdentitiesMatch([]*topo.Body{tooth(source)}))
	require.True(t, loftcylinder.SourceIdentitiesMatch([]*topo.Body{tooth(source), tooth(source)}))
	require.False(t, loftcylinder.SourceIdentitiesMatch([]*topo.Body{tooth(nil)}), "no source identity")
	require.False(t, loftcylinder.SourceIdentitiesMatch([]*topo.Body{
		tooth(source), tooth(&loftrecord.SourceIdentity{Origin: 4})}), "an equal but distinct identity")
	require.False(t, loftcylinder.SourceIdentitiesMatch([]*topo.Body{
		tooth(source), {Payload: prismrecord.Payload{}}}), "a later tooth that is not a Loft")
}

// placedBlank is a 2 mm circle prism of height 3 whose frame sits at z=5.
func placedBlank(t *testing.T) *topo.Body {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{Z: 5}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return &topo.Body{Origin: provenance.Ref{Producer: 9}, Payload: prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Radius: units.Millimeters(2), CCW: true, TStart: 0, TEnd: 1},
		}}},
		Frame: frame, Z0: 0, Z1: 3, Xform: r3.Identity(),
	}}
}

func TestLocalBlank(t *testing.T) {
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	t.Run("rebuilds the blank in the canonical frame", func(t *testing.T) {
		local, pose, ok, err := loftcylinder.LocalBlank(t.Context(), env, placedBlank(t))
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, r3.Vec{Z: 5}, pose.Translation())
		require.Equal(t, r3.Identity().Basis(), pose.Basis())
		require.Equal(t, provenance.Producer(9), local.Origin.Producer, "the blank's own producer")
		pp, isPrism := local.Payload.(prismrecord.Payload)
		require.True(t, isPrism)
		require.Equal(t, r3.Vec{}, pp.Frame.Origin())
		require.Equal(t, r3.Identity(), pp.Xform)
		require.Equal(t, 3.0, pp.Z1)
	})
	refusals := []struct {
		name string
		edit func(pp *prismrecord.Payload)
	}{
		{name: "identity pose", edit: func(pp *prismrecord.Payload) {
			frame, err := loftcylinder.CanonicalFrame(0)
			require.NoError(t, err)
			pp.Frame = frame
		}},
		{name: "extent off the frame", edit: func(pp *prismrecord.Payload) { pp.Z0 = 1 }},
		{name: "clockwise circle", edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments = []sectionrecord.CurveSegment{
				sectionrecord.CircleSeg{Radius: units.Millimeters(2), TStart: 0, TEnd: 1}}
		}},
		{name: "off-center circle", edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments = []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
				Center: sectionrecord.Point2{V: 0.5}, Radius: units.Millimeters(2), CCW: true, TStart: 0, TEnd: 1}}
		}},
		{name: "section displacement", edit: func(pp *prismrecord.Payload) { pp.SectionDelta = 0.25 }},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			blank := placedBlank(t)
			pp, ok := blank.Payload.(prismrecord.Payload)
			require.True(t, ok)
			tc.edit(&pp)
			blank.Payload = pp
			local, _, ok, err := loftcylinder.LocalBlank(t.Context(), env, blank)
			require.NoError(t, err)
			require.False(t, ok)
			require.Nil(t, local)
		})
	}
}

func TestLocalToothRefusesUnmatchedSources(t *testing.T) {
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	pose, err := r3.Translation(r3.Vec{Z: 5})
	require.NoError(t, err)
	frame0, err := loftcylinder.CanonicalFrame(5)
	require.NoError(t, err)
	frame1, err := loftcylinder.CanonicalFrame(8)
	require.NoError(t, err)
	plane := func(f r3.Frame) sectionrecord.PlaneRecord {
		return sectionrecord.PlaneRecord{Origin: f.Origin(), U: f.U(), V: f.V()}
	}
	matched := func() loftrecord.Payload {
		lp := placedToothPayload()
		lp.Frame0, lp.Frame1 = frame0, frame1
		lp.Plane0, lp.Plane1 = plane(frame0), plane(frame1)
		lp.Xform = r3.Identity()
		return lp
	}
	cases := []struct {
		name string
		edit func(lp *loftrecord.Payload)
	}{
		{name: "unauthenticated", edit: func(lp *loftrecord.Payload) { lp.AuthenticatedReconstruction = false }},
		{name: "no source identity", edit: func(lp *loftrecord.Payload) { lp.SourceIdentity = nil }},
		{name: "start frame off its plane", edit: func(lp *loftrecord.Payload) { lp.Plane0 = plane(frame1) }},
		{name: "end frame off its plane", edit: func(lp *loftrecord.Payload) { lp.Plane1 = plane(frame0) }},
		{name: "end section at another height", edit: func(lp *loftrecord.Payload) {
			other, err := loftcylinder.CanonicalFrame(9)
			require.NoError(t, err)
			lp.Frame1, lp.Plane1 = other, plane(other)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			lp := matched()
			tc.edit(&lp)
			local, ok, err := loftcylinder.LocalTooth(t.Context(), env, &topo.Body{Payload: lp}, pose, 3)
			require.NoError(t, err)
			require.False(t, ok)
			require.Nil(t, local)
		})
	}
	local, ok, err := loftcylinder.LocalTooth(t.Context(), env, &topo.Body{Payload: prismrecord.Payload{}}, pose, 3)
	require.NoError(t, err)
	require.False(t, ok, "not a Loft")
	require.Nil(t, local)
}
