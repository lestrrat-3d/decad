package loftcylinder_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// boreToolPayload is a 1 mm centered full-circle tool running from z=-1 to
// z=3 in the canonical XY frame.
func boreToolPayload(t *testing.T) prismrecord.Payload {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1},
		}}},
		Frame: frame, Z0: -1, Z1: 3, Xform: r3.Identity(),
	}
}

func TestBoreTool(t *testing.T) {
	t.Run("a centered full circle", func(t *testing.T) {
		radius, ok, err := loftcylinder.BoreTool(boreToolPayload(t), 2)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, 1.0, radius)
	})
	t.Run("a reversed full circle", func(t *testing.T) {
		pp := boreToolPayload(t)
		pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(1), TStart: 1, TEnd: 0}
		radius, ok, err := loftcylinder.BoreTool(pp, 2)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, 1.0, radius)
	})
	t.Run("a radius that is not a length", func(t *testing.T) {
		pp := boreToolPayload(t)
		pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Degrees(1), TStart: 0, TEnd: 1}
		_, ok, err := loftcylinder.BoreTool(pp, 2)
		require.Error(t, err)
		require.False(t, ok)
	})
	refusals := []struct {
		name  string
		blank float64
		edit  func(pp *prismrecord.Payload)
	}{
		{name: "surface result", blank: 2, edit: func(pp *prismrecord.Payload) { pp.SurfaceResult = true }},
		{name: "section displacement", blank: 2, edit: func(pp *prismrecord.Payload) { pp.SectionDelta = 0.5 }},
		{name: "start displacement", blank: 2, edit: func(pp *prismrecord.Payload) { pp.Z0Delta = 0.5 }},
		{name: "end displacement", blank: 2, edit: func(pp *prismrecord.Payload) { pp.Z1Delta = 0.5 }},
		{name: "a hole", blank: 2, edit: func(pp *prismrecord.Payload) {
			pp.Profile.Holes = []sectionrecord.LoopRecord{pp.Profile.Outer}
		}},
		{name: "two segments", blank: 2, edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments = append(pp.Profile.Outer.Segments, pp.Profile.Outer.Segments[0])
		}},
		{name: "not a circle", blank: 2, edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.LineSeg{}
		}},
		{name: "off-center circle", blank: 2, edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 0.1},
				Radius: units.Millimeters(1), TStart: 0, TEnd: 1}
		}},
		{name: "partial circle", blank: 2, edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(1), TStart: 0, TEnd: 0.5}
		}},
		{name: "radius reaches the blank", blank: 1},
		{name: "zero radius", blank: 2, edit: func(pp *prismrecord.Payload) {
			pp.Profile.Outer.Segments[0] = sectionrecord.CircleSeg{Radius: units.Millimeters(0), TStart: 0, TEnd: 1}
		}},
		{name: "non-finite end", blank: 2, edit: func(pp *prismrecord.Payload) { pp.Z1 = math.Inf(1) }},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			pp := boreToolPayload(t)
			if tc.edit != nil {
				tc.edit(&pp)
			}
			radius, ok, err := loftcylinder.BoreTool(pp, tc.blank)
			require.NoError(t, err)
			require.False(t, ok)
			require.Zero(t, radius)
		})
	}
}

func TestBoreThrough(t *testing.T) {
	require.True(t, loftcylinder.BoreThrough(0, -1, 3, 0, 2))
	require.True(t, loftcylinder.BoreThrough(1, -2, 2, 0, 2), "the origin level shifts both ends")
	require.False(t, loftcylinder.BoreThrough(0, 0, 3, 0, 2), "a tool starting on the bottom cap")
	require.False(t, loftcylinder.BoreThrough(0, -1, 2, 0, 2), "a tool ending on the top cap")
	require.False(t, loftcylinder.BoreThrough(1, -1, 3, 0, 2), "a tool starting at the bottom cap after its shift")
	// 1+2⁻⁶⁰ rounds to 1 in float; the exact end still lies above the top cap.
	require.True(t, loftcylinder.BoreThrough(1, -2, 0x1p-60, 0, 1), "the end sum is exact, not rounded")
	// -1-2⁻⁶⁰ rounds to -1 in float; the exact start still lies below the bottom cap.
	require.True(t, loftcylinder.BoreThrough(-1, -0x1p-60, 3, -1, 1), "the start sum is exact, not rounded")
}

func TestBoreCutLeavesOtherOperands(t *testing.T) {
	joined := facetedrecord.Payload{ContactAudited: true, Xform: r3.Identity(),
		LoftCylinderJoin: &facetedrecord.LoftCylinderJoinRecord{Radius: 2, Z0: 0, Z1: 2}}
	// Every tool but the last carries the cylindrical side wall BoreCut looks
	// for, so each case is refused by its own gate: past them all, the empty
	// join record is admitted and its build fails.
	tool := func(edit func(pp *prismrecord.Payload)) *topo.Body {
		pp := boreToolPayload(t)
		if edit != nil {
			edit(&pp)
		}
		wall := &topo.Face{Surface: surfacegeom.Cylinder{Axis: r3.Vec{Z: 1}, Radius: units.Millimeters(1)},
			Origins: []provenance.Ref{{Producer: 5, Role: prismrecord.OuterFirstSideRole}}}
		return &topo.Body{Payload: pp,
			Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{wall}}}}}}
	}
	t.Run("an admitted tool reaches the build", func(t *testing.T) {
		body, admitted, err := loftcylinder.BoreCut(t.Context(), topo.Env{}, joined, tool(nil))
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.True(t, admitted)
		require.Nil(t, body)
	})
	cases := []struct {
		name   string
		joined func(fp facetedrecord.Payload) facetedrecord.Payload
		tool   *topo.Body
	}{
		{name: "unaudited join", tool: tool(nil),
			joined: func(fp facetedrecord.Payload) facetedrecord.Payload { fp.ContactAudited = false; return fp }},
		{name: "no join record", tool: tool(nil),
			joined: func(fp facetedrecord.Payload) facetedrecord.Payload { fp.LoftCylinderJoin = nil; return fp }},
		{name: "tool is not a prism", tool: &topo.Body{Payload: loftrecord.Payload{}}},
		{name: "tool off the axis", tool: tool(func(pp *prismrecord.Payload) {
			frame, err := r3.NewFrame(r3.Vec{X: 0.5}, r3.Vec{X: 1}, r3.Vec{Y: 1})
			require.NoError(t, err)
			pp.Frame = frame
		})},
		{name: "tool ends inside the join", tool: tool(func(pp *prismrecord.Payload) { pp.Z1 = 1 })},
		{name: "tool has no side wall face", tool: &topo.Body{Payload: boreToolPayload(t)}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			fp := joined
			if tc.joined != nil {
				fp = tc.joined(fp)
			}
			body, admitted, err := loftcylinder.BoreCut(t.Context(), topo.Env{}, fp, tc.tool)
			require.NoError(t, err)
			require.False(t, admitted)
			require.Nil(t, body)
		})
	}
}
