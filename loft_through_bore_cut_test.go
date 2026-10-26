package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// throughBoreFixture is a 10 x 10 axial square Loft two tall and a circular
// through-tool of radius 1 at (1, 0.5) that spans both caps, as the public
// Cut sees them.
func throughBoreFixture(t *testing.T) (*Document, loftPayload, prismPayload) {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	top, err := world.CreateOffsetPlane(base, 2)
	require.NoError(t, err)
	square := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		r := s.CreateRectangle(-5, -5, 5, 5)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	s0, p0 := square(base)
	s1, p1 := square(top)
	doc := New()
	blank, err := Loft(t.Context(), doc, s0, p0, s1, p1)
	require.NoError(t, err)

	toolFrame, err := r3.NewFrame(r3.NewVec(0, 0, -1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	toolPlane, err := world.CreatePlaneFromFrame(toolFrame)
	require.NoError(t, err)
	ts, err := world.CreateSketch(toolPlane)
	require.NoError(t, err)
	center := ts.CreatePoint(1, 0.5)
	ts.Fix(center)
	ts.CreateCircle(center, 1)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	return doc, blank.n.Payload.(loftPayload), tool.n.Payload.(prismPayload)
}

// TestLoftThroughBoreCutGates breaks one fact of the admitted pair at a time.
// The tool's own placement and frame gates are implied by the exact center
// and cap checks that follow them, so no input isolates them, and the partial
// cancellation path of the containment audit needs a context that expires
// mid-call.
func TestLoftThroughBoreCutGates(t *testing.T) {
	doc, loft, tool := throughBoreFixture(t)
	cut := func(ctx context.Context, a loftPayload, b prismPayload) (bool, error) {
		node, ok, err := loftrecord.ThroughBoreCut(ctx, envOf(documentNode(doc), doc.nextProducerID()), a, b)
		if ok {
			require.NotNil(t, node)
			p, isLoft := node.Payload.(loftPayload)
			require.True(t, isLoft)
			require.True(t, p.ThroughBoreCut)
			require.Len(t, p.Profile0.Holes, 1)
			require.Len(t, p.Profile1.Holes, 1)
			require.True(t, p.AnalyticBore, "the fixture's axis and frames are exact")
		} else {
			require.Nil(t, node)
		}
		return ok, err
	}

	ok, err := cut(t.Context(), loft, tool)
	require.NoError(t, err)
	require.True(t, ok, "the fixture is the admitted class")

	flipU, err := r3.NewFrame(loft.Frame1.Origin(), r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	flipV, err := r3.NewFrame(loft.Frame1.Origin(), r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
	require.NoError(t, err)
	off0, err := r3.NewFrame(r3.NewVec(0.1, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	off1, err := r3.NewFrame(r3.NewVec(0.1, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	shifted, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	toolFlipU, err := r3.NewFrame(tool.Frame.Origin(), r3.NewVec(-1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	toolFlipV, err := r3.NewFrame(tool.Frame.Origin(), r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0))
	require.NoError(t, err)
	raised, err := r3.NewFrame(r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	withHole := loft.Profile0
	withHole.Holes = append([]sectionrecord.LoopRecord(nil), loft.Profile0.Outer)
	other := loft.Profile1
	other.Outer.Segments = append([]sectionrecord.CurveSegment(nil), other.Outer.Segments[:len(other.Outer.Segments)-1]...)
	line := sectionrecord.LineSeg{End: sectionrecord.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1}
	circle := tool.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)

	loftGates := []struct {
		name string
		edit func(*loftPayload)
	}{
		{"surface result", func(p *loftPayload) { p.SurfaceResult = true }},
		{"a section hole", func(p *loftPayload) { p.Profile0, p.Profile1 = withHole, withHole }},
		{"sections that differ", func(p *loftPayload) { p.Profile1 = other }},
		{"frames with different U", func(p *loftPayload) { p.Frame1 = flipU }},
		{"frames with different V", func(p *loftPayload) { p.Frame1 = flipV }},
		{"frames whose lifted tool center is rounded", func(p *loftPayload) { p.Frame0, p.Frame1 = off0, off1 }},
		{"a placed loft", func(p *loftPayload) { p.Xform = shifted }},
		{"an aligned section start", func(p *loftPayload) { p.Alignment = []int{1} }},
		{"caps in the wrong order", func(p *loftPayload) { p.Frame0, p.Frame1 = p.Frame1, p.Frame0 }},
		{"a hole the section cannot hold", func(p *loftPayload) { p.Profile0.Outer.Segments = nil }},
	}
	for _, tc := range loftGates {
		t.Run(tc.name, func(t *testing.T) {
			p := loft
			tc.edit(&p)
			ok, err := cut(t.Context(), p, tool)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}

	toolGates := []struct {
		name string
		edit func(*prismPayload)
	}{
		{"a surface result tool", func(p *prismPayload) { p.SurfaceResult = true }},
		{"a placed tool", func(p *prismPayload) { p.Xform = shifted }},
		{"a displaced section", func(p *prismPayload) { p.SectionDelta = 1e-3 }},
		{"a displaced start level", func(p *prismPayload) { p.Z0Delta = 1e-3 }},
		{"a displaced end level", func(p *prismPayload) { p.Z1Delta = 1e-3 }},
		{"a tool frame with a flipped U", func(p *prismPayload) { p.Frame = toolFlipU }},
		{"a tool frame with a flipped V", func(p *prismPayload) { p.Frame = toolFlipV }},
		{"a tool with a hole", func(p *prismPayload) { p.Profile.Holes = []sectionrecord.LoopRecord{p.Profile.Outer} }},
		{"a tool with two segments", func(p *prismPayload) {
			p.Profile.Outer.Segments = append(append([]sectionrecord.CurveSegment(nil), p.Profile.Outer.Segments...), line)
		}},
		{"a straight tool segment", func(p *prismPayload) { p.Profile.Outer.Segments = []sectionrecord.CurveSegment{line} }},
		{"a partial circle", func(p *prismPayload) {
			half := circle
			half.TEnd = 0.5
			p.Profile.Outer.Segments = []sectionrecord.CurveSegment{half}
		}},
		{"a tool that stops short of the top cap", func(p *prismPayload) { p.Z1 = 1 }},
		{"a tool that starts above the bottom cap", func(p *prismPayload) { p.Z0 = 1.5 }},
		{"a tool frame raised above the bottom cap", func(p *prismPayload) { p.Frame = raised }},
		{"a circle too big for the section", func(p *prismPayload) {
			big := circle
			big.Radius = units.Millimeters(9)
			p.Profile.Outer.Segments = []sectionrecord.CurveSegment{big}
		}},
	}
	for _, tc := range toolGates {
		t.Run(tc.name, func(t *testing.T) {
			p := tool
			tc.edit(&p)
			ok, err := cut(t.Context(), loft, p)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}

	t.Run("not a loft", func(t *testing.T) {
		node, ok, err := loftrecord.ThroughBoreCut(t.Context(), envOf(documentNode(doc), doc.nextProducerID()), tool, tool)
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, node)
	})
	t.Run("not a prism tool", func(t *testing.T) {
		node, ok, err := loftrecord.ThroughBoreCut(t.Context(), envOf(documentNode(doc), doc.nextProducerID()), loft, loft)
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, node)
	})
	t.Run("a canceled context", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		ok, err := cut(ctx, loft, tool)
		require.False(t, ok)
		require.ErrorIs(t, err, context.Canceled)
	})
}

// TestPlacedLoftBoreChargesPlacement places a certified through-bore Loft.
// The rebuild keeps the analytic bore mark, so the placed cylinder must carry
// the departure the rotation leaves on its tag; the unplaced one has none.
func TestPlacedLoftBoreChargesPlacement(t *testing.T) {
	doc, loft, tool := throughBoreFixture(t)
	env := envOf(documentNode(doc), doc.nextProducerID())
	node, ok, err := loftrecord.ThroughBoreCut(t.Context(), env, loft, tool)
	require.NoError(t, err)
	require.True(t, ok)
	cylinderBound := func(body *topo.Body) float64 {
		t.Helper()
		var bounds []float64
		for _, face := range topobuild.Faces(body) {
			if _, isCylinder := face.Surface.(surfacegeom.Cylinder); isCylinder {
				bounds = append(bounds, face.NormalBound)
			}
		}
		require.Len(t, bounds, 1)
		return bounds[0]
	}
	require.Zero(t, cylinderBound(node))
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Radians(0.7))
	require.NoError(t, err)
	placed, err := node.Payload.(loftPayload).Placed(t.Context(), envOf(documentNode(doc), doc.nextProducerID()), rotation)
	require.NoError(t, err)
	want, ok := loftrecord.BoreCylinderNormalBound(loft.Frame0, rotation, 1, 0.5, 2, 1)
	require.True(t, ok)
	require.Positive(t, want)
	require.Equal(t, want, cylinderBound(placed))
}
