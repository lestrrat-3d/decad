package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// allEdgeChamferFixture is a 40x20x20 box with one through-y bore of radius 3
// at (20, 10), the shape TryAllEdgeChamfer accepts, with its twelve straight
// edges.
func allEdgeChamferFixture(t *testing.T) (*topo.Body, brepPayload, []*topo.Edge) {
	t.Helper()
	return allEdgeChamferBore(t, 40, 20, 10)
}

// allEdgeChamferBore is the fixture with a box of the given x length and the
// bore centred at (cx, cz).
func allEdgeChamferBore(t *testing.T, length, cx, cz float64) (*topo.Body, brepPayload, []*topo.Edge) {
	t.Helper()
	doc := New()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, length, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	plane, err := world.CreateOffsetPlane(world.XZ(), -10)
	require.NoError(t, err)
	ds, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := ds.CreatePoint(cx, cz)
	ds.Fix(center)
	ds.CreateCircle(center, 3)
	_, err = ds.Solve(t.Context())
	require.NoError(t, err)
	drill, err := Extrude(t.Context(), doc, ds, ds.Profiles()[0], Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)
	bar, err := Cut(t.Context(), box, drill)
	require.NoError(t, err)
	bp, ok := bar.n.Payload.(brepPayload)
	require.True(t, ok)
	selected, err := Edges(ParallelTo(r3.NewVec(1, 0, 0))).Or(ParallelTo(r3.NewVec(0, 1, 0))).
		Or(ParallelTo(r3.NewVec(0, 0, 1))).Exactly(12).SelectEdges(bar)
	require.NoError(t, err)
	return &bar.n, bp, edgeNodes(selected)
}

// TestTryAllEdgeChamferGates breaks one fact of the accepted box-with-bore
// at a time. A broken fact makes the arm decline (matched false, no error)
// unless it is a setback the arm owns, which it refuses.
func TestTryAllEdgeChamferGates(t *testing.T) {
	t.Parallel()

	run := func(t *testing.T, edit func(*topo.Body, *brepPayload, *[]*topo.Edge, *float64, *float64, *bool)) (*topo.Body, bool, error) {
		t.Helper()
		body, bp, edges := allEdgeChamferFixture(t)
		bp.Faces = append([]breprecord.Face(nil), bp.Faces...)
		distance, delta, asym := 1.0, 0.0, false
		if edit != nil {
			edit(body, &bp, &edges, &distance, &delta, &asym)
		}
		return breprecord.TryAllEdgeChamfer(t.Context(), envOf(body.Doc, body.Doc.NextProducer), body, bp,
			edges, distance, delta, asym)
	}
	faceWith := func(body *topo.Body, match func(*topo.Face) bool) *topo.Face {
		for _, f := range topobuild.Faces(body) {
			if match(f) {
				return f
			}
		}
		return nil
	}
	yPlane := func(f *topo.Face) bool {
		p, ok := f.Surface.(surfacegeom.Plane)
		return ok && (p.Frame.N() == r3.Vec{Y: 1} || p.Frame.N() == r3.Vec{Y: -1})
	}
	xPlane := func(f *topo.Face) bool {
		p, ok := f.Surface.(surfacegeom.Plane)
		return ok && (p.Frame.N() == r3.Vec{X: 1} || p.Frame.N() == r3.Vec{X: -1})
	}
	cylinder := func(f *topo.Face) bool { _, ok := f.Surface.(surfacegeom.Cylinder); return ok }

	t.Run("accepts the box with its bore", func(t *testing.T) {
		got, matched, err := run(t, nil)
		require.NoError(t, err)
		require.True(t, matched)
		require.NotNil(t, got)
	})

	declines := []struct {
		name string
		edit func(*topo.Body, *brepPayload, *[]*topo.Edge, *float64, *float64, *bool)
	}{
		{"eleven edges", func(_ *topo.Body, _ *brepPayload, e *[]*topo.Edge, _, _ *float64, _ *bool) { *e = (*e)[:11] }},
		{"asymmetric setbacks", func(_ *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, a *bool) { *a = true }},
		{"six faces", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.Faces = bp.Faces[:6]
		}},
		{"placed record", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.Xform, _ = r3.Translation(r3.Vec{X: 1})
		}},
		{"section displacement", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.Faces[0].Delta = 1
		}},
		{"level displacement", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.Faces[0].Z1Delta = 1
		}},
		{"loop bands", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.LoopBands = []breprecord.LoopBand{{}}
		}},
		{"boss shell", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.BossShell = &prismshell.BossShellBand{}
		}},
		{"pocket shell", func(_ *topo.Body, bp *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			bp.PocketShell = &prismshell.PocketShellBand{}
		}},
		{"held bounds", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			b.Bounds.Bound = units.Millimeters(1)
		}},
		{"setback wider than the box", func(_ *topo.Body, _ *brepPayload, _ *[]*topo.Edge, d, _ *float64, _ *bool) {
			*d = 15
		}},
		{"repeated edge", func(_ *topo.Body, _ *brepPayload, e *[]*topo.Edge, _, _ *float64, _ *bool) {
			(*e)[1] = (*e)[0]
		}},
		{"non-box edge", func(b *topo.Body, _ *brepPayload, e *[]*topo.Edge, _, _ *float64, _ *bool) {
			(*e)[0] = faceWith(b, cylinder).Loops[0].CoEdges[0].Edge
		}},
		{"bore plane without its bore loop", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, yPlane)
			f.Loops = f.Loops[:1]
		}},
		{"bore loop with two edges", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, yPlane)
			f.Loops[1].CoEdges = append(f.Loops[1].CoEdges, f.Loops[1].CoEdges[0])
		}},
		{"bore circle off its plane", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			ce := &faceWith(b, yPlane).Loops[1].CoEdges[0]
			moved := *ce.Edge
			circle := moved.Curve.(surfacegeom.Circle3)
			circle.Center.Y += 1
			moved.Curve = circle
			ce.Edge = &moved
		}},
		{"extra loop on a side plane", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, xPlane)
			f.Loops = append(f.Loops, f.Loops[0])
		}},
		{"bore wall not reversed", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			faceWith(b, cylinder).Reversed = false
		}},
		{"bore wall with one loop", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, cylinder)
			f.Loops = f.Loops[:1]
		}},
		{"bore wall radius differs", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, cylinder)
			s := f.Surface.(surfacegeom.Cylinder)
			s.Radius = units.Millimeters(2)
			f.Surface = s
		}},
		{"bore rim with a held curve bound", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			e := faceWith(b, cylinder).Loops[0].CoEdges[0].Edge
			e.CurveBounded, e.CurveBound = true, 1e-3
		}},
		{"bore rim with an unbounded curve", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			faceWith(b, cylinder).Loops[0].CoEdges[0].Edge.CurveBounded = false
		}},
		{"bore rim off the box centre", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, cylinder)
			rim := *f.Loops[0].CoEdges[0].Edge
			circle := rim.Curve.(surfacegeom.Circle3)
			circle.Center.X += 1
			rim.Curve = circle
			f.Loops[0].CoEdges[0].Edge = &rim
		}},
		{"bore rim of another radius", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, cylinder)
			rim := *f.Loops[0].CoEdges[0].Edge
			circle := rim.Curve.(surfacegeom.Circle3)
			circle.Radius = units.Millimeters(2)
			rim.Curve = circle
			f.Loops[0].CoEdges[0].Edge = &rim
		}},
		{"bore wall loop with two edges", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			l := faceWith(b, cylinder).Loops[0]
			l.CoEdges = append(l.CoEdges, l.CoEdges[0])
		}},
		{"bore rims on one side", func(b *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, _ *float64, _ *bool) {
			f := faceWith(b, cylinder)
			f.Loops[1].CoEdges[0].Edge = f.Loops[0].CoEdges[0].Edge
		}},
	}
	for _, tc := range declines {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, matched, err := run(t, tc.edit)
			require.NoError(t, err)
			require.False(t, matched)
		})
	}

	refusals := []struct {
		name string
		msg  string
		edit func(*topo.Body, *brepPayload, *[]*topo.Edge, *float64, *float64, *bool)
	}{
		{"setback with a held displacement", "exact millimetre setback", func(_ *topo.Body, _ *brepPayload, _ *[]*topo.Edge, _, delta *float64, _ *bool) {
			*delta = 1e-9
		}},
		{"setback not exactly representable", "an all-edge chamfer corner is not exactly", func(_ *topo.Body, _ *brepPayload, _ *[]*topo.Edge, d, _ *float64, _ *bool) {
			*d = 0.1
		}},
		{"bore reaches the chamfer", "the bore reaches", func(_ *topo.Body, _ *brepPayload, _ *[]*topo.Edge, d, _ *float64, _ *bool) {
			*d = 8
		}},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, matched, err := run(t, tc.edit)
			require.True(t, matched)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, tc.msg)
		})
	}

	// The bore reaches the chamfer on any of the four sides the bore can
	// approach: the box's x ends, and the z ends.
	for _, tc := range []struct {
		name         string
		length, x, z float64
	}{
		{"low x end", 20, 5, 10},
		{"high x end", 20, 15, 10},
		{"low z end", 40, 20, 4},
		{"high z end", 40, 20, 16},
	} {
		t.Run("bore reaches the chamfer at the "+tc.name, func(t *testing.T) {
			t.Parallel()
			body, bp, edges := allEdgeChamferBore(t, tc.length, tc.x, tc.z)
			_, matched, err := breprecord.TryAllEdgeChamfer(t.Context(), envOf(body.Doc, body.Doc.NextProducer),
				body, bp, edges, 3, 0, false)
			require.True(t, matched)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, "the bore reaches")
		})
	}
}
