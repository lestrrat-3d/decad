package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-modify-reach-design.md §12 for increment
// M4: Fillet and Chamfer of an open rounded outline of a mesh Boolean result.
// The open-corner box is 40x20x20 with its corner at the origin rounded at
// r = 3 and a Ø8 bore through y. Its top and bottom outlines are each a line of
// 37 mm, a quarter arc and a line of 17 mm, ending on the sharp box corners
// (40,0,z) and (0,20,z).

const (
	openBoxR     = 3.0
	openBoxLines = 37 + 17
)

// openChainBox returns the box and the seed edge: the corner arc at height z.
func openChainBox(t *testing.T, z float64) (*decad.Body, decad.EdgeSelector) {
	t.Helper()
	drilled := openCornerBox(t, newRetentionScene())
	return drilled, arcSeedAt(t, drilled, z)
}

// openCornerBox builds the open-corner box in sc.
func openCornerBox(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	drilled, err := decad.Cut(t.Context(), roundedCornerBox(t, sc), sc.tool(t, 20))
	require.NoError(t, err)
	return drilled
}

// roundedCornerBox is the 40x20x20 box with only the corner at the origin
// rounded.
func roundedCornerBox(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	corner, err := decad.Fillet(t.Context(), box,
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.Vec{})).Exactly(1),
		units.Millimeters(openBoxR))
	require.NoError(t, err)
	return corner
}

// arcSeedAt selects the one Arc3 edge whose centre lies at height z.
func arcSeedAt(t *testing.T, b *decad.Body, z float64) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.Z-z) < 1e-9 {
			return decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
		}
	}
	require.Fail(t, "no arc at that height")
	return nil
}

// openFilletRemoved is the volume a fillet of radius r removes along the open
// outline: the corner area times the path length at the corner's centroid.
func openFilletRemoved(r float64) float64 {
	af := r * r * (1 - math.Pi/4)
	xbar := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	return af * (openBoxLines + math.Pi/2*(openBoxR-xbar))
}

// openChamferRemoved is the same for a chamfer of setback tp on the base and
// tw on the walls.
func openChamferRemoved(tp, tw float64) float64 {
	return tp * tw / 2 * (openBoxLines + math.Pi/2*(openBoxR-tp/3))
}

// rectBox extrudes the rectangle (x0,y0)-(x1,y1) from z = 0 up by h.
func rectBox(t *testing.T, w *sketch.World, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// placed moves b by (0, 0, dz) so a Cut with it stays on the mesh path.
func placed(t *testing.T, b *decad.Body, dz float64) *decad.Body {
	t.Helper()
	move, err := r3.Translation(r3.Vec{Z: dz})
	require.NoError(t, err)
	out, err := decad.Transform(t.Context(), b, move)
	require.NoError(t, err)
	return out
}

// roundedPocket is a 40x40x10 plate with a placed, rounded tool cut 5 mm deep.
// closed leaves the tool inside the plate, so the pocket floor's outline is a
// closed concave chain; otherwise the tool runs out through the x = 0 wall and
// the outline is open.
func roundedPocket(t *testing.T, closed bool) *decad.Body {
	t.Helper()
	w, doc := sketch.NewWorld(), decad.New()
	plate := rectBox(t, w, doc, 0, 0, 40, 40, 10)
	x0 := 10.0
	if !closed {
		x0 = -10
	}
	tool := rectBox(t, w, doc, x0, 10, 30, 30, 20)
	vertical := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)))
	if closed {
		vertical = vertical.Exactly(4)
	} else {
		up := decad.ParallelTo(r3.NewVec(0, 0, 1))
		vertical = decad.Edges(up, decad.EndpointAt(r3.NewVec(30, 10, 0))).
			Or(up, decad.EndpointAt(r3.NewVec(30, 30, 0))).Exactly(2)
	}
	tool, err := decad.Fillet(t.Context(), tool, vertical, units.Millimeters(3))
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), plate, placed(t, tool, 5))
	require.NoError(t, err)
	return out
}

func TestFacetedRetainedBlendOpenChain(t *testing.T) {
	t.Parallel()

	t.Run("seed expands to the open outline and fillets it", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 20)
		before, err := drilled.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), drilled, seed, units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, openFilletRemoved(r))
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 2, cylinders)
		require.Equal(t, 1, tori)
		require.InDelta(t, openBoxR-r, majors[0], 1e-12)
		requireVerifiedMesh(t, out)
	})

	t.Run("the surface area changes by the exact bands, blend face and end corners", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 20)
		before, err := drilled.Area()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), drilled, seed, units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Area()
		require.NoError(t, err)
		// The lines add the quarter cylinder's face and take a band of r from
		// the base and from the wall. The arc does the same on the torus, the
		// annulus about the corner axis and the corner cylinder. Each end
		// face loses the corner of the blend's section.
		span := math.Pi / 2
		lines := openBoxLines * (r*math.Pi/2 - 2*r)
		arc := span * (r*((openBoxR-r)*math.Pi/2+r) - (openBoxR*openBoxR-(openBoxR-r)*(openBoxR-r))/2 - openBoxR*r)
		corners := 2 * r * r * (1 - math.Pi/4)
		want := lines + arc - corners
		got := after.Value.Mag() - before.Value.Mag()
		growth := after.Bound.Mag() - before.Bound.Mag()
		t.Logf("area change %.9f, want %.9f, bound growth %.3g", got, want, growth)
		require.InDelta(t, want, got, growth)
		require.Less(t, growth, 1.0)
	})

	t.Run("junction and end edges are quarter circles of the blend radius", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 20)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), drilled, seed, units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		// Two junctions between a line blend and the corner torus, and one end
		// curve of each line blend on its end face.
		want := map[r3.Vec]r3.Vec{
			r3.NewVec(3, 1, 19):  r3.NewVec(1, 0, 0),
			r3.NewVec(1, 3, 19):  r3.NewVec(0, 1, 0),
			r3.NewVec(40, 1, 19): r3.NewVec(1, 0, 0),
			r3.NewVec(1, 20, 19): r3.NewVec(0, 1, 0),
		}
		found := 0
		for _, e := range out.Edges() {
			arc, ok := e.Curve().(decad.Arc3)
			if !ok || math.Abs(arc.Radius.Base()-r) > 1e-12 {
				continue
			}
			found++
			var centre, axis r3.Vec
			for c, a := range want {
				if arc.Center.Sub(c).Len() < 1e-9 {
					centre, axis = c, a
				}
			}
			require.NotZero(t, centre, "an unexpected arc about %v", arc.Center)
			require.InDelta(t, 1, math.Abs(arc.Axis.Dot(axis)), 1e-12)
			// The arc runs counter-clockwise about its axis from Start to End.
			from := e.Start().Position().Value.Sub(arc.Center)
			to := e.End().Position().Value.Sub(arc.Center)
			require.InDelta(t, r, from.Len(), 1e-9)
			require.InDelta(t, r, to.Len(), 1e-9)
			require.Positive(t, from.Cross(to).Dot(arc.Axis))
			require.InDelta(t, math.Pi/2, math.Acos(from.Dot(to)/(r*r)), 1e-9, "a quarter turn")
		}
		require.Equal(t, 4, found)
	})

	t.Run("the bottom outline runs against its face loop", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 0)
		before, err := drilled.Volume()
		require.NoError(t, err)
		out, err := decad.Fillet(t.Context(), drilled, seed, units.Millimeters(1), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, openFilletRemoved(1))
		requireVerifiedMesh(t, out)
	})

	t.Run("naming the three edges builds without the option", func(t *testing.T) {
		t.Parallel()
		drilled, _ := openChainBox(t, 20)
		before, err := drilled.Volume()
		require.NoError(t, err)
		outline := decad.Edges(decad.EndpointAt(r3.NewVec(40, 0, 20)), decad.EndpointAt(r3.NewVec(3, 0, 20))).
			Or(decad.EndpointAt(r3.NewVec(3, 0, 20)), decad.EndpointAt(r3.NewVec(0, 3, 20))).
			Or(decad.EndpointAt(r3.NewVec(0, 3, 20)), decad.EndpointAt(r3.NewVec(0, 20, 20)))
		out, err := decad.Fillet(t.Context(), drilled, outline.Exactly(3), units.Millimeters(1))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, openFilletRemoved(1))
	})

	t.Run("a chamfer of the open outline, equal setbacks", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 20)
		before, err := drilled.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), drilled, seed, units.Millimeters(1), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, openChamferRemoved(1, 1))
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer with the base as reference takes both distances", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 20)
		before, err := drilled.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), drilled, seed, units.Millimeters(2), decad.WithTangentChain(),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1)))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, openChamferRemoved(2, 1))
	})

	t.Run("the arc alone without the option names the option", func(t *testing.T) {
		t.Parallel()
		drilled, seed := openChainBox(t, 20)
		_, err := decad.Fillet(t.Context(), drilled, seed, units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "WithTangentChain")
	})

	t.Run("a chain that ends on an arc edge is refused", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		corner := roundedCornerBox(t, sc)
		// A placed block takes the part of the corner arc below x = 1.5 away, so
		// the arc ends on the new face x = 1.5.
		block := placed(t, rectBox(t, sc.w, sc.doc, -5, -5, 1.5, 25, 30), -5)
		trimmed, err := decad.Cut(t.Context(), corner, block)
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), trimmed, arcSeedAt(t, trimmed, 20), units.Millimeters(1),
			decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "only a straight edge end is built")
	})

	t.Run("a concave closed chain is refused", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, true)
		seed := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(r3.NewVec(13, 10, 5))).Exactly(1)
		_, err := decad.Fillet(t.Context(), pocket, seed, units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "concave tangent chain")
	})

	t.Run("a concave open chain is refused", func(t *testing.T) {
		t.Parallel()
		pocket := roundedPocket(t, false)
		seed := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(r3.NewVec(27, 10, 5))).Exactly(1)
		_, err := decad.Fillet(t.Context(), pocket, seed, units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "concave tangent chain")
	})

	t.Run("a junction the tags nearly meet at is refused with its text", func(t *testing.T) {
		t.Parallel()
		// The fillet radius 2.7 is not a float the corner cylinder's origin
		// reaches exactly, so the line and arc tags miss tangency by a rounding.
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		r := s.CreateRectangle(0, 0, 40.1, 20)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)
		rounded, err := decad.Fillet(t.Context(), box,
			decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(2.7))
		require.NoError(t, err)
		drilled, err := decad.Cut(t.Context(), rounded, sc.tool(t, 20))
		require.NoError(t, err)
		var outline *decad.EdgeQuery
		named := 0
		for _, e := range drilled.Edges() {
			p, q := e.Start().Position().Value, e.End().Position().Value
			if math.Abs(p.Z-20) > 1e-9 || math.Abs(q.Z-20) > 1e-9 {
				continue
			}
			named++
			if outline == nil {
				outline = decad.Edges(decad.EndpointAt(p), decad.EndpointAt(q))
				continue
			}
			outline = outline.Or(decad.EndpointAt(p), decad.EndpointAt(q))
		}
		require.Equal(t, 8, named)
		_, err = decad.Fillet(t.Context(), drilled, outline.Exactly(named), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "is nearly tangent and the tags cannot decide it")
		require.ErrorContains(t, err, "2.7,0,20")
	})
}
