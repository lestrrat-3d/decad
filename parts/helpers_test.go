package parts_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/parts"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	namePlate = "plate"
	namePin   = "pin"
	nameBase  = "base"
	nameStrap = "strap"
)

// block extrudes the rectangle x0..x1, y0..y1 from height z0 by height.
func block(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// cylinder extrudes a circle about the vertical line through (cx, 10).
func cylinder(t *testing.T, doc *decad.Document, cx, radius, z0, height float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, 10)
	s.Fix(c)
	s.CreateCircle(c, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// boredPlate is a 20 x 20 x 10 plate with a vertical bore of radius 3.2 at
// (10, 10).
func boredPlate(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	plate := block(t, doc, 0, 0, 20, 20, 0, 10)
	tool := cylinder(t, doc, 10, 3.2, -1, 12)
	out, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return out
}

func mustAdd(t *testing.T, s *parts.Set, ps ...parts.Part) {
	t.Helper()
	for _, p := range ps {
		require.NoError(t, s.Add(p), p.Name)
	}
}

func mustJoin(t *testing.T, s *parts.Set, js ...parts.Joint) {
	t.Helper()
	for _, j := range js {
		require.NoError(t, s.Join(j), j.Name)
	}
}

func partByName(t *testing.T, ex *parts.Exploded, name string) parts.ExplodedPart {
	t.Helper()
	for _, p := range ex.Parts {
		if p.Name == name {
			return p
		}
	}
	require.Failf(t, "missing part", "no part %q in the exploded view", name)
	return parts.ExplodedPart{}
}

// facetedCopy returns a solid of doc made from b's mesh: every face reads as
// Faceted.
func facetedCopy(t *testing.T, doc *decad.Document, b *decad.Body) *decad.Body {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.01))
	require.NoError(t, err)
	mb, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
	require.NoError(t, err)
	out, err := decad.SolidFromMesh(t.Context(), doc, mb)
	require.NoError(t, err)
	return out
}

func unitsDegrees(d float64) units.Value { return units.Degrees(d) }

func translationOf(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	tr, err := r3.Translation(v)
	require.NoError(t, err)
	return tr
}

func frameXY(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	return f
}
