package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// benchBoxBody and benchRodBody are boxBody/diskBody's own fixtures, built
// against *testing.B rather than *testing.T so the benchmarks below can build
// fresh operands inside the timed-out setup of each b.Loop() iteration.

// benchBoxBody extrudes an axis-aligned rectangle into doc.
func benchBoxBody(b *testing.B, doc *decad.Document, x0, y0, x1, y1, h float64) *decad.Body {
	b.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	rect := s.CreateRectangle(x0, y0, x1, y1)
	s.Fix(rect.A)
	_, err = s.Solve(b.Context())
	require.NoError(b, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(b, err)
	return body
}

// benchRodBody extrudes a radius-r circle at (cx, cy) to a 20 mm prism and
// translates it so it spans z ∈ [−6, 14] — tall enough to pierce every plate
// benchmarked here, and off the z = 0 plane so the pair is never two
// co-directional coplanar prisms (which the analytic prism boolean, not
// meshBoolean, would answer).
func benchRodBody(b *testing.B, doc *decad.Document, cx, cy, r float64) *decad.Body {
	b.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(b, err)
	center := s.CreatePoint(cx, cy)
	s.Fix(center)
	s.CreateCircle(center, r)
	_, err = s.Solve(b.Context())
	require.NoError(b, err)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(b, err)
	tr, err := r3.Translation(r3.Vec{X: 0, Y: 0, Z: -6})
	require.NoError(b, err)
	moved, err := body.Placed(tr)
	require.NoError(b, err)
	return moved
}

// BenchmarkBooleanUnionRodThroughPlate measures a mesh-path union where only
// one operand's wall is chorded: a 20×20×8 mm plate, held exactly, against a
// radius-6 rod. Every plate-face/rod-wall pair carries positive slack, so it
// runs the proximity gate (facesNearMiss) before the mesh pass (meshBoolean)
// — the two call sites contactMemo (boolean_mesh.go) now shares a facet
// pair's classification across. This fixture is dominated by other boolean
// work, so it is a no-regression guard rather than the memo's headline case.
func BenchmarkBooleanUnionRodThroughPlate(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		plate := benchBoxBody(b, doc, 0, 0, 20, 20, 8)
		rod := benchRodBody(b, doc, 12, 10, 6)
		b.StartTimer()
		got, err := decad.Union(plate, rod)
		require.NoError(b, err)
		require.Len(b, got.Lumps(), 1)
	}
}

// BenchmarkBooleanUnionCrossedRods is the memo's headline case: BOTH operands
// carry a chorded cylinder wall, so every wall/wall facet pair the proximity
// gate examines is one the mesh pass classifies again.
func BenchmarkBooleanUnionCrossedRods(b *testing.B) {
	for b.Loop() {
		b.StopTimer()
		doc := decad.New()
		a := benchRodBody(b, doc, 0, 0, 4)
		c := benchRodBody(b, doc, 0, 0, 3)
		rot, err := r3.Rotation(r3.Vec{X: 1, Y: 0, Z: 0}, units.Degrees(90))
		require.NoError(b, err)
		tr, err := r3.Translation(r3.Vec{X: 0, Y: -4, Z: 2})
		require.NoError(b, err)
		xf, err := rot.Then(tr)
		require.NoError(b, err)
		c, err = c.Placed(xf)
		require.NoError(b, err)
		b.StartTimer()
		got, err := decad.Union(a, c)
		require.NoError(b, err)
		require.NotNil(b, got)
	}
}
