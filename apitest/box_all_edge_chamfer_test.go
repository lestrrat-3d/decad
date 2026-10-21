package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/modify-general-design.md §4.3b's public-surface
// tests for the bare box: Chamfer of all twelve straight edges of an exact
// axis-aligned box builds the same eighteen-plane polyhedron as the
// cross-drilled box, without the bore.

const (
	bareLx, bareLy, bareLz = 40.0, 30.0, 10.0
)

func bareBox(tb testing.TB) (*decad.Document, *decad.Body) {
	tb.Helper()
	doc := decad.New()
	return doc, decadtest.NewBlock(tb, doc, 0, 0, bareLx, bareLy, units.Millimeters(bareLz))
}

func allEdges() *decad.EdgeQuery { return decad.Edges(decad.Convex()).Exactly(12) }

func TestChamferAllEdgesOfBareBox(t *testing.T) {
	t.Parallel()
	doc, box := bareBox(t)
	out, err := decad.Chamfer(t.Context(), box, allEdges(), units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	// L*L*L - 2d^2 (Lx+Ly+Lz) + 6 d^3: twelve wedges, minus the overlaps the
	// wedges share at the eight corners.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(bareLx*bareLy*bareLz-2*(bareLx+bareLy+bareLz)+6), decadtest.Exactly())
	decadtest.MeasuresBounds(t, out, r3.Vec{}, r3.Vec{X: bareLx, Y: bareLy, Z: bareLz}, decadtest.Exactly())
	decadtest.MeasuresCentroid(t, out, r3.Vec{X: bareLx / 2, Y: bareLy / 2, Z: bareLz / 2}, decadtest.Exactly())
	// Six inset faces and twelve strips of width d*sqrt(2).
	inset := 2 * ((bareLx-2)*(bareLy-2) + (bareLy-2)*(bareLz-2) + (bareLx-2)*(bareLz-2))
	strips := 4 * math.Sqrt2 * ((bareLx + bareLy + bareLz) - 3*1.5)
	decadtest.MeasuresArea(t, out, units.SquareMillimeters(inset+strips))
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 18})
}

func TestChamferAllEdgesOfBareBoxMeshes(t *testing.T) {
	t.Parallel()
	_, box := bareBox(t)
	out, err := decad.Chamfer(t.Context(), box, allEdges(), units.Millimeters(2))
	require.NoError(t, err)
	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	want := bareLx*bareLy*bareLz - 2*4*(bareLx+bareLy+bareLz) + 6*8
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(want), decadtest.Exactly())
}

func TestChamferAllEdgesOfBareBoxStaysPlaced(t *testing.T) {
	t.Parallel()
	doc, box := bareBox(t)
	out, err := decad.Chamfer(t.Context(), box, allEdges(), units.Millimeters(1))
	require.NoError(t, err)
	// A quarter turn about x takes y to z and z to -y. Its matrix is exact, as the
	// route needs, and it is not a turn that keeps a bore along y.
	turn, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)}, r3.Vec{})
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), out, turn)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{moved}, doc.Bodies())
	decadtest.MeasuresVolume(t, moved, units.CubicMillimeters(bareLx*bareLy*bareLz-2*(bareLx+bareLy+bareLz)+6), decadtest.Exactly())
	decadtest.MeasuresBounds(t, moved, r3.Vec{Y: -bareLz}, r3.Vec{X: bareLx, Y: 0, Z: bareLy}, decadtest.Exactly())
	copied, err := decad.Copy(t.Context(), moved)
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, copied, units.CubicMillimeters(bareLx*bareLy*bareLz-2*(bareLx+bareLy+bareLz)+6), decadtest.Exactly())
}

func TestChamferAllEdgesOfBareBoxIsSound(t *testing.T) {
	t.Parallel()
	doc, box := bareBox(t)
	out, err := decad.Chamfer(t.Context(), box, allEdges(), units.Millimeters(1))
	require.NoError(t, err)
	report := decadtest.Verify(t, doc)
	// The area is a sum of square roots of two, so the report holds it as
	// Approximate and has no tolerance reference to judge it by.
	decadtest.HasOnlyDiagnostics(t, report, decad.DiagToleranceReferenceUnavailable)
	body := decadtest.FindBodyReport(t, report, out)
	require.NotEqual(t, decad.Unsound, body.Status)
	require.NotEqual(t, decad.Violating, body.Status)
}

func TestChamferAllEdgesOfBareBoxRefusals(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		d    units.Value
		text string
	}{
		{"setback not exactly representable", units.Millimeters(0.1), "not exactly representable"},
		{"setback reaches half the shortest side", units.Millimeters(5), "mixes lateral edges"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, box := bareBox(t)
			_, err := decad.Chamfer(t.Context(), box, allEdges(), tc.d)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.text)
			require.Equal(t, []*decad.Body{box}, doc.Bodies(), "a refused chamfer leaves the box live")
		})
	}
}

// Only the complete set of twelve edges takes this route. A part of it still
// meets the ordinary refusal.
func TestChamferPartOfBareBoxEdgesStaysRefused(t *testing.T) {
	t.Parallel()
	_, box := bareBox(t)
	eight := decad.Edges(decad.ParallelTo(r3.Vec{X: 1})).Or(decad.ParallelTo(r3.Vec{Z: 1})).Exactly(8)
	_, err := decad.Chamfer(t.Context(), box, eight, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "mixes lateral edges")
}

// A bored box keeps the earlier route, and a drilled box that is not that box
// does not take this one.
func TestChamferAllEdgesOfCrossDrilledBarKeepsItsBore(t *testing.T) {
	t.Parallel()
	_, bar := crossDrilledBar(t)
	out, err := decad.Chamfer(t.Context(), bar, allEdges(), units.Millimeters(1))
	require.NoError(t, err)
	decadtest.HasSurfaceKinds(t, out, map[decad.SurfaceKind]int{decad.KindPlane: 18, decad.KindCylinder: 1})
}
