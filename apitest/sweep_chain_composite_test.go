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

func cardinalLineArcChain(t *testing.T) (*sketch.Sketch, *sketch.Chain, *decad.Path) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(4, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	path, err := decad.NewPath(r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 20)},
		decad.ArcThrough{Through: r3.NewVec(5, 0, 25), End: r3.NewVec(10, 0, 20)})
	require.NoError(t, err)
	return s, s.Chains()[0], path
}

func TestSweepChainCardinalLineArcSheet(t *testing.T) {
	t.Parallel()
	s, chain, path := cardinalLineArcChain(t)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.Len(t, sheet.Faces(), 2)
	_, err = decad.Edges(decad.Free()).Exactly(6).SelectEdges(sheet)
	require.NoError(t, err)
	area, err := sheet.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(80+12*math.Pi)), area.Bound.Base()+1e-12)
	box, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, 0, 0), box.Min)
	require.Equal(t, r3.NewVec(10, 0, 25), box.Max)
	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	require.NotEmpty(t, mesh.Triangles())
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	for _, source := range mesh.SourceFaces() {
		require.Contains(t, sheet.Faces(), source)
	}
	before := len(doc.Bodies())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	bodyReport, err := report.ForBody(sheet)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityUndecided, bodyReport.Validity.Outcome)
	require.Equal(t, before, len(doc.Bodies()))
	motion, err := r3.Translation(r3.NewVec(1, 0, 0))
	require.NoError(t, err)
	_, err = decad.Transform(t.Context(), sheet, motion)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, len(doc.Bodies()))
}
