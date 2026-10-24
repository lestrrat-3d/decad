package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStraightSweepChamferSelectedWallsAndVerifyAll(t *testing.T) {
	t.Parallel()
	doc, source := straightSweepSquare(t)
	vertical := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex()).Exactly(4)
	beveled, err := decad.Chamfer(t.Context(), source, vertical, units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{beveled}, doc.Bodies())
	decadtest.MeasuresVolume(t, beveled, units.CubicMillimeters(3980), decadtest.Exactly())
	decadtest.MeasuresBounds(t, beveled, r3.Vec{}, r3.NewVec(20, 20, 10), decadtest.Exactly())
	require.Len(t, beveled.Faces(), 10)
	mesh, err := beveled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestStraightSweepChamferSingleCapEdgeAndVerifyAll(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		shift r3.Vec
	}{
		{name: "source"},
		{name: "translated", shift: r3.NewVec(5, -3, 7)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := straightSweepSquare(t)
			if tc.shift != (r3.Vec{}) {
				motion, err := r3.Translation(tc.shift)
				require.NoError(t, err)
				source, err = decad.Copy(t.Context(), source, decad.WithCopyPlacement(motion))
				require.NoError(t, err)
			}
			corner := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
				decad.EndpointAt(r3.NewVec(0, 0, 10).Add(tc.shift))).Exactly(1)
			edges, err := corner.SelectEdges(source)
			require.NoError(t, err)
			require.Len(t, edges, 1)
			beveled, err := decad.Chamfer(t.Context(), source, corner, units.Millimeters(1))
			require.NoError(t, err)
			decadtest.MeasuresVolume(t, beveled, units.CubicMillimeters(3990), decadtest.Exactly())
			require.Len(t, beveled.Faces(), 7)
			mesh, err := beveled.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			require.Contains(t, doc.Bodies(), beveled)
			require.NotContains(t, doc.Bodies(), source)
		})
	}
}

func TestStraightSweepChamferOneEdgeAndPlacedReplay(t *testing.T) {
	t.Parallel()
	doc, source := straightSweepSquare(t)
	shift, err := r3.Translation(r3.NewVec(5, -3, 7))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), source, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.NewVec(5, -3, 7))).Exactly(1)
	beveled, err := decad.Chamfer(t.Context(), placed, corner, units.Millimeters(1))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, beveled, units.CubicMillimeters(3995), decadtest.Exactly())
	decadtest.MeasuresBounds(t, beveled, r3.NewVec(5, -3, 7),
		r3.NewVec(25, 17, 17), decadtest.Exactly())
	mesh, err := beveled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	replayed, err := decad.Copy(t.Context(), beveled, decad.WithCopyPlacement(shift))
	require.NoError(t, err)
	decadtest.MeasuresVolume(t, replayed, units.CubicMillimeters(3995), decadtest.Exactly())
	replayedMesh, err := replayed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, replayedMesh.BoundaryVerified())
	require.True(t, replayedMesh.VolumeVerified())
	require.Equal(t, []*decad.Body{source, beveled, replayed}, doc.Bodies())
}

func TestStraightSweepChamferRefusesOtherEdges(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		sel  *decad.EdgeQuery
	}{
		{"partial cap edges", decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).AtLeast(1)},
		{"mixed cap and lateral edges", decad.Edges().AtLeast(1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := straightSweepSquare(t)
			out, err := decad.Chamfer(t.Context(), source, tc.sel, units.Millimeters(1))
			require.Nil(t, out)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}

func TestStraightSweepChamferRefusesAsymmetricReference(t *testing.T) {
	t.Parallel()
	doc, source := straightSweepSquare(t)
	corner := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)),
		decad.EndpointAt(r3.Vec{})).Exactly(1)
	wall := decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))).Exactly(1)
	out, err := decad.Chamfer(t.Context(), source, corner, units.Millimeters(1),
		decad.WithAsymmetricChamfer(wall, units.Millimeters(2)))
	require.Nil(t, out)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{source}, doc.Bodies())
}

func TestStraightSweepChamferRefusesOtherSweeps(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		build func(*testing.T) (*decad.Document, *decad.Body)
	}{
		{"one arc path", func(t *testing.T) (*decad.Document, *decad.Body) {
			s, profile := plateSketch(t)
			path, err := decad.NewPath(r3.NewVec(0, 5, 0), decad.ArcThrough{
				Through: r3.NewVec(0, 3, 4), End: r3.NewVec(0, 0, 5)})
			require.NoError(t, err)
			doc := decad.New()
			body, err := decad.Sweep(t.Context(), doc, s, profile, path)
			require.NoError(t, err)
			return doc, body
		}},
		{"composite", func(t *testing.T) (*decad.Document, *decad.Body) {
			s, profile := orthogonalSweepProfile(t)
			path, err := decad.NewPath(r3.Vec{},
				decad.ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
				decad.LineTo{End: r3.NewVec(8, 0, 5)})
			require.NoError(t, err)
			doc := decad.New()
			body, err := decad.Sweep(t.Context(), doc, s, profile, path)
			require.NoError(t, err)
			return doc, body
		}},
		{"mitred", func(t *testing.T) (*decad.Document, *decad.Body) {
			s, profile := squareSweepSketch(t, 2)
			path := mustPath(t, r3.Vec{}, r3.NewVec(0, 0, 10), r3.NewVec(6, 0, 10))
			doc := decad.New()
			body, err := decad.Sweep(t.Context(), doc, s, profile, path, decad.WithMitredJoins())
			require.NoError(t, err)
			return doc, body
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc, source := tc.build(t)
			out, err := decad.Chamfer(t.Context(), source, decad.Edges().AtLeast(1), units.Millimeters(1))
			require.Nil(t, out)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Equal(t, []*decad.Body{source}, doc.Bodies())
		})
	}
}
