package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepArcAroundHalfTurn(t *testing.T) {
	s, profile := orthogonalSweepProfile(t)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.ArcAround{Center: r3.NewVec(5, 0, 5), Axis: r3.NewVec(0, 2, 0), End: r3.NewVec(10, 0, 5)},
		decad.LineTo{End: r3.NewVec(10, 0, 0)})
	require.NoError(t, err)
	body, err := decad.Sweep(t.Context(), decad.New(), s, profile, path)
	require.NoError(t, err)
	volume, err := body.Volume()
	require.NoError(t, err)
	want := 4 * (10 + 5*math.Pi)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-10)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := map[*decad.Face]struct{}{}
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}
	for _, x := range []float64{0, 10} {
		for _, dx := range []float64{-1, 1} {
			for _, y := range []float64{-1, 1} {
				point := r3.NewVec(x+dx, y, 5)
				matches := 0
				for _, vertex := range mesh.Vertices() {
					if vertex.Sub(point).Len() < 1e-9 {
						matches++
					}
				}
				require.Equal(t, 1, matches, "join point %v", point)
			}
		}
	}
}

func TestArcAroundInputAndOwnership(t *testing.T) {
	arc := decad.ArcAround{Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 2), End: r3.NewVec(0, 5, 0)}
	path, err := decad.NewPath(r3.NewVec(5, 0, 0), &arc)
	require.NoError(t, err)
	arc.End = r3.Vec{}
	require.Equal(t, decad.ArcAround{Center: r3.Vec{}, Axis: r3.NewVec(0, 0, 2), End: r3.NewVec(0, 5, 0)}, path.Segments()[0])
	var nilArc *decad.ArcAround
	_, err = decad.NewPath(r3.NewVec(5, 0, 0), nilArc)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	for _, tc := range []struct {
		name string
		arc  decad.ArcAround
		want error
	}{
		{"zero axis", decad.ArcAround{End: r3.NewVec(0, 5, 0)}, decad.ErrDegenerate},
		{"unequal radius", decad.ArcAround{Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(0, 4, 0)}, decad.ErrDegenerate},
		{"one-bit radius difference", decad.ArcAround{Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(0, math.Nextafter(5, math.Inf(1)), 0)}, decad.ErrDegenerate},
		{"axis not perpendicular", decad.ArcAround{Axis: r3.NewVec(1, 0, 1), End: r3.NewVec(0, 5, 0)}, decad.ErrDegenerate},
		{"one-bit axis component", decad.ArcAround{Axis: r3.NewVec(math.SmallestNonzeroFloat64, 0, 1), End: r3.NewVec(0, 5, 0)}, decad.ErrDegenerate},
		{"same endpoint", decad.ArcAround{Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(5, 0, 0)}, decad.ErrDegenerate},
		{"zero radius", decad.ArcAround{Center: r3.NewVec(5, 0, 0), Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(5, 1, 0)}, decad.ErrDegenerate},
		{"nonfinite", decad.ArcAround{Axis: r3.NewVec(0, 0, math.Inf(1)), End: r3.NewVec(0, 5, 0)}, decad.ErrNotFinite},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := decad.NewPath(r3.NewVec(5, 0, 0), tc.arc)
			require.ErrorIs(t, err, tc.want)
		})
	}
	_, err = decad.NewPath(r3.NewVec(5, 0, 0), decad.ArcAround{Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(3, 4, 0)})
	require.NoError(t, err)
}

func TestSweepDisplacedArcThroughRefuses(t *testing.T) {
	s, profile := orthogonalSweepProfile(t)
	arc := decad.ArcThrough{Through: r3.NewVec(5, 0, math.Nextafter(10, math.Inf(1))), End: r3.NewVec(10, 0, 5)}
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)}, arc,
		decad.LineTo{End: r3.NewVec(10, 0, 0)})
	require.NoError(t, err)
	doc := decad.New()
	_, err = decad.Sweep(t.Context(), doc, s, profile, path)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "not tangent")
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.OperationHistory().Operations())
	require.Equal(t, arc, path.Segments()[1])
	around, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.ArcAround{Center: r3.NewVec(5, 0, 5), Axis: r3.NewVec(0, -1, 0), End: r3.NewVec(10, 0, 5)})
	require.NoError(t, err)
	_, err = decad.Sweep(t.Context(), doc, s, profile, around)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "not tangent")
	require.Empty(t, doc.Bodies())
	require.Empty(t, doc.OperationHistory().Operations())
}

func TestPathPatternArcAround(t *testing.T) {
	_, block := patternBlock(t)
	path, err := decad.NewPath(r3.NewVec(10, 0, 0), decad.ArcAround{
		Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(-10, 0, 0),
	})
	require.NoError(t, err)
	copies, err := decad.PathPatternCopies(t.Context(), block, path, 3)
	require.NoError(t, err)
	require.Len(t, copies, 2)
	requireBoundsNear(t, copies[0], r3.NewVec(-10, 10, 0), r3.NewVec(-6, 14, 2))
	requireBoundsNear(t, copies[1], r3.NewVec(-20, 0, 0), r3.NewVec(-16, 4, 2))
	spaced, err := decad.PathPatternSpaced(t.Context(), block, path, units.Millimeters(5*math.Pi))
	require.NoError(t, err)
	require.Len(t, spaced, 2)
	requireBoundsNear(t, spaced[0], r3.NewVec(-10, 10, 0), r3.NewVec(-6, 14, 2))
	reverse, err := decad.NewPath(r3.NewVec(10, 0, 0), decad.ArcAround{
		Axis: r3.NewVec(0, 0, -7), End: r3.NewVec(-10, 0, 0),
	})
	require.NoError(t, err)
	reversed, err := decad.PathPatternCopies(t.Context(), block, reverse, 3)
	require.NoError(t, err)
	requireBoundsNear(t, reversed[0], r3.NewVec(-10, -10, 0), r3.NewVec(-6, -6, 2))
	quarter, err := decad.NewPath(r3.NewVec(10, 0, 0), decad.ArcAround{
		Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(0, 10, 0),
	})
	require.NoError(t, err)
	follow, err := decad.PathPatternCopies(t.Context(), block, quarter, 2, decad.WithPathFollow())
	require.NoError(t, err)
	lo, hi := turnedBlockBounds(math.Pi/2, r3.Vec{})
	requireBoundsNear(t, follow[0], lo, hi)
}

func TestPathPatternArcAroundUnrepresentableSampler(t *testing.T) {
	doc, block := patternBlock(t)
	path, err := decad.NewPath(r3.NewVec(math.SmallestNonzeroFloat64, 0, 0), decad.ArcAround{
		Axis: r3.NewVec(0, 0, 1), End: r3.NewVec(-math.SmallestNonzeroFloat64, 0, 0),
	})
	require.NoError(t, err)
	_, err = decad.PathPatternCopies(t.Context(), block, path, 3)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, []*decad.Body{block}, doc.Bodies())
}

func TestSweepChainArcAroundShortcutRefuses(t *testing.T) {
	s, chain, _ := cardinalLineArcChain(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 20)},
		decad.ArcAround{Center: r3.NewVec(5, 0, 20), Axis: r3.NewVec(0, 1, 0), End: r3.NewVec(10, 0, 20)})
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	_, err = sheet.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{sheet}, doc.Bodies())
}

func TestSweepArcAroundClosedCircle(t *testing.T) {
	s, profile := closedCircleSection(t, 9, 11)
	quarter, err := decad.NewPath(r3.NewVec(10, 0, 0),
		decad.ArcAround{Axis: r3.NewVec(0, -1, 0), End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	partial, err := decad.Sweep(t.Context(), decad.New(), s, profile, quarter)
	require.NoError(t, err)
	partialVolume, err := partial.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(partialVolume.Value.Base()-20*math.Pi), partialVolume.Bound.Base()+1e-10)
	partialMesh, err := partial.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, partialMesh.BoundaryVerified())
	require.True(t, partialMesh.VolumeVerified())
	path, err := decad.NewPath(r3.NewVec(10, 0, 0),
		decad.ArcAround{Axis: r3.NewVec(0, -1, 0), End: r3.NewVec(-10, 0, 0)},
		decad.ArcAround{Axis: r3.NewVec(0, -1, 0), End: r3.NewVec(10, 0, 0)})
	require.NoError(t, err)
	body, err := decad.Sweep(t.Context(), decad.New(), s, profile, path)
	require.NoError(t, err)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-80*math.Pi), volume.Bound.Base()+1e-10)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
