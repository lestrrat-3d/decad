package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The public Sketch, Extrude, Copy and Union producers must agree on the
// boundary and volume of a small radial prism pattern.
func TestPrismPatternUnionOfCopiedExtrudes(t *testing.T) {
	doc, blank, tooth, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	toothVolume, err := tooth.Volume()
	require.NoError(t, err)
	result, err := Union(t.Context(), blank, tooth, rotated)
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	volume, err := result.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base(), blankVolume.Value.Base())
	require.Less(t, volume.Value.Base(), blankVolume.Value.Base()+2*toothVolume.Value.Base())
	mesh, err := result.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestPrismPatternUnionRefusesOutwardRootWithoutRetiringSources(t *testing.T) {
	doc, blank, tooth, rotated := prismPatternTabBodies(t, 2.999)
	_, err := Union(t.Context(), blank, tooth, rotated)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "root is not strictly inside")
	require.Len(t, doc.Bodies(), 3)
}

// A root arc that lies on the blank's rim is a tangent contact the prism-pattern
// route refuses, and the union of the sections joins it.
func TestPrismPatternUnionOfTangentRootJoinsBySectionUnion(t *testing.T) {
	_, blank, tooth, rotated := prismPatternTabBodies(t, 3)
	var parts float64
	var slack float64
	for _, b := range []*Body{blank, tooth, rotated} {
		v, err := b.Volume()
		require.NoError(t, err)
		parts += v.Value.Base()
		slack += v.Bound.Base()
	}
	joined, err := Union(t.Context(), blank, tooth, rotated)
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	got, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(got.Value.Base()-parts), slack+got.Bound.Base())
}

func TestPrismPatternUnionSelectsProvenAxialChains(t *testing.T) {
	_, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	require.Len(t, joined.Edges(), 36)
	selected, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	require.Len(t, selected, 12)
	require.Equal(t, 24, len(joined.Edges())-len(selected))
	for _, edge := range selected {
		_, faceted := edge.n.Curve.(surfacegeom.FacetedCurve)
		require.True(t, faceted)
		start, end := edge.Start().Position().Value, edge.End().Position().Value
		require.Equal(t, start.X, end.X)
		require.Equal(t, start.Y, end.Y)
		require.NotEqual(t, start.Z, end.Z)
	}
}

func prismPatternTabBodies(t *testing.T, radius float64) (*Document, *Body, *Body, *Body) {
	t.Helper()
	ctx := t.Context()
	world := sketch.NewWorld()
	s, profile := loftCylinderTabProfile(t, ctx, world, world.XY(), 0)
	doc := New()
	tooth, err := Extrude(ctx, doc, s, profile, Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	blankSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := blankSketch.CreatePoint(0, 0)
	blankSketch.Fix(center)
	blankSketch.CreateCircle(center, radius)
	_, err = blankSketch.Solve(ctx)
	require.NoError(t, err)
	require.Len(t, blankSketch.Profiles(), 1)
	blank, err := Extrude(ctx, doc, blankSketch, blankSketch.Profiles()[0],
		Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	rotated, err := Copy(ctx, tooth, WithCopyPlacement(turn))
	require.NoError(t, err)
	return doc, blank, tooth, rotated
}
