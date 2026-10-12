package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestLoftCylinderUnionCentroidDiameter builds one public, fitted-side tab
// against a circular blank. The structural result's mass-property proof
// needs an upper diameter that covers both its held mesh and true boundary.
func TestLoftCylinderUnionCentroidDiameter(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	_, joined := loftCylinderTabJoin(t)
	payload, ok := joined.payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, payload.loftCylinderJoin)
	require.Positive(t, payload.volSymDiff)
	// The source tab reaches radius 4 and the blank reaches radius 3 on the
	// opposite side; the certified pair diameter must exceed 7 mm.
	require.Greater(t, payload.dPair, 7.0)
	centroid, err := joined.Centroid()
	require.NoError(t, err)
	require.Greater(t, centroid.Bound.Base(), 0.1)
	mesh, err := joined.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	move, err := r3.Translation(r3.NewVec(1e6, -2e6, 3e6))
	require.NoError(t, err)
	placed, err := joined.Placed(ctx, move)
	require.NoError(t, err)
	placedPayload, ok := placed.payload.(facetedPayload)
	require.True(t, ok)
	// Placement rounds the held vertices and adds their displacement to the
	// boundary bound; the centroid's diameter must cover that moved source.
	require.GreaterOrEqual(t, placedPayload.dPair,
		facetedSourceDiameterUpper(placedPayload.verts, placedPayload.meshBound))
	placedCentroid, err := placed.Centroid()
	require.NoError(t, err)
	require.Greater(t, placedCentroid.Bound.Base(), 0.1)
}

func loftCylinderTabJoin(t *testing.T) (*Document, *Body) {
	t.Helper()
	ctx := t.Context()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	top, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s0, p0 := loftCylinderTabProfile(t, ctx, world, world.XY(), 0)
	s1, p1 := loftCylinderTabProfile(t, ctx, world, top, 10*math.Pi/180)
	doc := New()
	tab, err := doc.Loft(ctx, s0, p0, s1, p1)
	require.NoError(t, err)

	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	blank, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	joined, err := Union(ctx, blank, tab)
	require.NoError(t, err)
	return doc, joined
}

func TestLoftCylinderBoreCentroidDiameter(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc, joined := loftCylinderTabJoin(t)
	w := sketch.NewWorld()
	base, err := w.CreateOffsetPlane(w.XY(), -1)
	require.NoError(t, err)
	s, err := w.CreateSketch(base)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 0.5)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	tool, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	bored, err := Cut(ctx, joined, tool)
	require.NoError(t, err)
	fp, ok := bored.payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, fp.loftCylinderBore)
	require.GreaterOrEqual(t, fp.dPair, facetedSourceDiameterUpper(fp.verts, fp.meshBound))
	centroid, err := bored.Centroid()
	require.NoError(t, err)
	require.Positive(t, centroid.Bound.Base())
	mesh, err := bored.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func loftCylinderTabProfile(t *testing.T, ctx context.Context, world *sketch.World,
	plane *sketch.Plane, rotation float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	point := func(radius, angle float64) *sketch.Point {
		p := s.CreatePoint(radius*math.Cos(angle+rotation), radius*math.Sin(angle+rotation))
		s.Fix(p)
		return p
	}
	rootRight, rootLeft := point(3, -0.14), point(3, 0.14)
	right := []*sketch.Point{point(3.35, -0.14), point(3.55, -0.137), point(3.75, -0.13), point(4, -0.12)}
	left := []*sketch.Point{point(3.35, 0.14), point(3.55, 0.137), point(3.75, 0.13), point(4, 0.12)}
	s.CreateLine(rootRight, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], rootLeft)
	s.CreateArc(center, rootRight, rootLeft)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.True(t, s.Profiles()[0].Valid)
	require.Len(t, s.Profiles()[0].Outer, 6)
	return s, s.Profiles()[0]
}
