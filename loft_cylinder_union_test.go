package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	payload, ok := joined.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, payload.LoftCylinderJoin)
	require.Positive(t, payload.VolSymDiff)
	// The source tab reaches radius 4 and the blank reaches radius 3 on the
	// opposite side; the certified pair diameter must exceed 7 mm.
	require.Greater(t, payload.DPair, 7.0)
	centroid, err := joined.Centroid()
	require.NoError(t, err)
	require.Greater(t, centroid.Bound.Base(), 0.1)
	mesh, err := joined.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	move, err := r3.Translation(r3.NewVec(1e6, -2e6, 3e6))
	require.NoError(t, err)
	placed, err := Transform(ctx, joined, move)
	require.NoError(t, err)
	placedPayload, ok := placed.n.Payload.(facetedPayload)
	require.True(t, ok)
	// Placement rounds the held vertices and adds their displacement to the
	// boundary bound; the centroid's diameter must cover that moved source.
	require.GreaterOrEqual(t, placedPayload.DPair,
		facetedrecord.SourceDiameterUpper(placedPayload.Verts, placedPayload.MeshBound))
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
	blank, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
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
	tool, err := Extrude(ctx, doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	bored, err := Cut(ctx, joined, tool)
	require.NoError(t, err)
	fp, ok := bored.n.Payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, fp.LoftCylinderBore)
	require.GreaterOrEqual(t, fp.DPair, facetedrecord.SourceDiameterUpper(fp.Verts, fp.MeshBound))
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

// loftCylinderTriangleMinRadiusReference is the rational minimum projected
// radius loftcylinder.TriangleMinRadius reads: the nearest point on each closed
// edge, or zero when the origin is inside.
func loftCylinderTriangleMinRadiusReference(verts []r3.Vec, tri [3]int) float64 {
	type point struct{ x, y *big.Rat }
	var p [3]point
	for i, index := range tri {
		p[i] = point{proofarith.FloatRat(verts[index].X), proofarith.FloatRat(verts[index].Y)}
	}
	best := new(big.Rat)
	for i := range 3 {
		a, b := p[i], p[(i+1)%3]
		dx, dy := new(big.Rat).Sub(b.x, a.x), new(big.Rat).Sub(b.y, a.y)
		den := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		t := new(big.Rat)
		if den.Sign() > 0 {
			num := new(big.Rat).Neg(new(big.Rat).Add(
				new(big.Rat).Mul(a.x, dx), new(big.Rat).Mul(a.y, dy)))
			t.Quo(num, den)
			if t.Sign() < 0 {
				t.SetInt64(0)
			} else if t.Cmp(big.NewRat(1, 1)) > 0 {
				t.SetInt64(1)
			}
		}
		qx := new(big.Rat).Add(a.x, new(big.Rat).Mul(t, dx))
		qy := new(big.Rat).Add(a.y, new(big.Rat).Mul(t, dy))
		q := new(big.Rat).Add(new(big.Rat).Mul(qx, qx), new(big.Rat).Mul(qy, qy))
		if i == 0 || q.Cmp(best) < 0 {
			best = q
		}
	}
	var signs [3]int
	for i := range 3 {
		a, b := p[i], p[(i+1)%3]
		cross := new(big.Rat).Sub(new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x))
		signs[i] = cross.Sign()
	}
	if (signs[0] > 0 && signs[1] > 0 && signs[2] > 0) ||
		(signs[0] < 0 && signs[1] < 0 && signs[2] < 0) {
		return 0
	}
	return proofbound.RatSqrtDown(best)
}

// TestLoftCylinderTriangleMinRadiusMatchesRational covers an origin inside,
// on an edge and on a vertex, a nearest vertex, a nearest edge interior, a
// degenerate edge, and coordinates with very different binary exponents.
func TestLoftCylinderTriangleMinRadiusMatchesRational(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		tri  [3]r3.Vec
	}{
		{"origin inside", [3]r3.Vec{{X: -1, Y: -1}, {X: 2, Y: -1}, {X: 0, Y: 3}}},
		{"origin on edge", [3]r3.Vec{{X: -1, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 3}}},
		{"origin on vertex", [3]r3.Vec{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: 3}}},
		{"nearest vertex", [3]r3.Vec{{X: 3, Y: 4}, {X: 6, Y: 4}, {X: 5, Y: 9}}},
		{"nearest edge interior", [3]r3.Vec{{X: 2, Y: -5}, {X: 2, Y: 5}, {X: 9, Y: 0.5}}},
		{"degenerate edge", [3]r3.Vec{{X: 3, Y: 1}, {X: 3, Y: 1}, {X: 5, Y: 8}}},
		{"mixed exponents", [3]r3.Vec{{X: 1e-3, Y: 3e2}, {X: 3e2, Y: 1e-3}, {X: 4e2, Y: 5e2}}},
		{"tiny and large", [3]r3.Vec{{X: 1e-9, Y: 2e-9}, {X: 7e3, Y: 1e-7}, {X: 1e-5, Y: 9e3}}},
		{"dyadic fractions", [3]r3.Vec{{X: 0.1, Y: 0.7}, {X: 0.3, Y: -0.9}, {X: 1.1, Y: 0.2}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			verts := tc.tri[:]
			tri := [3]int{0, 1, 2}
			require.Equal(t, loftCylinderTriangleMinRadiusReference(verts, tri),
				loftcylinder.TriangleMinRadius(verts, tri))
			// An edge-only call (the repeated index form used by seams).
			edge := [3]int{0, 1, 1}
			require.Equal(t, loftCylinderTriangleMinRadiusReference(verts, edge),
				loftcylinder.TriangleMinRadius(verts, edge))
		})
	}
}
