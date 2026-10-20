package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrismPatternFilletOfCopiedExtrudes(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	before, err := joined.Volume()
	require.NoError(t, err)
	selection := prismPatternRootSelection(t, joined)
	rounded, err := Fillet(t.Context(), joined, selection, units.Millimeters(0.05))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	after, err := rounded.Volume()
	require.NoError(t, err)
	require.Greater(t, after.Value.Base(), before.Value.Base())
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestPrismPatternFilletAcrossCircularWallStations(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	selection := prismPatternRootSelection(t, joined)
	edges, err := selection.SelectEdges(joined)
	require.NoError(t, err)
	contacts, err := prismpattern.FilletContacts(t.Context(), joined.n.Payload.(facetedPayload), edgeNodes(edges), 0.14)
	require.NoError(t, err)
	// Selecting one root twice is refused rather than rounded twice.
	_, err = prismpattern.FilletContacts(t.Context(), joined.n.Payload.(facetedPayload),
		edgeNodes(append([]*Edge{edges[0]}, edges...)), 0.14)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "selects one root twice")
	multiStation := false
	for _, contact := range contacts {
		multiStation = multiStation || len(contact.RingCut.Positions) > 2
	}
	require.True(t, multiStation)
	rounded, err := Fillet(t.Context(), joined, selection,
		units.Millimeters(0.14))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestPrismPatternFilletSectorBeyondRootDisk(t *testing.T) {
	doc, blank, section, rotated := prismPatternNarrowTabBodies(t)
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	// The old 3r disk reaches the fitted flank. The analytic arc and its
	// connector cutback stay inside a narrower radial and angular envelope.
	const radius = 0.055
	require.Greater(t, 3+3*radius, 3.15)
	require.Greater(t, 2*3*math.Sin(0.14), 6*radius)
	rounded, err := Fillet(t.Context(), joined, prismPatternRootSelection(t, joined),
		units.Millimeters(radius))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	mesh, err := rounded.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func prismPatternNarrowTabBodies(t *testing.T) (*Document, *Body, *Body, *Body) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	point := func(radius, angle float64) *sketch.Point {
		p := s.CreatePoint(radius*math.Cos(angle), radius*math.Sin(angle))
		s.Fix(p)
		return p
	}
	rootRight, rootLeft := point(3, -0.14), point(3, 0.14)
	right := []*sketch.Point{point(3.15, -0.14), point(3.55, -0.137),
		point(3.75, -0.13), point(4, -0.12)}
	left := []*sketch.Point{point(3.15, 0.14), point(3.55, 0.137),
		point(3.75, 0.13), point(4, 0.12)}
	s.CreateLine(rootRight, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], rootLeft)
	s.CreateArc(center, rootRight, rootLeft)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := New()
	section, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	blankSketch, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	c := blankSketch.CreatePoint(0, 0)
	blankSketch.Fix(c)
	blankSketch.CreateCircle(c, math.Nextafter(3, math.Inf(1)))
	_, err = blankSketch.Solve(t.Context())
	require.NoError(t, err)
	blank, err := Extrude(t.Context(), doc, blankSketch, blankSketch.Profiles()[0],
		Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/2))
	require.NoError(t, err)
	rotated, err := Copy(t.Context(), section, WithCopyPlacement(turn))
	require.NoError(t, err)
	return doc, blank, section, rotated
}

func TestPrismPatternFilletRefusesUnprovedCornerAndLargeRadius(t *testing.T) {
	doc, blank, section, rotated := prismPatternTabBodies(t, math.Nextafter(3, math.Inf(1)))
	joined, err := Union(t.Context(), blank, section, rotated)
	require.NoError(t, err)
	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var exterior *Edge
	for _, edge := range axial {
		p := edge.Start().Position().Value
		if math.Hypot(p.X, p.Y) > 3.3 {
			exterior = edge
			break
		}
	}
	require.NotNil(t, exterior)
	_, err = Fillet(t.Context(), joined, Edges(ParallelTo(r3.Vec{Z: 1}),
		EndpointAt(exterior.Start().Position().Value)), units.Millimeters(0.05))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
	_, err = Fillet(t.Context(), joined, prismPatternRootSelection(t, joined), units.Millimeters(0.2))
	require.ErrorIs(t, err, ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
}

func prismPatternRootSelection(t *testing.T, joined *Body) *EdgeQuery {
	t.Helper()
	axial, err := Edges(ParallelTo(r3.Vec{Z: 1})).SelectEdges(joined)
	require.NoError(t, err)
	var roots []*Edge
	for _, edge := range axial {
		p := edge.Start().Position().Value
		if math.Abs(math.Hypot(p.X, p.Y)-3) < 1e-8 {
			roots = append(roots, edge)
		}
	}
	require.Len(t, roots, 4)
	selection := Edges(ParallelTo(r3.Vec{Z: 1}), EndpointAt(roots[0].Start().Position().Value))
	for _, edge := range roots[1:] {
		selection.Or(ParallelTo(r3.Vec{Z: 1}), EndpointAt(edge.Start().Position().Value))
	}
	return selection.Exactly(len(roots))
}
