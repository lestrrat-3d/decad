package apitest_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func fittedSectorPointLoft(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0),
		r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	center := fixed(0, 0)
	innerRight, innerLeft := fixed(2.2, -0.2), fixed(2.2, 0.2)
	right := []*sketch.Point{fixed(2.8, -0.24), fixed(3.5, -0.3), fixed(4.2, -0.35)}
	left := []*sketch.Point{fixed(2.8, 0.24), fixed(3.5, 0.3), fixed(4.2, 0.35)}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[2], left[2])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(innerRight, right[0])
	s.CreateLine(left[0], innerLeft)
	s.CreateArc(center, innerRight, innerLeft)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	require.Len(t, s.Profiles()[0].Outer, 6)
	for _, edge := range s.Profiles()[0].Outer {
		require.False(t, edge.Partial)
	}
	loft, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return loft
}

func axialSliceSheet(t *testing.T, doc *decad.Document, x float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	a, b := s.CreatePoint(x, -20), s.CreatePoint(x, 20)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	sheet, err := decad.Extrude(t.Context(), doc, s, s.Chains()[0], decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(20)},
		Two: decad.DistanceSide{D: units.Millimeters(20)},
	})
	require.NoError(t, err)
	return sheet
}

func finiteConicalBlank(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	section := []decad.Point2{
		{U: 3.2}, {U: 6}, {U: 6, V: 3.5},
		{U: 5.8, V: 3.3}, {U: 5.7, V: 2.28},
		{U: 4.7, V: 1.88}, {U: 3.2, V: 3.2},
	}
	points := make([]*sketch.Point, len(section))
	for i, p := range section {
		points[i] = s.CreatePoint(p.U, p.V)
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	blank, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return blank
}

func TestCurvedLoftFacesUnionStopsAtTheChainDepthGate(t *testing.T) {
	doc := decad.New()
	inner := fittedSectorPointLoft(t, doc)
	slabs := make([]*decad.Body, 0, 8)
	for k := 1; k <= 8; k++ {
		parts, err := decad.Split(t.Context(), inner, axialSliceSheet(t, doc, 8-0.7*float64(k)))
		require.NoError(t, err)
		require.Len(t, parts, 2)
		inner = parts[0]
		slabs = append(slabs, parts[1])
	}
	sections := make([]*decad.Face, 0, 9)
	sections = append(sections, pointSlabCap(t, slabs[7], decad.CapStart(slabs[7])))
	for i := range slices.Backward(slabs) {
		sections = append(sections, pointSlabCap(t, slabs[i], decad.CapEnd(slabs[i])))
	}
	loft, err := decad.LoftFaces(t.Context(), doc, sections...)
	require.NoError(t, err)
	toe, err := decad.Cut(t.Context(), loft, pointConeTool(t, doc, 6.5))
	require.NoError(t, err)
	trimmed, err := decad.Intersect(t.Context(), toe, pointConeTool(t, doc, 8))
	require.NoError(t, err)
	blank := finiteConicalBlank(t, doc)
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	require.Positive(t, blankVolume.Value.Base())
	trimmedVolume, err := trimmed.Volume()
	require.NoError(t, err)
	require.Positive(t, trimmedVolume.Value.Base())
	before := doc.Bodies()
	_, err = decad.Union(t.Context(), blank, trimmed)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "coarser than this pair's chord tolerance")
	require.Equal(t, before, doc.Bodies())

	move, err := r3.Translation(r3.NewVec(0, 0.05, 0))
	require.NoError(t, err)
	shifted, err := decad.Copy(t.Context(), blank, decad.WithCopyPlacement(move))
	require.NoError(t, err)
	_, err = decad.Union(t.Context(), shifted, trimmed)
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
}
