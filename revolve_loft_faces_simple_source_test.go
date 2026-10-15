package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func neutralSimpleSourceSketch(t *testing.T, extra bool) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	fixed := func(u, v float64) *sketch.Point {
		point := s.CreatePoint(u, v)
		s.Fix(point)
		return point
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
	if extra {
		s.CreateLine(fixed(5, 1), fixed(6, 1))
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func neutralSimpleSourceRecord(t *testing.T, s *sketch.Sketch,
	profile *sketch.Profile) *loftFacesTrimRecord {
	t.Helper()
	recorded, plane, _, err := sectioncert.RecordProfile(s, profile)
	require.NoError(t, err)
	frame, err := s.Plane().Frame()
	require.NoError(t, err)
	return &loftFacesTrimRecord{Source: &pointSectionRecord{
		profile: recorded, plane: plane,
		source: &pointSectionSource{sketch: s, profile: profile,
			revision: s.Revision(), frame: frame,
			entities: append([]sketch.Entity(nil), profile.Entities...)},
	}}
}

func TestRevolveLoftFacesSimpleSourceCertifiesWholeCurveLoop(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	loop, certified, err := revolveLoftFacesSimpleSourceLoop(t.Context(),
		neutralSimpleSourceRecord(t, s, profile))
	require.NoError(t, err)
	require.True(t, certified)
	require.True(t, loop.ccw)
	require.Len(t, loop.cells, 8, "two fitted flanks each reconstruct as two Bezier spans")
	for i, cell := range loop.cells {
		next := loop.cells[(i+1)%len(loop.cells)]
		require.True(t, revolveLoftFacesSimpleNeighbors(cell, next), "joint %d", i)
	}
}

func TestRevolveLoftFacesSimpleSourceRefusesExtraSketchEntity(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, true)
	_, certified, err := revolveLoftFacesSimpleSourceLoop(t.Context(),
		neutralSimpleSourceRecord(t, s, profile))
	require.NoError(t, err)
	require.False(t, certified, "an extra entity may change the true region")
}

func TestRevolveLoftFacesSimpleSourceRefusesChangedRecordedArc(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	trim := neutralSimpleSourceRecord(t, s, profile)
	arc := trim.Source.profile.Outer.Segments[1].(arcSeg)
	arc.Center.U = 2
	trim.Source.profile.Outer.Segments[1] = arc
	require.True(t, pointSourceCurrent(trim.Source),
		"the existing revision gate does not compare stored curve geometry")
	_, certified, err := revolveLoftFacesSimpleSourceLoop(t.Context(), trim)
	require.NoError(t, err)
	require.False(t, certified, "the live Sketch arc still has its original center")
}

func TestRevolveLoftFacesSimpleNeighborsRequireOppositeInteriorSides(t *testing.T) {
	span := func(a, b float64) loftface.SimpleCell {
		cell, ok := loftface.SimpleBezier(0, 0, freeform.BezierSpan{
			{U: lfF(a), V: lfZero()}, {U: lfF(b), V: lfZero()}})
		require.True(t, ok)
		return cell
	}
	first := span(0, 1)
	require.True(t, revolveLoftFacesSimpleNeighbors(first, span(1, 2)))
	require.False(t, revolveLoftFacesSimpleNeighbors(first, span(1, 0)),
		"a curve that doubles back shares more than the endpoint")
}

func TestRevolveLoftFacesSimpleSeparatedRequiresStrictAxisGap(t *testing.T) {
	span := func(startU, startV, endU, endV float64) loftface.SimpleCell {
		cell, ok := loftface.SimpleBezier(0, 0, freeform.BezierSpan{
			{U: lfF(startU), V: lfF(startV)},
			{U: lfF(endU), V: lfF(endV)}})
		require.True(t, ok)
		return cell
	}
	first := span(0, 0, 1, 1)
	require.True(t, loftface.SimpleSeparated(first, span(2, 0, 3, 1)))
	require.False(t, loftface.SimpleSeparated(first, span(1, 0, 2, 1)),
		"a shared box boundary cannot certify separation")
	require.False(t, loftface.SimpleSeparated(first, span(0.5, 0.5, 1.5, 1.5)),
		"overlap in both coordinate boxes needs a stronger proof")
}
