package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChainGateDiameterCurvedExtrudeVerify(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	s.Fix(p0)
	_, err = s.CreateSpline(p0, s.CreatePoint(10, 5), s.CreatePoint(20, 8), s.CreatePoint(30, 9))
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	doc := decad.New()
	body, err := doc.ExtrudeChain(s, chains[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
	require.Len(t, body.Faces(), 1)
	area, err := body.Area()
	require.NoError(t, err)
	require.Greater(t, area.Value.Base()-area.Bound.Base(), 0.0)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed(), "%+v", report.Diagnostics)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), rotation)
	require.NoError(t, err)
	placedReport, err := doc.Verify(t.Context())
	require.NoError(t, err)
	_, err = placedReport.ForBody(placed)
	require.NoError(t, err)
	require.True(t, placedReport.Passed(), "%+v", placedReport.Diagnostics)
}

func TestChainGateDiameterFullRevolveVerify(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p0 := s.CreatePoint(0, 0)
	s.Fix(p0)
	s.CreateLine(p0, s.CreatePoint(4, 3))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	doc := decad.New()
	body, err := doc.RevolveChain(s, chains[0],
		decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}},
		decad.FullRevolution{})
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
	require.Len(t, body.Faces(), 1)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, area.Value.Base()-area.Bound.Base(), 15*math.Pi)
	require.GreaterOrEqual(t, area.Value.Base()+area.Bound.Base(), 15*math.Pi)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed(), "%+v", report.Diagnostics)
}

func TestChainGateDiameterLoftHasToleranceReference(t *testing.T) {
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	makeLine := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Chain) {
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		p0, p1 := s.CreatePoint(0, 0), s.CreatePoint(40, 0)
		s.Fix(p0)
		s.Fix(p1)
		s.CreateLine(p0, p1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		chains := s.Chains()
		require.Len(t, chains, 1)
		return s, chains[0]
	}
	s0, c0 := makeLine(w.XY())
	s1, c1 := makeLine(top)
	doc := decad.New()
	body, err := doc.LoftChain(t.Context(), s0, c0, s1, c1)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
	require.Len(t, body.Faces(), 2)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, area.Value.Base()-area.Bound.Base(), 400.0)
	require.GreaterOrEqual(t, area.Value.Base()+area.Bound.Base(), 400.0)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.True(t, report.Passed(), "%+v", report.Diagnostics)
}

func TestChainGateDiameterPartialRevolveHasToleranceReference(t *testing.T) {
	for _, degrees := range []float64{90, 270} {
		t.Run(units.Degrees(degrees).String(), func(t *testing.T) {
			s, chain, axis := coneChainAtRadius(t, 0)
			doc := decad.New()
			body, err := doc.RevolveChain(s, chain, axis,
				decad.AngleExtent{A: units.Degrees(degrees), Dir: decad.Along})
			require.NoError(t, err)
			require.Equal(t, decad.BodySheet, body.Kind())
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			for _, diag := range report.Diagnostics {
				require.NotEqual(t, decad.DiagToleranceReferenceUnavailable, diag.Code)
			}
		})
	}
}

func TestChainGateDiameterPlacedRevolveHasToleranceReference(t *testing.T) {
	s, chain, axis := coneChainAtRadius(t, 0)
	doc := decad.New()
	body, err := doc.RevolveChain(s, chain, axis, decad.FullRevolution{})
	require.NoError(t, err)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), rotation)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, placed.Kind())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	for _, diag := range report.Diagnostics {
		require.NotEqual(t, decad.DiagToleranceReferenceUnavailable, diag.Code)
	}
}
