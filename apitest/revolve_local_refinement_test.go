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

func torusArcBody(t *testing.T, doc *decad.Document, sweep float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 4)
	s.Fix(c)
	s.CreateCircle(c, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.ConstructionAxis{Dir: r3.Vec{X: 1}},
		decad.AngleExtent{A: units.Radians(sweep), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func radialStemBody(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(7), Dir: decad.Along})
	require.NoError(t, err)
	tr, err := r3.Translation(r3.Vec{Y: 4})
	require.NoError(t, err)
	moved, err := decad.Transform(t.Context(), body, tr)
	require.NoError(t, err)
	return moved
}

func TestUnionTorusArcWithStemShortSweep(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arc := torusArcBody(t, doc, math.Pi/9)
	stem := radialStemBody(t, doc)
	out, err := decad.Union(t.Context(), arc, stem)
	require.NoError(t, err)
	vol, err := out.Volume()
	require.NoError(t, err)
	arcVol := math.Pi * 4 * (math.Pi / 9)
	stemVol := 7 * math.Pi
	got := vol.Value.Base()
	// The union holds the whole stem plus the part of the arc it does not cover.
	require.Greater(t, got, stemVol)
	require.Less(t, got, arcVol+stemVol)
}

// A facet-pair audit that exceeds its work ceiling is final. A finer angular
// count only adds facets, so refining on it would charge every retry's whole
// mesh to the cumulative facet ceiling and report that ceiling instead.
func TestUnionTorusArcAuditWorkCeilingIsReportedAsItself(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arc := torusArcBody(t, doc, math.Pi/3)
	stem := radialStemBody(t, doc)
	_, err := decad.Union(t.Context(), arc, stem)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "contact audit exceeds its fixed work ceiling")
}
