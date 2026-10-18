package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExtrudeRejectsOverflowedVolume(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 1e103)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(1e103), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrNotFinite)
	require.Nil(t, body)
}

func TestRevolveRejectsOverflowedVolume(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(2e103, 0)
	s.Fix(center)
	s.CreateCircle(center, 1e103)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := decad.New()
	axis := decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}}
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.ErrorIs(t, err, decad.ErrNotFinite)
	require.Nil(t, body)
}

func TestExtrudeCoalescedEdgesCarryLengthBounds(t *testing.T) {
	t.Parallel()
	s, profile := polygonSketch(t, [][2]float64{
		{0, 0}, {1, 1}, {2, 2}, {1, 3}, {0, 2},
	})
	body, err := decad.Extrude(t.Context(), decad.New(), s, profile,
		decad.Distance{D: units.Millimeters(1), Dir: decad.Along})
	require.NoError(t, err)

	coalesced := 0
	for _, edge := range body.Edges() {
		length, err := edge.Length()
		require.NoError(t, err)
		got, err := length.Value.In(units.Millimeter)
		require.NoError(t, err)
		if math.Abs(got-2*math.Sqrt2) > 1e-12 {
			continue
		}
		coalesced++
		require.Positive(t, length.Bound.Mag())
	}
	require.Equal(t, 2, coalesced)
}
