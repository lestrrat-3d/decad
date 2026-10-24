package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func holePlateFixture(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) == 1 {
			profile = candidate
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	body, err := decad.Extrude(t.Context(), doc, s, profile,
		decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}
