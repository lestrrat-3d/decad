package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func pinAlongX(t *testing.T, doc *decad.Document, x0, length float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.YZ(), x0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(10, 5)
	s.Fix(center)
	s.CreateCircle(center, 3)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{
		D: units.Millimeters(length), Dir: decad.Along,
	})
	require.NoError(t, err)
	return body
}

func TestClassBRootedPinBooleanVolumes(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		op   func(*testing.T, *decad.Body, *decad.Body) (*decad.Body, error)
		want float64
	}{
		{name: "union_volume", op: func(t *testing.T, a, b *decad.Body) (*decad.Body, error) {
			return decad.Union(t.Context(), a, b)
		}, want: 8000 + 90*math.Pi},
		{name: "cut_volume", op: func(t *testing.T, a, b *decad.Body) (*decad.Body, error) {
			return decad.Cut(t.Context(), a, b)
		}, want: 8000 - 90*math.Pi},
		{name: "intersection_volume", op: func(t *testing.T, a, b *decad.Body) (*decad.Body, error) {
			return decad.Intersect(t.Context(), a, b)
		}, want: 90 * math.Pi},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 40, 20, 10)
			pin := pinAlongX(t, doc, 30, 20)
			got, err := tc.op(t, plate, pin)
			require.NoError(t, err)
			volume, err := got.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-tc.want), boundMM3(t, volume))
		})
	}
}
