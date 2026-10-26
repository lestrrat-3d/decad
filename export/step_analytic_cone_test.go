package export

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// coneFrustum revolves a trapezoid, radius r0 at x = 0 and r1 at x = 10, about +X.
func coneFrustum(t *testing.T, r0, r1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b, c, d := s.CreatePoint(0, 0), s.CreatePoint(10, 0), s.CreatePoint(10, r1), s.CreatePoint(0, r0)
	for _, p := range []*sketch.Point{a, b, c, d} {
		s.Fix(p)
	}
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestAnalyticConeFrustumWritesAConicalSurface(t *testing.T) {
	t.Parallel()
	f := bandFile(t, coneFrustum(t, 5, 3))
	byName := bandEntities(f)
	require.Len(t, byName["CONICAL_SURFACE"], 1)
	require.Equal(t, []float64{0}, bandReals(byName["CONICAL_SURFACE"], 2), "the reference circle is at the apex")
	require.InDelta(t, math.Atan(0.2), bandReals(byName["CONICAL_SURFACE"], 3)[0], 1e-12)
	require.Len(t, byName["CIRCLE"], 2)
	require.Len(t, byName["ADVANCED_FACE"], 3)
}

// TestAnalyticConeFrustumReadsBackToItsVolume writes a frustum that narrows and
// one that widens along its axis, and reads each file with decad.ReadSTEP: the
// mesh is closed and holds the frustum's volume less the chord tolerance's sliver.
// Shown to fail with the sense of the face inverted (the wall's triangles then
// face inward and the volume reads negative or the mesh opens).
func TestAnalyticConeFrustumReadsBackToItsVolume(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		r0, r1 float64
	}{{"narrowing", 5, 3}, {"widening", 3, 5}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			f := bandFile(t, coneFrustum(t, tc.r0, tc.r1))
			data, err := f.Marshal()
			require.NoError(t, err)
			mesh, err := decad.ReadSTEP(t.Context(), bytes.NewReader(data))
			require.NoError(t, err)
			require.True(t, mesh.Report().Closed)
			volume, err := mesh.Volume()
			require.NoError(t, err)
			exact := math.Pi * 10 / 3 * (tc.r0*tc.r0 + tc.r0*tc.r1 + tc.r1*tc.r1)
			require.Less(t, volume.Value.Base(), exact)
			require.Greater(t, volume.Value.Base(), 0.995*exact)
		})
	}
}

func TestAnalyticConeToAnApexKeepsFaceted(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin, apex, top := sk.CreatePoint(0, 0), sk.CreatePoint(10, 0), sk.CreatePoint(0, 5)
	sk.Fix(origin)
	sk.CreateLine(origin, apex)
	sk.CreateLine(apex, top)
	sk.CreateLine(top, origin)
	_, err = sk.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.Revolve(t.Context(), decad.New(), sk, sk.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	ok, err := supportsAnalyticSTEP(t.Context(), body)
	require.NoError(t, err)
	require.False(t, ok, "a cone that reaches its apex has one loop, which the analytic arm does not take")
}
