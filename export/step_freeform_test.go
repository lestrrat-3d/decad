package export_test

import (
	"bytes"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSTEPFreeformSolidRoundTrip(t *testing.T) {
	for _, tc := range []struct {
		name  string
		bulge float64
	}{{"outward", -3}, {"inward", 3}} {
		t.Run(tc.name, func(t *testing.T) { checkSTEPFreeformSolid(t, tc.bulge) })
	}
}

func checkSTEPFreeformSolid(t *testing.T, bulge float64) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b, c, d := s.CreatePoint(0, 0), s.CreatePoint(10, 0), s.CreatePoint(10, 10), s.CreatePoint(0, 10)
	s.Fix(a)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	_, err = s.CreateSpline(d, s.CreatePoint(bulge, 7), s.CreatePoint(bulge, 3), a)
	require.NoError(t, err)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(4), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	tol := units.Millimeters(.2)
	source, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	original, err := decad.NewMeshBody(t.Context(), source.Vertices(), source.Triangles())
	require.NoError(t, err)
	originalVolume, err := original.Volume()
	require.NoError(t, err)
	file, err := export.NewSTEPFile(t.Context(), body, tol, header())
	require.NoError(t, err)
	data, err := file.Marshal()
	require.NoError(t, err)
	require.Contains(t, string(data), "faceted decad solid")
	result, err := decad.ReadSTEP(t.Context(), bytes.NewReader(data))
	require.NoError(t, err)
	require.True(t, result.Report().Closed)
	require.Equal(t, len(source.Triangles()), result.Report().Triangles)
	volume, err := result.Volume()
	require.NoError(t, err)
	require.InDelta(t, originalVolume.Value.Base(), volume.Value.Base(), 1e-8)
}
