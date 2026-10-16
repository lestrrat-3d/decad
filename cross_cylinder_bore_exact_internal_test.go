package decad

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func crossBoreExactRecord(t *testing.T, radius, bore, height, centerZ float64) *crossCylinderBoreRecord {
	t.Helper()
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	target, err := Extrude(t.Context(), doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.YZ())
	require.NoError(t, err)
	toolCenter := ts.CreatePoint(0, centerZ)
	ts.Fix(toolCenter)
	ts.CreateCircle(toolCenter, bore)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Symmetric{D: units.Millimeters(2*radius + 2)})
	require.NoError(t, err)
	rec := recognizeCrossCylinderBore(target, tool)
	require.NotNil(t, rec)
	return rec
}

func TestCrossBoreStationErrorsMatchSerial(t *testing.T) {
	for _, tc := range []struct {
		R, r, H, c, setback float64
	}{
		{5, 1, 10, 5, 0.5},
		{8, 1.5, 16, 8, 0.1875},
	} {
		t.Run(fmt.Sprintf("%gx%g", tc.R, tc.r), func(t *testing.T) {
			rec := crossBoreExactRecord(t, tc.R, tc.r, tc.H, tc.c)
			serial, err := buildCrossBoreChamferMeshWorkers(t.Context(), rec, tc.setback, 1)
			require.NoError(t, err)
			fanned, err := buildCrossBoreChamferMeshWorkers(t.Context(), rec, tc.setback, 4)
			require.NoError(t, err)
			require.Equal(t, serial.Verts, fanned.Verts)
			require.Equal(t, serial.Tris, fanned.Tris)
			require.Equal(t, serial.Src, fanned.Src)
			require.Equal(t, math.Float64bits(serial.OuterStationError), math.Float64bits(fanned.OuterStationError))

			stations, err := crossbore.StationPlan(rec, tc.setback)
			require.NoError(t, err)
			radius := new(big.Rat).SetFloat64(rec.Radius)
			want := 0.0
			for _, station := range stations {
				x, y := rec.Radius*math.Cos(station.Phi), rec.Radius*math.Sin(station.Phi)
				upper := station.Lower
				if station.Lower.Sub(station.Upper).Len() >= 1e-13 {
					upper = station.Upper
				}
				for _, p := range []r3.Vec{{X: x, Y: y}, station.Lower, upper, {X: x, Y: y, Z: rec.Height}} {
					e, ok := crossbore.StationError(p, radius, station.Phi)
					require.True(t, ok)
					want = max(want, e)
				}
			}
			require.Equal(t, math.Float64bits(want), math.Float64bits(serial.OuterStationError))
		})
	}
}
