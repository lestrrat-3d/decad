package stationbound_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChordStationBoundEnclosesCircleAndArcStations(t *testing.T) {
	t.Parallel()
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 2, V: -3}, Radius: units.Millimeters(5), CCW: true, TEnd: 0.25}
	angle := math.Pi / 4
	heldU, heldV := 2+5*math.Cos(angle), -3+5*math.Sin(angle)
	got := stationbound.ChordStationBound(circle, 1, 2, heldU, heldV)
	require.True(t, got.Derivable())
	limit := 4 * proofbound.UlpOf(5)
	require.LessOrEqual(t, math.Abs(got.U), limit)
	require.LessOrEqual(t, math.Abs(got.V), limit)

	off := stationbound.ChordStationBound(circle, 1, 2, heldU+1e-6, heldV)
	require.Greater(t, off.U, 9e-7)
	require.LessOrEqual(t, math.Abs(off.V), limit)

	arc := sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: 3}, End: sectionrecord.Point2{V: 3}, TEnd: 1}
	for station := 1; station < 4; station++ {
		angle := float64(station) * math.Pi / 8
		bound := stationbound.ChordStationBound(arc, station, 4, 3*math.Cos(angle), 3*math.Sin(angle))
		require.Truef(t, bound.Derivable(), "station %d", station)
		require.Positivef(t, math.Max(bound.U, bound.V), "station %d", station)
		require.Lessf(t, math.Max(bound.U, bound.V), 1e-12, "station %d", station)
	}
}

func TestChordStationBoundRejectsUnprovableStations(t *testing.T) {
	t.Parallel()
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TEnd: 1}
	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{}, End: sectionrecord.Point2{U: 1}, TEnd: 1}
	for _, row := range []struct {
		name string
		seg  sectionrecord.CurveSegment
		k, n int
	}{
		{name: "line", seg: line, k: 1, n: 2},
		{name: "start", seg: circle, k: 0, n: 4},
		{name: "end", seg: circle, k: 4, n: 4},
		{name: "empty", seg: circle, k: 1, n: 0},
	} {
		t.Run(row.name, func(t *testing.T) {
			bound := stationbound.ChordStationBound(row.seg, row.k, row.n, 0, 0)
			require.False(t, bound.Derivable())
			require.True(t, math.IsInf(bound.U, 1))
			require.True(t, math.IsInf(bound.V, 1))
		})
	}
}
