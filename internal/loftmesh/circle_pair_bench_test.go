package loftmesh_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// BenchmarkCircularCellStationsPair measures the complete paired-circle
// station generator from two recorded sections and their actual walks.
func BenchmarkCircularCellStationsPair(b *testing.B) {
	first := sectionrecord.CircleSeg{Radius: units.Millimeters(10), TStart: 0, TEnd: 1, CCW: true}
	second := sectionrecord.CircleSeg{Radius: units.Millimeters(5), TStart: 0, TEnd: 1, CCW: true}
	w0, err := boundarywalk.WalkOf(first, nil)
	if err != nil {
		b.Fatal(err)
	}
	w1, err := boundarywalk.WalkOf(second, nil)
	if err != nil {
		b.Fatal(err)
	}
	a, c := loftmesh.RecordCircularSide(w0, first), loftmesh.RecordCircularSide(w1, second)
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		stations0, stations1, _, _, _, err := loftmesh.LoftCircularCellStations(a, c, 0.25)
		if err != nil || len(stations0) != len(stations1) || len(stations0) < 4 {
			b.Fatalf("unexpected paired circle stations: %d, %d, %v", len(stations0), len(stations1), err)
		}
	}
}
