package coil_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

var arcAreaBenchmarkSink coil.Iv

func arcAreaBenchmarkInputs() []struct {
	name    string
	loop    sectionrecord.LoopRecord
	segment int
} {
	return []struct {
		name    string
		loop    sectionrecord.LoopRecord
		segment int
	}{
		{name: "circle", loop: roundWire()},
		{name: "semicircle", loop: slot(), segment: 1},
		{name: "reversed-arc", loop: bite(), segment: 1},
	}
}

func BenchmarkArcAreaRepresentative(b *testing.B) {
	pitch, turns := big.NewRat(3, 2), big.NewRat(5, 1)
	for _, fixture := range arcAreaBenchmarkInputs() {
		profile, err := coil.Loops(fixture.loop, nil, 16)
		if err != nil {
			b.Fatal(err)
		}
		segment := profile.Segments[fixture.segment]
		for _, setting := range []struct {
			name    string
			workers int
		}{{"workers-1", 1}, {"workers-4", 4}} {
			b.Run(fixture.name+"/"+setting.name, func(b *testing.B) {
				ctx := orderedwork.WithWorkers(b.Context(), setting.workers)
				b.ReportAllocs()
				b.ResetTimer()
				for range b.N {
					area, ok, err := coil.ArcArea(ctx, segment, vAxis(), pitch, turns)
					if err != nil {
						b.Fatal(err)
					}
					if !ok || area.Lo.Cmp(area.Hi) > 0 {
						b.Fatal("ArcArea did not return an ordered enclosure")
					}
					arcAreaBenchmarkSink = area
				}
			})
		}
	}
}
