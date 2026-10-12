package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
)

func TestCurvedMappedSourceConsumesLiveProducer(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	if err != nil || !ok {
		t.Fatalf("live source was not admitted: %v, %t", err, ok)
	}
	section := proof.section.spans[proof.section.rootSpan]
	cells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	if err != nil || !ok {
		t.Fatalf("live source did not reach mapped finite-span consumer: %v, %t", err, ok)
	}
	if len(cells) == 0 {
		t.Fatal("mapped finite-span consumer returned no cells")
	}
	t.Logf("root section %d mapped %d source cells", proof.section.rootSpan, len(cells))
	radial := 0
	for segment, entity := range trim.source.source.entities {
		if _, isArc := entity.(*sketch.Arc); !isArc {
			continue
		}
		for band := 0; band+1 < len(trim.maps); band++ {
			if _, certified := mappedArcRadialTimeBand(t.Context(), trim, segment, band, section); certified {
				radial++
			}
		}
	}
	if radial == 0 {
		t.Fatal("no live mapped arc reached the radial source consumer")
	}
	t.Logf("mapped %d whole-arc bands", radial)
}
