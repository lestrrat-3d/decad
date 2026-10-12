package decad

import "testing"

func TestCurvedVerticalSourceConsumesLiveProducer(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, certified, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	if err != nil || !certified {
		t.Fatalf("live source was not admitted: %v, %t", err, certified)
	}
	manifest, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
	if err != nil || !complete || manifest == nil {
		t.Fatalf("live source did not reach all-side consumer: %v, %t", err, complete)
	}
	if len(manifest.sideOutcome) != len(proof.section.spans) {
		t.Fatal("all-side record does not cover the live Revolve section")
	}
	contacts, misses := 0, 0
	for _, entry := range manifest.vertical {
		if entry.sectionIndex < 0 || entry.sectionIndex >= len(proof.section.spans) ||
			entry.outcome == revolveLoftFacesVerticalContact && entry.side == nil ||
			entry.outcome == revolveLoftFacesVerticalMiss && entry.side != nil {
			t.Fatal("vertical answer lacks its live side record")
		}
		switch entry.outcome {
		case revolveLoftFacesVerticalContact:
			contacts++
		case revolveLoftFacesVerticalMiss:
			misses++
		default:
			t.Fatal("undecidable vertical side was admitted")
		}
	}
	if contacts == 0 || misses == 0 {
		t.Fatalf("real source did not exercise both vertical answers: contacts=%d misses=%d", contacts, misses)
	}
}
