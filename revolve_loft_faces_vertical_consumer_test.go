package decad

import (
	"slices"
	"testing"
)

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
	t.Run("extra blank face", func(t *testing.T) {
		shell := blank.Shells()[0]
		original := shell.faces
		defer func() { shell.faces = original }()
		shell.faces = append(slices.Clone(original), &Face{
			body: blank, origins: []FeatureRef{{Role: "unmatched side"}},
		})
		_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("an unclassified live blank face was admitted: %v, %t", err, complete)
		}
	})
	t.Run("wrong vertical triangle owner", func(t *testing.T) {
		contactIndex := -1
		var ref FeatureRef
		var owner *Face
		for _, entry := range manifest.vertical {
			if entry.outcome != revolveLoftFacesVerticalContact {
				continue
			}
			for _, cell := range entry.side.cells {
				if len(cell.retained) > 0 {
					contactIndex, ref, owner = entry.sectionIndex, cell.ref, cell.face
					break
				}
			}
			if contactIndex >= 0 {
				break
			}
		}
		if contactIndex < 0 || owner == nil {
			t.Fatal("the live vertical contact has no retained source face")
		}
		rejectChangedCurvedSourceOwner(t, tooth, ref, owner, func() bool {
			side, outcome, err := revolveLoftFacesVerticalSideClassify(t.Context(), proof, contactIndex)
			return err == nil && side != nil && outcome == revolveLoftFacesVerticalContact
		})
	})
	t.Run("wrong nonvertical triangle owner", func(t *testing.T) {
		var ref FeatureRef
		var owner *Face
		for _, entry := range manifest.nonvertical {
			for _, cell := range entry.cells {
				if cell.mapped.segment != proof.crossings[0].segment &&
					cell.mapped.segment != proof.crossings[1].segment {
					ref, owner = cell.ref, cell.face
					break
				}
			}
			if owner != nil {
				break
			}
		}
		if owner == nil {
			t.Fatal("the live nonvertical side has no retained noncrossing source face")
		}
		rejectChangedCurvedSourceOwner(t, tooth, ref, owner, func() bool {
			_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
			return err == nil && complete
		})
	})
}

func rejectChangedCurvedSourceOwner(t *testing.T, tooth *Body, ref FeatureRef,
	owner *Face, accepts func() bool) {
	t.Helper()
	original := tooth.payload
	defer func() { tooth.payload = original }()
	payload := original.(facetedPayload)
	payload.faceOf = slices.Clone(payload.faceOf)
	other := -1
	for i, face := range tooth.Faces() {
		if face != owner {
			other = i
			break
		}
	}
	if other < 0 {
		t.Fatal("the live source has no other face")
	}
	changed := false
	for i, group := range payload.src {
		if slices.Contains(payload.groups[group].origins, ref) {
			payload.faceOf[i] = other
			changed = true
			break
		}
	}
	if !changed {
		t.Fatal("the retained source has no triangle")
	}
	tooth.payload = payload
	if accepts() {
		t.Fatal("changed live triangle owner was admitted")
	}
}
