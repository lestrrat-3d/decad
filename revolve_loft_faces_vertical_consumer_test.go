package decad

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
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
	if len(manifest.sideOutcome) != len(proof.section.Spans) {
		t.Fatal("all-side record does not cover the live Revolve section")
	}
	contacts, misses := 0, 0
	for _, entry := range manifest.vertical {
		if entry.sectionIndex < 0 || entry.sectionIndex >= len(proof.section.Spans) ||
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
	t.Run("changed toe cone slope", func(t *testing.T) {
		original := proof.trim.Toe.slope
		defer func() { proof.trim.Toe.slope = original }()
		proof.trim.Toe.slope = lfAdd(original, lfF(0.001))
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("changed heel cone apex", func(t *testing.T) {
		original := proof.trim.Heel.apex
		defer func() { proof.trim.Heel.apex = original }()
		proof.trim.Heel.apex = lfAdd(original, lfF(0.001))
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("changed toe tool", func(t *testing.T) {
		original := proof.trim.ToeTool
		defer func() { proof.trim.ToeTool = original }()
		proof.trim.ToeTool = proof.trim.HeelTool
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("cap triangle on source face", func(t *testing.T) {
		mutateCurvedCapFaceOwner(t, tooth, proof, true)
	})
	t.Run("source triangle on cap face", func(t *testing.T) {
		mutateCurvedCapFaceOwner(t, tooth, proof, false)
	})
	t.Run("cap face index outside live faces", func(t *testing.T) {
		original := tooth.n.Payload
		defer func() { tooth.n.Payload = original }()
		payload := original.(facetedPayload)
		payload.FaceOf = slices.Clone(payload.FaceOf)
		changed := false
		for i, group := range payload.Src {
			if group == proof.trim.ToeCapGroup {
				payload.FaceOf[i] = len(tooth.Faces()) + 1
				changed = true
			}
		}
		if !changed {
			t.Fatal("the real producer lacks a toe cap triangle")
		}
		tooth.n.Payload = payload
		assertChangedCurvedTrimRefuses(t, tooth, proof)
	})
	t.Run("extra blank face", func(t *testing.T) {
		shell := blank.Shells()[0]
		original := shell.n.Faces
		defer func() { shell.n.Faces = original }()
		shell.n.Faces = append(slices.Clone(original), faceNode(bornFace(&Face{n: topo.Face{Origins: []provenance.Ref{{Role: "unmatched side"}}, Body: bodyNode(blank)}})))
		_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("an unclassified live blank face was admitted: %v, %t", err, complete)
		}
	})
	t.Run("changed root section", func(t *testing.T) {
		original := proof.section.RootStart
		defer func() { proof.section.RootStart = original }()
		proof.section.RootStart.U += 0.125
		_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("changed live root section was admitted: %v, %t", err, complete)
		}
	})
	t.Run("changed root cone", func(t *testing.T) {
		original := proof.section.RootCone
		defer func() { proof.section.RootCone = original }()
		proof.section.RootCone.Origin.X += 0.125
		_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("changed live root cone was admitted: %v, %t", err, complete)
		}
	})
	t.Run("changed tooth transform", func(t *testing.T) {
		original := tooth.n.Payload
		defer func() { tooth.n.Payload = original }()
		payload := original.(facetedPayload)
		shift, err := r3.Translation(r3.Vec{X: 1})
		if err != nil {
			t.Fatal(err)
		}
		payload.Xform = shift
		tooth.n.Payload = payload
		if revolveLoftFacesTrimCurrent(tooth, proof.trim) {
			t.Fatal("changed tooth transform retained a current source record")
		}
		_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
		if err != nil || complete {
			t.Fatalf("changed tooth transform was admitted: %v, %t", err, complete)
		}
	})
	t.Run("wrong vertical triangle owner", func(t *testing.T) {
		contactIndex := -1
		var ref provenance.Ref
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
		var ref provenance.Ref
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

func assertChangedCurvedTrimRefuses(t *testing.T, tooth *Body,
	proof *revolveLoftFacesSourceCertificate) {
	t.Helper()
	if revolveLoftFacesTrimCurrent(tooth, proof.trim) {
		t.Fatal("changed trim retained a current source record")
	}
	_, certified, err := certifyRevolveLoftFacesSource(t.Context(), proof.blank, tooth)
	if err != nil || certified {
		t.Fatalf("changed trim reached source admission: %v, %t", err, certified)
	}
	_, complete, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
	if err != nil || complete {
		t.Fatalf("changed trim reached all-side consumer: %v, %t", err, complete)
	}
}

func mutateCurvedCapFaceOwner(t *testing.T, tooth *Body,
	proof *revolveLoftFacesSourceCertificate, capToSource bool) {
	t.Helper()
	original := tooth.n.Payload
	defer func() { tooth.n.Payload = original }()
	payload := original.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
	capGroup := proof.trim.ToeCapGroup
	capTriangle, sourceTriangle := -1, -1
	for i, group := range payload.Src {
		if group == capGroup && capTriangle < 0 {
			capTriangle = i
		}
		if group != proof.trim.ToeCapGroup && group != proof.trim.HeelCapGroup && sourceTriangle < 0 {
			sourceTriangle = i
		}
	}
	if capTriangle < 0 || sourceTriangle < 0 ||
		payload.FaceOf[capTriangle] == payload.FaceOf[sourceTriangle] {
		t.Fatal("the real producer lacks separate cap and source faces")
	}
	if capToSource {
		payload.FaceOf[capTriangle] = payload.FaceOf[sourceTriangle]
	} else {
		payload.FaceOf[sourceTriangle] = payload.FaceOf[capTriangle]
	}
	tooth.n.Payload = payload
	assertChangedCurvedTrimRefuses(t, tooth, proof)
}

func rejectChangedCurvedSourceOwner(t *testing.T, tooth *Body, ref provenance.Ref,
	owner *Face, accepts func() bool) {
	t.Helper()
	original := tooth.n.Payload
	defer func() { tooth.n.Payload = original }()
	payload := original.(facetedPayload)
	payload.FaceOf = slices.Clone(payload.FaceOf)
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
	for i, group := range payload.Src {
		if slices.Contains(payload.Groups[group].Origins, ref) {
			payload.FaceOf[i] = other
			changed = true
			break
		}
	}
	if !changed {
		t.Fatal("the retained source has no triangle")
	}
	tooth.n.Payload = payload
	if accepts() {
		t.Fatal("changed live triangle owner was admitted")
	}
}
