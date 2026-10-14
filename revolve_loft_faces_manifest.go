package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/provenance"
)

// This is a source-only record. A mapped cell names its complete Sketch
// entity, Bezier span, LoftFaces band, source parameter interval and band-time
// interval. Contact cells also name the exact live endpoint that owns them.
// Tool clipping and held-switch construction are separate certificates.
type revolveLoftFacesSourceCell struct {
	mapped revolveLoftFacesMappedCell
	ref    provenance.Ref
	face   *Face
}

type revolveLoftFacesNonverticalSide struct {
	sectionIndex int
	face         *Face
	origins      []provenance.Ref
	cells        []revolveLoftFacesSourceCell
}

func revolveLoftFacesNonverticalManifest(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) ([]revolveLoftFacesNonverticalSide, bool, error) {
	if ctx == nil || proof == nil || proof.blank == nil || proof.tooth == nil || proof.trim == nil ||
		proof.section.blank != proof.blank || len(proof.section.spans) == 0 ||
		len(proof.sourceRefs) != len(proof.trim.maps)-1 {
		return nil, false, nil
	}
	p, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || p.loftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) ||
		!pointSourceCurrent(proof.trim.source) {
		return nil, false, nil
	}
	liveRefs, ok := revolveLoftFacesSourceRefs(proof.trim, proof.tooth)
	if !ok || len(liveRefs) != len(proof.sourceRefs) {
		return nil, false, nil
	}
	for band := range liveRefs {
		if !slices.Equal(liveRefs[band], proof.sourceRefs[band]) {
			return nil, false, nil
		}
	}
	current := false
	for _, candidate := range revolveRootCandidates(proof.blank) {
		if candidate.rootSpan != proof.section.rootSpan ||
			len(candidate.spans) != len(proof.section.spans) {
			continue
		}
		matches := true
		for i, span := range candidate.spans {
			old := proof.section.spans[i]
			matches = matches && span.face == old.face &&
				span.start == old.start && span.end == old.end &&
				slices.Equal(span.sourceEdges, old.sourceEdges) &&
				slices.Equal(span.origins, old.origins)
		}
		if matches {
			current = true
			break
		}
	}
	if !current {
		return nil, false, nil
	}
	faces := proof.tooth.Faces()
	var manifest []revolveLoftFacesNonverticalSide
	for sectionIndex, section := range proof.section.spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if section.face == nil || section.start.U == section.end.U ||
			section.start.V <= 0 || section.end.V <= 0 {
			continue
		}
		mapped, ok, err := revolveLoftFacesFiniteSpanCells(ctx, proof.trim, section)
		if err != nil {
			return nil, false, fmt.Errorf("finite Revolve source side %d: %w", sectionIndex, err)
		}
		if !ok || len(mapped) == 0 {
			return nil, false, fmt.Errorf("%w: finite Revolve source side %d has undecidable cells",
				ErrUnsupported, sectionIndex)
		}
		entry := revolveLoftFacesNonverticalSide{sectionIndex: sectionIndex,
			face: section.face, origins: append([]provenance.Ref(nil), section.origins...)}
		for _, cell := range mapped {
			if cell.band < 0 || cell.band >= len(proof.sourceRefs) ||
				cell.segment < 0 || cell.segment >= len(proof.sourceRefs[cell.band]) ||
				cell.sLo == nil || cell.sHi == nil || cell.tLo == nil || cell.tHi == nil ||
				cell.side == revolveLoftFacesJointContact &&
					(cell.jointKind == 0 ||
						cell.jointKind == revolveLoftFacesWholeArcContact && cell.jointArc == nil ||
						cell.jointKind != revolveLoftFacesWholeArcContact && cell.jointPoint == nil) {
				return nil, false, nil
			}
			ref := liveRefs[cell.band][cell.segment]
			index, valid := revolveLoftFacesRefFaceIndex(p, ref, len(faces))
			face, originValid := revolveLoftFacesUniqueOriginFace(proof.tooth, ref)
			if !valid || !originValid || (index >= 0) != (face != nil) ||
				index >= 0 && faces[index] != face {
				return nil, false, nil
			}
			if face == nil {
				continue // the authenticated source wall has no retained face
			}
			entry.cells = append(entry.cells, revolveLoftFacesSourceCell{
				mapped: cell, ref: ref, face: face})
		}
		manifest = append(manifest, entry)
	}
	return manifest, len(manifest) > 0, nil
}
