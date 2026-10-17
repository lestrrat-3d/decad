package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// This is a source-only record. A mapped cell names its complete Sketch
// entity, Bezier span, LoftFaces band, source parameter interval and band-time
// interval. Contact cells also name the exact live endpoint that owns them.
// Tool clipping and held-switch construction are separate certificates.
type revolveLoftFacesSourceCell struct {
	mapped revolveloftfaces.RevolveLoftFacesMappedCell
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
		proof.section.Blank != bodyNode(proof.blank) || len(proof.section.Spans) == 0 ||
		len(proof.sourceRefs) != len(proof.trim.Maps)-1 {
		return nil, false, nil
	}
	p, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) ||
		!pointSourceCurrent(proof.trim.Source) {
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
		if candidate.RootSpan != proof.section.RootSpan ||
			len(candidate.Spans) != len(proof.section.Spans) {
			continue
		}
		matches := true
		for i, span := range candidate.Spans {
			old := proof.section.Spans[i]
			matches = matches && span.Face == old.Face &&
				span.Start == old.Start && span.End == old.End &&
				slices.Equal(span.SourceEdges, old.SourceEdges) &&
				slices.Equal(span.Origins, old.Origins)
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
	for sectionIndex, section := range proof.section.Spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if section.Face == nil || section.Start.U == section.End.U ||
			section.Start.V <= 0 || section.End.V <= 0 {
			continue
		}
		mapped, ok, err := revolveloftfaces.RevolveLoftFacesFiniteSpanCells(ctx, proof.trim, section)
		if err != nil {
			return nil, false, fmt.Errorf("finite Revolve source side %d: %w", sectionIndex, err)
		}
		if !ok || len(mapped) == 0 {
			return nil, false, fmt.Errorf("%w: finite Revolve source side %d has undecidable cells",
				ErrUnsupported, sectionIndex)
		}
		entry := revolveLoftFacesNonverticalSide{sectionIndex: sectionIndex,
			face: faceHandle(section.Face), origins: append([]provenance.Ref(nil), section.Origins...)}
		for _, cell := range mapped {
			if cell.Band < 0 || cell.Band >= len(proof.sourceRefs) ||
				cell.Segment < 0 || cell.Segment >= len(proof.sourceRefs[cell.Band]) ||
				cell.SLo == nil || cell.SHi == nil || cell.TLo == nil || cell.THi == nil ||
				cell.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
					(cell.JointKind == 0 ||
						cell.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact && cell.JointArc == nil ||
						cell.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact && cell.JointPoint == nil) {
				return nil, false, nil
			}
			ref := liveRefs[cell.Band][cell.Segment]
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
