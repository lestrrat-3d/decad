package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
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
		!facetedrecord.PointSourceCurrent(proof.trim.Source) {
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
	for _, candidate := range revolveshell.RootCandidates(bodyNode(proof.blank)) {
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
			index, valid := revolveloftfaces.RefFaceIndex(p, ref, len(faces))
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

// A contact manifest keeps the two finite Revolve-side proofs separate.
// Tool-cap circles and the held seam need their own later certificates.
type revolveLoftFacesContactManifest struct {
	nonvertical []revolveLoftFacesNonverticalSide
	vertical    []revolveLoftFacesVerticalEntry
	sideOutcome []revolveLoftFacesSideOutcome // indexed by the authenticated Revolve section
}

type revolveLoftFacesSideOutcome uint8

const (
	revolveLoftFacesSideRefusal revolveLoftFacesSideOutcome = iota
	// A retained source contour is present. Its solid contact and held seam
	// still need separate proof.
	revolveLoftFacesSideSourceCandidate
	// A strict gap proves the complete finite Revolve face misses the solid.
	revolveLoftFacesSideMiss
)

type revolveLoftFacesVerticalEntry struct {
	sectionIndex int
	outcome      revolveloftfaces.VerticalOutcome
	side         *revolveLoftFacesVerticalSide // present only for a contact
}

// Every live Revolve boundary face gets exactly one source-side answer.
// A vertical miss proves that complete blank face misses the trimmed solid;
// a contact retains its exact clipped source cells for seam construction.
func revolveLoftFacesAllSideManifest(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate) (*revolveLoftFacesContactManifest, bool, error) {
	if ctx == nil || proof == nil {
		return nil, false, nil
	}
	if !revolveLoftFacesSectionCurrent(proof) || !revolveLoftFacesSectionFacesComplete(proof) {
		return nil, false, nil
	}
	nonvertical, ok, err := revolveLoftFacesNonverticalManifest(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	manifest := &revolveLoftFacesContactManifest{
		nonvertical: nonvertical,
		sideOutcome: make([]revolveLoftFacesSideOutcome, len(proof.section.Spans)),
	}
	for _, side := range nonvertical {
		if side.sectionIndex < 0 || side.sectionIndex >= len(manifest.sideOutcome) ||
			manifest.sideOutcome[side.sectionIndex] != revolveLoftFacesSideRefusal ||
			len(side.cells) == 0 {
			return nil, false, nil
		}
		manifest.sideOutcome[side.sectionIndex] = revolveLoftFacesSideSourceCandidate
	}
	for index, span := range proof.section.Spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if span.Face == nil {
			continue // the authenticated Revolve axis has no boundary face
		}
		if manifest.sideOutcome[index] != revolveLoftFacesSideRefusal {
			continue
		}
		if span.Start.U != span.End.U {
			return nil, false, fmt.Errorf("%w: finite Revolve side %d has no source cells",
				ErrUnsupported, index)
		}
		side, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, index)
		if err != nil {
			return nil, false, err
		}
		if outcome == revolveloftfaces.VerticalRefusal ||
			outcome == revolveloftfaces.VerticalContact && side == nil ||
			outcome == revolveloftfaces.VerticalMiss && side != nil {
			return nil, false, fmt.Errorf("%w: vertical Revolve side %d is undecidable",
				ErrUnsupported, index)
		}
		manifest.vertical = append(manifest.vertical, revolveLoftFacesVerticalEntry{
			sectionIndex: index, outcome: outcome, side: side})
		if outcome == revolveloftfaces.VerticalMiss {
			manifest.sideOutcome[index] = revolveLoftFacesSideMiss
		} else {
			manifest.sideOutcome[index] = revolveLoftFacesSideSourceCandidate
		}
	}
	return manifest, true, nil
}

// A section walk cannot describe the whole blank while a live face is
// omitted or assigned to more than one source side.
func revolveLoftFacesSectionFacesComplete(proof *revolveLoftFacesSourceCertificate) bool {
	if proof == nil || proof.blank == nil {
		return false
	}
	faces := proof.blank.Faces()
	if len(faces) == 0 {
		return false
	}
	seen := make(map[*topo.Face]bool, len(faces))
	for _, span := range proof.section.Spans {
		if span.Face == nil {
			continue // the Revolve axis has no boundary face
		}
		if span.Face.Body != bodyNode(proof.blank) || seen[span.Face] {
			return false
		}
		seen[span.Face] = true
	}
	if len(seen) != len(faces) {
		return false
	}
	for _, face := range faces {
		if face == nil || face.n.Body != bodyNode(proof.blank) || !seen[faceNode(face)] {
			return false
		}
	}
	return true
}

// The source record is rebuilt before shell construction. A copied face
// label with different source cells cannot select a retained side or seam.
func revolveLoftFacesSameShellManifest(a, b *revolveLoftFacesContactManifest) bool {
	if a == nil || b == nil || !slices.Equal(a.sideOutcome, b.sideOutcome) ||
		len(a.nonvertical) != len(b.nonvertical) || len(a.vertical) != len(b.vertical) {
		return false
	}
	for i, side := range a.nonvertical {
		other := b.nonvertical[i]
		if side.sectionIndex != other.sectionIndex || side.face != other.face ||
			!slices.Equal(side.origins, other.origins) || len(side.cells) != len(other.cells) {
			return false
		}
		for j, cell := range side.cells {
			match := other.cells[j]
			if cell.ref != match.ref || cell.face != match.face ||
				!revolveloftfaces.SameMappedCell(cell.mapped, match.mapped) {
				return false
			}
		}
	}
	for i, entry := range a.vertical {
		other := b.vertical[i]
		if entry.sectionIndex != other.sectionIndex || entry.outcome != other.outcome ||
			!revolveLoftFacesSameVerticalSide(entry.side, other.side) {
			return false
		}
	}
	return true
}

func revolveLoftFacesSameVerticalSide(a, b *revolveLoftFacesVerticalSide) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	if a.revolveLoftFacesConstantXSource == nil || b.revolveLoftFacesConstantXSource == nil {
		return false
	}
	if a.face != b.face || !slices.Equal(a.origins, b.origins) ||
		a.band != b.band || a.hasRetained != b.hasRetained ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.time, b.time) ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.x, b.x) ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.lower, b.lower) ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.upper, b.upper) ||
		!loftface.AffineEqual(a.yzAffine, b.yzAffine) ||
		len(a.cells) != len(b.cells) {
		return false
	}
	for i, cell := range a.cells {
		other := b.cells[i]
		if cell.segment != other.segment || cell.span != other.span ||
			cell.band != other.band || cell.ref != other.ref || cell.face != other.face ||
			!revolveloftfaces.RevolveLoftFacesSameBezier(cell.source, other.source) ||
			!loftface.LimitEqual(cell.lower, other.lower) ||
			!loftface.LimitEqual(cell.upper, other.upper) ||
			len(cell.retained) != len(other.retained) {
			return false
		}
		for j, piece := range cell.retained {
			match := other.retained[j]
			if !loftface.IntervalEqual(piece.Source, match.Source) ||
				piece.LoOpen != match.LoOpen || piece.HiOpen != match.HiOpen ||
				!revolveloftfaces.CapBoundaryBoxEqual(piece.Box, match.Box) {
				return false
			}
		}
	}
	return true
}
