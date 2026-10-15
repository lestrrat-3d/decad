package decad

import (
	"context"
	"fmt"
)

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
	outcome      revolveLoftFacesVerticalOutcome
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
		if outcome == revolveLoftFacesVerticalRefusal ||
			outcome == revolveLoftFacesVerticalContact && side == nil ||
			outcome == revolveLoftFacesVerticalMiss && side != nil {
			return nil, false, fmt.Errorf("%w: vertical Revolve side %d is undecidable",
				ErrUnsupported, index)
		}
		manifest.vertical = append(manifest.vertical, revolveLoftFacesVerticalEntry{
			sectionIndex: index, outcome: outcome, side: side})
		if outcome == revolveLoftFacesVerticalMiss {
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
	seen := make(map[*Face]bool, len(faces))
	for _, span := range proof.section.Spans {
		if span.Face == nil {
			continue // the Revolve axis has no boundary face
		}
		if span.Face.n.Body != bodyNode(proof.blank) || seen[span.Face] {
			return false
		}
		seen[span.Face] = true
	}
	if len(seen) != len(faces) {
		return false
	}
	for _, face := range faces {
		if face == nil || face.n.Body != bodyNode(proof.blank) || !seen[face] {
			return false
		}
	}
	return true
}
