package revolvecontact

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// This is a source-only record. A mapped cell names its complete Sketch
// entity, Bezier span, LoftFaces band, source parameter interval and band-time
// interval. Contact cells also name the exact live endpoint that owns them.
// Tool clipping and held-switch construction are separate certificates.
type SourceCell struct {
	Mapped revolveloftfaces.RevolveLoftFacesMappedCell
	Ref    provenance.Ref
	Face   *topo.Face
}

type NonverticalSide struct {
	SectionIndex int
	Face         *topo.Face
	Origins      []provenance.Ref
	Cells        []SourceCell
}

func NonverticalManifest(ctx context.Context,
	proof *SourceCertificate) ([]NonverticalSide, bool, error) {
	if ctx == nil || proof == nil || proof.Blank == nil || proof.Tooth == nil || proof.Trim == nil ||
		proof.Section.Blank != proof.Blank || len(proof.Section.Spans) == 0 ||
		len(proof.SourceRefs) != len(proof.Trim.Maps)-1 {
		return nil, false, nil
	}
	p, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || p.LoftFacesTrim != proof.Trim ||
		!TrimCurrent(proof.Tooth, proof.Trim) ||
		!facetedrecord.PointSourceCurrent(proof.Trim.Source) {
		return nil, false, nil
	}
	liveRefs, ok := SourceRefs(proof.Trim, proof.Tooth)
	if !ok || len(liveRefs) != len(proof.SourceRefs) {
		return nil, false, nil
	}
	for band := range liveRefs {
		if !slices.Equal(liveRefs[band], proof.SourceRefs[band]) {
			return nil, false, nil
		}
	}
	current := false
	for _, candidate := range revolveshell.RootCandidates(proof.Blank) {
		if candidate.RootSpan != proof.Section.RootSpan ||
			len(candidate.Spans) != len(proof.Section.Spans) {
			continue
		}
		matches := true
		for i, span := range candidate.Spans {
			old := proof.Section.Spans[i]
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
	faces := topobuild.Faces(proof.Tooth)
	var manifest []NonverticalSide
	for sectionIndex, section := range proof.Section.Spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if section.Face == nil || section.Start.U == section.End.U ||
			section.Start.V <= 0 || section.End.V <= 0 {
			continue
		}
		mapped, ok, err := revolveloftfaces.RevolveLoftFacesFiniteSpanCells(ctx, proof.Trim, section)
		if err != nil {
			return nil, false, fmt.Errorf("finite Revolve source side %d: %w", sectionIndex, err)
		}
		if !ok || len(mapped) == 0 {
			return nil, false, fmt.Errorf("%w: finite Revolve source side %d has undecidable cells",
				decaderr.ErrUnsupported, sectionIndex)
		}
		entry := NonverticalSide{SectionIndex: sectionIndex,
			Face: section.Face, Origins: append([]provenance.Ref(nil), section.Origins...)}
		for _, cell := range mapped {
			if cell.Band < 0 || cell.Band >= len(proof.SourceRefs) ||
				cell.Segment < 0 || cell.Segment >= len(proof.SourceRefs[cell.Band]) ||
				cell.SLo == nil || cell.SHi == nil || cell.TLo == nil || cell.THi == nil ||
				cell.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
					(cell.JointKind == 0 ||
						cell.JointKind == revolveloftfaces.RevolveLoftFacesWholeArcContact && cell.JointArc == nil ||
						cell.JointKind != revolveloftfaces.RevolveLoftFacesWholeArcContact && cell.JointPoint == nil) {
				return nil, false, nil
			}
			ref := liveRefs[cell.Band][cell.Segment]
			index, valid := revolveloftfaces.RefFaceIndex(p, ref, len(faces))
			face, originValid := UniqueOriginFace(proof.Tooth, ref)
			if !valid || !originValid || (index >= 0) != (face != nil) ||
				index >= 0 && faces[index] != face {
				return nil, false, nil
			}
			if face == nil {
				continue // the authenticated source wall has no retained face
			}
			entry.Cells = append(entry.Cells, SourceCell{
				Mapped: cell, Ref: ref, Face: face})
		}
		manifest = append(manifest, entry)
	}
	return manifest, len(manifest) > 0, nil
}

// A contact manifest keeps the two finite Revolve-side proofs separate.
// Tool-cap circles and the held seam need their own later certificates.
type ContactManifest struct {
	Nonvertical []NonverticalSide
	Vertical    []VerticalEntry
	SideOutcome []SideOutcome // indexed by the authenticated Revolve section
}

type SideOutcome uint8

const (
	SideRefusal SideOutcome = iota
	// A retained source contour is present. Its solid contact and held seam
	// still need separate proof.
	SideSourceCandidate
	// A strict gap proves the complete finite Revolve face misses the solid.
	SideMiss
)

type VerticalEntry struct {
	SectionIndex int
	Outcome      revolveloftfaces.VerticalOutcome
	Side         *VerticalSide // present only for a contact
}

// Every live Revolve boundary face gets exactly one source-side answer.
// A vertical miss proves that complete blank face misses the trimmed solid;
// a contact retains its exact clipped source cells for seam construction.
func AllSideManifest(ctx context.Context,
	proof *SourceCertificate) (*ContactManifest, bool, error) {
	if ctx == nil || proof == nil {
		return nil, false, nil
	}
	if !SectionCurrent(proof) || !SectionFacesComplete(proof) {
		return nil, false, nil
	}
	nonvertical, ok, err := NonverticalManifest(ctx, proof)
	if err != nil || !ok {
		return nil, false, err
	}
	manifest := &ContactManifest{
		Nonvertical: nonvertical,
		SideOutcome: make([]SideOutcome, len(proof.Section.Spans)),
	}
	for _, side := range nonvertical {
		if side.SectionIndex < 0 || side.SectionIndex >= len(manifest.SideOutcome) ||
			manifest.SideOutcome[side.SectionIndex] != SideRefusal ||
			len(side.Cells) == 0 {
			return nil, false, nil
		}
		manifest.SideOutcome[side.SectionIndex] = SideSourceCandidate
	}
	for index, span := range proof.Section.Spans {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if span.Face == nil {
			continue // the authenticated Revolve axis has no boundary face
		}
		if manifest.SideOutcome[index] != SideRefusal {
			continue
		}
		if span.Start.U != span.End.U {
			return nil, false, fmt.Errorf("%w: finite Revolve side %d has no source cells",
				decaderr.ErrUnsupported, index)
		}
		side, outcome, err := VerticalSideClassify(ctx, proof, index)
		if err != nil {
			return nil, false, err
		}
		if outcome == revolveloftfaces.VerticalRefusal ||
			outcome == revolveloftfaces.VerticalContact && side == nil ||
			outcome == revolveloftfaces.VerticalMiss && side != nil {
			return nil, false, fmt.Errorf("%w: vertical Revolve side %d is undecidable",
				decaderr.ErrUnsupported, index)
		}
		manifest.Vertical = append(manifest.Vertical, VerticalEntry{
			SectionIndex: index, Outcome: outcome, Side: side})
		if outcome == revolveloftfaces.VerticalMiss {
			manifest.SideOutcome[index] = SideMiss
		} else {
			manifest.SideOutcome[index] = SideSourceCandidate
		}
	}
	return manifest, true, nil
}

// A section walk cannot describe the whole blank while a live face is
// omitted or assigned to more than one source side.
func SectionFacesComplete(proof *SourceCertificate) bool {
	if proof == nil || proof.Blank == nil {
		return false
	}
	faces := topobuild.Faces(proof.Blank)
	if len(faces) == 0 {
		return false
	}
	seen := make(map[*topo.Face]bool, len(faces))
	for _, span := range proof.Section.Spans {
		if span.Face == nil {
			continue // the Revolve axis has no boundary face
		}
		if span.Face.Body != proof.Blank || seen[span.Face] {
			return false
		}
		seen[span.Face] = true
	}
	if len(seen) != len(faces) {
		return false
	}
	for _, face := range faces {
		if face == nil || face.Body != proof.Blank || !seen[face] {
			return false
		}
	}
	return true
}

// The source record is rebuilt before shell construction. A copied face
// label with different source cells cannot select a retained side or seam.
func SameShellManifest(a, b *ContactManifest) bool {
	if a == nil || b == nil || !slices.Equal(a.SideOutcome, b.SideOutcome) ||
		len(a.Nonvertical) != len(b.Nonvertical) || len(a.Vertical) != len(b.Vertical) {
		return false
	}
	for i, side := range a.Nonvertical {
		other := b.Nonvertical[i]
		if side.SectionIndex != other.SectionIndex || side.Face != other.Face ||
			!slices.Equal(side.Origins, other.Origins) || len(side.Cells) != len(other.Cells) {
			return false
		}
		for j, cell := range side.Cells {
			match := other.Cells[j]
			if cell.Ref != match.Ref || cell.Face != match.Face ||
				!revolveloftfaces.SameMappedCell(cell.Mapped, match.Mapped) {
				return false
			}
		}
	}
	for i, entry := range a.Vertical {
		other := b.Vertical[i]
		if entry.SectionIndex != other.SectionIndex || entry.Outcome != other.Outcome ||
			!SameVerticalSide(entry.Side, other.Side) {
			return false
		}
	}
	return true
}

func SameVerticalSide(a, b *VerticalSide) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	if a.ConstantXSource == nil || b.ConstantXSource == nil {
		return false
	}
	if a.Face != b.Face || !slices.Equal(a.Origins, b.Origins) ||
		a.Band != b.Band || a.HasRetained != b.HasRetained ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.Time, b.Time) ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.X, b.X) ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.Lower, b.Lower) ||
		!revolveloftfaces.RevolveLoftFacesSameRat(a.Upper, b.Upper) ||
		!loftface.AffineEqual(a.YzAffine, b.YzAffine) ||
		len(a.Cells) != len(b.Cells) {
		return false
	}
	for i, cell := range a.Cells {
		other := b.Cells[i]
		if cell.Segment != other.Segment || cell.Span != other.Span ||
			cell.Band != other.Band || cell.Ref != other.Ref || cell.Face != other.Face ||
			!revolveloftfaces.RevolveLoftFacesSameBezier(cell.Source, other.Source) ||
			!loftface.LimitEqual(cell.Lower, other.Lower) ||
			!loftface.LimitEqual(cell.Upper, other.Upper) ||
			len(cell.Retained) != len(other.Retained) {
			return false
		}
		for j, piece := range cell.Retained {
			match := other.Retained[j]
			if !loftface.IntervalEqual(piece.Source, match.Source) ||
				piece.LoOpen != match.LoOpen || piece.HiOpen != match.HiOpen ||
				!revolveloftfaces.CapBoundaryBoxEqual(piece.Box, match.Box) {
				return false
			}
		}
	}
	return true
}
