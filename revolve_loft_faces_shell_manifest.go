package decad

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

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
				!revolveLoftFacesSameMappedCell(cell.mapped, match.mapped) {
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

func revolveLoftFacesSameInterval(a, b loftFaceInterval) bool {
	return revolveloftfaces.RevolveLoftFacesSameRat(a.Lo, b.Lo) && revolveloftfaces.RevolveLoftFacesSameRat(a.Hi, b.Hi)
}

func revolveLoftFacesSameBox(a, b [3]loftFaceInterval) bool {
	for axis := range a {
		if !revolveLoftFacesSameInterval(a[axis], b[axis]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesSameCrossing(a, b loftface.RootCrossing) bool {
	return a.Span == b.Span && revolveloftfaces.RevolveLoftFacesSameRat(a.Lo, b.Lo) &&
		revolveloftfaces.RevolveLoftFacesSameRat(a.Hi, b.Hi)
}

func revolveLoftFacesSameMappedCell(a, b revolveloftfaces.RevolveLoftFacesMappedCell) bool {
	return a.Segment == b.Segment && a.Span == b.Span && a.Band == b.Band &&
		a.Side == b.Side && a.JointKind == b.JointKind &&
		a.JointPoint == b.JointPoint && a.JointArc == b.JointArc &&
		revolveLoftFacesSameCrossing(a.First, b.First) &&
		revolveLoftFacesSameCrossing(a.Last, b.Last) &&
		revolveLoftFacesSameInterval(a.XFirst, b.XFirst) &&
		revolveLoftFacesSameInterval(a.XLast, b.XLast) &&
		a.XDirection == b.XDirection &&
		revolveloftfaces.RevolveLoftFacesSameRat(a.SLo, b.SLo) &&
		revolveloftfaces.RevolveLoftFacesSameRat(a.SHi, b.SHi) &&
		revolveloftfaces.RevolveLoftFacesSameRat(a.TLo, b.TLo) &&
		revolveloftfaces.RevolveLoftFacesSameRat(a.THi, b.THi) &&
		a.TLoOpen == b.TLoOpen && a.THiOpen == b.THiOpen
}

func revolveLoftFacesSameVerticalLimit(a, b loftface.VerticalLimit) bool {
	return a.Side == b.Side && a.Before == b.Before && a.After == b.After &&
		revolveLoftFacesSameInterval(a.Root, b.Root) &&
		revolveLoftFacesSameBox(a.Box, b.Box)
}

func revolveLoftFacesSameVerticalAffine(a, b *loftface.VerticalAffine) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	for i := range a.Offset {
		if !revolveloftfaces.RevolveLoftFacesSameRat(a.Offset[i], b.Offset[i]) {
			return false
		}
		for j := range a.Column[i] {
			if !revolveloftfaces.RevolveLoftFacesSameRat(a.Column[i][j], b.Column[i][j]) {
				return false
			}
		}
	}
	return revolveloftfaces.RevolveLoftFacesSameRat(a.Det, b.Det)
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
		!revolveLoftFacesSameVerticalAffine(a.yzAffine, b.yzAffine) ||
		len(a.cells) != len(b.cells) {
		return false
	}
	for i, cell := range a.cells {
		other := b.cells[i]
		if cell.segment != other.segment || cell.span != other.span ||
			cell.band != other.band || cell.ref != other.ref || cell.face != other.face ||
			!revolveloftfaces.RevolveLoftFacesSameBezier(cell.source, other.source) ||
			!revolveLoftFacesSameVerticalLimit(cell.lower, other.lower) ||
			!revolveLoftFacesSameVerticalLimit(cell.upper, other.upper) ||
			len(cell.retained) != len(other.retained) {
			return false
		}
		for j, piece := range cell.retained {
			match := other.retained[j]
			if !revolveLoftFacesSameInterval(piece.Source, match.Source) ||
				piece.LoOpen != match.LoOpen || piece.HiOpen != match.HiOpen ||
				!revolveLoftFacesSameBox(piece.Box, match.Box) {
				return false
			}
		}
	}
	return true
}
