package decad

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
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

func revolveLoftFacesSameRat(a, b *big.Rat) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	return a.Cmp(b) == 0
}

func revolveLoftFacesSameInterval(a, b loftFaceInterval) bool {
	return revolveLoftFacesSameRat(a.lo, b.lo) && revolveLoftFacesSameRat(a.hi, b.hi)
}

func revolveLoftFacesSameBox(a, b [3]loftFaceInterval) bool {
	for axis := range a {
		if !revolveLoftFacesSameInterval(a[axis], b[axis]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesSameCrossing(a, b spiralRootCrossing) bool {
	return a.span == b.span && revolveLoftFacesSameRat(a.lo, b.lo) &&
		revolveLoftFacesSameRat(a.hi, b.hi)
}

func revolveLoftFacesSameMappedCell(a, b revolveLoftFacesMappedCell) bool {
	return a.segment == b.segment && a.span == b.span && a.band == b.band &&
		a.side == b.side && a.jointKind == b.jointKind &&
		a.jointPoint == b.jointPoint && a.jointArc == b.jointArc &&
		revolveLoftFacesSameCrossing(a.first, b.first) &&
		revolveLoftFacesSameCrossing(a.last, b.last) &&
		revolveLoftFacesSameInterval(a.xFirst, b.xFirst) &&
		revolveLoftFacesSameInterval(a.xLast, b.xLast) &&
		a.xDirection == b.xDirection &&
		revolveLoftFacesSameRat(a.sLo, b.sLo) &&
		revolveLoftFacesSameRat(a.sHi, b.sHi) &&
		revolveLoftFacesSameRat(a.tLo, b.tLo) &&
		revolveLoftFacesSameRat(a.tHi, b.tHi) &&
		a.tLoOpen == b.tLoOpen && a.tHiOpen == b.tHiOpen
}

func revolveLoftFacesSameBezier(a, b freeform.BezierSpan) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !revolveLoftFacesSameRat(a[i].U, b[i].U) ||
			!revolveLoftFacesSameRat(a[i].V, b[i].V) {
			return false
		}
	}
	return true
}

func revolveLoftFacesSameVerticalLimit(a, b revolveLoftFacesVerticalLimit) bool {
	return a.side == b.side && a.before == b.before && a.after == b.after &&
		revolveLoftFacesSameInterval(a.root, b.root) &&
		revolveLoftFacesSameBox(a.box, b.box)
}

func revolveLoftFacesSameVerticalAffine(a, b *revolveLoftFacesVerticalAffine) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	for i := range a.offset {
		if !revolveLoftFacesSameRat(a.offset[i], b.offset[i]) {
			return false
		}
		for j := range a.column[i] {
			if !revolveLoftFacesSameRat(a.column[i][j], b.column[i][j]) {
				return false
			}
		}
	}
	return revolveLoftFacesSameRat(a.det, b.det)
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
		!revolveLoftFacesSameRat(a.time, b.time) ||
		!revolveLoftFacesSameRat(a.x, b.x) ||
		!revolveLoftFacesSameRat(a.lower, b.lower) ||
		!revolveLoftFacesSameRat(a.upper, b.upper) ||
		!revolveLoftFacesSameVerticalAffine(a.yzAffine, b.yzAffine) ||
		len(a.cells) != len(b.cells) {
		return false
	}
	for i, cell := range a.cells {
		other := b.cells[i]
		if cell.segment != other.segment || cell.span != other.span ||
			cell.band != other.band || cell.ref != other.ref || cell.face != other.face ||
			!revolveLoftFacesSameBezier(cell.source, other.source) ||
			!revolveLoftFacesSameVerticalLimit(cell.lower, other.lower) ||
			!revolveLoftFacesSameVerticalLimit(cell.upper, other.upper) ||
			len(cell.retained) != len(other.retained) {
			return false
		}
		for j, piece := range cell.retained {
			match := other.retained[j]
			if !revolveLoftFacesSameInterval(piece.source, match.source) ||
				piece.loOpen != match.loOpen || piece.hiOpen != match.hiOpen ||
				!revolveLoftFacesSameBox(piece.box, match.box) {
				return false
			}
		}
	}
	return true
}
