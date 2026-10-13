package decad

import "github.com/lestrrat-3d/decad/internal/loftface"

// A constant-X source section is private mutable data. Compare every field
// with a fresh classification before its root boxes become cap endpoints.
func revolveLoftFacesToolCapSourceCellsEqual(a, b *revolveLoftFacesConstantXSource) bool {
	if a == nil || b == nil || a.band != b.band ||
		!loftface.RatEqual(a.time, b.time) ||
		!loftface.RatEqual(a.x, b.x) ||
		!loftface.RatEqual(a.lower, b.lower) ||
		!loftface.RatEqual(a.upper, b.upper) ||
		a.hasRetained != b.hasRetained || len(a.cells) != len(b.cells) ||
		(a.cells == nil) != (b.cells == nil) ||
		!loftface.AffineEqual(a.yzAffine, b.yzAffine) {
		return false
	}
	for i := range a.cells {
		if !revolveLoftFacesToolCapCellEqual(a.cells[i], b.cells[i]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapCellEqual(a, b revolveLoftFacesVerticalCell) bool {
	if a.segment != b.segment || a.span != b.span || a.band != b.band ||
		a.ref != b.ref || a.face != b.face || len(a.source) != len(b.source) ||
		(a.source == nil) != (b.source == nil) ||
		len(a.retained) != len(b.retained) ||
		(a.retained == nil) != (b.retained == nil) ||
		!loftface.LimitEqual(a.lower, b.lower) ||
		!loftface.LimitEqual(a.upper, b.upper) {
		return false
	}
	for i := range a.source {
		if !loftface.RatEqual(a.source[i].U, b.source[i].U) ||
			!loftface.RatEqual(a.source[i].V, b.source[i].V) {
			return false
		}
	}
	for i := range a.retained {
		if !loftface.PieceEqual(a.retained[i], b.retained[i]) {
			return false
		}
	}
	return true
}
