package decad

// A constant-X source section is private mutable data. Compare every field
// with a fresh classification before its root boxes become cap endpoints.
func revolveLoftFacesToolCapSourceCellsEqual(a, b *revolveLoftFacesConstantXSource) bool {
	if a == nil || b == nil || a.band != b.band ||
		!revolveLoftFacesToolCapRatEqual(a.time, b.time) ||
		!revolveLoftFacesToolCapRatEqual(a.x, b.x) ||
		!revolveLoftFacesToolCapRatEqual(a.lower, b.lower) ||
		!revolveLoftFacesToolCapRatEqual(a.upper, b.upper) ||
		a.hasRetained != b.hasRetained || len(a.cells) != len(b.cells) ||
		!revolveLoftFacesToolCapAffineEqual(a.yzAffine, b.yzAffine) {
		return false
	}
	for i := range a.cells {
		if !revolveLoftFacesToolCapCellEqual(a.cells[i], b.cells[i]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapAffineEqual(a, b *revolveLoftFacesVerticalAffine) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	if !revolveLoftFacesToolCapRatEqual(a.det, b.det) {
		return false
	}
	for axis := range a.offset {
		if !revolveLoftFacesToolCapRatEqual(a.offset[axis], b.offset[axis]) {
			return false
		}
	}
	for sourceAxis := range a.column {
		for worldAxis := range a.column[sourceAxis] {
			if !revolveLoftFacesToolCapRatEqual(a.column[sourceAxis][worldAxis],
				b.column[sourceAxis][worldAxis]) {
				return false
			}
		}
	}
	return true
}

func revolveLoftFacesToolCapCellEqual(a, b revolveLoftFacesVerticalCell) bool {
	if a.segment != b.segment || a.span != b.span || a.band != b.band ||
		a.ref != b.ref || a.face != b.face || len(a.source) != len(b.source) ||
		len(a.retained) != len(b.retained) ||
		!revolveLoftFacesToolCapLimitEqual(a.lower, b.lower) ||
		!revolveLoftFacesToolCapLimitEqual(a.upper, b.upper) {
		return false
	}
	for i := range a.source {
		if !revolveLoftFacesToolCapRatEqual(a.source[i].U, b.source[i].U) ||
			!revolveLoftFacesToolCapRatEqual(a.source[i].V, b.source[i].V) {
			return false
		}
	}
	for i := range a.retained {
		if !revolveLoftFacesToolCapPieceEqual(a.retained[i], b.retained[i]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapLimitEqual(a, b revolveLoftFacesVerticalLimit) bool {
	if a.side != b.side || a.before != b.before || a.after != b.after ||
		!revolveLoftFacesToolCapIntervalEqual(a.root, b.root) {
		return false
	}
	for axis := range a.box {
		if !revolveLoftFacesToolCapIntervalEqual(a.box[axis], b.box[axis]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapPieceEqual(a, b revolveLoftFacesVerticalPiece) bool {
	if a.loOpen != b.loOpen || a.hiOpen != b.hiOpen ||
		!revolveLoftFacesToolCapIntervalEqual(a.source, b.source) {
		return false
	}
	for axis := range a.box {
		if !revolveLoftFacesToolCapIntervalEqual(a.box[axis], b.box[axis]) {
			return false
		}
	}
	return true
}
