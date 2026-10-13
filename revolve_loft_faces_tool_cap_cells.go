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
		(a.cells == nil) != (b.cells == nil) ||
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
	if !revolveLoftFacesToolCapRatEqual(a.Det, b.Det) {
		return false
	}
	for axis := range a.Offset {
		if !revolveLoftFacesToolCapRatEqual(a.Offset[axis], b.Offset[axis]) {
			return false
		}
	}
	for sourceAxis := range a.Column {
		for worldAxis := range a.Column[sourceAxis] {
			if !revolveLoftFacesToolCapRatEqual(a.Column[sourceAxis][worldAxis],
				b.Column[sourceAxis][worldAxis]) {
				return false
			}
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
	if a.Side != b.Side || a.Before != b.Before || a.After != b.After ||
		!revolveLoftFacesToolCapIntervalEqual(a.Root, b.Root) {
		return false
	}
	for axis := range a.Box {
		if !revolveLoftFacesToolCapIntervalEqual(a.Box[axis], b.Box[axis]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapPieceEqual(a, b revolveLoftFacesVerticalPiece) bool {
	if a.LoOpen != b.LoOpen || a.HiOpen != b.HiOpen ||
		!revolveLoftFacesToolCapIntervalEqual(a.Source, b.Source) {
		return false
	}
	for axis := range a.Box {
		if !revolveLoftFacesToolCapIntervalEqual(a.Box[axis], b.Box[axis]) {
			return false
		}
	}
	return true
}
