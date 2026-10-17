package momentinput

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// AxisRect is the exact footprint AxisRectangle certifies: the recorded
// rectangle's least and greatest u and v.
type AxisRect struct{ U0, V0, U1, V1 float64 }

// CrossingBlindAxisLines accepts profiles whose loops contain exactly four
// axis-aligned line records for the crossing-blind Cut route.
func CrossingBlindAxisLines(p Profile) bool {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		if len(loop.Segments) != 4 {
			return false
		}
		for _, segment := range loop.Segments {
			line, ok := segment.(sectionrecord.LineSeg)
			if !ok || (line.Start.U != line.End.U && line.Start.V != line.End.V) {
				return false
			}
		}
	}
	return true
}

// AxisRectangle accepts only four complete line records, one on each side
// of a nondegenerate axis-aligned rectangle. Profile validation already proves
// a closed outer walk; this check proves its precise rectangular footprint.
func AxisRectangle(p Profile) (AxisRect, bool) {
	if len(p.Holes) != 0 || len(p.Outer.Segments) != 4 {
		return AxisRect{}, false
	}
	var lines [4]LineSeg
	for i, segment := range p.Outer.Segments {
		line, ok := segment.(LineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return AxisRect{}, false
		}
		for _, value := range []float64{line.Start.U, line.Start.V, line.End.U, line.End.V} {
			if math.IsNaN(value) || math.IsInf(value, 0) {
				return AxisRect{}, false
			}
		}
		lines[i] = line
	}
	r := AxisRect{U0: lines[0].Start.U, U1: lines[0].Start.U,
		V0: lines[0].Start.V, V1: lines[0].Start.V}
	for _, line := range lines {
		for _, point := range [...]sectionrecord.Point2{line.Start, line.End} {
			r.U0, r.U1 = min(r.U0, point.U), max(r.U1, point.U)
			r.V0, r.V1 = min(r.V0, point.V), max(r.V1, point.V)
		}
	}
	if r.U0 == r.U1 || r.V0 == r.V1 {
		return AxisRect{}, false
	}
	var sides uint8
	for _, line := range lines {
		s, e := line.Start, line.End
		switch {
		case s.U == r.U0 && e.U == r.U0 &&
			((s.V == r.V0 && e.V == r.V1) || (s.V == r.V1 && e.V == r.V0)):
			sides |= 1
		case s.U == r.U1 && e.U == r.U1 &&
			((s.V == r.V0 && e.V == r.V1) || (s.V == r.V1 && e.V == r.V0)):
			sides |= 2
		case s.V == r.V0 && e.V == r.V0 &&
			((s.U == r.U0 && e.U == r.U1) || (s.U == r.U1 && e.U == r.U0)):
			sides |= 4
		case s.V == r.V1 && e.V == r.V1 &&
			((s.U == r.U0 && e.U == r.U1) || (s.U == r.U1 && e.U == r.U0)):
			sides |= 8
		default:
			return AxisRect{}, false
		}
	}
	return r, sides == 15
}
