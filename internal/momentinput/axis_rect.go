package momentinput

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// AxisRect is the exact footprint AxisRectangle certifies: the recorded
// rectangle's least and greatest u and v.
type AxisRect struct{ U0, V0, U1, V1 float64 }

// CrossingBlindLoops accepts profiles whose loops hold only axis-aligned line,
// circle and arc records, for the crossing-blind Cut route
// (agent-docs/general-boolean-design.md §3 A7). An oblique line, an ellipse or a
// free-form segment is refused: the A1 BRep build states no exact crossing for
// it. Whether two circles cross, and whether each crossing is certified, is the
// private scene's answer, read later.
func CrossingBlindLoops(p Profile) bool {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, segment := range loop.Segments {
			switch s := segment.(type) {
			case sectionrecord.LineSeg:
				if s.Start.U != s.End.U && s.Start.V != s.End.V {
					return false
				}
			case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
			default:
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

// PrismUnionFilletRadiusFits checks that a radius fits both rectangle spans.
func PrismUnionFilletRadiusFits(rect AxisRect, radius float64) bool {
	r := proofarith.FloatRat(radius)
	twice := new(big.Rat).Mul(r, big.NewRat(2, 1))
	width := new(big.Rat).Sub(proofarith.FloatRat(rect.U1), proofarith.FloatRat(rect.U0))
	height := new(big.Rat).Sub(proofarith.FloatRat(rect.V1), proofarith.FloatRat(rect.V0))
	return twice.Cmp(width) < 0 && twice.Cmp(height) < 0
}

// PrismUnionFilletExactShift checks the exact inward corner coordinate.
func PrismUnionFilletExactShift(c, radius float64, direction int) (float64, bool) {
	sum := new(big.Rat).Set(proofarith.FloatRat(c))
	if direction > 0 {
		sum.Add(sum, proofarith.FloatRat(radius))
	} else {
		sum.Sub(sum, proofarith.FloatRat(radius))
	}
	held, _ := sum.Float64()
	return held, !proofbound.IsNonFinite(held) && proofarith.FloatRat(held).Cmp(sum) == 0
}
