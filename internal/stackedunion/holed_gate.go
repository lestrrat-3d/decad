package stackedunion

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// gatedArrangementSegments is the arrangement cap of a holed union that
// passed holedCandidate: twice prismcells.MaxArrangementSegments. It applies
// to every scene of that build.
//
// Measured arrangement times (CellProfiles): the spoked-disc Ø14 slab scene
// (6,664 counted segments) takes 3.5 to 4.3 ms; a plate with 31 apart
// circular holes and a circle (8,196 counted) takes 10 ms; 32 mutually
// crossing circles (8,192 counted, 977 cells) take 59 ms. The cap therefore
// bounds a gated arrangement near 60 ms.
const gatedArrangementSegments = 2 * prismcells.MaxArrangementSegments

// hasHoledRecord reports whether any region of any slab of either operand has
// a hole.
func (st *stackedUnionState) hasHoledRecord() bool {
	for _, op := range []Operand{st.va, st.vb} {
		for _, slab := range op.Slabs {
			for _, region := range slab.Regions {
				if len(region.Holes) != 0 {
					return true
				}
			}
		}
	}
	return false
}

// holedCandidate is the reject-only record pre-filter of a holed union. It
// reports whether some hole segment of a region of one operand and some outer
// segment of a region of the other operand lie on one carrier kind within a
// loose band: circles and arcs whose centre distance plus radius difference
// is at most 1e-9·max(1, radius), lines whose both endpoints (the hole
// line's) lie within 1e-9·max(1, |coordinates|) of the outer line's infinite
// line. It computes in decad for cost alone, sparing the arrangement for
// pairs that cannot share a span; the band only rejects, and sketch's span
// decides admission.
func holedCandidate(va, vb Operand) bool {
	return holesMeetOuters(va, vb) || holesMeetOuters(vb, va)
}

// holesMeetOuters reports a candidate pair between a hole segment of holed
// and an outer segment of other.
func holesMeetOuters(holed, other Operand) bool {
	var holes, outers []sectionrecord.CurveSegment
	for _, slab := range holed.Slabs {
		for _, region := range slab.Regions {
			for _, hole := range region.Holes {
				holes = append(holes, hole.Segments...)
			}
		}
	}
	if len(holes) == 0 {
		return false
	}
	for _, slab := range other.Slabs {
		for _, region := range slab.Regions {
			outers = append(outers, region.Outer.Segments...)
		}
	}
	for _, h := range holes {
		for _, o := range outers {
			if looseSameCarrier(h, o) {
				return true
			}
		}
	}
	return false
}

// looseSameCarrier is the band test for one hole segment and one outer
// segment.
func looseSameCarrier(hole, outer sectionrecord.CurveSegment) bool {
	hl, hLine := hole.(sectionrecord.LineSeg)
	ol, oLine := outer.(sectionrecord.LineSeg)
	if hLine || oLine {
		return hLine && oLine && looseSameLine(hl, ol)
	}
	hw, err := boundarywalk.WalkOf(hole, nil)
	if err != nil || !hw.IsCircular() {
		return false
	}
	ow, err := boundarywalk.WalkOf(outer, nil)
	if err != nil || !ow.IsCircular() {
		return false
	}
	scale := math.Max(1, math.Max(hw.Radius, ow.Radius))
	return math.Hypot(hw.CU-ow.CU, hw.CV-ow.CV)+math.Abs(hw.Radius-ow.Radius) <= 1e-9*scale
}

// looseSameLine reports whether both endpoints of hole lie within the band of
// the infinite line through outer.
func looseSameLine(hole, outer sectionrecord.LineSeg) bool {
	dx, dy := outer.End.U-outer.Start.U, outer.End.V-outer.Start.V
	length := math.Hypot(dx, dy)
	if !(length > 0) {
		return false
	}
	scale := 1.0
	for _, v := range []float64{outer.Start.U, outer.Start.V, outer.End.U, outer.End.V,
		hole.Start.U, hole.Start.V, hole.End.U, hole.End.V} {
		scale = math.Max(scale, math.Abs(v))
	}
	for _, p := range []sectionrecord.Point2{hole.Start, hole.End} {
		dist := math.Abs(dx*(p.V-outer.Start.V)-dy*(p.U-outer.Start.U)) / length
		if !(dist <= 1e-9*scale) {
			return false
		}
	}
	return true
}
