package decad

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// placedAxisProfileAdmitted is the additional sufficient section certificate
// for prisms whose stored transforms differ only by an axial translation.
// The pre-existing same-transform shared-axis arm does not use this gate.
func placedAxisProfileAdmitted(op meshbool.OperationKind, a, b prismPayload) bool {
	if a.SectionDelta != 0 || b.SectionDelta != 0 ||
		a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 {
		return false
	}
	ra, aok := axisRectangle(a.Profile)
	rb, bok := axisRectangle(b.Profile)
	if !aok || !bok {
		return false
	}
	switch op {
	case meshbool.OpUnion, meshbool.OpIntersect:
		return ra == rb
	case meshbool.OpCut:
		return ra.U0 < rb.U0 && rb.U1 < ra.U1 && ra.V0 < rb.V0 && rb.V1 < ra.V1
	default:
		return false
	}
}

type axisRect struct{ U0, V0, U1, V1 float64 }

// axisRectangle accepts only four complete line records, one on each side
// of a nondegenerate axis-aligned rectangle. Profile validation already proves
// a closed outer walk; this check proves its precise rectangular footprint.
func axisRectangle(p profileRecord) (axisRect, bool) {
	if len(p.Holes) != 0 || len(p.Outer.Segments) != 4 {
		return axisRect{}, false
	}
	var lines [4]lineSeg
	for i, segment := range p.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return axisRect{}, false
		}
		for _, value := range []float64{line.Start.U, line.Start.V, line.End.U, line.End.V} {
			if math.IsNaN(value) || math.IsInf(value, 0) {
				return axisRect{}, false
			}
		}
		lines[i] = line
	}
	r := axisRect{U0: lines[0].Start.U, U1: lines[0].Start.U,
		V0: lines[0].Start.V, V1: lines[0].Start.V}
	for _, line := range lines {
		for _, point := range [...]sectionrecord.Point2{line.Start, line.End} {
			r.U0, r.U1 = min(r.U0, point.U), max(r.U1, point.U)
			r.V0, r.V1 = min(r.V0, point.V), max(r.V1, point.V)
		}
	}
	if r.U0 == r.U1 || r.V0 == r.V1 {
		return axisRect{}, false
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
			return axisRect{}, false
		}
	}
	return r, sides == 15
}
