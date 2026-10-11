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
	if a.sectionDelta != 0 || b.sectionDelta != 0 ||
		a.z0Delta != 0 || a.z1Delta != 0 || b.z0Delta != 0 || b.z1Delta != 0 {
		return false
	}
	ra, aok := axisRectangle(a.profile)
	rb, bok := axisRectangle(b.profile)
	if !aok || !bok {
		return false
	}
	switch op {
	case meshbool.OpUnion, meshbool.OpIntersect:
		return ra == rb
	case meshbool.OpCut:
		return ra.u0 < rb.u0 && rb.u1 < ra.u1 && ra.v0 < rb.v0 && rb.v1 < ra.v1
	default:
		return false
	}
}

type axisRect struct{ u0, v0, u1, v1 float64 }

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
	r := axisRect{u0: lines[0].Start.U, u1: lines[0].Start.U,
		v0: lines[0].Start.V, v1: lines[0].Start.V}
	for _, line := range lines {
		for _, point := range [...]sectionrecord.Point2{line.Start, line.End} {
			r.u0, r.u1 = min(r.u0, point.U), max(r.u1, point.U)
			r.v0, r.v1 = min(r.v0, point.V), max(r.v1, point.V)
		}
	}
	if r.u0 == r.u1 || r.v0 == r.v1 {
		return axisRect{}, false
	}
	var sides uint8
	for _, line := range lines {
		s, e := line.Start, line.End
		switch {
		case s.U == r.u0 && e.U == r.u0 &&
			((s.V == r.v0 && e.V == r.v1) || (s.V == r.v1 && e.V == r.v0)):
			sides |= 1
		case s.U == r.u1 && e.U == r.u1 &&
			((s.V == r.v0 && e.V == r.v1) || (s.V == r.v1 && e.V == r.v0)):
			sides |= 2
		case s.V == r.v0 && e.V == r.v0 &&
			((s.U == r.u0 && e.U == r.u1) || (s.U == r.u1 && e.U == r.u0)):
			sides |= 4
		case s.V == r.v1 && e.V == r.v1 &&
			((s.U == r.u0 && e.U == r.u1) || (s.U == r.u1 && e.U == r.u0)):
			sides |= 8
		default:
			return axisRect{}, false
		}
	}
	return r, sides == 15
}
