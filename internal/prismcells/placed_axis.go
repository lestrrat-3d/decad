package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
)

// PlacedAxisProfileAdmitted is the additional sufficient section certificate
// for prisms whose stored transforms differ only by an axial translation.
// The pre-existing same-transform shared-axis arm does not use this gate.
func PlacedAxisProfileAdmitted(op meshbool.OperationKind, a, b prismrecord.Payload) bool {
	if a.SectionDelta != 0 || b.SectionDelta != 0 ||
		a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 {
		return false
	}
	ra, aok := momentinput.AxisRectangle(a.Profile)
	rb, bok := momentinput.AxisRectangle(b.Profile)
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
