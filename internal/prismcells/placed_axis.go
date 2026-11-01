package prismcells

import (
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PlacedAxisProfileAdmitted is the additional sufficient section certificate
// for prisms whose stored transforms differ only by an axial translation.
// The pre-existing same-transform shared-axis arm does not use this gate.
func PlacedAxisProfileAdmitted(budget *proofbound.WorkBudget, op meshbool.OperationKind, a, b prismrecord.Payload) (bool, error) {
	if a.SectionDelta != 0 || b.SectionDelta != 0 ||
		a.Z0Delta != 0 || a.Z1Delta != 0 || b.Z0Delta != 0 || b.Z1Delta != 0 {
		return false, nil
	}
	ra, aok := momentinput.AxisRectangle(a.Profile)
	rb, bok := momentinput.AxisRectangle(b.Profile)
	switch op {
	case meshbool.OpUnion:
		if aok && bok && ra == rb {
			return true, nil
		}
		return momentinput.ExactProfileEqual(budget, a.Profile, b.Profile)
	case meshbool.OpIntersect:
		return aok && bok && ra == rb, nil
	case meshbool.OpCut:
		return aok && bok && ra.U0 < rb.U0 && rb.U1 < ra.U1 && ra.V0 < rb.V0 && rb.V1 < ra.V1, nil
	default:
		return false, nil
	}
}
