package prismrecord

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SameSet is the exact set-identity test of two prism records: equal frames,
// levels and placements, no section displacement on either, and equal
// profiles walked loop by loop under the budget. Equal records are a
// certificate of equal SETS only while each record IS the set it denotes; a
// payload carrying a section displacement (agent-docs/prism-boolean-design.md §7)
// denotes a set its record is only within that displacement of, so such a pair
// stays undecided.
func (pp Payload) SameSet(budget *proofbound.WorkBudget, other Payload) (bool, error) {
	if pp.Frame != other.Frame || pp.Z0 != other.Z0 || pp.Z1 != other.Z1 || pp.Xform != other.Xform {
		return false, nil
	}
	if pp.SectionDelta != 0 || other.SectionDelta != 0 {
		return false, nil
	}
	return momentinput.ExactProfileEqual(budget, pp.Profile, other.Profile)
}
