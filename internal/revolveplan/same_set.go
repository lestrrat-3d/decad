package revolveplan

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// SameSet is the exact set-identity test of two revolve records: equal
// frames, axes, angles and placements, no section displacement on either, and
// equal profiles walked loop by loop under the budget. As for a prism, equal
// records certify equal sets only while each record is the set it denotes.
func (rp Payload) SameSet(budget *proofbound.WorkBudget, other Payload) (bool, error) {
	if rp.Frame != other.Frame || rp.Ax != other.Ax || rp.Phi0 != other.Phi0 ||
		rp.Phi1 != other.Phi1 || rp.Full != other.Full || rp.Xform != other.Xform {
		return false, nil
	}
	if rp.SectionDelta != 0 || other.SectionDelta != 0 {
		return false, nil
	}
	return momentinput.ExactProfileEqual(budget, rp.Profile, other.Profile)
}
