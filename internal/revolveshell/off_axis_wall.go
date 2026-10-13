package revolveshell

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// OffAxisWall is the wall region of a meridian that is its own
// effective meridian: the tube section of modify Table B, {P, Q} inward and
// {Q, P} outward, the inner loop walked as a hole. The offset runs the prism
// shell's S11a construction and §5 audit (S8, S11b, S9) unchanged, and so does
// its displacement: the prism cup's offset2d.OffsetSectionDelta over the same
// loop. tol is the shell tolerance both offset constructions bind.
func OffAxisWall(budget *proofbound.WorkBudget, profile momentinput.Profile, s, tmm, tDelta, tol float64) (momentinput.Profile, float64, error) {
	offset, err := offset2d.OffsetProfile(budget, profile, s, tmm, tol)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	if err := offset2d.AuditOffsetSectionBudget(budget, profile, offset); err != nil {
		return momentinput.Profile{}, 0, err
	}
	delta, err := offset2d.OffsetSectionDelta(budget, profile, s, tmm, tDelta, tol)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	outer, inner := profile.Outer, offset.Outer
	if s < 0 {
		outer, inner = offset.Outer, profile.Outer
	}
	hole, err := offset2d.ReverseLoopRecordBudget(budget, inner)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	return momentinput.Profile{Outer: outer, Holes: []sectionrecord.LoopRecord{hole}}, delta, nil
}
