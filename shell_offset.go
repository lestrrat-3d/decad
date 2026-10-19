package decad

import (
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file binds the exact section offset of docs/modify-design.md §7 to
// shellTol; the construction, its S11a/S11 refusals and the §5 audit live in
// internal/offset2d/profile_offset.go.

// offsetProfile binds offset2d.OffsetProfile to shellTol.
func offsetProfile(budget *proofbound.WorkBudget, profile profileRecord, s, t float64) (profileRecord, error) {
	return offset2d.OffsetProfile(budget, profile, s, t, shellTol)
}

// offsetSectionDelta binds offset2d.OffsetSectionDelta to shellTol.
func offsetSectionDelta(budget *proofbound.WorkBudget, profile profileRecord, s, t, tDelta float64) (float64, error) {
	return offset2d.OffsetSectionDelta(budget, profile, s, t, tDelta, shellTol)
}
