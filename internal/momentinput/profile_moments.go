package momentinput

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// WithIntegrals returns a new walk-cache snapshot that also owns this
// successful plane-local integration. pw must not be nil. The caller supplies
// the result it just computed for pw.Profile at order.
// Existing snapshots and their immutable walks are never changed.
func (pw *ProfileWalks) WithIntegrals(ig Integrals, order freeform.MomentIntegralOrder) *ProfileWalks {
	out := *pw
	ig = cloneProfileIntegrals(ig)
	out.integrals, out.integralOrder = &ig, order
	out.integralProfile = Profile{Outer: sectionrecord.CloneLoopRecord(pw.Profile.Outer)}
	if pw.Profile.Holes != nil {
		out.integralProfile.Holes = make([]LoopRecord, len(pw.Profile.Holes))
		for i, hole := range pw.Profile.Holes {
			out.integralProfile.Holes[i] = sectionrecord.CloneLoopRecord(hole)
		}
	}
	return &out
}

// EvaluatorIntegralsUncheckedContext reads a bit-identical record's integration
// after the original field validation and work charges. The cache owns a
// separate record snapshot so aliased source edits cannot change its key.
// Returned rationals are owned. See agent-docs/evaluator-design.md §8.
func (pw *ProfileWalks) EvaluatorIntegralsUncheckedContext(ctx context.Context, profile Profile,
	order freeform.MomentIntegralOrder, work *freeform.FreeformWork) (Integrals, error) {
	pre, err := ValidateFieldsWithPoll(ctx.Err, profile, work)
	if err != nil {
		return Integrals{}, err
	}
	if pw.Reusable(profile) && pw.integrals != nil && pw.integralOrder == order &&
		sectionrecord.IdenticalRecord(pw.integralProfile, profile) {
		if err := ctx.Err(); err != nil {
			return Integrals{}, err
		}
		return cloneProfileIntegrals(*pw.integrals), nil
	}
	return integrateMomentRecordUncheckedContext(ctx, pre, order)
}

func cloneProfileIntegrals(ig Integrals) Integrals {
	fields := [6]**big.Rat{&ig.Exact.Area, &ig.Exact.Mu, &ig.Exact.Mv, &ig.Exact.Muu, &ig.Exact.Muv, &ig.Exact.Mvv}
	for _, field := range fields {
		if *field != nil {
			*field = new(big.Rat).Set(*field)
		}
	}
	for i := range ig.Third {
		if ig.Third[i].Lo != nil {
			ig.Third[i].Lo = new(big.Rat).Set(ig.Third[i].Lo)
		}
		if ig.Third[i].Hi != nil {
			ig.Third[i].Hi = new(big.Rat).Set(ig.Third[i].Hi)
		}
	}
	return ig
}
