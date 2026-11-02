package momentinput

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ReplayAnalyticProfileViews reserves and constructs short-lived typed views.
func ReplayAnalyticProfileViews(ctx context.Context, profiles []Profile, budget proof.ReplayKeyBudget) ([]sectionrecord.AnalyticProfileView, error) {
	if budget == nil || len(profiles) == 0 || len(profiles) > 4096 {
		return nil, decaderr.ErrUnsupported
	}
	n := uint64(len(profiles))
	size := uint64(unsafe.Sizeof(sectionrecord.AnalyticProfileView{}))
	if err := budget.SpendWork(ctx, n+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, 2*n*size+128); err != nil {
		return nil, err
	}
	out := make([]sectionrecord.AnalyticProfileView, len(profiles))
	for i, p := range profiles {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		out[i] = sectionrecord.AnalyticProfileView{Outer: p.Outer, Holes: p.Holes}
	}
	return out, nil
}

// AnalyticSealView returns the recorded loops without copying or retaining them.
func (p Profile) AnalyticSealView() sectionrecord.AnalyticProfileView {
	return sectionrecord.AnalyticProfileView{Outer: p.Outer, Holes: p.Holes}
}
