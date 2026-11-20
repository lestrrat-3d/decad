package proofbound

import "context"

// BeginRetainedRimPublication admits final single-shell assembly for a checked
// local edit. All source and crossing checks have already paid the ordinary
// strict allowance. The same counters continue through diameter, moments,
// topology and publication. No failed or previously extended meter qualifies.
func (b *ReplayBudget) BeginRetainedRimPublication(ctx context.Context, vertices, facets int) (bool, error) {
	if e := ctx.Err(); e != nil {
		return false, e
	}
	if b == nil || !b.strict || b.circle {
		return false, nil
	}
	if b.failure != nil {
		return false, b.failure
	}
	if b.shared.maxWork != ReplayTrialWork || b.shared.maxRecords != ReplayTrialRecords || b.maxBytes != 0 || vertices < 4 || vertices > 16384 || facets < 4 || facets > 32768 {
		return false, nil
	}
	b.shared.maxWork = 320000000
	b.shared.maxRecords = 16 << 20
	b.maxBytes = 7 << 29
	return true, nil
}
