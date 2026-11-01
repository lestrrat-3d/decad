package facetproof

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type replayExactMeter struct{ budget *proofbound.WorkBudget }

func (m replayExactMeter) ReserveRecords(_ context.Context, n uint64) error {
	return m.budget.Reserve(n)
}
func (m replayExactMeter) ReserveBytes(_ context.Context, n uint64) error {
	return m.budget.ReserveBytes(n)
}
func (m replayExactMeter) SpendWork(_ context.Context, n uint64) error {
	_, err := m.budget.StepN(n)
	return err
}
