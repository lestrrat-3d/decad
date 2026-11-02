package booleaneval

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type brakeCandidateGroupError struct{ index int }

func (e *brakeCandidateGroupError) Error() string {
	return fmt.Sprintf("%s: candidate source group %d changed", decaderr.ErrUnsupported, e.index)
}
func (*brakeCandidateGroupError) Unwrap() error { return decaderr.ErrUnsupported }

func newBrakeCandidateGroupError(ctx context.Context, budget *proofbound.ReplayBudget, index int) error {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(brakeCandidateGroupError{}))); err != nil {
		return err
	}
	return &brakeCandidateGroupError{index: index}
}
