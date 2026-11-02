package decad

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type brakeAncestryErrorKind uint8

const (
	brakeFacetAncestryMismatch brakeAncestryErrorKind = iota
	brakeSourceGroupMissing
	brakeSourceGroupProducer
	brakeSourceGroupFace
)

type brakeAncestryError struct {
	kind                     brakeAncestryErrorKind
	index, producer, operand int
}

func (e *brakeAncestryError) Error() string {
	switch e.kind {
	case brakeFacetAncestryMismatch:
		return fmt.Sprintf("%s: refined facet ancestry mismatch at %d", ErrUnsupported, e.index)
	case brakeSourceGroupMissing:
		return fmt.Sprintf("%s: refined clevis source group %d missing", ErrUnsupported, e.index)
	case brakeSourceGroupProducer:
		return fmt.Sprintf("%s: refined clevis source group %d producer=%d operand=%d", ErrUnsupported, e.index, e.producer, e.operand)
	default:
		return fmt.Sprintf("%s: refined clevis source group %d face mismatch", ErrUnsupported, e.index)
	}
}

func (*brakeAncestryError) Unwrap() error { return ErrUnsupported }

func newBrakeAncestryError(ctx context.Context, budget *proofbound.ReplayBudget, kind brakeAncestryErrorKind, index, producer, operand int) error {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(brakeAncestryError{}))); err != nil {
		return err
	}
	return &brakeAncestryError{kind: kind, index: index, producer: producer, operand: operand}
}
