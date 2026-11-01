package facetedtopology

import (
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func replayAppend[T any](budget *proofbound.WorkBudget, s []T, v T) ([]T, error) {
	if !budget.Replay {
		return append(s, v), nil
	}
	if len(s) == cap(s) {
		next := max(1, 2*cap(s))
		if next <= cap(s) {
			return nil, fmt.Errorf("%w: replay topology buffer overflow", decaderr.ErrUnsupported)
		}
		if err := budget.Reserve(uint64(next)); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytesMul(uint64(next), uint64(unsafe.Sizeof(v))); err != nil {
			return nil, err
		}
		if _, err := budget.StepN(uint64(len(s))); err != nil {
			return nil, err
		}
		p := make([]T, len(s), next)
		copy(p, s)
		s = p
	}
	return append(s, v), nil
}
