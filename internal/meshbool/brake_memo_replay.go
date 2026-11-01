package meshbool

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// NewReplayContactMemo allocates the identity shell used by replay executors.
// Replay disables memo lookup and storage, so it needs no map.
func NewReplayContactMemo(ctx context.Context, a, b *BoolMesh, budget *proofbound.ReplayBudget) (*ContactMemo, error) {
	if a == nil || b == nil || budget == nil {
		return nil, errReplayMissingReplayContactMemoInputs
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ContactMemo{}))); err != nil {
		return nil, err
	}
	return &ContactMemo{Ma: a, Mb: b, Cols: len(b.Tris)}, nil
}
