package facetedrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func reserveReplayNode(b *proofbound.WorkBudget, alloc topo.Alloc, kind string) error {
	n, err := topo.ReplayNodeBytes(alloc, kind)
	if err != nil {
		return fmt.Errorf("%w: %v", decaderr.ErrUnsupported, err)
	}
	if err := b.Reserve(1); err != nil {
		return err
	}
	return b.ReserveBytesMul(1, n)
}
