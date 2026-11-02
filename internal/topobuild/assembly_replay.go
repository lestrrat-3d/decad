package topobuild

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// AttachFaceLoopsReplay registers each coedge under the caller's shared
// replay meter. A failed registration leaves only the uncommitted body changed.
func AttachFaceLoopsReplay(ctx context.Context, faces []*topo.Face,
	budget *proofbound.ReplayBudget) error {
	if budget == nil {
		return decaderr.ErrUnsupported
	}
	for _, face := range faces {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if face == nil {
			return decaderr.ErrUnsupported
		}
		for _, loop := range face.Loops {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if loop == nil {
				return decaderr.ErrUnsupported
			}
			for _, ce := range loop.CoEdges {
				if err := budget.ReserveBundle(ctx, 1, 1, 0); err != nil {
					return err
				}
				if ce.Edge == nil {
					return decaderr.ErrUnsupported
				}
				if len(ce.Edge.Faces) == cap(ce.Edge.Faces) {
					if cap(ce.Edge.Faces) > int(^uint(0)>>1)/2 {
						return decaderr.ErrUnsupported
					}
					capacity := max(1, 2*cap(ce.Edge.Faces))
					if capacity < len(ce.Edge.Faces)+1 {
						return decaderr.ErrUnsupported
					}
					if err := budget.ReserveArray(ctx, uint64(capacity),
						uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
						return err
					}
				}
				ce.Edge.Faces = append(ce.Edge.Faces, face)
			}
		}
	}
	return nil
}
