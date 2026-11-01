package topobuild

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errReplayVertexBounds1 = fmt.Errorf("%w: missing replay mesh", decaderr.ErrUnsupported)
	errReplayVertexBounds2 = fmt.Errorf("%w: invalid vertex bounds", decaderr.ErrBooleanFailed)
	errReplayVertexBounds3 = fmt.Errorf("%w: invalid vertex bound", decaderr.ErrBooleanFailed)
	errReplayVertexBounds4 = fmt.Errorf("%w: invalid face sources", decaderr.ErrBooleanFailed)
	errReplayVertexBounds5 = fmt.Errorf("%w: a mesh facet's source face states no displacement bound", decaderr.ErrBooleanFailed)
	errReplayVertexBounds6 = fmt.Errorf("%w: invalid face bound", decaderr.ErrBooleanFailed)
	errReplayVertexBounds7 = fmt.Errorf("%w: invalid mesh vertex", decaderr.ErrBooleanFailed)
)

// VertexBoundsReplay reads or derives vertex bounds under the replay budget.
func VertexBoundsReplay(ctx context.Context, m *topo.Mesh, b *proofbound.ReplayBudget) ([]float64, error) {
	if m == nil || b == nil {
		return nil, errReplayVertexBounds1
	}
	if err := b.ReserveRecords(ctx, uint64(len(m.Vertices))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(m.Vertices)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return nil, err
	}
	if m.VertexBound != nil {
		if len(m.VertexBound) != len(m.Vertices) {
			return nil, errReplayVertexBounds2
		}
		if err := b.SpendWork(ctx, 2*uint64(len(m.Vertices))); err != nil {
			return nil, err
		}
		for _, v := range m.VertexBound {
			if !(v >= 0) || math.IsInf(v, 0) {
				return nil, errReplayVertexBounds3
			}
		}
		out := make([]float64, len(m.VertexBound))
		copy(out, m.VertexBound)
		return out, nil
	}
	out := make([]float64, len(m.Vertices))
	if len(m.Source) != len(m.Triangles) {
		return nil, errReplayVertexBounds4
	}
	for i, tri := range m.Triangles {
		if err := b.SpendWork(ctx, 3); err != nil {
			return nil, err
		}
		d, ok, err := m.ReplayFaceBound(ctx, m.Source[i], b)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, errReplayVertexBounds5
		}
		if !(d >= 0) || math.IsInf(d, 0) {
			return nil, errReplayVertexBounds6
		}
		for _, v := range tri {
			if v < 0 || v >= len(out) {
				return nil, errReplayVertexBounds7
			}
			out[v] = max(out[v], d)
		}
	}
	return out, nil
}
