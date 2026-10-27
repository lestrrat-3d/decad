package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// cutProjectionScratch owns temporary 2D coordinates for consecutive facet
// subdivisions, plus bounded vertex, edge and index storage for its cutter.
// CutRegion copies 3D points and partner triangles, so a completed subdivision
// keeps no reference to these mutable records or rationals. One worker owns
// the scratch and resets it only after the preceding CutTriangle returns.
type cutProjectionScratch struct {
	rats   []*big.Rat
	used   int
	cutter TriCutter
}

// cutWorkspaceRetainLimit bounds the cutter entries retained between facets.
// A large facet may grow beyond this; the next facet drops that storage.
const cutWorkspaceRetainLimit = 256

func newTriCutter(ctx context.Context, projections *cutProjectionScratch) *TriCutter {
	var tc *TriCutter
	if projections == nil {
		tc = new(TriCutter)
	} else {
		tc = &projections.cutter
		projections.used = 0
	}
	if tc.index.promoted != nil {
		tc.index.promoted = resetCutMap(tc.index.promoted)
	}
	if tc.index.exactHead != nil {
		tc.index.exactHead = resetCutMap(tc.index.exactHead)
	}
	*tc = TriCutter{
		Verts:        resetCutSlice(tc.Verts),
		Edges:        resetCutSlice(tc.Edges),
		regionPoints: resetCutSlice(tc.regionPoints),
		index: cutVertIndex{
			floatKeys: projections != nil,
			promoted:  tc.index.promoted,
			exactHead: tc.index.exactHead,
			head:      resetCutMap(tc.index.head),
			next:      resetCutSlice(tc.index.next),
		},
		raw: cutRawIndex{
			head:    resetCutMap(tc.raw.head),
			entries: resetCutSlice(tc.raw.entries),
		},
		seen:        resetCutMap(tc.seen),
		anchors:     resetCutMap(tc.anchors),
		Work:        proofbound.NewWorkBudget(ctx),
		projections: projections,
	}
	return tc
}

func resetCutSlice[T any](v []T) []T {
	if cap(v) > cutWorkspaceRetainLimit {
		return nil
	}
	clear(v)
	return v[:0]
}

func resetCutMap[K comparable, V any](m map[K]V) map[K]V {
	if m == nil || len(m) > cutWorkspaceRetainLimit {
		return map[K]V{}
	}
	clear(m)
	return m
}

func (s *cutProjectionScratch) rat() *big.Rat {
	if s.used == len(s.rats) {
		s.rats = append(s.rats, new(big.Rat))
	}
	r := s.rats[s.used]
	s.used++
	return r
}
