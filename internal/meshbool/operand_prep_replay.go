package meshbool

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func prepContextReplay(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int, b *proofbound.ReplayBudget) (*BoolMesh, error) {
	if len(tris) > 65536 {
		return nil, errReplayBoundedMeshPreparationFacetCapExceeded
	}
	if err := b.ReserveRecords(ctx, 1+uint64(len(verts))+4*uint64(len(tris))); err != nil {
		return nil, err
	}
	for _, a := range [][2]uint64{{1, uint64(unsafe.Sizeof(BoolMesh{}))}, {uint64(len(verts)), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(len(tris)), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(len(tris)), uint64(unsafe.Sizeof(r3.Vec{}))}, {uint64(len(tris)), uint64(unsafe.Sizeof(bool(false)))}, {uint64(len(tris)), uint64(unsafe.Sizeof([2]r3.Vec{}))}} {
		if err := b.ReserveArray(ctx, a[0], a[1]); err != nil {
			return nil, err
		}
	}
	if err := b.SpendWork(ctx, uint64(len(verts))+4*uint64(len(tris))); err != nil {
		return nil, err
	}
	owner, err := newReplayTable[[2]int, int](ctx, 3*len(tris), b, replayEdgeHash)
	if err != nil {
		return nil, err
	}
	bm := &BoolMesh{Verts: verts, Tris: tris, Src: src, Xverts: make([]proof.Xpt, len(verts)), Norms: make([]proof.Xpt, len(tris)), Fnorms: make([]r3.Vec, len(tris)), FnormsReady: make([]bool, len(tris)), Boxes: make([][2]r3.Vec, len(tris)), replayOwner: owner}
	for i, v := range verts {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
			return nil, errReplayAMeshVertexIsNotFinite
		}
		p, e := proof.XptOfBounded(v, func(n uint64) error {
			if n > ^uint64(0)/64 {
				return errReplayExactPointByteReservationOverflow
			}
			if e := b.ReserveBytes(ctx, n*64); e != nil {
				return e
			}
			return b.ReserveRecords(ctx, n)
		})
		if e != nil {
			return nil, e
		}
		bm.Xverts[i] = p
	}
	for i, t := range tris {
		if err := b.SpendWork(ctx, 4); err != nil {
			return nil, err
		}
		for _, v := range t {
			if v < 0 || v >= len(verts) {
				return nil, errReplayInvalidOperandFacet
			}
		}
		n, e := proof.TriangleNormalReplay(verts[t[0]], verts[t[1]], verts[t[2]], func(n uint64) error {
			if n > ^uint64(0)/64 {
				return errReplayExactNormalByteReservationOverflow
			}
			if e := b.ReserveBytes(ctx, n*64); e != nil {
				return e
			}
			return b.ReserveRecords(ctx, n)
		}, func(n uint64) error { return b.SpendWork(ctx, n) })
		if e != nil {
			return nil, e
		}
		if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
			return nil, errReplayAnOperandHoldsACollapsedFacet
		}
		bm.Norms[i] = n
		bm.Boxes[i] = TriBox(verts, t)
		for k := range 3 {
			if e := owner.Set(ctx, [2]int{t[k], t[(k+1)%3]}, i); e != nil {
				return nil, e
			}
		}
	}
	return bm, nil
}

func (bm *BoolMesh) twinFacetReplay(ctx context.Context, f, k int) (int, bool, error) {
	if bm.replayOwner == nil {
		return 0, false, errReplayMissingReplayEdgeOwner
	}
	tri := bm.Tris[f]
	t, ok, err := bm.replayOwner.Get(ctx, [2]int{tri[(k+1)%3], tri[k]})
	return t, ok && t != f, err
}

func (bm *BoolMesh) ensureReplayOwner(ctx context.Context, b *proofbound.ReplayBudget) error {
	if bm.replayOwner != nil && bm.replayOwner.budget == b {
		return nil
	}
	table, err := newReplayTable[[2]int, int](ctx, 3*len(bm.Tris), b, replayEdgeHash)
	if err != nil {
		return err
	}
	for i, tri := range bm.Tris {
		for k := range 3 {
			if err := table.Set(ctx, [2]int{tri[k], tri[(k+1)%3]}, i); err != nil {
				return err
			}
		}
	}
	bm.replayOwner = table
	return nil
}

func edgeCrossesReplay(ctx context.Context, m *BoolMesh, f, k int, other *BoolMesh, partner int,
	b *proofbound.ReplayBudget) ([2]int, bool, error) {
	if err := m.ensureReplayOwner(ctx, b); err != nil {
		return [2]int{}, false, err
	}
	tri := m.Tris[f]
	u, v := tri[k], tri[(k+1)%3]
	key := [2]int{min(u, v), max(u, v)}
	twin, ok, err := m.twinFacetReplay(ctx, f, k)
	if err != nil {
		return key, false, err
	}
	if !ok {
		return key, false, errReplayAnInPlaneFacetEdgeHasNo
	}
	tt, pt := m.Tris[twin], other.Tris[partner]
	a, c := tri[0]+tri[1]+tri[2]-u-v, tt[0]+tt[1]+tt[2]-u-v
	s0, err := proof.ReplayOrientSignPrepared(ctx, other.Verts[pt[0]], other.Verts[pt[1]], other.Verts[pt[2]], m.Verts[a], other.Xverts[pt[0]], m.Xverts[a], other.Norms[partner], b)
	if err != nil {
		return key, false, err
	}
	s1, err := proof.ReplayOrientSignPrepared(ctx, other.Verts[pt[0]], other.Verts[pt[1]], other.Verts[pt[2]], m.Verts[c], other.Xverts[pt[0]], m.Xverts[c], other.Norms[partner], b)
	return key, s0*s1 < 0, err
}

func facetAdjacencyReplay(ctx context.Context, m *BoolMesh, b *proofbound.ReplayBudget) ([][]int, error) {
	if err := m.ensureReplayOwner(ctx, b); err != nil {
		return nil, err
	}
	n := len(m.Tris)
	if err := b.ReserveRecords(ctx, 4*uint64(n)); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof([]int(nil)))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 3*uint64(n), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	adj := make([][]int, n)
	neighbors := make([]int, 3*n)
	for i := range m.Tris {
		for k := range 3 {
			j, ok, err := m.twinFacetReplay(ctx, i, k)
			if err != nil {
				return nil, err
			}
			if ok {
				if adj[i] == nil {
					adj[i] = neighbors[3*i : 3*i : 3*i+3]
				}
				adj[i] = append(adj[i], j)
			}
		}
	}
	return adj, nil
}
