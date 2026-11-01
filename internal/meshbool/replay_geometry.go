package meshbool

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ReplayGeometry owns a sealed copy of one preparation's exact geometry.
// A view gets independent exact integers and mutable float-normal caches.
type ReplayGeometry struct {
	budget        *proofbound.ReplayBudget
	verts         []r3.Vec
	tris          [][3]int
	src           []int
	xverts, norms []proof.Xpt
	boxes         [][2]r3.Vec
	owner         *replayTable[[2]int, int]
}

func replayCopyXpts(ctx context.Context, values []proof.Xpt, b *proofbound.ReplayBudget) ([]proof.Xpt, error) {
	if err := b.ReserveRecords(ctx, uint64(len(values))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(values)), uint64(unsafe.Sizeof(proof.Xpt{}))); err != nil {
		return nil, err
	}
	if err := b.SpendWork(ctx, uint64(len(values))); err != nil {
		return nil, err
	}
	out := make([]proof.Xpt, len(values))
	for i, v := range values {
		if err := b.ReserveRecords(ctx, 4); err != nil {
			return nil, err
		}
		if err := b.ReserveArray(ctx, 4, uint64(unsafe.Sizeof(big.Int{}))); err != nil {
			return nil, err
		}
		var copyOf [4]*big.Int
		for k, x := range [4]*big.Int{v.X, v.Y, v.Z, v.W} {
			if x == nil {
				return nil, errReplayMissingCachedExactCoordinate
			}
			words := uint64(len(x.Bits()))
			if err := b.SpendWork(ctx, words+1); err != nil {
				return nil, err
			}
			if err := b.ReserveArray(ctx, words+4, uint64(unsafe.Sizeof(big.Word(0)))); err != nil {
				return nil, err
			}
			copyOf[k] = new(big.Int).Set(x)
		}
		out[i] = proof.Xpt{X: copyOf[0], Y: copyOf[1], Z: copyOf[2], W: copyOf[3]}
	}
	return out, nil
}

// CaptureReplayGeometry freezes geometry only after successful preparation.
func CaptureReplayGeometry(ctx context.Context, bm *BoolMesh, b *proofbound.ReplayBudget) (*ReplayGeometry, error) {
	if bm == nil || b == nil || bm.replayOwner == nil || bm.replayOwner.budget != b ||
		len(bm.Verts) != len(bm.Xverts) || len(bm.Tris) != len(bm.Norms) ||
		len(bm.Tris) != len(bm.Boxes) || len(bm.Src) != len(bm.Tris) {
		return nil, errReplayInvalidPreparedReplayGeometry
	}
	for _, spec := range [][2]uint64{{1, uint64(unsafe.Sizeof(ReplayGeometry{}))},
		{uint64(len(bm.Verts)), uint64(unsafe.Sizeof(r3.Vec{}))},
		{uint64(len(bm.Tris)), uint64(unsafe.Sizeof([3]int{}))},
		{uint64(len(bm.Src)), uint64(unsafe.Sizeof(int(0)))},
		{uint64(len(bm.Boxes)), uint64(unsafe.Sizeof([2]r3.Vec{}))}} {
		if err := b.ReserveArray(ctx, spec[0], spec[1]); err != nil {
			return nil, err
		}
		if err := b.ReserveRecords(ctx, spec[0]); err != nil {
			return nil, err
		}
	}
	if err := b.SpendWork(ctx, uint64(len(bm.Verts)+3*len(bm.Tris))); err != nil {
		return nil, err
	}
	xverts, err := replayCopyXpts(ctx, bm.Xverts, b)
	if err != nil {
		return nil, err
	}
	norms, err := replayCopyXpts(ctx, bm.Norms, b)
	if err != nil {
		return nil, err
	}
	return &ReplayGeometry{budget: b, verts: append([]r3.Vec(nil), bm.Verts...), tris: append([][3]int(nil), bm.Tris...),
		src: append([]int(nil), bm.Src...), xverts: xverts, norms: norms, boxes: append([][2]r3.Vec(nil), bm.Boxes...), owner: bm.replayOwner}, nil
}

// View checks the seal and returns fresh attempt-local mutable state. The
// caller attaches the current bounds, slabs and gate after this returns.
func (g *ReplayGeometry) View(ctx context.Context, verts []r3.Vec, tris [][3]int, src []int,
	b *proofbound.ReplayBudget) (*BoolMesh, bool, error) {
	if g == nil || b == nil || g.budget != b || len(verts) != len(g.verts) || len(tris) != len(g.tris) || len(src) != len(g.src) {
		return nil, false, nil
	}
	for i, v := range verts {
		if err := b.SpendWork(ctx, 3); err != nil {
			return nil, false, err
		}
		old := g.verts[i]
		if math.Float64bits(v.X) != math.Float64bits(old.X) || math.Float64bits(v.Y) != math.Float64bits(old.Y) || math.Float64bits(v.Z) != math.Float64bits(old.Z) {
			return nil, false, nil
		}
	}
	for i, tri := range tris {
		if err := b.SpendWork(ctx, 4); err != nil {
			return nil, false, err
		}
		if tri != g.tris[i] || src[i] != g.src[i] {
			return nil, false, nil
		}
	}
	for _, spec := range [][2]uint64{{1, uint64(unsafe.Sizeof(BoolMesh{}))},
		{uint64(len(tris)), uint64(unsafe.Sizeof(r3.Vec{}))},
		{uint64(len(tris)), uint64(unsafe.Sizeof(false))},
		{uint64(len(tris)), uint64(unsafe.Sizeof([2]r3.Vec{}))}} {
		if err := b.ReserveArray(ctx, spec[0], spec[1]); err != nil {
			return nil, false, err
		}
		if err := b.ReserveRecords(ctx, spec[0]); err != nil {
			return nil, false, err
		}
	}
	if err := b.SpendWork(ctx, 3*uint64(len(tris))+1); err != nil {
		return nil, false, err
	}
	xverts, err := replayCopyXpts(ctx, g.xverts, b)
	if err != nil {
		return nil, false, err
	}
	norms, err := replayCopyXpts(ctx, g.norms, b)
	if err != nil {
		return nil, false, err
	}
	return &BoolMesh{Verts: verts, Tris: tris, Src: src, Xverts: xverts, Norms: norms,
		Fnorms: make([]r3.Vec, len(tris)), FnormsReady: make([]bool, len(tris)),
		Boxes: append([][2]r3.Vec(nil), g.boxes...), replayOwner: g.owner}, true, nil
}
