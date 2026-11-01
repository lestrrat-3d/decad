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

type replayHeldRepair struct {
	ctx    context.Context //nolint:containedctx // The value lives only for one bounded replay call.
	budget *proofbound.ReplayBudget
	h      HeldRounding
	grid   *heldEmbedding
	mesh   BoolMesh
	inc    [][]int
}

func newReplayHeldRepair(ctx context.Context, h HeldRounding, b *proofbound.ReplayBudget) (*replayHeldRepair, error) {
	nv, nt := len(h.Verts), len(h.Tris)
	if b == nil || len(h.Moved) != nv || len(h.Movable) != nv || len(h.Exact) != nv || nt == 0 || nt > 65536 {
		return nil, errReplayInvalidReplayRepairInput
	}
	for _, t := range h.Tris {
		if err := b.SpendWork(ctx, 3); err != nil {
			return nil, err
		}
		for _, v := range t {
			if v < 0 || v >= nv {
				return nil, errReplayInvalidReplayRepairFacet
			}
		}
	}
	// All vertex/facet scratch arrays and incidence backing are reserved before allocation.
	arrays := [][2]uint64{{1, uint64(unsafe.Sizeof(replayHeldRepair{}))}, {uint64(nv), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(nt), uint64(unsafe.Sizeof(proof.Xpt{}))}, {uint64(nt), uint64(unsafe.Sizeof(r3.Vec{}))}, {uint64(nt), uint64(unsafe.Sizeof(false))}, {uint64(nt), uint64(unsafe.Sizeof([2]r3.Vec{}))}, {uint64(nv), uint64(unsafe.Sizeof([]int{}))}, {uint64(nv), uint64(unsafe.Sizeof(int(0)))}, {3 * uint64(nt), uint64(unsafe.Sizeof(int(0)))}, {uint64(nt), uint64(unsafe.Sizeof(int(0)))}, {uint64(nt), uint64(unsafe.Sizeof(int(0)))}}
	for _, a := range arrays {
		if err := b.ReserveRecords(ctx, a[0]); err != nil {
			return nil, err
		}
		if err := b.ReserveArray(ctx, a[0], a[1]); err != nil {
			return nil, err
		}
	}
	r := &replayHeldRepair{ctx: ctx, budget: b, h: h, inc: make([][]int, nv)}
	r.mesh = BoolMesh{Verts: h.Verts, Tris: h.Tris, Xverts: make([]proof.Xpt, nv), Norms: make([]proof.Xpt, nt), Fnorms: make([]r3.Vec, nt), FnormsReady: make([]bool, nt), Boxes: make([][2]r3.Vec, nt)}
	counts := make([]int, nv)
	if err := b.SpendWork(ctx, 6*uint64(nt)+uint64(nv)); err != nil {
		return nil, err
	}
	for _, t := range h.Tris {
		for _, v := range t {
			counts[v]++
		}
	}
	for v, n := range counts {
		r.inc[v] = make([]int, 0, n)
	}
	for i, t := range h.Tris {
		for _, v := range t {
			r.inc[v] = append(r.inc[v], i)
		}
	}
	for i := range h.Verts {
		if err := r.refreshVertex(i); err != nil {
			return nil, err
		}
	}
	for i := range h.Tris {
		if err := r.refreshFacet(i); err != nil {
			return nil, err
		}
	}
	if err := r.buildGrid(); err != nil {
		return nil, err
	}
	return r, nil
}
func (r *replayHeldRepair) reserve(n uint64) error {
	if n > ^uint64(0)/8 {
		return errReplayReplayExactLiftWorkOverflow
	}
	if err := r.budget.SpendWork(r.ctx, 8*n); err != nil {
		return err
	}
	if err := r.budget.ReserveRecords(r.ctx, n); err != nil {
		return err
	}
	return r.budget.ReserveArray(r.ctx, n, 64)
}
func (r *replayHeldRepair) refreshVertex(i int) error {
	p, err := proof.XptOfBounded(r.h.Verts[i], r.reserve)
	if err == nil {
		r.mesh.Xverts[i] = p
	}
	return err
}
func (r *replayHeldRepair) refreshFacet(i int) error {
	t := r.h.Tris[i]
	p, err := proof.TriangleNormalReplay(r.h.Verts[t[0]], r.h.Verts[t[1]], r.h.Verts[t[2]], r.reserve, func(n uint64) error { return r.budget.SpendWork(r.ctx, n) })
	if err != nil {
		return err
	}
	r.mesh.Norms[i] = p
	r.mesh.FnormsReady[i] = false
	r.mesh.Boxes[i] = TriBox(r.h.Verts, t)
	return nil
}
func (r *replayHeldRepair) setVertex(i int, p r3.Vec) error {
	r.h.Verts[i] = p
	if err := r.refreshVertex(i); err != nil {
		return err
	}
	for _, f := range r.inc[i] {
		if err := r.refreshFacet(f); err != nil {
			return err
		}
	}
	return nil
}
func (r *replayHeldRepair) buildGrid() error {
	nt := len(r.h.Tris)
	maxAbs, maxDev := 0.0, 0.0
	for i, p := range r.h.Verts {
		if err := r.budget.SpendWork(r.ctx, 1); err != nil {
			return err
		}
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(p.X), math.Max(math.Abs(p.Y), math.Abs(p.Z))))
		if !r.h.Movable[i] {
			continue
		}
		q := r.h.Exact[i]
		if q.X == nil || q.Y == nil || q.Z == nil || q.W == nil || q.W.Sign() <= 0 {
			return errReplayInvalidReplayExactPoint
		}
		var vals [3]float64
		for k, num := range [3]*big.Int{q.X, q.Y, q.Z} {
			f, err := proof.ReplayFractionFloat64(r.ctx, num, q.W, r.budget)
			if err != nil {
				return err
			}
			vals[k] = f
		}
		maxDev = math.Max(maxDev, math.Max(math.Abs(p.X-vals[0]), math.Max(math.Abs(p.Y-vals[1]), math.Abs(p.Z-vals[2]))))
	}
	pad := 2*maxDev + 8*proofbound.UlpOf(maxAbs)
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, box := range r.mesh.Boxes {
		if err := r.budget.SpendWork(r.ctx, 1); err != nil {
			return err
		}
		lo = r3.Vec{X: math.Min(lo.X, box[0].X), Y: math.Min(lo.Y, box[0].Y), Z: math.Min(lo.Z, box[0].Z)}
		hi = r3.Vec{X: math.Max(hi.X, box[1].X), Y: math.Max(hi.Y, box[1].Y), Z: math.Max(hi.Z, box[1].Z)}
	}
	lo = lo.Sub(r3.Vec{X: pad, Y: pad, Z: pad})
	hi = hi.Add(r3.Vec{X: pad, Y: pad, Z: pad})
	ext := hi.Sub(lo)
	for _, value := range [...]float64{pad, lo.X, lo.Y, lo.Z, hi.X, hi.Y, hi.Z, ext.X, ext.Y, ext.Z} {
		if proofbound.IsNonFinite(value) {
			return errReplayReplayRepairGridIsNotFinite
		}
	}
	longest := math.Max(ext.X, math.Max(ext.Y, ext.Z))
	cell := math.Cbrt(ext.X * ext.Y * ext.Z / float64(nt))
	if !(cell > 0) || math.IsInf(cell, 0) {
		cell = longest / math.Cbrt(float64(nt))
	}
	if !(cell > 0) || math.IsInf(cell, 0) {
		cell = 1
	}
	cell = math.Max(cell, longest/1024)
	var dims [3]int
	for k, x := range [3]float64{ext.X, ext.Y, ext.Z} {
		dims[k] = max(1, min(1024, int(math.Ceil(x/cell))))
	}
	for dims[0]*dims[1]*dims[2] > 8*nt+64 {
		if err := r.budget.SpendWork(r.ctx, 1); err != nil {
			return err
		}
		cell *= 2
		for k, x := range [3]float64{ext.X, ext.Y, ext.Z} {
			dims[k] = max(1, int(math.Ceil(x/cell)))
		}
	}
	count := dims[0] * dims[1] * dims[2]
	for _, a := range [][2]uint64{{1, uint64(unsafe.Sizeof(heldEmbedding{}))}, {1, 1024}, {uint64(count), uint64(unsafe.Sizeof([]int32{}))}, {uint64(count), uint64(unsafe.Sizeof(int(0)))}, {uint64(nt), uint64(unsafe.Sizeof(int32(0)))}} {
		if err := r.budget.ReserveRecords(r.ctx, a[0]); err != nil {
			return err
		}
		if err := r.budget.ReserveArray(r.ctx, a[0], a[1]); err != nil {
			return err
		}
	}
	e := &heldEmbedding{h: r.h, budget: r.budget.WorkBudget(r.ctx), pad: pad, origin: lo, cell: cell, dims: dims, cells: make([][]int32, count), stamp: make([]int, nt), cands: make([]int, 0, nt), wide: make([]int32, 0, nt)}
	counts := make([]int, count)
	visit := func(box [2]r3.Vec, fn func(int)) (bool, error) {
		a, b := e.cellRange(box)
		span := (b[0] - a[0] + 1) * (b[1] - a[1] + 1) * (b[2] - a[2] + 1)
		if span > heldGridCellSpan {
			return true, nil
		}
		if err := r.budget.SpendWork(r.ctx, uint64(span)); err != nil {
			return false, err
		}
		for x := a[0]; x <= b[0]; x++ {
			for y := a[1]; y <= b[1]; y++ {
				for z := a[2]; z <= b[2]; z++ {
					k := (x*dims[1]+y)*dims[2] + z
					fn(k)
				}
			}
		}
		return false, nil
	}
	for i, box := range r.mesh.Boxes {
		wide, err := visit(box, func(k int) { counts[k]++ })
		if err != nil {
			return err
		}
		if wide {
			e.wide = append(e.wide, int32(i))
		}
	}
	var entries uint64
	for _, n := range counts {
		entries += uint64(n)
	}
	if err := r.budget.ReserveRecords(r.ctx, entries); err != nil {
		return err
	}
	if err := r.budget.ReserveArray(r.ctx, entries, uint64(unsafe.Sizeof(int32(0)))); err != nil {
		return err
	}
	if err := r.budget.SpendWork(r.ctx, uint64(count)); err != nil {
		return err
	}
	for i, n := range counts {
		e.cells[i] = make([]int32, 0, n)
	}
	for i, box := range r.mesh.Boxes {
		if _, err := visit(box, func(k int) { e.cells[k] = append(e.cells[k], int32(i)) }); err != nil {
			return err
		}
	}
	r.grid = e
	return nil
}

func (r *replayHeldRepair) scanFacet(i int, visit func(int)) (bool, error) {
	if err := r.budget.ReserveRecords(r.ctx, 1); err != nil {
		return false, err
	}
	if err := r.budget.ReserveBytes(r.ctx, 128); err != nil {
		return false, err
	}
	if zeroXpt(r.mesh.Norms[i]) {
		return true, nil
	}
	candidates, err := r.grid.candidates(i)
	if err != nil {
		return false, err
	}
	for _, j := range candidates {
		if err := r.budget.SpendWork(r.ctx, 5); err != nil {
			return false, err
		}
		if i == j || !BoxesOverlap(r.mesh.Boxes[i], r.mesh.Boxes[j]) {
			continue
		}
		ok, err := replayHeldPairEmbedded(r.ctx, &r.mesh, i, j, r.budget)
		if err != nil {
			return false, err
		}
		if !ok {
			visit(j)
		}
	}
	return false, nil
}
func (r *replayHeldRepair) vertexFailures(v int) (int, error) {
	n := 0
	for _, f := range r.inc[v] {
		var visitErr error
		collapsed, err := r.scanFacet(f, func(j int) {
			if visitErr != nil {
				return
			}
			// Both incident facets are counted only at their lower index.
			if j < f {
				for _, other := range r.inc[v] {
					if err := r.budget.SpendWork(r.ctx, 1); err != nil {
						visitErr = err
						return
					}
					if other == j {
						return
					}
				}
			}
			n++
		})
		if err != nil {
			return 0, err
		}
		if visitErr != nil {
			return 0, visitErr
		}
		if collapsed {
			n++
		}
	}
	return n, nil
}
func (r *replayHeldRepair) improve(v int) (bool, error) {
	orig := r.h.Verts[v]
	best, err := r.vertexFailures(v)
	if err != nil || best == 0 {
		return false, err
	}
	corners, n, err := replayFloatBoxCorners(r.ctx, r.h.Exact[v], r.budget)
	if err != nil {
		return false, err
	}
	bestAt := orig
	for _, c := range corners[:n] {
		if err := r.budget.SpendWork(r.ctx, 1); err != nil {
			return false, err
		}
		if c == orig {
			continue
		}
		occupied := false
		for i, p := range r.h.Verts {
			if err := r.budget.SpendWork(r.ctx, 1); err != nil {
				return false, err
			}
			if i != v && p == c {
				occupied = true
				break
			}
		}
		if occupied {
			continue
		}
		if err := r.setVertex(v, c); err != nil {
			r.h.Verts[v] = orig
			return false, err
		}
		count, trialErr := r.vertexFailures(v)
		if err := r.setVertex(v, orig); err != nil {
			r.h.Verts[v] = orig
			return false, err
		}
		if trialErr != nil {
			return false, trialErr
		}
		if count < best {
			best, bestAt = count, c
		}
	}
	if bestAt == orig {
		return false, nil
	}
	if err := r.setVertex(v, bestAt); err != nil {
		r.h.Verts[v] = orig
		return false, err
	}
	return true, nil
}

func replayRepairHeldEmbedding(ctx context.Context, h HeldRounding, b *proofbound.ReplayBudget) (int, error) {
	r, err := newReplayHeldRepair(ctx, h, b)
	if err != nil {
		return 0, err
	}
	nt, nv := len(h.Tris), len(h.Verts)
	for _, a := range [][2]uint64{{3 * uint64(nt), uint64(unsafe.Sizeof(int(0)))}, {2 * uint64(nt), uint64(unsafe.Sizeof(false))}, {2 * uint64(nv), uint64(unsafe.Sizeof(false))}} {
		if err := b.ReserveRecords(ctx, a[0]); err != nil {
			return 0, err
		}
		if err := b.ReserveArray(ctx, a[0], a[1]); err != nil {
			return 0, err
		}
	}
	check := make([]int, 0, nt)
	next := make([]bool, nt)
	offenders := make([]bool, nv)
	moved := make([]bool, nv)
	for i, t := range h.Tris {
		if err := b.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		if h.Moved[t[0]] || h.Moved[t[1]] || h.Moved[t[2]] {
			check = append(check, i)
		}
	}
	for range heldEmbeddingSweeps {
		if err := b.SpendWork(ctx, uint64(nt)+uint64(nv)); err != nil {
			return 0, err
		}
		clear(next)
		clear(offenders)
		failed := false
		add := func(f int) {
			for _, v := range h.Tris[f] {
				if h.Movable[v] {
					offenders[v] = true
				}
			}
		}
		for _, i := range check {
			facetFailed := false
			collapsed, err := r.scanFacet(i, func(j int) { facetFailed = true; add(j) })
			if err != nil {
				return 0, err
			}
			if collapsed || facetFailed {
				failed = true
				next[i] = true
				add(i)
			}
		}
		if !failed {
			n := 0
			for _, v := range moved {
				if v {
					n++
				}
			}
			return n, nil
		}
		changed := false
		for v := range offenders {
			if err := b.SpendWork(ctx, 1); err != nil {
				return 0, err
			}
			if !offenders[v] {
				continue
			}
			placed, err := r.improve(v)
			if err != nil {
				return 0, err
			}
			if !placed {
				continue
			}
			moved[v] = true
			changed = true
			if err := b.SpendWork(ctx, uint64(len(r.inc[v]))); err != nil {
				return 0, err
			}
			for _, f := range r.inc[v] {
				next[f] = true
			}
		}
		check = check[:0]
		for i, on := range next {
			if err := b.SpendWork(ctx, 1); err != nil {
				return 0, err
			}
			if on {
				check = append(check, i)
			}
		}
		if !changed {
			break
		}
	}
	// The ordinary search checks once more after its last sweep. Reach repair is
	// not admitted by replay, so a remaining fold refuses before that stage.
	for _, i := range check {
		failed := false
		collapsed, err := r.scanFacet(i, func(int) { failed = true })
		if err != nil {
			return 0, err
		}
		if collapsed || failed {
			return 0, errReplayReplayFloatBoxRepairDidNotClear
		}
	}
	n := 0
	for _, v := range moved {
		if v {
			n++
		}
	}
	return n, nil
}
