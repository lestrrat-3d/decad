package revolvemesh

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

//nolint:containedctx // The context is confined to this synchronous replay.
type replayDyadicOps struct {
	ctx    context.Context
	budget *proofbound.ReplayBudget
}

// These refusals are constructed once so rejected replay does not allocate.
var (
	errReplayFacetArea1 = fmt.Errorf("%w: invalid revolve facet vertex", decaderr.ErrUnsupported)
	errReplayFacetArea2 = fmt.Errorf("%w: invalid facet area root", decaderr.ErrUnsupported)
)

func (m replayDyadicOps) op(a, b proof.Dyadic, k byte) (proof.Dyadic, error) {
	return proof.ReplayDyadicArithmetic(m.ctx, a, b, k, m.budget)
}
func (m replayDyadicOps) sub(a, b proof.DyV3) (proof.DyV3, error) {
	var v proof.DyV3
	for i := range 3 {
		x, e := m.op(a[i], b[i], '-')
		if e != nil {
			return v, e
		}
		v[i] = x
	}
	return v, nil
}
func (m replayDyadicOps) cross(a, b proof.DyV3) (proof.DyV3, error) {
	var v proof.DyV3
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		x, e := m.op(a[j], b[k], '*')
		if e != nil {
			return v, e
		}
		y, e := m.op(a[k], b[j], '*')
		if e != nil {
			return v, e
		}
		v[i], e = m.op(x, y, '-')
		if e != nil {
			return v, e
		}
	}
	return v, nil
}
func (m replayDyadicOps) dot(a, b proof.DyV3) (proof.Dyadic, error) {
	var sum proof.Dyadic
	for i := range 3 {
		x, e := m.op(a[i], b[i], '*')
		if e != nil {
			return sum, e
		}
		sum, e = m.op(sum, x, '+')
		if e != nil {
			return sum, e
		}
	}
	return sum, nil
}
func (m replayDyadicOps) length(a proof.DyV3, down bool) (float64, error) {
	sq, e := m.dot(a, a)
	if e != nil {
		return 0, e
	}
	return proof.ReplaySqrtDyadic(m.ctx, sq, down, m.budget)
}

func newRevolveAuditTriReplay(ctx context.Context, verts []r3.Vec, tri [3]int, b *proofbound.ReplayBudget) (RevolveAuditTri, error) {
	var out RevolveAuditTri
	if b == nil {
		return out, decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return out, err
	}
	m := replayDyadicOps{ctx, b}
	for k, vi := range tri {
		if vi < 0 || vi >= len(verts) || !proofbound.FiniteVec(verts[vi]) {
			return out, errReplayFacetArea1
		}
		if err := b.SpendWork(ctx, 3); err != nil {
			return out, err
		}
		out.P[k] = proof.DyVec(verts[vi])
		out.Fp[k] = verts[vi]
	}
	var err error
	out.U, err = m.sub(out.P[1], out.P[0])
	if err != nil {
		return out, err
	}
	out.V, err = m.sub(out.P[2], out.P[0])
	if err != nil {
		return out, err
	}
	out.W, err = m.sub(out.P[2], out.P[1])
	if err != nil {
		return out, err
	}
	out.N, err = m.cross(out.U, out.V)
	if err != nil {
		return out, err
	}
	out.Fu = RevIvPointDiff(out.Fp[1], out.Fp[0])
	out.Fv = RevIvPointDiff(out.Fp[2], out.Fp[0])
	out.Fw = RevIvPointDiff(out.Fp[2], out.Fp[1])
	out.Fn = RevIvCross(out.Fu, out.Fv)
	out.FnProj[0], out.FnProj[1] = RevIvProject(out.Fp, out.Fn)
	out.Lu, err = m.length(out.U, false)
	if err != nil {
		return out, err
	}
	out.Lv, err = m.length(out.V, false)
	if err != nil {
		return out, err
	}
	out.Lw, err = m.length(out.W, false)
	if err != nil {
		return out, err
	}
	out.Box = meshbool.TriBox(verts, tri)
	out.Off[0] = [2]RevolveOffset{{V: out.U, F: out.Fu, Length: out.Lu}, {V: out.V, F: out.Fv, Length: out.Lv}}
	for i := 1; i < 3; i++ {
		j := 0
		for k := range 3 {
			if k == i {
				continue
			}
			v, e := m.sub(out.P[k], out.P[i])
			if e != nil {
				return out, e
			}
			// Reversing an edge preserves its exact squared length.
			var length float64
			switch {
			case i == 1 && k == 0:
				length = out.Lu
			case i == 2 && k == 0:
				length = out.Lv
			default:
				length = out.Lw
			}
			out.Off[i][j] = RevolveOffset{V: v, F: RevIvPointDiff(out.Fp[k], out.Fp[i]), Length: length}
			j++
		}
	}
	return out, nil
}
func requireRevolveFacetAreaReplay(ctx context.Context, t RevolveAuditTri, i int, delta float64, b *proofbound.ReplayBudget) error {
	_, err := requireRevolveFacetAreaUpperReplay(ctx, t, i, delta, b, false)
	return err
}

func requireRevolveFacetAreaUpperReplay(ctx context.Context, t RevolveAuditTri, i int, delta float64,
	b *proofbound.ReplayBudget, wantUpper bool) (float64, error) {
	m := replayDyadicOps{ctx, b}
	sq, err := m.dot(t.N, t.N)
	if err != nil {
		return 0, err
	}
	held, err := proof.ReplaySqrtDyadic(ctx, sq, true, b)
	if err != nil {
		return 0, err
	}
	allow := proofbound.ProductUpper(2, proofbound.AbsSumUpper(proofbound.ProductUpper(delta, proofbound.AbsSumUpper(t.Lu, t.Lv)), proofbound.ProductUpper(proofbound.ProductUpper(2, delta), delta)))
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(allow) || held <= allow {
		return 0, proofbound.ReplayIndexedRefusal(ctx, b, "revolve facet ", " does not keep a positive area under the coordinate displacement this mesh carries", "", i, 0, false)
	}
	if !wantUpper {
		return 0, nil
	}
	// For a normal result, scaling by one half preserves the directed float
	// rounding. Subnormal and overflow cases use the independently rounded
	// squared-area path instead.
	if held < 0x1p-1021 || math.IsInf(math.Nextafter(held, math.Inf(1)), 1) {
		return proof.ReplaySqrtDyadic(ctx, proof.DyShift(sq, -2), false, b)
	}
	v, ok := proof.DyOf(held)
	if !ok {
		return 0, errReplayFacetArea2
	}
	square, err := m.op(v, v, '*')
	if err != nil {
		return 0, err
	}
	diff, err := m.op(square, sq, '-')
	if err != nil {
		return 0, err
	}
	if diff.Sign() != 0 {
		held = math.Nextafter(held, math.Inf(1))
	}
	return held * .5, nil
}
