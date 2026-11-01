package revolvemesh

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// replayFloatAxis carries only the outward-rounded filters. An undecided
// candidate refuses before the ordinary exact dyadic fallback.
type replayFloatAxis struct {
	f             RevIvVec
	length, drift float64
}

func replayAxisSide(a replayFloatAxis, o RevolveOffset, e float64) (int, bool) {
	allow := PerturbBilinearAllow(a.length, o.Length, a.drift, e)
	if proofbound.IsNonFinite(allow) {
		return 0, false
	}
	s, ok := RevIvSide(RevIvDot(a.f, o.F), allow)
	return s, ok && s != 0
}
func replayNormal(t RevolveAuditTri, d float64) replayFloatAxis {
	e := proofbound.ProductUpper(2, d)
	return replayFloatAxis{t.Fn, proofbound.ProductUpper(t.Lu, t.Lv), PerturbBilinearAllow(t.Lu, t.Lv, e, e)}
}
func replayFan(t RevolveAuditTri, o RevolveOffset, d float64) replayFloatAxis {
	e := proofbound.ProductUpper(2, d)
	n := replayNormal(t, d)
	return replayFloatAxis{RevIvCross(n.f, o.F), proofbound.ProductUpper(n.length, o.Length), PerturbBilinearAllow(n.length, o.Length, n.drift, e)}
}
func replayVertexIsolated(ctx context.Context, meter *proofbound.ReplayBudget,
	a RevolveAuditTri, ta [3]int, b RevolveAuditTri, tb [3]int, shared int, d float64) (bool, error) {
	ai, bi := TriangleVertexSlot(ta, shared), TriangleVertexSlot(tb, shared)
	if ai < 0 || bi < 0 {
		return false, nil
	}
	ao, bo := a.Off[ai], b.Off[bi]
	e := proofbound.ProductUpper(2, d)
	signed := func(ax replayFloatAxis, offs []RevolveOffset) (int, bool) {
		side := 0
		for _, o := range offs {
			s, ok := replayAxisSide(ax, o, e)
			if !ok {
				return 0, false
			}
			if side == 0 {
				side = s
			} else if side != s {
				return 0, false
			}
		}
		return side, true
	}
	test := func(ax replayFloatAxis, check []RevolveOffset) bool {
		side, ok := signed(ax, check)
		if !ok {
			return false
		}
		want, ok := signed(ax, bo[:])
		return ok && want != 0 && (side == 0 || side != want)
	}
	// A normal test has at most two interval dot products and their
	// displacement allowances. Its 384 units cover the normal, both
	// three-component dot products, outward rounding and branch checks.
	// Each later vertex axis reserves 512 units for its cross products and
	// up to four interval dot products before evaluating the axis.
	if err := meter.SpendWork(ctx, 384); err != nil {
		return false, err
	}
	if test(replayNormal(a, d), nil) {
		return true, nil
	}
	for k := range 2 {
		if err := meter.SpendWork(ctx, 512); err != nil {
			return false, err
		}
		if test(replayFan(a, ao[k], d), []RevolveOffset{ao[1-k]}) {
			return true, nil
		}
	}
	if err := meter.SpendWork(ctx, 512); err != nil {
		return false, err
	}
	chord := RevolveOffset{F: RevIvSubVec(ao[0].F, ao[1].F), Length: proofbound.AbsSumUpper(ao[0].Length, ao[1].Length)}
	if test(replayFan(a, chord, d), ao[:]) {
		return true, nil
	}
	for k := range 2 {
		for j := range 2 {
			if err := meter.SpendWork(ctx, 512); err != nil {
				return false, err
			}
			ax := replayFloatAxis{RevIvCross(ao[k].F, bo[j].F), proofbound.ProductUpper(ao[k].Length, bo[j].Length), PerturbBilinearAllow(ao[k].Length, bo[j].Length, e, e)}
			sa, oka := replayAxisSide(ax, ao[1-k], e)
			sb, okb := replayAxisSide(ax, bo[1-j], e)
			if oka && okb && sa != sb {
				return true, nil
			}
		}
	}
	return false, nil
}
func replayEdgeIsolated(ctx context.Context, meter *proofbound.ReplayBudget,
	a RevolveAuditTri, ta [3]int, b RevolveAuditTri, tb [3]int, shared [3]int, d float64) (bool, error) {
	p0, p1 := TriangleVertexSlot(ta, shared[0]), TriangleVertexSlot(ta, shared[1])
	aa, bb := tessellation.TriangleApexIndex(ta, shared[0], shared[1]), tessellation.TriangleApexIndex(tb, shared[0], shared[1])
	if p0 < 0 || p1 < 0 || aa < 0 || bb < 0 {
		return false, nil
	}
	bi := TriangleVertexSlot(tb, bb)
	ai := TriangleVertexSlot(ta, aa)
	if bi < 0 || ai < 0 {
		return false, nil
	}
	e := proofbound.ProductUpper(2, d)
	offset := func(from, to int, t RevolveAuditTri) RevolveOffset {
		j := 0
		for k := range 3 {
			if k == from {
				continue
			}
			if k == to {
				return t.Off[from][j]
			}
			j++
		}
		return RevolveOffset{}
	}
	// Shared vertex coordinates are identical; b's own offset has the same
	// exact displacement and its precomputed outward length bound.
	b0 := TriangleVertexSlot(tb, shared[0])
	if b0 < 0 {
		return false, nil
	}
	ob := offset(b0, bi, b)
	// The normal reads one offset. The fallback reserves two cross products,
	// one interval dot product and their displacement allowances separately.
	if err := meter.SpendWork(ctx, 256); err != nil {
		return false, err
	}
	if _, ok := replayAxisSide(replayNormal(a, d), ob, e); ok {
		return true, nil
	}
	if err := meter.SpendWork(ctx, 512); err != nil {
		return false, err
	}
	edge, apex := offset(p0, p1, a), offset(p0, ai, a)
	cross := RevIvCross(edge.F, apex.F)
	cl := proofbound.ProductUpper(edge.Length, apex.Length)
	cd := PerturbBilinearAllow(edge.Length, apex.Length, e, e)
	ax := replayFloatAxis{RevIvCross(cross, edge.F), proofbound.ProductUpper(cl, edge.Length), PerturbBilinearAllow(cl, edge.Length, cd, e)}
	side, ok := replayAxisSide(ax, ob, e)
	return ok && side < 0, nil
}
func auditRevolvePairReplay(ctx context.Context, data []RevolveAuditTri, tris [][3]int, shared [3]int, count, i, j int, delta float64, b *proofbound.ReplayBudget) error {
	if err := b.SpendWork(ctx, 16); err != nil {
		return err
	}
	if err := b.ReserveRecords(ctx, 8); err != nil {
		return err
	}
	if err := b.ReserveArray(ctx, 8, uint64(unsafe.Sizeof(RevolveOffset{}))); err != nil {
		return err
	}
	a, c := data[i], data[j]
	admitted := false
	var err error
	switch count {
	case 0:
		// The generic disjoint test still reserves all 17 possible axes.
		if err := b.SpendWork(ctx, 1024); err != nil {
			return err
		}
		admitted = revolveSeparatedFloat(&a, &c, delta)
	case 1:
		admitted, err = replayVertexIsolated(ctx, b, a, tris[i], c, tris[j], shared[0], delta)
		if err == nil && !admitted {
			admitted, err = replayVertexIsolated(ctx, b, c, tris[j], a, tris[i], shared[0], delta)
		}
	case 2:
		admitted, err = replayEdgeIsolated(ctx, b, a, tris[i], c, tris[j], shared, delta)
		if err == nil && !admitted {
			admitted, err = replayEdgeIsolated(ctx, b, c, tris[j], a, tris[i], shared, delta)
		}
	}
	if err != nil {
		return err
	}
	if !admitted {
		switch count {
		case 3:
			return proofbound.ReplayIndexedRefusal(ctx, b, "revolve facets ", " and ", " are the same triangle", i, j, true)
		case 0:
			return proofbound.ReplayIndexedRefusal(ctx, b, "revolve facets ", " and ", " share no vertex, and this mesh cannot prove they stay apart under its own coordinate displacement", i, j, true)
		case 1:
			return proofbound.ReplayIndexedRefusal(ctx, b, "revolve facets ", " and ", " do not provably meet only at the vertex they share", i, j, true)
		default:
			return proofbound.ReplayIndexedRefusal(ctx, b, "revolve facets ", " and ", " do not provably meet only along the edge they share", i, j, true)
		}
	}
	return nil
}
