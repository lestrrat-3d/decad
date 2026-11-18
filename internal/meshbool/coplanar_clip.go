package meshbool

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// ClipPiece is the part of one coplanar edge that lies in a closed triangle:
// the segment P0 to P1, and whether each end lies on the triangle's own
// boundary.
type ClipPiece struct {
	P0, P1       proof.Xpt
	P0OnA, P1OnA bool
}

// CoplanarClip clips edge to the closed triangle xta, exactly
// (agent-docs/coplanar-contact-design.md §3.2). The edge and the triangle lie
// in one plane with exact normal n, and the clip runs in the dominant-axis
// projection of n. The answer is false when the piece is empty, a single
// point, or runs along an edge of xta: a piece on the triangle's own boundary
// is no cut. Every value is an exact rational.
//
// An end of the piece that is an end of the edge is that edge end itself, so
// a corner shared with the mesh keeps its point identity.
func CoplanarClip(edge [2]proof.Xpt, xta [3]proof.Xpt, n proof.Xpt) (ClipPiece, bool, error) {
	u, v := ProjAxes(n)
	var tri [3]Xp2
	for i := range 3 {
		tri[i] = NewXP2(RatCoordOf(xta[i], u), RatCoordOf(xta[i], v))
	}
	a := NewXP2(RatCoordOf(edge[0], u), RatCoordOf(edge[0], v))
	b := NewXP2(RatCoordOf(edge[1], u), RatCoordOf(edge[1], v))
	ccw := Cross2xSign(tri[0], tri[1], tri[2])
	if ccw == 0 {
		return ClipPiece{}, false, fmt.Errorf(`%w: a coplanar clip met a degenerate facet`, decaderr.ErrBooleanFailed)
	}
	lo := ClipFrac{Num: big.NewInt(0), Den: big.NewInt(1)}
	hi := ClipFrac{Num: big.NewInt(1), Den: big.NewInt(1)}
	for i := range 3 {
		e0, e1 := tri[i], tri[(i+1)%3]
		sa, sb := Cross2xSign(e0, e1, a), Cross2xSign(e0, e1, b)
		if ccw < 0 {
			sa, sb = -sa, -sb
		}
		switch {
		case sa >= 0 && sb >= 0:
			continue
		case sa < 0 && sb < 0:
			return ClipPiece{}, false, nil
		}
		// Exactly one sign is negative, so the clip parameter
		// t = -fa/(fb - fa) is defined, as in SegTriOverlap2.
		fa, fb := EdgeCross2Fracs(e0, e1, a, b)
		naWb := new(big.Int).Mul(fa.Num, fb.Den)
		t := ClipFrac{
			Num: new(big.Int).Neg(naWb),
			Den: new(big.Int).Sub(new(big.Int).Mul(fb.Num, fa.Den), naWb),
		}
		if t.Den.Sign() < 0 {
			t.Num.Neg(t.Num)
			t.Den.Neg(t.Den)
		}
		if sa < 0 {
			if CmpClipFrac(t, lo) > 0 {
				lo = t
			}
			continue
		}
		if CmpClipFrac(t, hi) < 0 {
			hi = t
		}
	}
	if CmpClipFrac(lo, hi) >= 0 || (CompareCoord(edge[0], edge[1], 0) == 0 &&
		CompareCoord(edge[0], edge[1], 1) == 0 && CompareCoord(edge[0], edge[1], 2) == 0) {
		return ClipPiece{}, false, nil
	}
	piece := ClipPiece{P0: edge[0], P1: edge[1]}
	if lo.Num.Sign() != 0 {
		piece.P0 = proof.Xlerp(edge[0], edge[1], lo.Num, lo.Den)
	}
	if hi.Num.Cmp(hi.Den) != 0 {
		piece.P1 = proof.Xlerp(edge[0], edge[1], hi.Num, hi.Den)
	}
	if SegAlongEdge(piece.P0, piece.P1, xta, n) >= 0 {
		return ClipPiece{}, false, nil
	}
	piece.P0OnA = OnTriBoundary(piece.P0, xta, n)
	piece.P1OnA = OnTriBoundary(piece.P1, xta, n)
	return piece, true, nil
}
