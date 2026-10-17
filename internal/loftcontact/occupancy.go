package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// CertifyCarrierOccupancy proves a strict partition of a fixed straight
// carrier by a moving convex line loop. At every height, the region inside
// the loop occupies exactly the open interval between the returned seams. The
// crossing edge indices follow carrier order; vertex sides are +1 to its
// left and -1 to its right. A crossing edge has opposite endpoint sides.
//
// The caller must authenticate paired full-domain Loft walls, their common
// axial frame, and the prism carrier. This is only a source certificate: it
// does not certify output closure, bounds, or a Boolean result.
func CertifyCarrierOccupancy(ctx context.Context, budget *proofbound.WorkBudget,
	bottom, top []sectionrecord.LineSeg, carrier [2]sectionrecord.Point2,
) ([2]int, [2]LineSeam, []int8, bool, error) {
	var noEdges [2]int
	var noSeams [2]LineSeam
	if err := ctx.Err(); err != nil {
		return noEdges, noSeams, nil, false, err
	}
	// The finite ceiling bounds exact arithmetic as well as the quadratic
	// edge/vertex scan. Larger loops need a separate work certificate.
	n := len(bottom)
	if n < 3 || n > 32 || len(top) != n {
		return noEdges, noSeams, nil, false, nil
	}
	c, ok := lift(carrier[0])
	if !ok {
		return noEdges, noSeams, nil, false, nil
	}
	d, ok := lift(carrier[1])
	if !ok {
		return noEdges, noSeams, nil, false, nil
	}
	v := sub(d, c)
	if v.u.Sign() == 0 && v.v.Sign() == 0 {
		return noEdges, noSeams, nil, false, nil
	}
	starts0, ends0 := make([]point, n), make([]point, n)
	starts1, ends1 := make([]point, n), make([]point, n)
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return noEdges, noSeams, nil, false, err
		}
		if bottom[i].TStart != 0 || bottom[i].TEnd != 1 ||
			top[i].TStart != 0 || top[i].TEnd != 1 {
			return noEdges, noSeams, nil, false, nil
		}
		var valid bool
		starts0[i], valid = lift(bottom[i].Start)
		if !valid {
			return noEdges, noSeams, nil, false, nil
		}
		ends0[i], valid = lift(bottom[i].End)
		if !valid {
			return noEdges, noSeams, nil, false, nil
		}
		starts1[i], valid = lift(top[i].Start)
		if !valid {
			return noEdges, noSeams, nil, false, nil
		}
		ends1[i], valid = lift(top[i].End)
		if !valid {
			return noEdges, noSeams, nil, false, nil
		}
	}
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return noEdges, noSeams, nil, false, err
		}
		next := (i + 1) % n
		if !samePoint(ends0[i], starts0[next]) || !samePoint(ends1[i], starts1[next]) {
			return noEdges, noSeams, nil, false, nil
		}
	}

	// Every other vertex must stay strictly on the same interior side of
	// every moving edge. Each orientation is quadratic in height. Strict
	// Bernstein coefficients certify a simple, strictly convex loop for
	// the whole interval, including between the two recorded sections.
	winding := 0
	for i := range bottom {
		e0, e1 := sub(ends0[i], starts0[i]), sub(ends1[i], starts1[i])
		for j := range bottom {
			if j == i || j == (i+1)%n {
				continue
			}
			if err := budget.Step(); err != nil {
				return noEdges, noSeams, nil, false, err
			}
			f0, f1 := sub(starts0[j], starts0[i]), sub(starts1[j], starts1[i])
			turn := crossMoving(e0, sub(e1, e0), f0, sub(f1, f0))
			if winding == 0 {
				winding = turn[0].Sign()
			}
			if winding == 0 || !sameSignBernstein(turn, winding) {
				return noEdges, noSeams, nil, false, nil
			}
		}
	}

	sides := make([]int8, n)
	var crossing [2]int
	count := 0
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return noEdges, noSeams, nil, false, err
		}
		g0 := cross(v, sub(starts0[i], c))
		g1 := cross(v, sub(starts1[i], c))
		if g0.Sign() == 0 || g1.Sign() != g0.Sign() {
			return noEdges, noSeams, nil, false, nil
		}
		sides[i] = int8(g0.Sign())
	}
	for i := range bottom {
		if sides[i] != sides[(i+1)%n] {
			if count == len(crossing) {
				return noEdges, noSeams, nil, false, nil
			}
			crossing[count] = i
			count++
		}
	}
	if count != 2 {
		return noEdges, noSeams, nil, false, nil
	}
	var first [2]LineSeam
	for k, i := range crossing {
		var admitted bool
		var err error
		first[k], admitted, err = CertifyLineSeam(ctx, budget,
			[2]sectionrecord.Point2{bottom[i].Start, bottom[i].End},
			[2]sectionrecord.Point2{top[i].Start, top[i].End}, carrier)
		if err != nil || !admitted {
			return noEdges, noSeams, nil, false, err
		}
	}
	_, v0, _ := first[0].At(big.NewRat(0, 1))
	_, v1, _ := first[1].At(big.NewRat(0, 1))
	if v0.Cmp(v1) == 0 {
		return noEdges, noSeams, nil, false, nil
	}
	if v0.Cmp(v1) > 0 {
		crossing[0], crossing[1] = crossing[1], crossing[0]
	}
	a, b := crossing[0], crossing[1]
	seams, admitted, err := CertifyOrderedLineSeams(ctx, budget,
		[2]sectionrecord.Point2{bottom[a].Start, bottom[a].End},
		[2]sectionrecord.Point2{top[a].Start, top[a].End},
		[2]sectionrecord.Point2{bottom[b].Start, bottom[b].End},
		[2]sectionrecord.Point2{top[b].Start, top[b].End}, carrier)
	if err != nil || !admitted {
		return noEdges, noSeams, nil, false, err
	}
	return crossing, seams, sides, true, nil
}

func samePoint(a, b point) bool { return a.u.Cmp(b.u) == 0 && a.v.Cmp(b.v) == 0 }

func crossMoving(a0, a1, b0, b1 point) quadratic {
	return quadratic{
		cross(a0, b0),
		add(cross(a0, b1), cross(a1, b0)),
		cross(a1, b1),
	}
}
