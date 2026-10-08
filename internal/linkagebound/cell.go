package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Midpoint is an interval's exact midpoint.
func Midpoint(iv proofbound.RatInterval) *big.Rat {
	mid := new(big.Rat).Add(iv.Lo, iv.Hi)
	return mid.Quo(mid, big.NewRat(2, 1))
}

// CellReach encloses a dependent joint around its cell-centre reading.
type CellReach struct {
	Centre, Half *big.Rat
}

// DependentCellReach reads the farthest hull end from the centre of m.
func DependentCellReach(m, hull proofbound.RatInterval) CellReach {
	lo, hi := m.Lo, m.Hi
	if hull.Lo.Cmp(lo) < 0 {
		lo = hull.Lo
	}
	if hull.Hi.Cmp(hi) > 0 {
		hi = hull.Hi
	}
	centre := Midpoint(m)
	h := new(big.Rat).Sub(hi, centre)
	if low := new(big.Rat).Sub(centre, lo); low.Cmp(h) > 0 {
		h = low
	}
	return CellReach{Centre: centre, Half: h}
}

// DependentCellDelta bounds the change from a centre value in m to any
// value in the whole cell's hull h.
func DependentCellDelta(m, h proofbound.RatInterval) *big.Rat {
	delta := new(big.Rat).Sub(h.Hi, m.Lo)
	if other := new(big.Rat).Sub(m.Hi, h.Lo); other.Cmp(delta) > 0 {
		delta = other
	}
	return delta
}

// RankAxes chooses the eligible joint with the largest bound-defect share.
// A zero share cannot improve a projection bound by splitting.
func RankAxes(axes []int, shares map[int]*big.Rat, projected bool, eligible func(int) bool) int {
	axis := -1
	var top *big.Rat
	for _, k := range axes {
		if !eligible(k) {
			continue
		}
		v := shares[k]
		if v == nil {
			v = new(big.Rat)
		}
		if projected && v.Sign() == 0 {
			continue
		}
		if axis < 0 || v.Cmp(top) > 0 {
			axis, top = k, v
		}
	}
	return axis
}

// ProjectionAxes maps the joints below an ancestor to cell split axes.
func ProjectionAxes(path []int, below int, axisOf func(int) int) []int {
	axes := make([]int, len(path)-below)
	for n, joint := range path[below:] {
		axes[n] = axisOf(joint)
	}
	return axes
}

// JointTerm is one joint's weighted travel, charged to a cell split axis.
type JointTerm struct {
	Axis  int
	Value *big.Rat
}

// TermsOnPath reads the weighted travel below an ancestor from a link bound.
func TermsOnPath(bound ReachBound, below int, span func(int) (int, *big.Rat)) []JointTerm {
	out := make([]JointTerm, 0, len(bound.Path)-below)
	for n := below; n < len(bound.Path); n++ {
		axis, term := span(bound.Path[n])
		if bound.Rho[n] != nil {
			term.Mul(term, bound.Rho[n])
		}
		out = append(out, JointTerm{Axis: axis, Value: term})
	}
	return out
}

// SumTerms adds every joint's travel share to its split axis.
func SumTerms(terms []JointTerm) map[int]*big.Rat {
	out := make(map[int]*big.Rat)
	for _, t := range terms {
		if cur, ok := out[t.Axis]; ok {
			cur.Add(cur, t.Value)
			continue
		}
		out[t.Axis] = new(big.Rat).Set(t.Value)
	}
	return out
}

// HalfSum is the one-sided chain travel from a cell's centre.
func HalfSum(terms []JointTerm) *big.Rat {
	sum := new(big.Rat)
	for _, t := range terms {
		sum.Add(sum, t.Value)
	}
	return sum.Quo(sum, big.NewRat(2, 1))
}
