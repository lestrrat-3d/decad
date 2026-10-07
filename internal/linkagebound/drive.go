package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/motionbound"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// JointReach is the largest joint distance from zero at its waypoints.
func JointReach(points []motionbound.MotionParam) *big.Rat {
	zero := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat)}
	var out *big.Rat
	for _, p := range points {
		if m := zero.SpanUpper(p); out == nil || m.Cmp(out) > 0 {
			out = m
		}
	}
	return out
}

// JointSpan bounds total joint travel between fractions, including reversals
// at waypoints strictly inside the interval.
func JointSpan(points []motionbound.MotionParam, sa, sb *big.Rat,
	paramAt func(*big.Rat) motionbound.MotionParam,
) *big.Rat {
	lo, hi := sa, sb
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	n := len(points) - 1
	sum := new(big.Rat)
	prev := paramAt(lo)
	for j := 1; j < n; j++ {
		w := big.NewRat(int64(j), int64(n))
		if w.Cmp(lo) <= 0 || w.Cmp(hi) >= 0 {
			continue
		}
		sum.Add(sum, prev.SpanUpper(points[j]))
		prev = points[j]
	}
	return sum.Add(sum, prev.SpanUpper(paramAt(hi)))
}

// PathTravel sums the joint travel terms below an ancestor. Span returns a
// fresh rational for each joint, or nil when that joint lacks a bound.
func PathTravel(path []int, rho []*big.Rat, below int, span func(joint int) *big.Rat) *big.Rat {
	sum := new(big.Rat)
	for n := below; n < len(path); n++ {
		term := span(path[n])
		if term == nil {
			return nil
		}
		if rho[n] != nil {
			term.Mul(term, rho[n])
		}
		sum.Add(sum, term)
	}
	return sum
}

// CommonDepth counts the leading joints shared by two paths.
func CommonDepth(a, b []int) int {
	n := 0
	for n < len(a) && n < len(b) && a[n] == b[n] {
		n++
	}
	return n
}

// JointStep encloses the signed change across one affine joint segment.
// It refuses a waypoint that bends the schedule strictly inside the interval.
func JointStep(points []motionbound.MotionParam, sa, sb *big.Rat,
	paramAt func(*big.Rat) motionbound.MotionParam,
) (proofbound.RatInterval, bool) {
	n := len(points) - 1
	for j := 1; j < n; j++ {
		w := big.NewRat(int64(j), int64(n))
		if w.Cmp(sa) <= 0 || w.Cmp(sb) >= 0 {
			continue
		}
		prev, at, next := points[j-1], points[j], points[j+1]
		bent := new(big.Rat).Sub(at.Turn, prev.Turn).Cmp(new(big.Rat).Sub(next.Turn, at.Turn)) != 0 ||
			new(big.Rat).Sub(at.Base, prev.Base).Cmp(new(big.Rat).Sub(next.Base, at.Base)) != 0
		if bent {
			return proofbound.RatInterval{}, false
		}
	}
	a, b := paramAt(sa), paramAt(sb)
	turn := new(big.Rat).Sub(b.Turn, a.Turn)
	base := new(big.Rat).Sub(b.Base, a.Base)
	step := proofbound.IntervalAdd(proofbound.IntervalScale(proofbound.TwoPiInterval(), turn), proofbound.PointInterval(base))
	return RoundOut(step)
}
