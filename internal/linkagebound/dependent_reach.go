package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// DependentReach is the reach guard of agent-docs/linkage-check-design.md
// §15.5 over a decomposed loop drive. Bound holds each dependent's reach m_j,
// the largest |value| in the hull of every ask the decomposition certified,
// and Certified the stretches [a, b] of the fraction it certified, in
// increasing order and overlapping at most at their ends. Every swept box,
// ball reading and layer exclusion is grown by Bound, so no claim may be
// made about a value past it.
type DependentReach struct {
	Bound     []*big.Rat
	Certified [][2]*big.Rat
}

// PointPast reports whether dependent j's enclosure v at one point reaches
// past m_j. A point is always held to m_j: its enclosure's midpoint is where
// §5.8's cell form expands the cell's corners.
func (r DependentReach) PointPast(j int, v proofbound.RatInterval) bool {
	return Magnitude(v).Cmp(r.Bound[j]) > 0
}

// StretchPast reports whether dependent j's hull h over the stretch [a, b]
// refuses it: h reaches past m_j and [a, b] does not lie inside the certified
// stretches. Inside them every exact value is one the decomposition's hull
// already encloses, so h can pass m_j only by its own pieces' overshoot,
// which says nothing about the values.
func (r DependentReach) StretchPast(j int, h proofbound.RatInterval, a, b *big.Rat) bool {
	return Magnitude(h).Cmp(r.Bound[j]) > 0 && !r.Covers(a, b)
}

// Covers reports whether every fraction of [a, b] lies in a certified
// stretch.
func (r DependentReach) Covers(a, b *big.Rat) bool {
	at := a
	for _, c := range r.Certified {
		if c[1].Cmp(at) < 0 {
			continue
		}
		if c[0].Cmp(at) > 0 {
			return false
		}
		at = c[1]
		if at.Cmp(b) >= 0 {
			return true
		}
	}
	return false
}
