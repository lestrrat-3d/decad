package capband

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// JoinTurnSinSq is the exact sin²θ between prev's walk-end tangent and cur's
// walk-start tangent at the recorded point where prev ends and cur starts
// (agent-docs/boss-root-overflow-design.md §5.5.2). It reads the same
// rationals JoinIsG1 and NearTangentCorner read: it is sin²θ = cross²/(|a|²|b|²)
// of the two exact tangents. ok is false when the two recorded ends differ, a
// coordinate denotes no rational, a segment is of a kind those tests do not
// pair, or the tangents do not run the same way (dot <= 0), where sin²θ would
// not measure a forward turn. The value decides nothing by itself: a caller
// admits a join only through JoinIsG1 or NearTangentCorner and reads this to
// bound the turn that admission left.
func JoinTurnSinSq(prev, cur sectionrecord.CurveSegment) (*big.Rat, bool) {
	pc, okP := recordedCorner(prev)
	cc, okC := recordedCorner(cur)
	if !okP || !okC {
		return nil, false
	}
	p, okP := joinEnds(pc)
	c, okC := joinEnds(cc)
	if !okP || !okC {
		return nil, false
	}
	if p.end[0].Cmp(c.start[0]) != 0 || p.end[1].Cmp(c.start[1]) != 0 {
		return nil, false
	}
	a, b := p.tanEnd, c.tanStart
	if ratDot(a, b).Sign() <= 0 {
		return nil, false
	}
	cross := new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0]))
	sinSq := new(big.Rat).Mul(cross, cross)
	return sinSq.Quo(sinSq, new(big.Rat).Mul(ratDot(a, a), ratDot(b, b))), true
}
