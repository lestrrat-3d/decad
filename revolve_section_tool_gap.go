package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// revolveSectionToolGap charges the finite meridian mismatch between each
// trimming cone and the two section spans meeting the selected root face.
// The endpoints come from the recorded section walk, not profile positions.
func revolveSectionToolGap(section revolveSectionCertificate,
	record *pointConeTrimRecord) (float64, bool) {
	if record == nil || record.lower == nil || record.upper == nil ||
		len(section.spans) < 4 || section.rootSpan < 0 ||
		section.rootSpan >= len(section.spans) {
		return 0, false
	}
	root := section.spans[section.rootSpan]
	var neighbors [2]planePoint
	if root.start.U < root.end.U {
		neighbors[0] = section.spans[(section.rootSpan+len(section.spans)-1)%len(section.spans)].start
		neighbors[1] = section.spans[(section.rootSpan+1)%len(section.spans)].end
	} else {
		neighbors[0] = section.spans[(section.rootSpan+1)%len(section.spans)].end
		neighbors[1] = section.spans[(section.rootSpan+len(section.spans)-1)%len(section.spans)].start
	}
	x0, x1 := proofarith.FloatRat(section.rootStart.U), proofarith.FloatRat(section.rootEnd.U)
	r0, r1 := proofarith.FloatRat(section.rootStart.V), proofarith.FloatRat(section.rootEnd.V)
	dx := new(big.Rat).Sub(x1, x0)
	if dx.Sign() <= 0 {
		return 0, false
	}
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), dx)
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	maxGap := 0.0
	for i, item := range []struct {
		limit *pointConeLimit
		root  planePoint
	}{{record.lower, section.rootStart}, {record.upper, section.rootEnd}} {
		outer := neighbors[i]
		if outer.V <= 0 || outer == item.root {
			return 0, false
		}
		residual := func(p planePoint) *big.Rat {
			value := new(big.Rat).Sub(new(big.Rat).Add(
				proofarith.FloatRat(p.U),
				new(big.Rat).Mul(item.limit.slope, proofarith.FloatRat(p.V))),
				item.limit.apex)
			return value.Abs(value)
		}
		maxGap = max(maxGap, proofbound.RatFloatUp(residual(item.root)),
			proofbound.RatFloatUp(residual(outer)))
		den := new(big.Rat).Add(big.NewRat(1, 1),
			new(big.Rat).Mul(item.limit.slope, m))
		if den.Sign() <= 0 {
			return 0, false
		}
		crossX := new(big.Rat).Quo(new(big.Rat).Sub(item.limit.apex,
			new(big.Rat).Mul(item.limit.slope, b)), den)
		xGap := new(big.Rat).Sub(crossX, proofarith.FloatRat(item.root.U))
		rGap := new(big.Rat).Mul(m, xGap)
		distance2 := new(big.Rat).Add(new(big.Rat).Mul(xGap, xGap),
			new(big.Rat).Mul(rGap, rGap))
		maxGap = max(maxGap, proofbound.RatSqrtUp(distance2))
	}
	return maxGap, !proofbound.IsNonFinite(maxGap)
}
