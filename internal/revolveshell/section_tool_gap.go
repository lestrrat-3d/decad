package revolveshell

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RevolveSectionToolGap charges the finite meridian mismatch between each
// trimming cone and the two section spans meeting the selected root face.
// The endpoints come from the recorded section walk, not profile positions.
func RevolveSectionToolGap(section facetedrecord.RevolveSectionCertificate,
	record *facetedrecord.PointConeTrimRecord) (float64, bool) {
	if record == nil || record.Lower == nil || record.Upper == nil ||
		len(section.Spans) < 4 || section.RootSpan < 0 ||
		section.RootSpan >= len(section.Spans) {
		return 0, false
	}
	root := section.Spans[section.RootSpan]
	var neighbors [2]sectionrecord.Point2
	if root.Start.U < root.End.U {
		neighbors[0] = section.Spans[(section.RootSpan+len(section.Spans)-1)%len(section.Spans)].Start
		neighbors[1] = section.Spans[(section.RootSpan+1)%len(section.Spans)].End
	} else {
		neighbors[0] = section.Spans[(section.RootSpan+1)%len(section.Spans)].End
		neighbors[1] = section.Spans[(section.RootSpan+len(section.Spans)-1)%len(section.Spans)].Start
	}
	x0, x1 := proofarith.FloatRat(section.RootStart.U), proofarith.FloatRat(section.RootEnd.U)
	r0, r1 := proofarith.FloatRat(section.RootStart.V), proofarith.FloatRat(section.RootEnd.V)
	dx := new(big.Rat).Sub(x1, x0)
	if dx.Sign() <= 0 {
		return 0, false
	}
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), dx)
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	maxGap := 0.0
	for i, item := range []struct {
		limit *facetedrecord.PointConeLimit
		root  sectionrecord.Point2
	}{{record.Lower, section.RootStart}, {record.Upper, section.RootEnd}} {
		outer := neighbors[i]
		if outer.V <= 0 || outer == item.root {
			return 0, false
		}
		residual := func(p sectionrecord.Point2) *big.Rat {
			value := new(big.Rat).Sub(new(big.Rat).Add(
				proofarith.FloatRat(p.U),
				new(big.Rat).Mul(item.limit.Slope, proofarith.FloatRat(p.V))),
				item.limit.Apex)
			return value.Abs(value)
		}
		maxGap = max(maxGap, proofbound.RatFloatUp(residual(item.root)),
			proofbound.RatFloatUp(residual(outer)))
		den := new(big.Rat).Add(big.NewRat(1, 1),
			new(big.Rat).Mul(item.limit.Slope, m))
		if den.Sign() <= 0 {
			return 0, false
		}
		crossX := new(big.Rat).Quo(new(big.Rat).Sub(item.limit.Apex,
			new(big.Rat).Mul(item.limit.Slope, b)), den)
		xGap := new(big.Rat).Sub(crossX, proofarith.FloatRat(item.root.U))
		rGap := new(big.Rat).Mul(m, xGap)
		distance2 := new(big.Rat).Add(new(big.Rat).Mul(xGap, xGap),
			new(big.Rat).Mul(rGap, rGap))
		maxGap = max(maxGap, proofbound.RatSqrtUp(distance2))
	}
	return maxGap, !proofbound.IsNonFinite(maxGap)
}
