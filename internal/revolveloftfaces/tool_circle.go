package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ToolCircleOnSpan finds a possible exact contact circle between a finite
// Revolve generator and a LoftFaces trim cone.
func ToolCircleOnSpan(tool *facetedrecord.PointConeLimit, span facetedrecord.RevolveSectionSpan) (
	*big.Rat, *big.Rat, bool, bool,
) {
	if tool == nil || tool.Apex == nil || tool.Far == nil || tool.Slope == nil ||
		tool.Slope.Sign() <= 0 || tool.Far.Cmp(tool.Apex) >= 0 ||
		span.Start == span.End || span.Start.V < 0 || span.End.V < 0 {
		return nil, nil, false, false
	}
	a := toolCircleResidual(tool, span.Start)
	b := toolCircleResidual(tool, span.End)
	if a.Sign() != 0 && a.Sign() == b.Sign() {
		return nil, nil, false, true
	}
	if a.Sign() == 0 || b.Sign() == 0 || a.Cmp(b) == 0 {
		return nil, nil, false, false
	}
	fraction := proofarith.QuoRat(new(big.Rat), a, lfSub(a, b))
	if fraction.Sign() <= 0 || fraction.Cmp(big.NewRat(1, 1)) >= 0 {
		return nil, nil, false, false
	}
	x := lfAdd(lfF(span.Start.U), lfMul(fraction,
		lfSub(lfF(span.End.U), lfF(span.Start.U))))
	radius := lfAdd(lfF(span.Start.V), lfMul(fraction,
		lfSub(lfF(span.End.V), lfF(span.Start.V))))
	if radius.Sign() <= 0 || x.Cmp(tool.Far) <= 0 || x.Cmp(tool.Apex) >= 0 {
		return nil, nil, false, false
	}
	return x, radius, true, true
}

func toolCircleResidual(tool *facetedrecord.PointConeLimit, point sectionrecord.Point2) *big.Rat {
	return lfSub(lfAdd(lfF(point.U), lfMul(tool.Slope, lfF(point.V))), tool.Apex)
}
